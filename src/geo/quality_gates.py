"""F.O.R.G.E — Reconstruction & Geospatial Quality Gates Evaluation Engine.

Provides automated multi-criteria geometric and geospatial quality verification:
1. Valid mesh geometry & non-empty point clouds (vertex, triangle, point count thresholds).
2. Bounding-box dimensions & extreme-coordinate needle outlier detection.
3. Topological integrity (boundary edge ratio, connected triangle cluster dominance).
4. Camera trajectory consistency & frame coverage.
5. Geospatial similarity-transform conditioning, inlier ratio, and residual RMSE.
6. Transformation orientation sanity (detecting degenerate 1D tilts into the vertical axis).

Distinguishes hard failures from warnings with scene-justified physical thresholds.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple, Union

import numpy as np
import open3d as o3d


@dataclass
class QualityCheck:
    """Individual quality gate test result."""
    name: str
    category: str
    status: str  # "PASS", "WARN", "FAIL"
    metric_value: Any
    threshold_str: str
    message: str


@dataclass
class QualityGateReport:
    """Comprehensive quality evaluation report for a reconstruction model."""
    target_id: str
    overall_verdict: str  # "PASS", "PASS_WITH_WARNINGS", "FAIL"
    passed: bool
    checks: List[QualityCheck] = field(default_factory=list)
    hard_failures: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "target_id": self.target_id,
            "overall_verdict": self.overall_verdict,
            "passed": self.passed,
            "num_checks": len(self.checks),
            "num_failures": len(self.hard_failures),
            "num_warnings": len(self.warnings),
            "hard_failures": self.hard_failures,
            "warnings": self.warnings,
            "checks": [asdict(c) for c in self.checks]
        }


def evaluate_reconstruction_quality_gate(
    target_id: str,
    mesh_path: Union[str, Path],
    point_cloud_path: Optional[Union[str, Path]] = None,
    camera_poses_path: Optional[Union[str, Path]] = None,
    alignment_metadata: Optional[Dict[str, Any]] = None
) -> QualityGateReport:
    """
    Evaluates comprehensive geometric, topological, and geospatial quality gates.

    Args:
        target_id: Identifier of the target scene.
        mesh_path: Path to reconstructed or aligned PLY mesh.
        point_cloud_path: Optional path to fused point cloud PLY.
        camera_poses_path: Optional path to camera poses NPZ.
        alignment_metadata: Optional dictionary of GPS alignment metadata if alignment was performed.

    Returns:
        QualityGateReport with detailed checks, status, warnings, and failures.
    """
    mesh_p = Path(mesh_path)
    checks: List[QualityCheck] = []
    hard_failures: List[str] = []
    warnings: List[str] = []

    # -------------------------------------------------------------------------
    # 1. Mesh File Existence and Loadability
    # -------------------------------------------------------------------------
    if not mesh_p.is_file():
        checks.append(QualityCheck(
            name="mesh_file_exists",
            category="file_io",
            status="FAIL",
            metric_value=False,
            threshold_str="File exists on disk",
            message=f"Mesh file does not exist: {mesh_p}"
        ))
        hard_failures.append(f"Mesh file not found: {mesh_p}")
        return QualityGateReport(
            target_id=target_id,
            overall_verdict="FAIL",
            passed=False,
            checks=checks,
            hard_failures=hard_failures,
            warnings=warnings
        )

    mesh = o3d.io.read_triangle_mesh(str(mesh_p))
    n_verts = len(mesh.vertices)
    n_tris = len(mesh.triangles)

    # Vertex count check
    if n_verts >= 20000:
        v_status = "PASS"
        v_msg = f"Dense vertex count: {n_verts:,}"
    elif n_verts >= 5000:
        v_status = "WARN"
        v_msg = f"Moderate vertex count: {n_verts:,} (sparse geometry)"
        warnings.append(v_msg)
    else:
        v_status = "FAIL"
        v_msg = f"Insufficient vertex count: {n_verts:,} (< 5,000 threshold)"
        hard_failures.append(v_msg)

    checks.append(QualityCheck(
        name="mesh_vertex_density",
        category="geometry",
        status=v_status,
        metric_value=n_verts,
        threshold_str=">= 20,000 (Pass), >= 5,000 (Warn)",
        message=v_msg
    ))

    # Triangle count check
    if n_tris >= 40000:
        t_status = "PASS"
        t_msg = f"Solid triangle coverage: {n_tris:,}"
    elif n_tris >= 10000:
        t_status = "WARN"
        t_msg = f"Low triangle coverage: {n_tris:,}"
        warnings.append(t_msg)
    else:
        t_status = "FAIL"
        t_msg = f"Severely deficient triangle coverage: {n_tris:,} (< 10,000 threshold)"
        hard_failures.append(t_msg)

    checks.append(QualityCheck(
        name="mesh_triangle_density",
        category="geometry",
        status=t_status,
        metric_value=n_tris,
        threshold_str=">= 40,000 (Pass), >= 10,000 (Warn)",
        message=t_msg
    ))

    # -------------------------------------------------------------------------
    # 2. Bounding Box & Aspect Ratio (Outlier Needle Spikes)
    # -------------------------------------------------------------------------
    if n_verts > 0:
        pts = np.asarray(mesh.vertices)
        ptp = np.ptp(pts, axis=0)  # extents [X, Y, Z]
        sorted_ext = np.sort(ptp)
        min_dim = max(float(sorted_ext[0]), 1e-3)
        max_dim = float(sorted_ext[-1])
        aspect_ratio = max_dim / min_dim

        if aspect_ratio < 25.0:
            asp_status = "PASS"
            asp_msg = f"Bounding box extents [{ptp[0]:.2f}, {ptp[1]:.2f}, {ptp[2]:.2f}], aspect ratio {aspect_ratio:.2f} is well-proportioned."
        elif aspect_ratio < 60.0:
            asp_status = "WARN"
            asp_msg = f"High aspect ratio ({aspect_ratio:.2f}); possible elongated corridor or background depth fringe."
            warnings.append(asp_msg)
        else:
            asp_status = "FAIL"
            asp_msg = f"Severe coordinate spike / needle geometry (aspect ratio {aspect_ratio:.2f} >= 60.0). Probable runaway ray artifact."
            hard_failures.append(asp_msg)

        checks.append(QualityCheck(
            name="bounding_box_aspect_ratio",
            category="geometry",
            status=asp_status,
            metric_value=round(aspect_ratio, 2),
            threshold_str="< 25.0 (Pass), < 60.0 (Warn)",
            message=asp_msg
        ))

    # -------------------------------------------------------------------------
    # 3. Topological Integrity & Disconnected Fragments
    # -------------------------------------------------------------------------
    if n_tris > 0:
        # Connected components clustering
        triangle_clusters, num_triangles, _ = mesh.cluster_connected_triangles()
        cluster_sizes = np.asarray(num_triangles)
        n_clusters = len(cluster_sizes)
        largest_cluster = cluster_sizes.max() if n_clusters > 0 else 0
        largest_cluster_pct = (largest_cluster / n_tris) * 100.0

        if largest_cluster_pct >= 75.0:
            cluster_status = "PASS"
            cluster_msg = f"Solid surface continuity: {largest_cluster_pct:.1f}% in main component ({n_clusters} clusters)."
        elif largest_cluster_pct >= 40.0:
            cluster_status = "WARN"
            cluster_msg = f"Moderate terrain segmentation: {largest_cluster_pct:.1f}% in main component ({n_clusters} clusters). Multi-bank or segmented features present."
            warnings.append(cluster_msg)
        else:
            cluster_status = "FAIL"
            cluster_msg = f"Severe speckling & fragmentation: only {largest_cluster_pct:.1f}% in main component (< 40% threshold, {n_clusters} clusters)."
            hard_failures.append(cluster_msg)

        checks.append(QualityCheck(
            name="topological_clustering",
            category="topology",
            status=cluster_status,
            metric_value=round(largest_cluster_pct, 1),
            threshold_str=">= 75% (Pass), >= 40% (Warn)",
            message=cluster_msg
        ))

        # Boundary edge ratio
        # Non-watertight surfaces have boundary edges where edge is shared by only 1 triangle
        # For terrain/corridors, boundary edges around perimeter are normal, but internal holes cause excess
        edge_to_triangles = {}
        tri_arr = np.asarray(mesh.triangles)
        for t in tri_arr:
            edges = [
                tuple(sorted([t[0], t[1]])),
                tuple(sorted([t[1], t[2]])),
                tuple(sorted([t[2], t[0]]))
            ]
            for e in edges:
                edge_to_triangles[e] = edge_to_triangles.get(e, 0) + 1

        boundary_edges = sum(1 for cnt in edge_to_triangles.values() if cnt == 1)
        boundary_ratio = (boundary_edges / n_tris) if n_tris > 0 else 0.0

        if boundary_ratio < 0.08:
            b_status = "PASS"
            b_msg = f"Solid mesh boundary ratio: {boundary_ratio * 100.0:.2f}% ({boundary_edges:,} open edges)."
        elif boundary_ratio < 0.20:
            b_status = "WARN"
            b_msg = f"Elevated open boundary edges: {boundary_ratio * 100.0:.2f}% ({boundary_edges:,} edges). Some surface holes present."
            warnings.append(b_msg)
        else:
            b_status = "FAIL"
            b_msg = f"Excessive mesh holes: {boundary_ratio * 100.0:.2f}% boundary ratio (> 20.0% threshold)."
            hard_failures.append(b_msg)

        checks.append(QualityCheck(
            name="surface_boundary_continuity",
            category="topology",
            status=b_status,
            metric_value=round(boundary_ratio * 100.0, 2),
            threshold_str="< 8.0% (Pass), < 20.0% (Warn)",
            message=b_msg
        ))

    # -------------------------------------------------------------------------
    # 4. Point Cloud Verification
    # -------------------------------------------------------------------------
    if point_cloud_path and Path(point_cloud_path).is_file():
        pcd = o3d.io.read_point_cloud(str(point_cloud_path))
        n_pts = len(pcd.points)
        if n_pts >= 50000:
            p_status = "PASS"
            p_msg = f"Dense point cloud: {n_pts:,} points."
        elif n_pts >= 10000:
            p_status = "WARN"
            p_msg = f"Moderate point cloud: {n_pts:,} points."
            warnings.append(p_msg)
        else:
            p_status = "FAIL"
            p_msg = f"Deficient point cloud: {n_pts:,} points (< 10,000 threshold)."
            hard_failures.append(p_msg)

        checks.append(QualityCheck(
            name="point_cloud_density",
            category="geometry",
            status=p_status,
            metric_value=n_pts,
            threshold_str=">= 50,000 (Pass), >= 10,000 (Warn)",
            message=p_msg
        ))

    # -------------------------------------------------------------------------
    # 5. Camera Poses & Trajectory Consistency
    # -------------------------------------------------------------------------
    if camera_poses_path and Path(camera_poses_path).is_file():
        poses_data = np.load(str(camera_poses_path))
        extr = poses_data.get("extrinsics")
        if extr is not None and len(extr) >= 3:
            # Check rotation determinants
            dets = [float(np.linalg.det(extr[i, :3, :3])) for i in range(len(extr))]
            all_valid_rot = all(abs(d - 1.0) < 1e-3 for d in dets)

            if all_valid_rot:
                pose_status = "PASS"
                pose_msg = f"Trajectory verified: {len(extr)} valid camera poses in SO(3)."
            else:
                pose_status = "FAIL"
                pose_msg = f"Invalid camera rotation matrices detected (determinants not 1.0)."
                hard_failures.append(pose_msg)

            checks.append(QualityCheck(
                name="camera_pose_validity",
                category="kinematics",
                status=pose_status,
                metric_value=len(extr),
                threshold_str=">= 3 frames & SO(3) valid rotations",
                message=pose_msg
            ))
        else:
            checks.append(QualityCheck(
                name="camera_pose_validity",
                category="kinematics",
                status="FAIL",
                metric_value=len(extr) if extr is not None else 0,
                threshold_str=">= 3 frames",
                message="Less than 3 camera poses available in poses NPZ."
            ))
            hard_failures.append("Insufficient camera poses (< 3).")

    # -------------------------------------------------------------------------
    # 6. Geospatial Alignment Quality Gates (when GPS was attempted)
    # -------------------------------------------------------------------------
    if alignment_metadata is not None:
        align_st = alignment_metadata.get("alignment_status", "")
        traj_cond = alignment_metadata.get("trajectory_conditioning", {})
        sim_xf = alignment_metadata.get("similarity_transformation", {})
        fit_st = alignment_metadata.get("fit_statistics", {})

        # Trajectory Baseline Conditioning
        cond_verdict = traj_cond.get("conditioning_verdict", "N/A")
        horiz_span_m = traj_cond.get("horizontal_span_gps_m", 0.0)

        if cond_verdict == "WELL_CONSTRAINED_TRANSLATION":
            geo_cond_status = "PASS"
            geo_cond_msg = f"Flight trajectory is well-conditioned: {horiz_span_m:.2f}m horizontal baseline span."
        elif cond_verdict == "WEAK_HORIZONTAL_BASELINE_HOVER":
            geo_cond_status = "WARN"
            geo_cond_msg = (
                f"Weak horizontal baseline ({horiz_span_m:.2f}m < 1.5m noise floor). "
                f"Stationary hover flight prevents reliable horizontal orientation."
            )
            warnings.append(geo_cond_msg)
        else:
            geo_cond_status = "FAIL"
            geo_cond_msg = f"Degenerate flight trajectory ({cond_verdict}): baseline translation is below sensor resolution."
            hard_failures.append(geo_cond_msg)

        checks.append(QualityCheck(
            name="gps_trajectory_conditioning",
            category="geospatial",
            status=geo_cond_status,
            metric_value=cond_verdict,
            threshold_str="WELL_CONSTRAINED_TRANSLATION (Pass)",
            message=geo_cond_msg
        ))

        # Scale Factor Sanity
        scale = sim_xf.get("scale_factor_meters_per_unit")
        if scale is not None:
            if 0.1 <= scale <= 2000.0:
                sc_status = "PASS"
                sc_msg = f"Plausible metric scale: {scale:.3f} m/unit."
            else:
                sc_status = "FAIL"
                sc_msg = f"Implausible metric scale: {scale:.3f} m/unit (outside [0.1, 2000] range)."
                hard_failures.append(sc_msg)

            checks.append(QualityCheck(
                name="gps_metric_scale_sanity",
                category="geospatial",
                status=sc_status,
                metric_value=round(scale, 3) if scale else None,
                threshold_str="0.1 <= s <= 2000.0 m/unit",
                message=sc_msg
            ))

        # Rotation Matrix Sanity & Vertical Tilt Artifact Detection
        R_sim = sim_xf.get("rotation_matrix")
        if R_sim is not None:
            R_arr = np.array(R_sim)
            # R[2, 2] represents how much of the model's +Z (optical look-at axis) projects into ENU +Up
            # For aerial drone flights pointing downward or oblique forward, the optical axis points down (negative Up)
            # or horizontal. If R[2,2] > 0.85, the horizontal optical axis was mathematically mapped straight up into sky!
            z_projection = float(R_arr[2, 2])
            det_R = float(np.linalg.det(R_arr))

            if abs(det_R - 1.0) > 1e-3:
                r_status = "FAIL"
                r_msg = f"Improper rotation: det(R) = {det_R:.4f} != 1.0 (reflection detected)."
                hard_failures.append(r_msg)
            elif z_projection > 0.85:
                r_status = "FAIL"
                r_msg = (
                    f"Collinear hover orientation defect: optical axis projected straight up into sky "
                    f"(R[2,2] = {z_projection:.3f} > 0.85). Background depth curtain falsely oriented as vertical height."
                )
                hard_failures.append(r_msg)
            else:
                r_status = "PASS"
                r_msg = f"Proper SO(3) rotation (det=1.0) with physically sound camera tilt (Z_proj={z_projection:.3f})."

            checks.append(QualityCheck(
                name="gps_rotation_and_tilt_sanity",
                category="geospatial",
                status=r_status,
                metric_value=round(z_projection, 3),
                threshold_str="det(R)=1.0 & R[2,2] <= 0.85",
                message=r_msg
            ))

        # Inlier Ratio & RMSE
        rmse = fit_st.get("residual_rmse_m")
        inlier_pct = fit_st.get("inlier_ratio_pct", 0.0)
        if rmse is not None:
            if rmse <= 0.50 and inlier_pct >= 80.0:
                fit_status = "PASS"
                fit_msg = f"Survey-grade alignment fit: RMSE {rmse:.3f}m, {inlier_pct:.1f}% inliers."
            elif rmse <= 2.50 and inlier_pct >= 50.0:
                fit_status = "WARN"
                fit_msg = f"Acceptable alignment fit: RMSE {rmse:.3f}m, {inlier_pct:.1f}% inliers."
                warnings.append(fit_msg)
            else:
                fit_status = "FAIL"
                fit_msg = f"Poor alignment fit: RMSE {rmse:.3f}m (> 2.5m threshold) or inlier ratio {inlier_pct:.1f}% (< 50%)."
                hard_failures.append(fit_msg)

            checks.append(QualityCheck(
                name="gps_fit_residual_and_inliers",
                category="geospatial",
                status=fit_status,
                metric_value=round(rmse, 3),
                threshold_str="RMSE <= 0.50m (Pass), <= 2.50m (Warn)",
                message=fit_msg
            ))

    # Overall Verdict
    if len(hard_failures) > 0:
        overall_verdict = "FAIL"
        passed = False
    elif len(warnings) > 0:
        overall_verdict = "PASS_WITH_WARNINGS"
        passed = True
    else:
        overall_verdict = "PASS"
        passed = True

    return QualityGateReport(
        target_id=target_id,
        overall_verdict=overall_verdict,
        passed=passed,
        checks=checks,
        hard_failures=hard_failures,
        warnings=warnings
    )
