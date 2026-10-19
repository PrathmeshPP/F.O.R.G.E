"""F.O.R.G.E — Dataset-Wide Batch Geospatial Alignment Engine.

Coordinates multi-video GPS alignment across all eligible project scenes:
1. Audits and validates per-video inputs (video file, telemetry tracks, reconstructed poses, mesh, point cloud).
2. Extracts and matches telemetry (embedded djmd protobuf or companion SRT/CSV sidecars).
3. Performs rigorous trajectory conditioning analysis (evaluating horizontal vs. vertical baseline, GPS noise floor).
4. Estimates 7-DoF similarity transformation via Umeyama–RANSAC.
5. Transforms 3D geometry and camera extrinsics consistently.
6. Exports aligned models, side-by-side comparison sheets, and per-video metadata.
7. Produces dataset-wide batch reports (JSON, CSV, Markdown) documenting successes, conditioning flags, and missing-telemetry blockers.
"""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, List, Optional, Tuple, Union

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

from src.geo.telemetry import (
    extract_dji_telemetry,
    match_telemetry_to_frame_timestamps,
    geodetic_to_enu,
    GPSTelemetryRecord
)
from src.geo.umeyama_ransac import (
    umeyama_ransac_alignment,
    apply_similarity_transform,
    transform_camera_poses,
    transform_mesh,
    AlignmentResult
)
from src.geo.quality_gates import evaluate_reconstruction_quality_gate


@dataclass
class VideoAlignmentConfig:
    """Configuration specification for a single video alignment target."""
    video_id: str
    display_name: str
    video_path: str
    camera_poses_npz_path: str
    mesh_ply_path: str
    point_cloud_ply_path: Optional[str] = None
    telemetry_source_path: Optional[str] = None  # If None, extracts from video_path
    video_time_offset_sec: float = 0.0          # Non-zero if frames came from a sliced segment
    target_frame_timestamps_sec: Optional[List[float]] = None
    inlier_threshold_m: float = 2.5
    min_translation_baseline_m: float = 1.0     # Minimum required flight translation to consider horizontal heading well-constrained


@dataclass
class TrajectoryConditioning:
    """Diagnostics on trajectory geometry and motion constraints."""
    num_poses: int
    net_translation_model: float
    model_extent_ptp: List[float]
    net_translation_gps_m: float
    horizontal_span_gps_m: float
    vertical_span_gps_m: float
    motion_type: str
    conditioning_verdict: str
    warnings: List[str]


def assess_trajectory_conditioning(
    src_camera_centers: np.ndarray,
    dst_gps_enu: np.ndarray,
    min_horizontal_baseline_m: float = 1.5
) -> TrajectoryConditioning:
    """
    Evaluates whether the trajectory geometry adequately constrains 7-DoF similarity.

    Args:
        src_camera_centers: (N, 3) reconstructed camera centers in model space.
        dst_gps_enu: (N, 3) GPS camera centers in local metric ENU meters.
        min_horizontal_baseline_m: Threshold below which horizontal heading is weakly constrained.

    Returns:
        TrajectoryConditioning dataclass with motion classifications and warnings.
    """
    num_poses = len(src_camera_centers)
    model_ext = np.ptp(src_camera_centers, axis=0) if num_poses > 0 else np.zeros(3)
    net_trans_model = float(np.linalg.norm(src_camera_centers[-1] - src_camera_centers[0])) if num_poses > 1 else 0.0

    if num_poses > 1:
        net_trans_gps = float(np.linalg.norm(dst_gps_enu[-1] - dst_gps_enu[0]))
        horiz_disp = dst_gps_enu[:, :2] - dst_gps_enu[0, :2]
        horiz_span = float(np.max(np.linalg.norm(horiz_disp, axis=1)))
        vert_span = float(np.ptp(dst_gps_enu[:, 2]))
    else:
        net_trans_gps = 0.0
        horiz_span = 0.0
        vert_span = 0.0

    warnings = []

    if net_trans_gps < 0.2:
        motion_type = "stationary_hover"
        conditioning_verdict = "DEGENERATE_ZERO_MOTION"
        warnings.append(f"Near-zero flight translation ({net_trans_gps:.2f}m). Scale and 3D rotation are ill-conditioned.")
    elif horiz_span < min_horizontal_baseline_m:
        motion_type = "vertical_hover_descent" if vert_span > 1.0 else "low_baseline_hover"
        conditioning_verdict = "WEAK_HORIZONTAL_BASELINE_HOVER"
        warnings.append(
            f"Horizontal flight span ({horiz_span:.2f}m) is below GPS noise floor ({min_horizontal_baseline_m}m). "
            f"Vertical scale is constrained by altitude change ({vert_span:.2f}m), but horizontal heading is weakly constrained."
        )
    else:
        motion_type = "translational_corridor_flight"
        conditioning_verdict = "WELL_CONSTRAINED_TRANSLATION"

    return TrajectoryConditioning(
        num_poses=num_poses,
        net_translation_model=round(net_trans_model, 4),
        model_extent_ptp=[round(float(x), 4) for x in model_ext],
        net_translation_gps_m=round(net_trans_gps, 3),
        horizontal_span_gps_m=round(horiz_span, 3),
        vertical_span_gps_m=round(vert_span, 3),
        motion_type=motion_type,
        conditioning_verdict=conditioning_verdict,
        warnings=warnings
    )


def render_mesh_ortho_views(mesh: o3d.geometry.TriangleMesh, width: int = 1280, height: int = 720) -> Dict[str, Image.Image]:
    """Renders Top, Frontal, and Oblique views of a mesh."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    mesh.compute_vertex_normals()
    vis.add_geometry(mesh)

    ctr = mesh.get_axis_aligned_bounding_box().get_center()
    views = {}
    view_configs = [
        ("top", [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
        ("frontal", [0.0, -1.0, 0.2], [0.0, 0.0, 1.0]),
        ("oblique", [0.6, -0.7, 0.5], [0.0, 0.0, 1.0])
    ]

    for vname, front, up in view_configs:
        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr)
        ctrl.set_front(front)
        ctrl.set_up(up)
        ctrl.set_zoom(0.75)
        vis.poll_events()
        vis.update_renderer()
        buf = vis.capture_screen_float_buffer(do_render=True)
        views[vname] = Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))

    vis.destroy_window()
    return views


def process_single_video_alignment(
    config: VideoAlignmentConfig,
    output_base_dir: Path,
    force_rerun: bool = False
) -> Dict[str, Any]:
    """
    Executes the complete 8-stage GPS alignment workflow for one video target safely.
    Never raises an unhandled exception to prevent crashing the batch.
    """
    video_out_dir = output_base_dir / config.video_id
    video_out_dir.mkdir(parents=True, exist_ok=True)
    t_start = time.perf_counter()

    aligned_ply_path = video_out_dir / "surface_mesh_gps_aligned.ply"
    metadata_out_path = video_out_dir / "gps_alignment_metadata.json"

    result_summary: Dict[str, Any] = {
        "video_id": config.video_id,
        "display_name": config.display_name,
        "video_path": str(Path(config.video_path).resolve()) if config.video_path and Path(config.video_path).exists() else str(config.video_path),
        "telemetry_source_path": str(Path(config.telemetry_source_path).resolve()) if config.telemetry_source_path and Path(config.telemetry_source_path).exists() else (str(Path(config.video_path).resolve()) if config.video_path and Path(config.video_path).exists() else str(config.video_path)),
        "extracted_frames_count": 0,
        "reconstructed_poses_count": 0,
        "telemetry_samples_count": 0,
        "matched_correspondences_count": 0,
        "reconstruction_status": "PENDING",
        "alignment_status": "PENDING",
        "quality_gate": None,
        "status_code": "unknown",
        "failure_reason": None,
        "scale_factor_meters_per_unit": None,
        "rotation_matrix": None,
        "translation_vector_enu_m": None,
        "ransac_inliers": 0,
        "total_correspondences": 0,
        "inlier_ratio_pct": 0.0,
        "residual_rmse_m": None,
        "residual_mae_m": None,
        "residual_max_m": None,
        "aligned_extents_m": None,
        "estimated_footprint_m2": None,
        "model_vertex_count": 0,
        "model_triangle_count": 0,
        "output_directory": str(video_out_dir.resolve()),
        "processing_time_sec": 0.0,
        "cached": False,
        "artifacts": {}
    }

    # Cache hit check
    if not force_rerun and aligned_ply_path.is_file() and metadata_out_path.is_file():
        try:
            with open(metadata_out_path, "r", encoding="utf-8") as f:
                cached_meta = json.load(f)
            fit_st = cached_meta.get("fit_statistics", {})
            model_st = cached_meta.get("aligned_model_statistics", {})
            sim_st = cached_meta.get("similarity_transformation", {})
            cond_st = cached_meta.get("trajectory_conditioning", {})
            src_st = cached_meta.get("source_data", {})

            result_summary.update({
                "reconstruction_status": "SUCCESS_RECONSTRUCTED",
                "alignment_status": cached_meta.get("alignment_status", "SUCCESS_METRIC_ALIGNED"),
                "quality_gate": cached_meta.get("quality_gate"),
                "status_code": "success_cached",
                "scale_factor_meters_per_unit": sim_st.get("scale_factor_meters_per_unit"),
                "rotation_matrix": sim_st.get("rotation_matrix"),
                "translation_vector_enu_m": sim_st.get("translation_vector_enu_m"),
                "ransac_inliers": fit_st.get("num_inliers", 0),
                "total_correspondences": fit_st.get("total_correspondences", 0),
                "inlier_ratio_pct": fit_st.get("inlier_ratio_pct", 0.0),
                "residual_rmse_m": fit_st.get("residual_rmse_m"),
                "residual_mae_m": fit_st.get("residual_mae_m"),
                "residual_max_m": fit_st.get("residual_max_m"),
                "aligned_extents_m": model_st.get("extents_enu_m"),
                "estimated_footprint_m2": model_st.get("estimated_footprint_m2"),
                "model_vertex_count": model_st.get("vertex_count", 0),
                "model_triangle_count": model_st.get("triangle_count", 0),
                "extracted_frames_count": src_st.get("matched_frames", 0),
                "reconstructed_poses_count": src_st.get("matched_frames", 0),
                "matched_correspondences_count": src_st.get("matched_frames", 0),
                "conditioning_verdict": cond_st.get("conditioning_verdict", "N/A"),
                "warnings": cond_st.get("warnings", []),
                "cached": True,
                "processing_time_sec": 0.01,
                "artifacts": cached_meta.get("artifacts", {})
            })
            return result_summary
        except Exception:
            # If cache read fails, continue with fresh execution
            pass

    try:
        # 1. Validate Input Reconstructed Geometry & Poses
        mesh_file = Path(config.mesh_ply_path) if config.mesh_ply_path else None
        poses_file = Path(config.camera_poses_npz_path) if config.camera_poses_npz_path else None

        if mesh_file is None or not mesh_file.is_file():
            result_summary["alignment_status"] = "UNPROCESSED_OR_MISSING_RECONSTRUCTION"
            result_summary["status_code"] = "reconstruction_missing"
            result_summary["failure_reason"] = f"Reconstructed mesh not found: {mesh_file}"
            return result_summary

        if poses_file is None or not poses_file.is_file():
            result_summary["alignment_status"] = "MISSING_CAMERA_POSES"
            result_summary["status_code"] = "poses_missing"
            result_summary["failure_reason"] = f"Camera poses NPZ not found: {poses_file}"
            return result_summary

        poses_data = np.load(poses_file)
        extrinsics = poses_data["extrinsics"]
        intrinsics = poses_data["intrinsics"] if "intrinsics" in poses_data else None
        num_frames = len(extrinsics)
        result_summary["reconstructed_poses_count"] = num_frames
        result_summary["extracted_frames_count"] = num_frames

        orig_m_preview = o3d.io.read_triangle_mesh(str(mesh_file))
        result_summary["model_vertex_count"] = len(orig_m_preview.vertices)
        result_summary["model_triangle_count"] = len(orig_m_preview.triangles)
        result_summary["reconstruction_status"] = "SUCCESS_NATIVE_COORDINATES"

        if num_frames < 3:
            result_summary["alignment_status"] = "INSUFFICIENT_GEOMETRY"
            result_summary["status_code"] = "insufficient_camera_poses"
            result_summary["failure_reason"] = f"Reconstructed camera poses count ({num_frames}) is less than minimal 3 points required for 3D alignment."
            return result_summary

        # 2. Extract Telemetry
        telemetry_source = config.telemetry_source_path if config.telemetry_source_path else config.video_path
        if not telemetry_source:
            qg = evaluate_reconstruction_quality_gate(
                target_id=config.video_id,
                mesh_path=mesh_file,
                point_cloud_path=config.point_cloud_ply_path,
                camera_poses_path=poses_file
            )
            result_summary["quality_gate"] = qg.to_dict()
            result_summary["alignment_status"] = "BLOCKED_MISSING_TELEMETRY"
            result_summary["status_code"] = "telemetry_missing"
            result_summary["failure_reason"] = "Telemetry unavailable (benchmark/terrestrial video). Model retained in native coordinate system."
            return result_summary

        tel_file = Path(telemetry_source)
        if not tel_file.is_file():
            qg = evaluate_reconstruction_quality_gate(
                target_id=config.video_id,
                mesh_path=mesh_file,
                point_cloud_path=config.point_cloud_ply_path,
                camera_poses_path=poses_file
            )
            result_summary["quality_gate"] = qg.to_dict()
            result_summary["alignment_status"] = "BLOCKED_MISSING_TELEMETRY"
            result_summary["status_code"] = "telemetry_source_missing"
            result_summary["failure_reason"] = f"Telemetry source file not found: {tel_file}. Model retained in native coordinate system."
            return result_summary

        telemetry_records = extract_dji_telemetry(str(tel_file))
        result_summary["telemetry_samples_count"] = len(telemetry_records)
        valid_telemetry = [r for r in telemetry_records if r.latitude_deg is not None and r.altitude_m is not None]

        if len(valid_telemetry) == 0:
            qg = evaluate_reconstruction_quality_gate(
                target_id=config.video_id,
                mesh_path=mesh_file,
                point_cloud_path=config.point_cloud_ply_path,
                camera_poses_path=poses_file
            )
            result_summary["quality_gate"] = qg.to_dict()
            result_summary["alignment_status"] = "BLOCKED_MISSING_TELEMETRY"
            result_summary["status_code"] = "telemetry_missing"
            result_summary["failure_reason"] = "Video container or sidecars contain no embedded GPS/altitude telemetry (public benchmark or stripped metadata). Model retained in native coordinate system."
            return result_summary

        # 3. Match Telemetry to Camera Frames
        if config.target_frame_timestamps_sec:
            target_times = [t + config.video_time_offset_sec for t in config.target_frame_timestamps_sec[:num_frames]]
        else:
            # Default fallback: 1 fps starting at offset
            target_times = [i * 1.0 + config.video_time_offset_sec for i in range(num_frames)]

        matched_telemetry = match_telemetry_to_frame_timestamps(valid_telemetry, target_times, max_time_tolerance_sec=1.0)
        matched_indices = [i for i, m in enumerate(matched_telemetry) if m is not None]

        if len(matched_indices) < 3:
            result_summary["alignment_status"] = "INSUFFICIENT_DATA"
            result_summary["status_code"] = "insufficient_timestamp_matches"
            result_summary["failure_reason"] = f"Only {len(matched_indices)} telemetry observations matched frame timestamps within tolerance (< 3 required)."
            return result_summary

        # 4. Formulate Coordinate Correspondences
        src_camera_centers = np.zeros((len(matched_indices), 3), dtype=np.float64)
        for k, frame_idx in enumerate(matched_indices):
            R_cam = extrinsics[frame_idx, :3, :3]
            t_cam = extrinsics[frame_idx, :3, 3]
            src_camera_centers[k] = -R_cam.T @ t_cam

        ref_fix = matched_telemetry[matched_indices[0]]
        ref_lat = ref_fix.latitude_deg
        ref_lon = ref_fix.longitude_deg
        ref_alt = ref_fix.altitude_m

        dst_gps_enu = np.zeros((len(matched_indices), 3), dtype=np.float64)
        for k, frame_idx in enumerate(matched_indices):
            m = matched_telemetry[frame_idx]
            e, n, u = geodetic_to_enu(m.latitude_deg, m.longitude_deg, m.altitude_m, ref_lat, ref_lon, ref_alt)
            dst_gps_enu[k] = [e, n, u]

        # 5. Trajectory Conditioning Analysis
        conditioning = assess_trajectory_conditioning(src_camera_centers, dst_gps_enu, min_horizontal_baseline_m=config.min_translation_baseline_m)

        # 6. Umeyama–RANSAC 7-DoF Alignment
        align_result = umeyama_ransac_alignment(
            src_pts=src_camera_centers,
            dst_pts=dst_gps_enu,
            inlier_threshold_m=config.inlier_threshold_m,
            max_iterations=2000,
            ref_geodetic_anchor={
                "latitude_deg": ref_lat,
                "longitude_deg": ref_lon,
                "altitude_m": ref_alt
            },
            random_seed=42
        )

        if not align_result.success:
            result_summary["alignment_status"] = "ALIGNMENT_FAILED"
            result_summary["status_code"] = "umeyama_fit_error"
            result_summary["failure_reason"] = align_result.status_message
            return result_summary

        scale = align_result.scale
        R_sim = np.array(align_result.rotation_matrix)
        t_sim = np.array(align_result.translation_vector)

        # 7. Apply Transformation to Geometry and Camera Poses
        orig_mesh = o3d.io.read_triangle_mesh(str(mesh_file))
        aligned_mesh = transform_mesh(orig_mesh, scale, R_sim, t_sim)

        aligned_pcd = None
        if config.point_cloud_ply_path and Path(config.point_cloud_ply_path).is_file():
            orig_pcd = o3d.io.read_point_cloud(config.point_cloud_ply_path)
            aligned_pcd = o3d.geometry.PointCloud(orig_pcd)
            aligned_pcd_pts = apply_similarity_transform(np.asarray(orig_pcd.points), scale, R_sim, t_sim)
            aligned_pcd.points = o3d.utility.Vector3dVector(aligned_pcd_pts)
            if orig_pcd.has_normals():
                aligned_pcd.normals = o3d.utility.Vector3dVector(np.asarray(orig_pcd.normals) @ R_sim.T)

        aligned_extrinsics = transform_camera_poses(extrinsics, scale, R_sim, t_sim)

        # 8. Export Aligned Artifacts
        aligned_ply_path = video_out_dir / "surface_mesh_gps_aligned.ply"
        aligned_obj_path = video_out_dir / "surface_mesh_gps_aligned.obj"
        aligned_pcd_path = video_out_dir / "point_cloud_gps_aligned.ply"
        aligned_poses_path = video_out_dir / "camera_poses_gps_aligned.npz"
        metadata_out_path = video_out_dir / "gps_alignment_metadata.json"

        o3d.io.write_triangle_mesh(str(aligned_ply_path), aligned_mesh, write_ascii=False, write_vertex_normals=True, write_vertex_colors=True)
        o3d.io.write_triangle_mesh(str(aligned_obj_path), aligned_mesh)
        if aligned_pcd:
            o3d.io.write_point_cloud(str(aligned_pcd_path), aligned_pcd)

        np.savez_compressed(
            str(aligned_poses_path),
            extrinsics=aligned_extrinsics,
            intrinsics=intrinsics,
            original_extrinsics=extrinsics,
            scale=scale,
            rotation_matrix=R_sim,
            translation_vector=t_sim
        )

        m_pts = np.asarray(aligned_mesh.vertices)
        m_ext = (m_pts.max(axis=0) - m_pts.min(axis=0)) if len(m_pts) > 0 else np.zeros(3)

        # Render preview views & comparison sheet
        comp_sheet_path = None
        try:
            aligned_views = render_mesh_ortho_views(aligned_mesh)
            for vname, img in aligned_views.items():
                img.save(video_out_dir / f"aligned_mesh_view_{vname}.png")

            orig_views = render_mesh_ortho_views(orig_mesh)
            cw, ch = 640, 360
            sheet = Image.new("RGB", (cw * 3, ch * 2 + 100), (15, 20, 30))
            draw = ImageDraw.Draw(sheet)
            draw.rectangle([0, 0, cw * 3, 90], fill=(24, 32, 48))
            draw.text((25, 15), f"F.O.R.G.E — Geospatial Alignment: {config.display_name}", fill=(255, 255, 255))
            draw.text(
                (25, 45),
                f"Scale: {scale:.2f} m/unit | Inliers: {align_result.num_inliers}/{align_result.num_total_correspondences} | "
                f"RMSE: {align_result.rmse_m:.3f}m | Extents: {m_ext[0]:.1f}m x {m_ext[1]:.1f}m x {m_ext[2]:.1f}m | Verdict: {conditioning.conditioning_verdict}",
                fill=(100, 220, 255)
            )

            vnames = ["top", "frontal", "oblique"]
            vlabels = ["Top (Plan / Overhead)", "Frontal (Elevation)", "Oblique (Perspective)"]
            for col, (vn, vl) in enumerate(zip(vnames, vlabels)):
                x = col * cw
                im_orig = orig_views[vn].resize((cw, ch), Image.Resampling.BILINEAR)
                sheet.paste(im_orig, (x, 95))
                draw.rectangle([x + 10, 105, x + 380, 140], fill=(0, 0, 0))
                draw.text((x + 18, 112), f"Unaligned (Model Space) — {vl}", fill=(255, 200, 100))

                im_alg = aligned_views[vn].resize((cw, ch), Image.Resampling.BILINEAR)
                sheet.paste(im_alg, (x, 95 + ch))
                draw.rectangle([x + 10, 105 + ch, x + 380, 140 + ch], fill=(0, 0, 0))
                draw.text((x + 18, 112 + ch), f"GPS-Aligned (Metric ENU) — {vl}", fill=(100, 255, 180))

            comp_sheet_path = video_out_dir / "alignment_comparison_sheet.png"
            sheet.save(comp_sheet_path)
        except Exception as render_err:
            conditioning.warnings.append(f"Visual QA rendering failed or headless display unavailable: {str(render_err)}")

        # Final metadata
        final_status = "SUCCESS_METRIC_ALIGNED" if conditioning.conditioning_verdict == "WELL_CONSTRAINED_TRANSLATION" else "UNRELIABLE_POORLY_CONSTRAINED"
        meta = {
            "video_id": config.video_id,
            "display_name": config.display_name,
            "alignment_status": final_status,
            "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "source_data": {
                "video_path": str(Path(config.video_path).resolve()) if config.video_path and Path(config.video_path).exists() else str(config.video_path),
                "telemetry_source": str(tel_file.resolve()),
                "telemetry_type": valid_telemetry[0].source_type if valid_telemetry else "none",
                "matched_frames": len(matched_indices),
                "geodetic_anchor": {
                    "latitude_deg": ref_lat,
                    "longitude_deg": ref_lon,
                    "altitude_wgs84_m": ref_alt
                }
            },
            "similarity_transformation": {
                "scale_factor_meters_per_unit": scale,
                "rotation_matrix": R_sim.round(6).tolist(),
                "rotation_determinant": round(float(np.linalg.det(R_sim)), 6),
                "translation_vector_enu_m": t_sim.round(6).tolist()
            },
            "fit_statistics": {
                "inlier_threshold_m": config.inlier_threshold_m,
                "num_inliers": align_result.num_inliers,
                "total_correspondences": align_result.num_total_correspondences,
                "inlier_ratio_pct": round(align_result.inlier_ratio * 100.0, 2),
                "residual_rmse_m": align_result.rmse_m,
                "residual_mae_m": align_result.mae_m,
                "residual_max_m": align_result.max_error_m,
                "residuals_m": align_result.residuals_m
            },
            "trajectory_conditioning": asdict(conditioning),
            "aligned_model_statistics": {
                "vertex_count": len(m_pts),
                "triangle_count": len(aligned_mesh.triangles),
                "extents_enu_m": [round(float(x), 3) for x in m_ext],
                "estimated_footprint_m2": round(float(m_ext[0] * m_ext[1]), 2),
                "ply_size_bytes": aligned_ply_path.stat().st_size,
                "obj_size_bytes": aligned_obj_path.stat().st_size
            },
            "artifacts": {
                "surface_mesh_ply": str(aligned_ply_path.resolve()),
                "surface_mesh_obj": str(aligned_obj_path.resolve()),
                "point_cloud_ply": str(aligned_pcd_path.resolve()) if aligned_pcd else None,
                "camera_poses_npz": str(aligned_poses_path.resolve()),
                "comparison_sheet_png": str(comp_sheet_path.resolve()) if comp_sheet_path and comp_sheet_path.is_file() else None
            }
        }

        # Comprehensive Quality Gate Assessment
        qg = evaluate_reconstruction_quality_gate(
            target_id=config.video_id,
            mesh_path=aligned_ply_path,
            point_cloud_path=aligned_pcd_path,
            camera_poses_path=aligned_poses_path,
            alignment_metadata=meta
        )
        meta["quality_gate"] = qg.to_dict()

        with open(metadata_out_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        result_summary["reconstruction_status"] = "SUCCESS_RECONSTRUCTED"
        result_summary["alignment_status"] = final_status
        result_summary["quality_gate"] = qg.to_dict()
        result_summary["status_code"] = "success" if final_status == "SUCCESS_METRIC_ALIGNED" else "poorly_constrained"
        if final_status == "UNRELIABLE_POORLY_CONSTRAINED":
            result_summary["failure_reason"] = (
                "Horizontal flight translation baseline (< 1.5m) is below GPS noise floor. "
                "Orientation and metric scale are ill-conditioned; unaligned native model must be used for reliable geometry."
            )
        result_summary["scale_factor_meters_per_unit"] = scale
        result_summary["rotation_matrix"] = R_sim.round(6).tolist()
        result_summary["translation_vector_enu_m"] = t_sim.round(6).tolist()
        result_summary["ransac_inliers"] = align_result.num_inliers
        result_summary["total_correspondences"] = align_result.num_total_correspondences
        result_summary["matched_correspondences_count"] = len(matched_indices)
        result_summary["inlier_ratio_pct"] = round(align_result.inlier_ratio * 100.0, 2)
        result_summary["residual_rmse_m"] = align_result.rmse_m
        result_summary["residual_mae_m"] = align_result.mae_m
        result_summary["residual_max_m"] = align_result.max_error_m
        result_summary["aligned_extents_m"] = [round(float(x), 2) for x in m_ext]
        result_summary["estimated_footprint_m2"] = round(float(m_ext[0] * m_ext[1]), 1)
        result_summary["model_vertex_count"] = len(m_pts)
        result_summary["model_triangle_count"] = len(aligned_mesh.triangles)
        result_summary["conditioning_verdict"] = conditioning.conditioning_verdict
        result_summary["warnings"] = conditioning.warnings
        result_summary["artifacts"] = meta["artifacts"]

    except Exception as e:
        result_summary["alignment_status"] = "ERROR"
        result_summary["status_code"] = "exception_error"
        result_summary["failure_reason"] = f"Unhandled error during processing: {str(e)}"

    result_summary["processing_time_sec"] = round(time.perf_counter() - t_start, 3)
    return result_summary


def execute_batch_alignment(
    configs: List[VideoAlignmentConfig],
    output_base_dir: Path,
    force_rerun: bool = False
) -> Dict[str, Any]:
    """
    Executes alignment across all provided video configurations, generating consolidated
    JSON and CSV reports.
    """
    output_base_dir.mkdir(parents=True, exist_ok=True)
    batch_t0 = time.perf_counter()

    all_results = []
    print(f"\n{'='*80}\nF.O.R.G.E DATASET-WIDE BATCH GPS ALIGNMENT ({len(configs)} TARGET VIDEOS)\n{'='*80}")

    for idx, cfg in enumerate(configs):
        print(f"\n[{idx+1}/{len(configs)}] Processing: {cfg.display_name} ({cfg.video_id})...")
        res = process_single_video_alignment(cfg, output_base_dir, force_rerun=force_rerun)
        all_results.append(res)
        print(f"  Reconstruction: {res.get('reconstruction_status', 'N/A')}")
        print(f"  Alignment:      {res['alignment_status']} ({res['status_code']})")
        qg_verdict = res.get('quality_gate', {}).get('overall_verdict', 'N/A') if res.get('quality_gate') else 'N/A'
        print(f"  Quality Gate:   {qg_verdict}")
        if res['scale_factor_meters_per_unit']:
            print(f"  Scale:          {res['scale_factor_meters_per_unit']:.3f} m/unit | Inliers: {res['ransac_inliers']}/{res['total_correspondences']} | RMSE: {res['residual_rmse_m']:.3f}m")
        if res['failure_reason']:
            print(f"  Reason/Notes:   {res['failure_reason']}")

    total_batch_time = time.perf_counter() - batch_t0

    # 1. Save JSON batch report
    batch_report = {
        "title": "F.O.R.G.E Dataset-Wide Batch Geospatial Alignment Report",
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "total_videos_audited": len(configs),
        "total_reconstructed": sum(1 for r in all_results if r.get("model_vertex_count", 0) > 0),
        "total_aligned_successfully": sum(1 for r in all_results if r["alignment_status"] == "SUCCESS_METRIC_ALIGNED"),
        "total_unreliable_constrained": sum(1 for r in all_results if r["alignment_status"] == "UNRELIABLE_POORLY_CONSTRAINED"),
        "total_blocked_missing_telemetry": sum(1 for r in all_results if r["alignment_status"] == "BLOCKED_MISSING_TELEMETRY"),
        "total_errors": sum(1 for r in all_results if r["alignment_status"] == "ERROR"),
        "total_processing_time_sec": round(total_batch_time, 3),
        "results": all_results
    }

    report_json_path = output_base_dir / "batch_alignment_report.json"
    with open(report_json_path, "w", encoding="utf-8") as f:
        json.dump(batch_report, f, indent=2)

    # 2. Save CSV summary
    report_csv_path = output_base_dir / "batch_alignment_summary.csv"
    with open(report_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Video ID", "Display Name", "Reconstruction Status", "Alignment Status", "Quality Gate",
            "Vertices", "Triangles", "Scale (m/unit)", "Inliers", "Total Poses", "Inlier Ratio (%)",
            "RMSE (m)", "Max Error (m)", "Footprint (m2)", "Extents (East x North x Up) m",
            "Failure Reason / Conditioning Warning"
        ])
        for r in all_results:
            ext_str = f"{r['aligned_extents_m'][0]}x{r['aligned_extents_m'][1]}x{r['aligned_extents_m'][2]}" if r.get('aligned_extents_m') else "N/A"
            qg_v = r.get("quality_gate", {}).get("overall_verdict", "N/A") if r.get("quality_gate") else "N/A"
            note = r.get("failure_reason") or ("; ".join(r.get("warnings", [])) if r.get("warnings") else "Clean fit")
            writer.writerow([
                r["video_id"],
                r["display_name"],
                r.get("reconstruction_status", "N/A"),
                r["alignment_status"],
                qg_v,
                r.get("model_vertex_count", 0),
                r.get("model_triangle_count", 0),
                f"{r['scale_factor_meters_per_unit']:.4f}" if r.get("scale_factor_meters_per_unit") else "N/A",
                r.get("ransac_inliers", 0),
                r.get("total_correspondences", 0),
                r.get("inlier_ratio_pct", 0.0),
                f"{r['residual_rmse_m']:.3f}" if r.get("residual_rmse_m") else "N/A",
                f"{r['residual_max_m']:.3f}" if r.get("residual_max_m") else "N/A",
                r.get("estimated_footprint_m2", "N/A"),
                ext_str,
                note
            ])

    print(f"\n{'='*80}\nBATCH PROCESSING COMPLETE ({total_batch_time:.2f}s)\n{'='*80}")
    print(f"Dataset-level Report JSON: {report_json_path}")
    print(f"Dataset-level Summary CSV: {report_csv_path}")
    return batch_report
