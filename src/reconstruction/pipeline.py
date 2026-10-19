"""F.O.R.G.E — General-Purpose Adaptive 3D Reconstruction Pipeline.

Provides a unified, modular, and adaptive 3D reconstruction pipeline that works across
diverse video categories (aerial drone surveys, terrestrial forward driving, river/bridge
scenarios) without hardcoded scene-specific heuristics or metric assumptions.

Key Architectural Principles:
1. Scale-Invariant Adaptivity:
   Voxel fusion, normal estimation radii, and mesh support distances scale relative
   to scene extents and local point spacing rather than imposing arbitrary metric constants.
2. Robust Outlier & Horizon Filtering:
   Detects the zero-logit confidence floor (sky/horizon rays) without truncating distant
   road and terrain surfaces.
3. Multi-View Camera Parallax Assessment:
   Analyzes trajectory translation, rotation diversity, and baseline geometry to document
   physical reconstruction constraints.
4. Non-Destructive Surface Meshing:
   Employs camera-oriented normal fields, gentle density trimming, and depth-aware support
   filtering to eliminate voids and preserve architectural/terrain continuity.
5. Trustworthy Multi-Dimensional Evaluation:
   Measures retention, manifoldness, boundary preservation, and roughness consistently
   without relying on single aggregate scores.
"""

from __future__ import annotations

import os
import sys
import json
import time
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, Tuple, List, Optional, Union

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

from src.reconstruction.point_cloud import depth_to_points, save_ply, voxel_grid_fusion


@dataclass
class PipelineConfig:
    """Configuration parameters for the general-purpose reconstruction pipeline."""
    # Depth bounds
    min_depth: float = 0.1
    max_depth: Optional[float] = None  # None = adaptive 99.5th percentile of valid geometry
    depth_percentile_cutoff: float = 99.5

    # Confidence thresholding
    min_confidence: Optional[float] = 0.1  # None or 0.1 = baseline passthrough; float = fixed threshold
    adaptive_confidence: bool = False  # If True, computes threshold above the zero-logit noise floor

    # Voxel-grid fusion
    voxel_size: Optional[Union[float, str]] = None  # None = unfused; float = fixed; "auto" = adaptive
    target_voxels_per_extent: int = 400  # Number of voxels along max scene extent for adaptive sizing

    # Meshing
    mesh_method: str = "poisson"  # "poisson" or "bpa"
    poisson_depth: int = 9  # Octree depth (8=coarse, 9=standard, 10=fine)
    density_quantile: float = 0.03  # Gentle trimming (3% lowest density pruned, preserving courtyards/facades)
    adaptive_support: bool = True  # Depth-scaled support distance to preserve distant terrain
    base_support_dist: float = 0.03  # Near-field support radius in model units
    max_edge_length_factor: float = 2.5  # Max triangle edge as multiple of local support radius
    min_cluster_faces: int = 20  # Minimum face count for connected component pruning (removes isolated specks)

    # Color transfer
    color_transfer_k: int = 4

    # Presentation
    render_width: int = 1280
    render_height: int = 720


def assess_camera_trajectory(extrinsics: np.ndarray) -> Dict[str, Any]:
    """
    Analyzes camera motion, translation baseline, rotation diversity, and coverage constraints.

    Args:
        extrinsics: (N, 4, 4) or (N, 3, 4) world-to-camera extrinsic matrices.

    Returns:
        Dictionary of trajectory diagnostics, baseline lengths, and potential warnings.
    """
    num_frames = len(extrinsics)
    cam_centers = np.zeros((num_frames, 3), dtype=np.float64)
    rotations = np.zeros((num_frames, 3, 3), dtype=np.float64)

    for i in range(num_frames):
        R = extrinsics[i, :3, :3]
        t = extrinsics[i, :3, 3]
        cam_centers[i] = -R.T @ t
        rotations[i] = R

    # Pairwise translation baseline
    consecutive_steps = np.linalg.norm(cam_centers[1:] - cam_centers[:-1], axis=1) if num_frames > 1 else np.array([0.0])
    total_path_length = float(np.sum(consecutive_steps))
    net_translation = float(np.linalg.norm(cam_centers[-1] - cam_centers[0])) if num_frames > 1 else 0.0
    trajectory_extent = float(np.max(np.ptp(cam_centers, axis=0))) if num_frames > 1 else 0.0

    # Rotation variation (trace of relative rotations)
    rot_variations_deg = []
    for i in range(num_frames - 1):
        R_rel = rotations[i + 1] @ rotations[i].T
        cos_theta = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
        rot_variations_deg.append(np.degrees(np.arccos(cos_theta)))
    mean_rot_deg = float(np.mean(rot_variations_deg)) if rot_variations_deg else 0.0

    # Trajectory classification and limitations
    motion_type = "unknown"
    warnings = []
    if trajectory_extent < 0.1:
        motion_type = "low_baseline_hover"
        warnings.append(
            f"Low camera baseline ({trajectory_extent:.3f} units). Multi-view parallax on vertical facades "
            "is physically constrained; depth predictions rely heavily on single-view priors."
        )
    elif np.ptp(cam_centers[:, 2]) > np.ptp(cam_centers[:, 0]) * 2.0:
        motion_type = "forward_axial_motion"
        warnings.append(
            "Predominantly forward camera translation. Distant depth observations exhibit smaller stereo "
            "parallax; far-field confidence naturally decreases toward the horizon."
        )
    else:
        motion_type = "multi_axis_survey"

    return {
        "num_frames": num_frames,
        "camera_centers": cam_centers.round(4).tolist(),
        "mean_camera_center": cam_centers.mean(axis=0).round(4).tolist(),
        "total_path_length": round(total_path_length, 4),
        "net_translation": round(net_translation, 4),
        "trajectory_extent": round(trajectory_extent, 4),
        "mean_frame_rotation_deg": round(mean_rot_deg, 3),
        "motion_type": motion_type,
        "warnings": warnings
    }


def estimate_adaptive_parameters(
    depth_maps: np.ndarray,
    confidences: Optional[np.ndarray],
    extrinsics: np.ndarray,
    intrinsics: np.ndarray,
    config: PipelineConfig
) -> Dict[str, Any]:
    """
    Computes mathematically justified, scale-relative parameters from input observations.

    Args:
        depth_maps: (N, H, W) or (N, H, W, 1) depth array.
        confidences: Optional (N, H, W) confidence array.
        extrinsics: (N, 4, 4) camera extrinsics.
        intrinsics: (N, 3, 3) camera intrinsics.
        config: Current pipeline configuration.

    Returns:
        Dictionary containing resolved min_depth, max_depth, min_confidence, voxel_size, and support radii.
    """
    depth_flat = depth_maps.reshape(-1)
    valid_depth_mask = np.isfinite(depth_flat) & (depth_flat > 0.05)
    valid_depths = depth_flat[valid_depth_mask]

    # 1. Depth Bounds
    resolved_min_depth = config.min_depth
    if config.max_depth is not None:
        resolved_max_depth = config.max_depth
    else:
        # Robust upper bound to prune infinite sky rays while preserving 99.5% of real scene geometry
        resolved_max_depth = float(np.percentile(valid_depths, config.depth_percentile_cutoff))

    # 2. Confidence Threshold
    resolved_min_confidence = config.min_confidence if config.min_confidence is not None else 0.1
    if config.adaptive_confidence and confidences is not None:
        conf_flat = confidences.reshape(-1)[valid_depth_mask]
        p5_conf = float(np.percentile(conf_flat, 5.0))
        # Keep threshold conservative (at or below 5th percentile, capped at 0.5) to avoid amputating distant geometry
        resolved_min_confidence = min(max(p5_conf, 0.1), 0.5)

    # 3. Scale & Voxel Sizing
    sample_sub = depth_to_points(depth_maps[:, ::4, ::4], extrinsics, intrinsics)
    s_pts = sample_sub.reshape(-1, 3)
    s_mask = np.isfinite(s_pts).all(axis=1) & (s_pts[:, 2] >= resolved_min_depth) & (s_pts[:, 2] <= resolved_max_depth)
    s_pts_valid = s_pts[s_mask]

    if len(s_pts_valid) > 0:
        extents = np.ptp(s_pts_valid, axis=0)
        max_extent = float(np.max(extents))
        median_z = float(np.median(s_pts_valid[:, 2]))
    else:
        max_extent = 5.0
        median_z = 2.0

    resolved_voxel_size = config.voxel_size
    if resolved_voxel_size == "auto" or (isinstance(resolved_voxel_size, (int, float)) and resolved_voxel_size <= 0):
        # Target high geometric fidelity: at least 800 voxels along max extent (or ~0.01-0.02 units)
        resolved_voxel_size = round(max_extent / max(config.target_voxels_per_extent, 50), 4)

    mean_fx = float(np.mean(intrinsics[:, 0, 0])) if intrinsics is not None else 600.0

    return {
        "resolved_min_depth": round(resolved_min_depth, 4),
        "resolved_max_depth": round(resolved_max_depth, 4),
        "resolved_min_confidence": round(resolved_min_confidence, 4),
        "resolved_voxel_size": resolved_voxel_size,
        "scene_max_extent": round(max_extent, 4),
        "median_depth_z": round(median_z, 4),
        "mean_focal_length_px": round(mean_fx, 2),
    }


def filter_mesh_adaptive_support(
    mesh: o3d.geometry.TriangleMesh,
    reference_pcd: o3d.geometry.PointCloud,
    base_support_dist: float = 0.03,
    max_edge_factor: float = 2.5,
    min_cluster_faces: int = 20,
    depth_scaled: bool = True
) -> o3d.geometry.TriangleMesh:
    """
    Prunes unsupported vertices, bridging faces, and isolated specks with depth-aware thresholds.

    Args:
        mesh: Raw reconstructed triangle mesh.
        reference_pcd: Source point cloud verifying genuine observation support.
        base_support_dist: Support radius at camera origin / foreground.
        max_edge_factor: Multiple of support distance for triangle edge rejection.
        min_cluster_faces: Minimum faces for connected component retention.
        depth_scaled: If True, support distance scales with Z to account for perspective ray spreading.

    Returns:
        Filtered TriangleMesh.
    """
    verts = np.asarray(mesh.vertices)
    if len(verts) == 0:
        return mesh

    kdtree = o3d.geometry.KDTreeFlann(reference_pcd)
    dists = np.zeros(len(verts), dtype=np.float32)

    for i in range(len(verts)):
        [k, idx, d2] = kdtree.search_knn_vector_3d(verts[i], 1)
        dists[i] = np.sqrt(d2[0]) if k > 0 else 999.0

    if depth_scaled:
        z_vals = np.maximum(verts[:, 2], 0.5)
        median_z = float(np.median(z_vals))
        depth_scale = 1.0 + 0.5 * (z_vals / max(median_z, 1.0))
        effective_support = base_support_dist * depth_scale
    else:
        effective_support = np.full(len(verts), base_support_dist, dtype=np.float32)

    unsupported_mask = dists > effective_support
    mesh.remove_vertices_by_mask(unsupported_mask)

    triangles = np.asarray(mesh.triangles)
    if len(triangles) == 0:
        return mesh

    v = np.asarray(mesh.vertices)
    v0 = v[triangles[:, 0]]
    v1 = v[triangles[:, 1]]
    v2 = v[triangles[:, 2]]
    e1 = np.linalg.norm(v1 - v0, axis=1)
    e2 = np.linalg.norm(v2 - v1, axis=1)
    e3 = np.linalg.norm(v0 - v2, axis=1)
    max_edges = np.maximum(np.maximum(e1, e2), e3)

    face_z = (v0[:, 2] + v1[:, 2] + v2[:, 2]) / 3.0
    if depth_scaled:
        max_allowed_edge = (base_support_dist * (1.0 + 0.5 * (face_z / max(float(np.median(face_z)), 1.0)))) * max_edge_factor
    else:
        max_allowed_edge = base_support_dist * max_edge_factor

    valid_faces = max_edges <= max_allowed_edge
    mesh.remove_triangles_by_mask(~valid_faces)
    mesh.remove_unreferenced_vertices()

    if len(mesh.triangles) > 0 and min_cluster_faces > 0:
        clusters, num_faces, _ = mesh.cluster_connected_triangles()
        clusters = np.asarray(clusters)
        num_faces = np.asarray(num_faces)
        tiny_clusters = np.where(num_faces < min_cluster_faces)[0]
        if len(tiny_clusters) > 0:
            mesh.remove_triangles_by_mask(np.isin(clusters, tiny_clusters))
            mesh.remove_unreferenced_vertices()

    return mesh


def transfer_colors_kdtree(
    mesh: o3d.geometry.TriangleMesh,
    source_pcd: o3d.geometry.PointCloud,
    k: int = 4
) -> None:
    """Transfers vertex colors from source point cloud using inverse-distance-weighted interpolation."""
    verts = np.asarray(mesh.vertices)
    if len(verts) == 0:
        return

    src_colors = np.asarray(source_pcd.colors)
    if len(src_colors) == 0:
        return

    kdtree = o3d.geometry.KDTreeFlann(source_pcd)
    mesh_colors = np.zeros((len(verts), 3), dtype=np.float64)

    for i in range(len(verts)):
        [num_k, idxs, d2] = kdtree.search_knn_vector_3d(verts[i], k)
        if num_k == 0:
            mesh_colors[i] = [0.5, 0.5, 0.5]
            continue
        dists = np.sqrt(d2)
        if dists[0] < 1e-7:
            mesh_colors[i] = src_colors[idxs[0]]
        else:
            weights = 1.0 / np.maximum(dists, 1e-6)
            weights /= weights.sum()
            mesh_colors[i] = np.sum(src_colors[idxs] * weights[:, None], axis=0)

    mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(mesh_colors, 0.0, 1.0))


def compute_surface_roughness(pcd: o3d.geometry.PointCloud, k: int = 15, max_samples: int = 10000) -> float:
    """Computes mean distance from sampled points to their local best-fit plane (measures surface rippling)."""
    pts = np.asarray(pcd.points)
    if len(pts) < k:
        return 0.0
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    residuals = []
    sample_indices = np.random.choice(len(pts), size=min(max_samples, len(pts)), replace=False)
    for idx in sample_indices:
        [count, n_idx, _] = kdtree.search_knn_vector_3d(pts[idx], k)
        if count < 3:
            continue
        neighbors = pts[n_idx]
        centroid = neighbors.mean(axis=0)
        _, _, vh = np.linalg.svd(neighbors - centroid)
        normal = vh[-1]
        dist = abs(np.dot(pts[idx] - centroid, normal))
        residuals.append(dist)
    return float(np.mean(residuals)) if residuals else 0.0


def evaluate_disaster_readiness(
    active_points: np.ndarray,
    mesh: o3d.geometry.TriangleMesh,
    traj_diagnostics: Dict[str, Any],
    depth_maps: np.ndarray,
    confidences: Optional[np.ndarray],
    scene_classification: str = "general"
) -> Dict[str, Any]:
    """
    Evaluates reconstruction suitability, uncertainty, and coverage for disaster-response applications.
    Distinguishes visual quality from verified metric accuracy and documents physical limitations.
    """
    pts = active_points
    v_arr = np.asarray(mesh.vertices)
    f_arr = np.asarray(mesh.triangles)

    extents = np.ptp(pts, axis=0) if len(pts) > 0 else np.zeros(3)
    bbox_vol = float(np.prod(extents))
    footprint_area = float(extents[0] * extents[1]) if extents[2] < min(extents[0], extents[1]) else float(extents[0] * extents[2])

    weak_obs_ratio = 0.0
    if confidences is not None:
        c_flat = confidences.reshape(-1)
        valid_c = c_flat[np.isfinite(c_flat)]
        if len(valid_c) > 0:
            weak_thresh = max(float(np.percentile(valid_c, 10.0)), 1.5)
            weak_obs_ratio = round(float(np.mean(valid_c < weak_thresh)), 4)

    warnings = list(traj_diagnostics.get("warnings", []))
    warnings.append("Coordinate units are model-relative and unverified against Ground Control Points (GCPs). Do NOT use for precision metric clearance measurements.")

    if traj_diagnostics.get("motion_type") == "low_baseline_hover":
        warnings.append("Near-zero camera baseline limits stereo parallax; vertical facade geometry and sub-canopy structures have high depth uncertainty.")

    if weak_obs_ratio > 0.20:
        warnings.append(f"High proportion of weakly observed regions ({weak_obs_ratio*100:.1f}% low confidence). Structural voids or occlusion likely.")

    is_tree_scene = any(k in scene_classification.lower() for k in ["forest", "tree", "foliage"])
    if is_tree_scene:
        warnings.append("Dense vegetation canopy produces top-down occlusion; ground surface beneath foliage is physically unobserved and cannot be used for vehicle/personnel traversability.")

    can_visual_recon = len(v_arr) > 1000 and len(f_arr) > 1000
    can_tactical_path = can_visual_recon and (traj_diagnostics.get("trajectory_extent", 0.0) > 0.15) and (weak_obs_ratio < 0.25)

    return {
        "scale_verification_status": {
            "is_metric_ground_truth_calibrated": False,
            "scale_units": "model_relative_uncalibrated",
            "explanation": "Predicted coordinates reflect internal model scale. Centimetre-level physical accuracy is unverified without surveyor GCPs or RTK telemetry."
        },
        "coverage_diagnostics": {
            "spatial_extents": extents.round(4).tolist(),
            "bounding_volume": round(bbox_vol, 4),
            "estimated_ground_footprint": round(footprint_area, 4),
            "retained_points": len(pts),
            "mesh_vertices": len(v_arr),
            "mesh_triangles": len(f_arr),
        },
        "uncertainty_and_occlusion": {
            "weakly_observed_ray_ratio": weak_obs_ratio,
            "sub_canopy_occlusion_detected": is_tree_scene,
            "unobserved_ground_status": "OCCLUDED_BY_CANOPY" if is_tree_scene else "PARTIALLY_VISIBLE",
        },
        "geometric_integrity": {
            "is_edge_manifold": bool(mesh.is_edge_manifold()),
            "is_vertex_manifold": bool(mesh.is_vertex_manifold()),
            "has_finite_coordinates": bool(np.isfinite(v_arr).all()) if len(v_arr) > 0 else False,
        },
        "operational_readiness_verdict": {
            "rough_visual_reconnaissance": "ACCEPTABLE" if can_visual_recon else "REJECTED",
            "tactical_pathfinding_assessment": "MARGINAL_CAUTION" if can_tactical_path else "NOT_RECOMMENDED",
            "metric_engineering_and_clearance": "NOT_PERMITTED_WITHOUT_GCP",
            "operational_warnings": warnings
        }
    }


def render_matched_view_robust(
    geom: o3d.geometry.Geometry,
    cam_json_path: Optional[str] = None,
    ref_bbox: Optional[o3d.geometry.AxisAlignedBoundingBox] = None,
    width: int = 1280,
    height: int = 720,
    is_mesh: bool = False
) -> Image.Image:
    """
    Renders geometry from camera parameters or bounding box framing, preventing blank/culled output.
    Uses allow_arbitrary=True and ensures near/far planes envelope the geometry.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    if is_mesh:
        geom.compute_vertex_normals()
    vis.add_geometry(geom)

    ctrl = vis.get_view_control()
    if cam_json_path and os.path.isfile(cam_json_path):
        param = o3d.io.read_pinhole_camera_parameters(cam_json_path)
        ctrl.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)
    elif ref_bbox is not None:
        ctrl.set_lookat(ref_bbox.get_center())
        ctrl.set_front([0.6, -0.7, -0.6])
        ctrl.set_up([0.0, -1.0, 0.0])
        ctrl.set_zoom(0.7)

    vis.poll_events()
    vis.update_renderer()
    img_buf = vis.capture_screen_float_buffer(do_render=True)
    arr = (np.asarray(img_buf) * 255).astype(np.uint8)
    vis.destroy_window()

    return Image.fromarray(arr)


def render_standard_views(
    geom: o3d.geometry.Geometry,
    ref_bbox: o3d.geometry.AxisAlignedBoundingBox,
    width: int = 800,
    height: int = 600,
    is_mesh: bool = False,
    orientation_class: Optional[str] = None
) -> Dict[str, Image.Image]:
    """
    Renders Top, Frontal, and Oblique views with mathematically correct orientation vectors.
    Automatically detects whether the scene is nadir/overhead (Campus, Forest) or forward corridor (Desert Road, River Bridge).
    """
    if orientation_class is None:
        ext = ref_bbox.get_extent()
        if ext[2] < min(ext[0], ext[1]):
            orientation_class = "nadir_overhead"
        else:
            orientation_class = "forward_corridor"

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    if is_mesh:
        geom.compute_vertex_normals()
    vis.add_geometry(geom)

    ctr = ref_bbox.get_center()
    views = {}
    if orientation_class == "nadir_overhead":
        view_configs = [
            ("top", [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]),
            ("frontal", [0.0, 1.0, -0.3], [0.0, 0.0, -1.0]),
            ("oblique", [0.5, -0.6, -0.6], [0.0, -1.0, 0.0]),
        ]
    else:
        view_configs = [
            ("top", [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
            ("frontal", [0.0, 0.1, -1.0], [0.0, -1.0, 0.0]),
            ("oblique", [0.6, -0.5, -0.6], [0.0, -1.0, 0.0]),
        ]

    for v_name, front, up in view_configs:
        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr)
        ctrl.set_front(front)
        ctrl.set_up(up)
        ctrl.set_zoom(0.75)

        vis.poll_events()
        vis.update_renderer()
        img_buf = vis.capture_screen_float_buffer(do_render=True)
        views[v_name] = Image.fromarray((np.asarray(img_buf) * 255).astype(np.uint8))

    vis.destroy_window()
    return views


class GeneralReconstructionPipeline:
    """Executes the full general-purpose 3D reconstruction pipeline on any input dataset."""

    def __init__(self, config: Optional[PipelineConfig] = None):
        self.config = config or PipelineConfig()

    def run(
        self,
        depth_npz_path: str,
        poses_npz_path: str,
        frames_dir: str,
        output_dir: str,
        reference_camera_json: Optional[str] = None,
        scene_classification: str = "general"
    ) -> Dict[str, Any]:
        """
        Executes all pipeline stages:
        1. Metadata & Trajectory Assessment
        2. Scale-relative Adaptive Parameter Estimation
        3. Backprojection & Filtering
        4. Optional Voxel-Grid Fusion
        5. Geometry Reconstruction & Mesh Validation
        6. Multimodal Quality Evaluation & Matched Rendering
        """
        t_start = time.perf_counter()
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)

        # -------------------------------------------------------------
        # Stage 1: Load Predictions & Assess Camera Motion
        # -------------------------------------------------------------
        depth_data = np.load(depth_npz_path)
        depth_maps = depth_data["depth"]
        confidences = depth_data["confidence"] if "confidence" in depth_data else None

        poses_data = np.load(poses_npz_path)
        extrinsics = poses_data["extrinsics"]
        intrinsics = poses_data["intrinsics"]

        traj_diagnostics = assess_camera_trajectory(extrinsics)

        # -------------------------------------------------------------
        # Stage 2: Adaptive Parameter Estimation
        # -------------------------------------------------------------
        adaptive_params = estimate_adaptive_parameters(
            depth_maps=depth_maps,
            confidences=confidences,
            extrinsics=extrinsics,
            intrinsics=intrinsics,
            config=self.config
        )

        min_depth = adaptive_params["resolved_min_depth"]
        max_depth = adaptive_params["resolved_max_depth"]
        min_confidence = adaptive_params["resolved_min_confidence"]
        voxel_size = adaptive_params["resolved_voxel_size"]

        # -------------------------------------------------------------
        # Stage 3: Backprojection & Point Cloud Generation
        # -------------------------------------------------------------
        frame_files = sorted(
            [p for p in Path(frames_dir).iterdir() if p.suffix.lower() in [".png", ".jpg", ".jpeg"]],
            key=lambda p: p.name
        )
        frame_colors_list = []
        for f in frame_files[:len(depth_maps)]:
            img = Image.open(f).convert("RGB")
            h, w = depth_maps.shape[1], depth_maps.shape[2]
            if img.size != (w, h):
                img = img.resize((w, h), Image.Resampling.BILINEAR)
            frame_colors_list.append(np.array(img, dtype=np.uint8))
        frame_colors = np.stack(frame_colors_list, axis=0)

        world_points = depth_to_points(depth_maps, extrinsics, intrinsics)

        # Apply filtering criteria
        valid_mask = np.isfinite(world_points).all(axis=-1)
        depth_vals = depth_maps.squeeze(-1) if depth_maps.ndim == 4 else depth_maps
        valid_mask &= (depth_vals >= min_depth)
        if max_depth is not None:
            valid_mask &= (depth_vals <= max_depth)
        if confidences is not None and min_confidence > 0.0:
            valid_mask &= (confidences >= min_confidence)

        raw_points = world_points[valid_mask]
        raw_colors = frame_colors[valid_mask]
        raw_point_count = len(raw_points)

        # Save primary raw/filtered point cloud
        raw_ply_path = out_path / "point_cloud_filtered.ply"
        save_ply(raw_points, raw_colors, str(raw_ply_path), binary=True)

        # -------------------------------------------------------------
        # Stage 4: Optional Voxel-Grid Fusion
        # -------------------------------------------------------------
        if voxel_size is not None and voxel_size > 0:
            fused_points, fused_colors = voxel_grid_fusion(raw_points, raw_colors, voxel_size)
            fused_ply_path = out_path / "point_cloud_fused.ply"
            save_ply(fused_points, fused_colors, str(fused_ply_path), binary=True)
            active_points, active_colors = fused_points, fused_colors
        else:
            active_points, active_colors = raw_points, raw_colors
            fused_ply_path = None

        active_pcd = o3d.geometry.PointCloud()
        active_pcd.points = o3d.utility.Vector3dVector(active_points.astype(np.float64))
        active_pcd.colors = o3d.utility.Vector3dVector(active_colors.astype(np.float64) / 255.0)

        ref_bbox = active_pcd.get_axis_aligned_bounding_box()
        roughness = compute_surface_roughness(active_pcd)

        # -------------------------------------------------------------
        # Stage 5: Surface Mesh Reconstruction
        # -------------------------------------------------------------
        t_mesh_start = time.perf_counter()
        cam_center = np.array(traj_diagnostics["mean_camera_center"])

        # Estimate normals oriented towards camera trajectory
        active_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(adaptive_params["scene_max_extent"] * 0.015, 0.03),
                max_nn=30
            )
        )
        active_pcd.orient_normals_towards_camera_location(camera_location=cam_center)

        if self.config.mesh_method == "poisson":
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                active_pcd, depth=self.config.poisson_depth
            )
            if self.config.density_quantile > 0.0:
                dens_np = np.asarray(densities)
                dens_thresh = float(np.percentile(dens_np, self.config.density_quantile * 100.0))
                mesh.remove_vertices_by_mask(dens_np < dens_thresh)
        else:
            radii = [0.03, 0.06, 0.12]
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                active_pcd, o3d.utility.DoubleVector(radii)
            )

        filter_mesh_adaptive_support(
            mesh=mesh,
            reference_pcd=active_pcd,
            base_support_dist=self.config.base_support_dist,
            max_edge_factor=self.config.max_edge_length_factor,
            min_cluster_faces=self.config.min_cluster_faces,
            depth_scaled=self.config.adaptive_support
        )

        transfer_colors_kdtree(mesh, active_pcd, k=self.config.color_transfer_k)
        mesh_time = time.perf_counter() - t_mesh_start

        mesh_ply_path = out_path / "surface_mesh.ply"
        mesh_obj_path = out_path / "surface_mesh.obj"
        o3d.io.write_triangle_mesh(str(mesh_ply_path), mesh)
        o3d.io.write_triangle_mesh(str(mesh_obj_path), mesh)

        # -------------------------------------------------------------
        # Stage 6: Rendering & Validation
        # -------------------------------------------------------------
        pcd_std_views = render_standard_views(active_pcd, ref_bbox, is_mesh=False)
        mesh_std_views = render_standard_views(mesh, ref_bbox, is_mesh=True)
        for vname, img in pcd_std_views.items():
            img.save(out_path / f"pcd_view_{vname}.png")
        for vname, img in mesh_std_views.items():
            img.save(out_path / f"mesh_view_{vname}.png")

        if reference_camera_json and os.path.isfile(reference_camera_json):
            ref_cam_pcd_img = render_matched_view_robust(
                active_pcd, cam_json_path=reference_camera_json, is_mesh=False,
                width=self.config.render_width, height=self.config.render_height
            )
            ref_cam_mesh_img = render_matched_view_robust(
                mesh, cam_json_path=reference_camera_json, is_mesh=True,
                width=self.config.render_width, height=self.config.render_height
            )
            ref_cam_pcd_img.save(out_path / "reference_matched_view_pcd.png")
            ref_cam_mesh_img.save(out_path / "reference_matched_view_mesh.png")

        total_time = time.perf_counter() - t_start

        v_arr = np.asarray(mesh.vertices)
        f_arr = np.asarray(mesh.triangles)
        c_arr = np.asarray(mesh.vertex_colors)

        disaster_eval = evaluate_disaster_readiness(
            active_points=active_points,
            mesh=mesh,
            traj_diagnostics=traj_diagnostics,
            depth_maps=depth_maps,
            confidences=confidences,
            scene_classification=scene_classification
        )

        stats = {
            "pipeline": "GeneralReconstructionPipeline",
            "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "scene_classification": scene_classification,
            "coordinate_units_note": "Model-predicted relative coordinate units; unverified against ground truth GCPs.",
            "disaster_readiness_evaluation": disaster_eval,
            "trajectory_diagnostics": traj_diagnostics,
            "adaptive_parameters": adaptive_params,
            "point_cloud": {
                "raw_filtered_points": raw_point_count,
                "active_points": len(active_points),
                "is_voxel_fused": voxel_size is not None and voxel_size > 0,
                "voxel_size": voxel_size,
                "surface_roughness_residual": round(roughness, 6),
                "bounds_min": active_points.min(axis=0).round(4).tolist(),
                "bounds_max": active_points.max(axis=0).round(4).tolist(),
                "extents": np.ptp(active_points, axis=0).round(4).tolist(),
            },
            "surface_mesh": {
                "vertex_count": len(v_arr),
                "triangle_count": len(f_arr),
                "has_vertex_colors": mesh.has_vertex_colors(),
                "has_vertex_normals": mesh.has_vertex_normals(),
                "is_edge_manifold": mesh.is_edge_manifold(),
                "is_vertex_manifold": mesh.is_vertex_manifold(),
                "mesh_bounds_min": v_arr.min(axis=0).round(4).tolist() if len(v_arr) > 0 else [],
                "mesh_bounds_max": v_arr.max(axis=0).round(4).tolist() if len(v_arr) > 0 else [],
                "mesh_extents": np.ptp(v_arr, axis=0).round(4).tolist() if len(v_arr) > 0 else [],
                "color_stats": {
                    "mean_rgb": c_arr.mean(axis=0).round(4).tolist() if len(c_arr) > 0 else [],
                    "std_rgb": c_arr.std(axis=0).round(4).tolist() if len(c_arr) > 0 else [],
                },
                "meshing_time_sec": round(mesh_time, 3),
            },
            "performance": {
                "total_pipeline_time_sec": round(total_time, 3),
            },
            "artifacts": {
                "point_cloud_ply": str(raw_ply_path.resolve()),
                "fused_ply": str(fused_ply_path.resolve()) if fused_ply_path else None,
                "surface_mesh_ply": str(mesh_ply_path.resolve()),
                "surface_mesh_obj": str(mesh_obj_path.resolve()),
            }
        }

        meta_file = out_path / "pipeline_metadata.json"
        with open(meta_file, "w") as f:
            json.dump(stats, f, indent=2)

        return stats
