"""
F.O.R.G.E — Sprint 1: Candidate 3D Model Generator & Multi-Scene Evaluator.
Generates and evaluates candidate 3D models (PCD, Poisson mesh, BPA mesh) across all 4 scenes:
1. Campus 0049
2. Desert Road
3. Forest Road
4. River Bridge

All outputs are saved strictly under:
outputs/accuracy_experiment/regression_diagnosis_v2/<scene>/<candidate>/
Preserves all baselines and previous outputs untouched.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Tuple

import cv2
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.point_cloud import depth_to_points, save_ply, voxel_grid_fusion
from src.reconstruction.pipeline import (
    PipelineConfig,
    assess_camera_trajectory,
    estimate_adaptive_parameters,
    filter_mesh_adaptive_support,
    transfer_colors_kdtree,
    render_standard_views,
    compute_surface_roughness,
    evaluate_disaster_readiness
)
from scripts.validate_sprint1_geometry_and_filtering import compute_relative_depth_jump


def generate_scene_candidates(
    scene_key: str,
    scene_cfg: Dict[str, Any],
    exp_root: Path
) -> Dict[str, Any]:
    print(f"\n{'='*80}\nGENERATING CANDIDATES: {scene_key.upper()}\n{'='*80}")
    scene_base_dir = exp_root / scene_key
    scene_base_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load Inputs
    d_data = np.load(scene_cfg["depth"])
    p_data = np.load(scene_cfg["poses"])
    depth_maps = d_data["depth"]
    confidences = d_data["confidence"] if "confidence" in d_data else None
    extrinsics = p_data["extrinsics"]
    intrinsics = p_data["intrinsics"]
    num_frames, H, W = depth_maps.shape[0], depth_maps.shape[1], depth_maps.shape[2]

    frame_files = sorted(
        [p for p in scene_cfg["frames"].iterdir() if p.suffix.lower() in [".png", ".jpg", ".jpeg"]],
        key=lambda p: p.name
    )[:num_frames]

    frame_rgbs = []
    for f in frame_files:
        im = Image.open(f).convert("RGB")
        if im.size != (W, H):
            im = im.resize((W, H), Image.Resampling.BILINEAR)
        frame_rgbs.append(np.array(im, dtype=np.uint8))
    frame_rgbs = np.stack(frame_rgbs, axis=0)

    traj_diag = assess_camera_trajectory(extrinsics)
    cam_centers_np = np.array(traj_diag["camera_centers"])
    mean_cam_center = np.array(traj_diag["mean_camera_center"])

    # 2. Backprojection
    world_points = depth_to_points(depth_maps, extrinsics, intrinsics)
    flat_world = world_points.reshape(-1, 3)
    flat_rgb = frame_rgbs.reshape(-1, 3)
    flat_depth = depth_maps.reshape(-1)

    # Base valid mask: depth > 0.05 & finite
    base_valid = np.isfinite(flat_world).all(axis=1) & (flat_depth > 0.05)

    # Orientation detection
    pcd_test = o3d.geometry.PointCloud()
    pcd_test.points = o3d.utility.Vector3dVector(flat_world[base_valid].astype(np.float64))
    ext_test = pcd_test.get_axis_aligned_bounding_box().get_extent()
    if ext_test[2] < min(ext_test[0], ext_test[1]):
        orientation_class = "nadir_overhead"
    else:
        orientation_class = "forward_corridor"

    # Depth-Edge filtering: use scene-adaptive rtol
    # For nadir aerial scenes (Campus, Forest), depth steps are small (0.04 is optimal)
    # For forward corridor scenes (Desert Road, River Bridge), perspective stretches depth (0.06 is optimal to preserve road/vehicle)
    rtol = 0.04 if orientation_class == "nadir_overhead" else 0.06

    all_jumps = []
    for i in range(num_frames):
        d_i = depth_maps[i, :, :, 0] if depth_maps.ndim == 4 else depth_maps[i]
        jump_i = compute_relative_depth_jump(d_i, kernel_size=3)
        all_jumps.append(jump_i)
    all_jumps = np.stack(all_jumps, axis=0)
    jump_mask = all_jumps.reshape(-1) > rtol

    # Distant horizon cap: cap depth at 99.0th percentile of valid ground to prune unconstrained sky rays
    d_valid = flat_depth[base_valid]
    horizon_cap = float(np.percentile(d_valid, 99.0))
    horizon_mask = flat_depth > horizon_cap

    # Combined clean mask
    clean_mask = base_valid & (~jump_mask) & (~horizon_mask)
    clean_pts = flat_world[clean_mask]
    clean_cols = flat_rgb[clean_mask]

    pcd_clean = o3d.geometry.PointCloud()
    pcd_clean.points = o3d.utility.Vector3dVector(clean_pts.astype(np.float64))
    pcd_clean.colors = o3d.utility.Vector3dVector(clean_cols.astype(np.float64) / 255.0)

    ref_bbox = pcd_clean.get_axis_aligned_bounding_box()
    scene_extents = ref_bbox.get_extent()
    clean_bounds = np.ptp(clean_pts, axis=0).round(4).tolist()

    # Voxel fusion for meshing stability
    voxel_size = max(float(np.max(scene_extents)) / 450.0, 0.008)
    fused_pts, fused_cols = voxel_grid_fusion(clean_pts, clean_cols, voxel_size=voxel_size)

    pcd_fused = o3d.geometry.PointCloud()
    pcd_fused.points = o3d.utility.Vector3dVector(fused_pts.astype(np.float64))
    pcd_fused.colors = o3d.utility.Vector3dVector(fused_cols.astype(np.float64) / 255.0)

    # Estimate normals oriented towards camera trajectory center
    search_radius = max(float(np.max(scene_extents)) * 0.015, 0.03)
    pcd_fused.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=30))
    pcd_fused.orient_normals_towards_camera_location(camera_location=mean_cam_center)

    candidates_summary = {}

    # -------------------------------------------------------------------------
    # Candidate 1: Optimized Point Cloud (candidate_pcd_filtered)
    # -------------------------------------------------------------------------
    cand1_dir = scene_base_dir / "candidate_pcd_filtered"
    cand1_dir.mkdir(parents=True, exist_ok=True)
    pcd_ply_path = cand1_dir / "point_cloud_filtered.ply"
    pcd_fused_path = cand1_dir / "point_cloud_fused.ply"
    save_ply(clean_pts, clean_cols, str(pcd_ply_path), binary=True)
    save_ply(fused_pts, fused_cols, str(pcd_fused_path), binary=True)

    views_cand1 = render_standard_views(pcd_fused, ref_bbox, is_mesh=False, orientation_class=orientation_class)
    for vname, img in views_cand1.items():
        img.save(cand1_dir / f"pcd_view_{vname}.png")

    roughness = compute_surface_roughness(pcd_fused)
    cand1_info = {
        "candidate_id": "candidate_pcd_filtered",
        "type": "point_cloud",
        "raw_points": len(clean_pts),
        "fused_points": len(fused_pts),
        "voxel_size": round(voxel_size, 4),
        "bounds_extents": clean_bounds,
        "roughness_residual": round(roughness, 6),
        "filtering_rtol": rtol,
        "horizon_cap_depth": round(horizon_cap, 4),
        "ply_path": str(pcd_fused_path.resolve()),
        "preview_path": str((cand1_dir / "pcd_view_top.png").resolve())
    }
    with open(cand1_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand1_info, f, indent=2)
    candidates_summary["candidate_pcd_filtered"] = cand1_info
    print(f"  [Cand 1] PCD Filtered: {len(clean_pts):,} raw pts, {len(fused_pts):,} fused pts")

    # -------------------------------------------------------------------------
    # Candidate 2: Adaptive Support-Constrained Poisson Mesh
    # -------------------------------------------------------------------------
    cand2_dir = scene_base_dir / "candidate_mesh_poisson_adaptive"
    cand2_dir.mkdir(parents=True, exist_ok=True)

    poisson_depth = 9
    mesh_p, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_fused, depth=poisson_depth)
    dens_np = np.asarray(densities)
    dens_thresh = float(np.percentile(dens_np, 5.0))  # Trim bottom 5% low-density boundary fringe
    mesh_p.remove_vertices_by_mask(dens_np < dens_thresh)

    # Adaptive support filtering to eliminate bridging and boundary curling
    base_support = max(voxel_size * 2.5, 0.02)
    filter_mesh_adaptive_support(
        mesh=mesh_p,
        reference_pcd=pcd_fused,
        base_support_dist=base_support,
        max_edge_factor=2.8,
        min_cluster_faces=50,
        depth_scaled=True
    )
    transfer_colors_kdtree(mesh_p, pcd_fused, k=4)

    m2_verts = np.asarray(mesh_p.vertices)
    m2_tris = np.asarray(mesh_p.triangles)
    m2_ext = np.ptp(m2_verts, axis=0).round(4).tolist() if len(m2_verts) > 0 else [0, 0, 0]

    mesh2_ply_path = cand2_dir / "surface_mesh.ply"
    mesh2_obj_path = cand2_dir / "surface_mesh.obj"
    o3d.io.write_triangle_mesh(str(mesh2_ply_path), mesh_p)
    o3d.io.write_triangle_mesh(str(mesh2_obj_path), mesh_p)

    views_cand2 = render_standard_views(mesh_p, ref_bbox, is_mesh=True, orientation_class=orientation_class)
    for vname, img in views_cand2.items():
        img.save(cand2_dir / f"mesh_view_{vname}.png")

    cand2_info = {
        "candidate_id": "candidate_mesh_poisson_adaptive",
        "type": "surface_mesh",
        "algorithm": "Screened Poisson (Adaptive Support Pruned)",
        "vertex_count": len(m2_verts),
        "triangle_count": len(m2_tris),
        "extents": m2_ext,
        "is_edge_manifold": bool(mesh_p.is_edge_manifold()),
        "is_vertex_manifold": bool(mesh_p.is_vertex_manifold()),
        "ply_path": str(mesh2_ply_path.resolve()),
        "obj_path": str(mesh2_obj_path.resolve()),
        "preview_path": str((cand2_dir / "mesh_view_top.png").resolve())
    }
    with open(cand2_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand2_info, f, indent=2)
    candidates_summary["candidate_mesh_poisson_adaptive"] = cand2_info
    print(f"  [Cand 2] Poisson Mesh: {len(m2_verts):,} verts, {len(m2_tris):,} tris, extents: {m2_ext}")

    # -------------------------------------------------------------------------
    # Candidate 3: Open-Surface Ball Pivoting Mesh (BPA)
    # -------------------------------------------------------------------------
    cand3_dir = scene_base_dir / "candidate_mesh_bpa_open"
    cand3_dir.mkdir(parents=True, exist_ok=True)

    # BPA radii scaled to point cloud density
    r0 = voxel_size * 1.5
    r1 = voxel_size * 3.0
    r2 = voxel_size * 6.0
    bpa_radii = [r0, r1, r2]

    mesh_bpa = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd_fused, o3d.utility.DoubleVector(bpa_radii)
    )
    mesh_bpa.remove_unreferenced_vertices()
    mesh_bpa.remove_degenerate_triangles()
    mesh_bpa.remove_duplicated_triangles()
    mesh_bpa.remove_duplicated_vertices()
    mesh_bpa.remove_non_manifold_edges()

    # Prune tiny disconnected specks
    if len(mesh_bpa.triangles) > 0:
        clusters, num_faces, _ = mesh_bpa.cluster_connected_triangles()
        clusters = np.asarray(clusters)
        num_faces = np.asarray(num_faces)
        tiny_clusters = np.where(num_faces < 30)[0]
        if len(tiny_clusters) > 0:
            mesh_bpa.remove_triangles_by_mask(np.isin(clusters, tiny_clusters))
            mesh_bpa.remove_unreferenced_vertices()

    transfer_colors_kdtree(mesh_bpa, pcd_fused, k=4)

    m3_verts = np.asarray(mesh_bpa.vertices)
    m3_tris = np.asarray(mesh_bpa.triangles)
    m3_ext = np.ptp(m3_verts, axis=0).round(4).tolist() if len(m3_verts) > 0 else [0, 0, 0]

    mesh3_ply_path = cand3_dir / "surface_mesh.ply"
    mesh3_obj_path = cand3_dir / "surface_mesh.obj"
    o3d.io.write_triangle_mesh(str(mesh3_ply_path), mesh_bpa)
    o3d.io.write_triangle_mesh(str(mesh3_obj_path), mesh_bpa)

    views_cand3 = render_standard_views(mesh_bpa, ref_bbox, is_mesh=True, orientation_class=orientation_class)
    for vname, img in views_cand3.items():
        img.save(cand3_dir / f"mesh_view_{vname}.png")

    cand3_info = {
        "candidate_id": "candidate_mesh_bpa_open",
        "type": "surface_mesh",
        "algorithm": "Ball Pivoting Algorithm (Open Manifold)",
        "radii": [round(r, 4) for r in bpa_radii],
        "vertex_count": len(m3_verts),
        "triangle_count": len(m3_tris),
        "extents": m3_ext,
        "is_edge_manifold": bool(mesh_bpa.is_edge_manifold()),
        "is_vertex_manifold": bool(mesh_bpa.is_vertex_manifold()),
        "ply_path": str(mesh3_ply_path.resolve()),
        "obj_path": str(mesh3_obj_path.resolve()),
        "preview_path": str((cand3_dir / "mesh_view_top.png").resolve())
    }
    with open(cand3_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand3_info, f, indent=2)
    candidates_summary["candidate_mesh_bpa_open"] = cand3_info
    print(f"  [Cand 3] BPA Open Mesh: {len(m3_verts):,} verts, {len(m3_tris):,} tris, extents: {m3_ext}")

    # -------------------------------------------------------------------------
    # Composite Comparison Sheet for the Scene
    # -------------------------------------------------------------------------
    tw, th = 400, 300
    sheet = Image.new("RGB", (tw * 3, th * 3 + 50), (14, 17, 23))
    draw = ImageDraw.Draw(sheet)
    draw.rectangle([0, 0, tw * 3, 45], fill=(21, 26, 36))
    draw.text((20, 12), f"F.O.R.G.E — Sprint 1 Candidate Evaluation: {scene_key.upper()} ({orientation_class.upper()})", fill=(255, 255, 255))

    rows = [
        ("Candidate 1: Filtered Point Cloud", views_cand1),
        ("Candidate 2: Adaptive Poisson Mesh", views_cand2),
        ("Candidate 3: Open BPA Terrain Mesh", views_cand3),
    ]
    cols = ["top", "frontal", "oblique"]

    for r_idx, (r_title, v_dict) in enumerate(rows):
        y_top = 50 + r_idx * th
        for c_idx, c_name in enumerate(cols):
            x_left = c_idx * tw
            img_tile = v_dict[c_name].resize((tw, th), Image.Resampling.LANCZOS)
            sheet.paste(img_tile, (x_left, y_top))
            draw.rectangle([x_left + 1, y_top + 1, x_left + tw - 1, y_top + th - 1], outline=(30, 40, 55))
            draw.rectangle([x_left + 8, y_top + 8, x_left + tw - 8, y_top + 38], fill=(0, 0, 0))
            draw.text((x_left + 12, y_top + 10), r_title, fill=(255, 220, 100))
            draw.text((x_left + 12, y_top + 24), f"View: {c_name.upper()}", fill=(180, 200, 230))

    comp_path = scene_base_dir / "candidate_comparison_sheet.png"
    sheet.save(comp_path)
    candidates_summary["comparison_sheet_path"] = str(comp_path.resolve())

    # Disaster readiness evaluation
    readiness = evaluate_disaster_readiness(
        active_points=fused_pts,
        mesh=mesh_p,
        traj_diagnostics=traj_diag,
        depth_maps=depth_maps,
        confidences=confidences,
        scene_classification=scene_key
    )
    candidates_summary["disaster_readiness"] = readiness

    with open(scene_base_dir / "scene_evaluation_summary.json", "w", encoding="utf-8") as f:
        json.dump(candidates_summary, f, indent=2)

    return candidates_summary


def main():
    exp_root = Path("outputs/accuracy_experiment/regression_diagnosis_v2")
    exp_root.mkdir(parents=True, exist_ok=True)

    scenes = {
        "campus_0049": {
            "frames": Path("outputs/dji_0049_validation/frames"),
            "depth": Path("outputs/dji_0049_validation/predictions/depth_predictions.npz"),
            "poses": Path("outputs/dji_0049_validation/predictions/camera_poses.npz")
        },
        "desert_road": {
            "frames": Path("outputs/desert_road_validation/frames"),
            "depth": Path("outputs/desert_road_validation/predictions/depth_predictions.npz"),
            "poses": Path("outputs/desert_road_validation/predictions/camera_poses.npz")
        },
        "forest_road": {
            "frames": Path("outputs/accuracy_experiment/generalization_disaster_validation_v1/forest_road/frames_uniform_span"),
            "depth": Path("outputs/accuracy_experiment/generalization_disaster_validation_v1/forest_road/predictions_uniform_span/depth_predictions.npz"),
            "poses": Path("outputs/accuracy_experiment/generalization_disaster_validation_v1/forest_road/predictions_uniform_span/camera_poses.npz")
        },
        "river_bridge": {
            "frames": Path("outputs/accuracy_experiment/river_bridge_flight_corridor/frames"),
            "depth": Path("outputs/accuracy_experiment/river_bridge_flight_corridor/predictions/depth_predictions.npz"),
            "poses": Path("outputs/accuracy_experiment/river_bridge_flight_corridor/predictions/camera_poses.npz")
        }
    }

    all_scene_results = {}
    for skey, scfg in scenes.items():
        res = generate_scene_candidates(skey, scfg, exp_root)
        all_scene_results[skey] = res

    with open(exp_root / "all_scenes_candidates_summary.json", "w", encoding="utf-8") as f:
        json.dump(all_scene_results, f, indent=2)

    print("\n" + "="*80)
    print("ALL CANDIDATES SUCCESSFULLY GENERATED & EVALUATED!")
    print(f"Summary JSON: {exp_root / 'all_scenes_candidates_summary.json'}")
    print("="*80)


if __name__ == "__main__":
    main()
