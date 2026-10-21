"""F.O.R.G.E — Sprint 6: Master Reconstruction and Quality Improvement for All Four Videos.

Generates for every scene under outputs/accuracy_experiment/all_scenes_quality_v1/<scene>/:
1. Unfused Dense Point Cloud (point_cloud_unfused.ply)
2. Fine-Scale Fused Point Cloud (point_cloud_fused.ply)
3. High-Fidelity Surface Mesh (surface_mesh_repaired.ply / .obj)
4. Multi-Material Photographic OBJ (multiview_textured_mesh.obj + .mtl)
5. Unified High-Resolution Texture Atlas OBJ (textured_mesh.obj + texture_atlas.png)
6. Photographic Vertex-Colored PLY (surface_mesh_photographic.ply)
7. Standard Perspective Views (render_view_top.png, render_view_frontal.png, render_view_oblique.png)
8. Close-up Magnification View (render_view_closeup.png)
9. 4-Panel Comparative Evaluation Contact Sheet (comparison_contact_sheet.png)
10. Comprehensive Candidate Metadata JSON (candidate_metadata.json)
"""

from __future__ import annotations

import os
import sys
import time
import json
import shutil
import argparse
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional

import cv2
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.point_cloud import depth_to_points, voxel_grid_fusion
from src.reconstruction.multiframe_stitcher import SurfaceConnectivityMesher
from src.reconstruction.photographic_texture import (
    compute_geometry_hash,
    extract_native_frames,
    scale_camera_intrinsics,
    MultiViewTextureEngine
)
from scripts.validate_sprint1_geometry_and_filtering import compute_relative_depth_jump

SPRINT6_SCENES = {
    "campus_0049": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4",
        "description": "Campus aerial drone survey: building facades, solar roofs, walkways, and courtyard",
        "orientation_class": "nadir_overhead",
        "num_frames": 8,
        "timestamps": [0.0, 2.302, 4.605, 6.907, 9.193, 11.495, 13.797, 16.099],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/predictions",
        "jump_rtol": 0.12,
        "depth_range": (0.05, 5.0),
        "conf_min": 1.0,
        "horizon_quantile": 99.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.002,
        "support_distance_factor": 4.5,
        "max_hole_vertices": 32,
        "atlas_size": 4096,
        "moving_object_strategy": None,
        "prior_best_mesh": PROJECT_ROOT / "outputs/accuracy_experiment/photographic_texturing_v1/campus_0049/surface_mesh_photographic.ply"
    },
    "desert_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\desert_road.mp4",
        "description": "Desert highway corridor: dark asphalt road, yellow dashed lines, blue Mini Cooper, and sand shoulders",
        "orientation_class": "forward_corridor",
        "num_frames": 8,
        "timestamps": [0.0, 2.836, 5.672, 8.508, 11.345, 14.181, 17.017, 19.853],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions",
        "jump_rtol": 0.08,
        "depth_range": (0.2, 3.5),
        "conf_min": 1.5,
        "horizon_quantile": 95.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.005,
        "support_distance_factor": 3.5,
        "max_hole_vertices": 24,
        "atlas_size": 2048,
        "moving_object_strategy": "desert_road_vehicle",
        "prior_best_mesh": PROJECT_ROOT / "outputs/accuracy_experiment/photographic_texturing_v1/desert_road/surface_mesh_photographic.ply"
    },
    "forest_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\forest_road.mp4",
        "description": "Forest canopy drone survey: coniferous & deciduous crowns, needle foliage, and asphalt roadway",
        "orientation_class": "nadir_overhead",
        "num_frames": 10,
        "timestamps": [0.0, 3.378, 6.757, 10.135, 13.513, 16.892, 20.270, 23.649, 27.027, 30.405],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/predictions",
        "jump_rtol": 0.12,
        "depth_range": (0.05, 3.5),
        "conf_min": 1.2,
        "horizon_quantile": 99.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.002,
        "support_distance_factor": 4.5,
        "max_hole_vertices": 32,
        "atlas_size": 4096,
        "moving_object_strategy": None,
        "prior_best_mesh": PROJECT_ROOT / "outputs/accuracy_experiment/photographic_texturing_v1/forest_road/surface_mesh_photographic.ply"
    },
    "river_bridge": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4",
        "description": "Bridge crossing & riverbed corridor: concrete construction, red dirt roadway, riverbank terrain",
        "orientation_class": "forward_corridor",
        "num_frames": 8,
        "timestamps": [17.985, 22.272, 26.560, 30.847, 35.135, 39.423, 43.710, 47.998],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/predictions",
        "jump_rtol": 0.08,
        "depth_range": (0.2, 4.2),
        "conf_min": 2.0,
        "horizon_quantile": 95.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.004,
        "support_distance_factor": 4.0,
        "max_hole_vertices": 24,
        "atlas_size": 4096,
        "moving_object_strategy": None,
        "prior_best_mesh": PROJECT_ROOT / "outputs/accuracy_experiment/photographic_texturing_v1/river_bridge/surface_mesh_photographic.ply"
    }
}


def render_scene_views(
    mesh: o3d.geometry.TriangleMesh,
    orientation_class: str,
    out_dir: Path
) -> Dict[str, Path]:
    """Renders top, frontal, oblique, and close-up perspective views."""
    views = {}
    W, H = 800, 600
    ref_bbox = mesh.get_axis_aligned_bounding_box()
    ctr_pt = ref_bbox.get_center()

    configs = {
        "top": {"front": [0.0, 0.0, -1.0], "up": [0.0, -1.0, 0.0], "zoom": 0.75},
        "frontal": {"front": [0.0, -0.98, -0.15], "up": [0.0, 0.0, -1.0], "zoom": 0.75} if orientation_class == "nadir_overhead" else {"front": [0.0, -0.15, -0.98], "up": [0.0, -1.0, 0.0], "zoom": 0.75},
        "oblique": {"front": [0.5, 0.5, -0.7], "up": [0.0, -1.0, 0.0], "zoom": 0.75} if orientation_class == "nadir_overhead" else {"front": [0.4, -0.3, -0.85], "up": [0.0, -1.0, 0.0], "zoom": 0.75},
        "closeup": {"front": [0.3, 0.4, -0.85], "up": [0.0, -1.0, 0.0], "zoom": 0.32} if orientation_class == "nadir_overhead" else {"front": [0.0, -0.3, -0.95], "up": [0.0, -1.0, 0.0], "zoom": 0.32}
    }

    for vname, cfg in configs.items():
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=W, height=H)
        vis.add_geometry(mesh)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.10, 0.14])
        opt.mesh_color_option = o3d.visualization.MeshColorOption.Color

        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr_pt)
        ctrl.set_front(cfg["front"])
        ctrl.set_up(cfg["up"])
        ctrl.set_zoom(cfg["zoom"])

        vis.poll_events()
        vis.update_renderer()
        buf = vis.capture_screen_float_buffer(do_render=True)
        vis.destroy_window()

        img_path = out_dir / f"render_view_{vname}.png"
        Image.fromarray((np.asarray(buf) * 255).astype(np.uint8)).save(img_path)
        views[vname] = img_path

    return views


def render_comparative_contact_sheet(
    prior_mesh_path: Path,
    new_mesh: o3d.geometry.TriangleMesh,
    atlas_image_path: Path,
    orientation_class: str,
    output_png: Path,
    scene_name: str,
    scene_desc: str
) -> Path:
    """Creates a matched 4-panel comparison contact sheet."""
    W_panel, H_panel = 640, 480
    header_h = 75
    canvas = Image.new("RGB", (W_panel * 4, header_h + H_panel), (14, 18, 24))
    draw = ImageDraw.Draw(canvas)

    # Title & Subtitle
    title = f"F.O.R.G.E SPRINT 6 MASTER RECONSTRUCTION COMPARISON — {scene_name.upper()}"
    subtitle = f"Prior Best Candidate vs Sprint 6 Quality Reconstructed & Textured Candidate | {scene_desc}"
    draw.text((25, 14), title, fill=(255, 255, 255))
    draw.text((25, 42), subtitle, fill=(180, 205, 230))

    ref_bbox = new_mesh.get_axis_aligned_bounding_box()
    ctr_pt = ref_bbox.get_center()

    def capture_view(m, closeup=False):
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=W_panel, height=H_panel)
        vis.add_geometry(m)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.10, 0.14])
        opt.mesh_color_option = o3d.visualization.MeshColorOption.Color

        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr_pt)
        if orientation_class == "nadir_overhead":
            if not closeup:
                ctrl.set_front([0.0, 0.0, -1.0])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.75)
            else:
                ctrl.set_front([0.3, 0.4, -0.85])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.32)
        else:
            if not closeup:
                ctrl.set_front([0.0, -0.2, -0.98])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.75)
            else:
                ctrl.set_front([0.0, -0.3, -0.95])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.32)

        vis.poll_events()
        vis.update_renderer()
        buf = vis.capture_screen_float_buffer(do_render=True)
        vis.destroy_window()
        return Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))

    # Panel 1: Prior Best
    if prior_mesh_path.is_file():
        prior_m = o3d.io.read_triangle_mesh(str(prior_mesh_path))
        img_p1 = capture_view(prior_m, closeup=False)
    else:
        img_p1 = Image.new("RGB", (W_panel, H_panel), (20, 24, 30))
    canvas.paste(img_p1, (0, header_h))

    # Panel 2: Sprint 6 Unified Model Overview
    img_p2 = capture_view(new_mesh, closeup=False)
    canvas.paste(img_p2, (W_panel, header_h))

    # Panel 3: Sprint 6 Close-Up Zoom (Detail Inspection)
    img_p3 = capture_view(new_mesh, closeup=True)
    canvas.paste(img_p3, (W_panel * 2, header_h))

    # Panel 4: Texture Atlas
    if atlas_image_path.is_file():
        atlas_img = Image.open(atlas_image_path).resize((W_panel, H_panel), Image.Resampling.LANCZOS)
    else:
        atlas_img = Image.new("RGB", (W_panel, H_panel), (20, 24, 30))
    canvas.paste(atlas_img, (W_panel * 3, header_h))

    # Badges
    cards = [
        (0, "1. Prior Candidate", "(Sprint 4 / Sprint 5 Baseline)", (240, 120, 100)),
        (W_panel, "2. Sprint 6 Candidate", "(Unified Mesh & Photographic Color)", (100, 240, 160)),
        (W_panel * 2, "3. Close-Up Inspection", "(Surface Detail & Sharp Boundaries)", (100, 220, 255)),
        (W_panel * 3, "4. UV Texture Atlas", f"({atlas_image_path.name})", (255, 220, 100))
    ]
    for x_off, c_title, c_sub, col in cards:
        draw.rectangle([x_off + 12, header_h + 12, x_off + 390, header_h + 54], fill=(10, 14, 20))
        draw.text((x_off + 20, header_h + 16), c_title, fill=col)
        draw.text((x_off + 20, header_h + 34), c_sub, fill=(200, 220, 240))

    output_png.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_png, format="PNG")
    return output_png


def process_scene_sprint6(
    scene_key: str,
    cfg: Dict[str, Any],
    out_root: Path
) -> Dict[str, Any]:
    t0_start = time.time()
    print("\n" + "=" * 80)
    print(f"F.O.R.G.E SPRINT 6 RECONSTRUCTION: {scene_key.upper()}")
    print(f"Objective: {cfg['description']}")
    print(f"Source Video: {cfg['video_path']}")
    print("=" * 80)

    scene_out = out_root / scene_key
    scene_out.mkdir(parents=True, exist_ok=True)

    # Step 1: Load Predictions and Frames
    print("\n[Step 1/6] Ingesting camera poses and depth predictions...")
    pred_dir = cfg["predictions_dir"]
    poses_data = np.load(pred_dir / "camera_poses.npz")
    depth_data = np.load(pred_dir / "depth_predictions.npz")

    extrinsics = poses_data["extrinsics"] # (N, 3, 4)
    intrinsics = poses_data["intrinsics"] # (N, 3, 3)
    d_maps = np.squeeze(depth_data["depth"]) # (N, H, W)
    c_maps = np.squeeze(depth_data["confidence"]) # (N, H, W)

    infer_h, infer_w = d_maps.shape[1], d_maps.shape[2]
    num_cams = len(extrinsics)

    # Extract Native High-Resolution Video Frames
    print(f"\n[Step 2/6] Extracting {num_cams} native uncompressed frames from source video...")
    native_dir = scene_out / "native_frames"
    extracted_frames = extract_native_frames(
        video_path=cfg["video_path"],
        timestamps=cfg["timestamps"][:num_cams],
        out_dir=native_dir
    )
    native_paths = [e[0] for e in extracted_frames]
    native_w, native_h = extracted_frames[0][1], extracted_frames[0][2]
    print(f"  Native Resolution: {native_w}x{native_h} | Inference Resolution: {infer_w}x{infer_h}")

    # Scale camera intrinsics to native resolution
    scaled_intrinsics = scale_camera_intrinsics(
        intrinsics=intrinsics,
        infer_res=(infer_w, infer_h),
        native_res=(native_w, native_h)
    )

    # Step 3: Backprojection & Scene-Adaptive Filtering
    print("\n[Step 3/6] Backprojecting 3D points and executing principled filtering...")
    all_raw_pts = []
    all_raw_cols = []
    all_clean_pts = []
    all_clean_cols = []

    # Detect vehicle mask for desert road
    car_masks_infer = []
    if cfg["moving_object_strategy"] == "desert_road_vehicle":
        for i in range(num_cams):
            im_native = cv2.imread(str(native_paths[i]))
            im_infer = cv2.resize(im_native, (infer_w, infer_h))
            hsv = cv2.cvtColor(im_infer, cv2.COLOR_BGR2HSV)
            mask = (hsv[:, :, 0] >= 95) & (hsv[:, :, 0] <= 130) & (hsv[:, :, 1] >= 55) & (hsv[:, :, 2] >= 35)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
            car_masks_infer.append(cv2.dilate(mask.astype(np.uint8), kernel).astype(bool))

    d_min, d_max = cfg["depth_range"]
    c_min = cfg["conf_min"]
    rtol = cfg["jump_rtol"]

    for i in range(num_cams):
        d_i = d_maps[i]
        c_i = c_maps[i]
        ext_i = extrinsics[i]
        K_i = intrinsics[i]

        im_native = cv2.imread(str(native_paths[i]))
        im_rgb = cv2.cvtColor(cv2.resize(im_native, (infer_w, infer_h)), cv2.COLOR_BGR2RGB)

        pts_i = depth_to_points(d_i, ext_i, K_i).reshape(infer_h, infer_w, 3)
        cols_i = im_rgb

        # Valid mask
        valid_i = (
            np.isfinite(pts_i).all(axis=-1)
            & (d_i >= d_min)
            & (d_i <= d_max)
            & (c_i >= c_min)
        )

        # Depth-jump filtering (preserves edges according to rtol)
        jump_i = compute_relative_depth_jump(d_i, kernel_size=3)
        edge_clean_i = valid_i & (jump_i <= rtol)

        # Horizon quantile ceiling
        valid_depths = d_i[valid_i]
        if len(valid_depths) > 0:
            h_cap = float(np.percentile(valid_depths, cfg["horizon_quantile"]))
            horizon_mask = d_i <= h_cap
        else:
            horizon_mask = np.ones_like(d_i, dtype=bool)

        clean_i = edge_clean_i & horizon_mask

        # Desert road dynamic object separation: in frames 1..7, exclude moving car
        if cfg["moving_object_strategy"] == "desert_road_vehicle" and i > 0:
            clean_i = clean_i & (~car_masks_infer[i])

        all_raw_pts.append(pts_i[valid_i])
        all_raw_cols.append(cols_i[valid_i])
        all_clean_pts.append(pts_i[clean_i])
        all_clean_cols.append(cols_i[clean_i])

    raw_pts_stacked = np.vstack(all_raw_pts)
    raw_cols_stacked = np.vstack(all_raw_cols)
    clean_pts_stacked = np.vstack(all_clean_pts)
    clean_cols_stacked = np.vstack(all_clean_cols)

    print(f"  Raw Points: {len(raw_pts_stacked):,} -> Clean Filtered Points: {len(clean_pts_stacked):,} ({len(clean_pts_stacked)/max(len(raw_pts_stacked),1)*100:.1f}%)")

    # Step 4: Scale-Aware Voxel Fusion
    print("\n[Step 4/6] Scale-aware voxel grid fusion...")
    extents = np.ptp(clean_pts_stacked, axis=0)
    max_extent = float(np.max(extents))
    voxel_size = max(max_extent / 1200.0, 0.005)
    fused_pts, fused_cols = voxel_grid_fusion(clean_pts_stacked, clean_cols_stacked, voxel_size=voxel_size)

    print(f"  Fused Points: {len(fused_pts):,} (voxel size: {voxel_size:.4f}m)")
    print(f"  Spatial Extents [X, Y, Z]: [{extents[0]:.2f}, {extents[1]:.2f}, {extents[2]:.2f}] m")

    # Save Point Clouds
    pcd_unfused = o3d.geometry.PointCloud()
    pcd_unfused.points = o3d.utility.Vector3dVector(raw_pts_stacked.astype(np.float64))
    pcd_unfused.colors = o3d.utility.Vector3dVector(raw_cols_stacked.astype(np.float64) / 255.0)

    pcd_fused = o3d.geometry.PointCloud()
    pcd_fused.points = o3d.utility.Vector3dVector(fused_pts.astype(np.float64))
    pcd_fused.colors = o3d.utility.Vector3dVector(fused_cols.astype(np.float64) / 255.0)

    pcd_unfused_path = scene_out / "point_cloud_unfused.ply"
    pcd_fused_path = scene_out / "point_cloud_fused.ply"
    o3d.io.write_point_cloud(str(pcd_unfused_path), pcd_unfused)
    o3d.io.write_point_cloud(str(pcd_fused_path), pcd_fused)

    # Step 5: Surface Reconstruction & Hole Closure
    print("\n[Step 5/6] Surface reconstruction with boundary repair and support-distance filtering...")
    # Estimate normals oriented towards camera trajectory
    cam_centers = []
    for i in range(num_cams):
        R = extrinsics[i, :3, :3]
        t = extrinsics[i, :3, 3]
        cam_centers.append(-R.T @ t)
    mean_cam_center = np.mean(cam_centers, axis=0)

    normal_radius = max(voxel_size * 2.5, 0.015)
    pcd_fused.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30))
    pcd_fused.orient_normals_towards_camera_location(camera_location=mean_cam_center)

    mesh_poisson, p_meta = SurfaceConnectivityMesher.build_poisson_hq(
        pcd_fused,
        depth=cfg["poisson_depth"],
        gentle_trim_quantile=cfg["gentle_trim_quantile"],
        support_distance_factor=cfg["support_distance_factor"]
    )

    # Boundary hole closure
    mesh_repaired, rep_meta = SurfaceConnectivityMesher.repair_surface_connectivity(
        mesh_poisson,
        max_hole_vertices=cfg["max_hole_vertices"]
    )
    print(f"  Repaired mesh: {len(mesh_repaired.vertices):,} vertices, {len(mesh_repaired.triangles):,} triangles ({rep_meta.get('holes_closed', 0)} holes closed)")

    mesh_repaired_ply = scene_out / "surface_mesh_repaired.ply"
    mesh_repaired_obj = scene_out / "surface_mesh_repaired.obj"
    o3d.io.write_triangle_mesh(str(mesh_repaired_ply), mesh_repaired)
    o3d.io.write_triangle_mesh(str(mesh_repaired_obj), mesh_repaired)

    # Step 6: Photographic Texture Reconstruction
    print("\n[Step 6/6] Executing multi-view photographic texturing (OBJ, Atlas, PLY)...")
    tex_engine = MultiViewTextureEngine(
        mesh=mesh_repaired,
        extrinsics=extrinsics,
        intrinsics_native=scaled_intrinsics,
        frame_paths=native_paths,
        depth_maps=d_maps,
        moving_object_strategy=cfg["moving_object_strategy"]
    )

    best_cam_per_face, q_scores, vis_stats = tex_engine.compute_face_visibility_and_scores()
    print(f"  Photographic Face Coverage: {vis_stats['photographic_coverage_pct']}% ({vis_stats['direct_photographic_faces']:,} / {vis_stats['total_faces']:,} faces)")

    # 6A: Multi-Material Uncompressed OBJ (1:1 native 4K/HD mapping)
    multiview_obj_path = scene_out / "multiview_textured_mesh.obj"
    tex_engine.export_multiview_obj(multiview_obj_path, best_cam_per_face)

    # 6B: Unified UV Texture Atlas OBJ
    atlas_obj_path = scene_out / "textured_mesh.obj"
    atlas_obj_path, atlas_png_path, atlas_stats = tex_engine.export_packed_atlas_obj(
        output_obj_path=atlas_obj_path,
        best_cam_per_face=best_cam_per_face,
        atlas_size=cfg["atlas_size"]
    )

    # 6C: Photographic Vertex-Colored PLY
    photo_ply_path = scene_out / "surface_mesh_photographic.ply"
    tex_engine.export_photographic_vertex_ply(photo_ply_path, best_cam_per_face)

    # Render standard perspective views and close-ups
    print("\nRendering preview perspectives and close-ups...")
    mesh_final = o3d.io.read_triangle_mesh(str(photo_ply_path))
    views = render_scene_views(mesh_final, cfg["orientation_class"], scene_out)

    # Render 4-Panel Comparative Contact Sheet
    contact_sheet_path = scene_out / "comparison_contact_sheet.png"
    render_comparative_contact_sheet(
        prior_mesh_path=cfg["prior_best_mesh"],
        new_mesh=mesh_final,
        atlas_image_path=atlas_png_path,
        orientation_class=cfg["orientation_class"],
        output_png=contact_sheet_path,
        scene_name=scene_key,
        scene_desc=cfg["description"]
    )
    print(f"  - Saved master contact sheet: {contact_sheet_path.name}")

    total_time = time.time() - t0_start
    final_hash = compute_geometry_hash(mesh_final)

    metadata = {
        "pipeline": "F.O.R.G.E Sprint 6: Master Reconstruction and Quality Improvement",
        "scene_name": scene_key,
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "description": cfg["description"],
        "source_video": cfg["video_path"],
        "frame_count_ingested": num_cams,
        "timestamps": cfg["timestamps"][:num_cams],
        "native_resolution": [native_w, native_h],
        "inference_resolution": [infer_w, infer_h],
        "point_clouds": {
            "unfused_path": str(pcd_unfused_path.resolve()),
            "unfused_points": len(raw_pts_stacked),
            "fused_path": str(pcd_fused_path.resolve()),
            "fused_points": len(fused_pts),
            "voxel_size_m": round(voxel_size, 4),
            "extents_xyz_m": [round(float(extents[i]), 3) for i in range(3)]
        },
        "surface_mesh": {
            "repaired_ply": str(mesh_repaired_ply.resolve()),
            "repaired_obj": str(mesh_repaired_obj.resolve()),
            "vertex_count": len(mesh_repaired.vertices),
            "triangle_count": len(mesh_repaired.triangles),
            "sha256_hash": final_hash,
            "holes_closed": rep_meta.get("holes_closed", 0)
        },
        "texture_deliverables": {
            "multiview_obj": str(multiview_obj_path.resolve()),
            "multiview_mtl": str(multiview_obj_path.with_suffix(".mtl").resolve()),
            "unified_atlas_obj": str(atlas_obj_path.resolve()),
            "unified_atlas_mtl": str(atlas_obj_path.with_suffix(".mtl").resolve()),
            "texture_atlas_png": str(atlas_png_path.resolve()),
            "atlas_resolution": [cfg["atlas_size"], cfg["atlas_size"]],
            "atlas_size_mb": atlas_stats["atlas_size_mb"],
            "photographic_vertex_ply": str(photo_ply_path.resolve()),
            "photographic_coverage_pct": vis_stats["photographic_coverage_pct"]
        },
        "preview_renders": {
            "top": str(views["top"].resolve()),
            "frontal": str(views["frontal"].resolve()),
            "oblique": str(views["oblique"].resolve()),
            "closeup": str(views["closeup"].resolve()),
            "comparison_contact_sheet": str(contact_sheet_path.resolve())
        },
        "total_processing_time_sec": round(total_time, 2)
    }

    meta_file = scene_out / "candidate_metadata.json"
    meta_file.write_text(json.dumps(metadata, indent=2))
    print(f"\n[DONE] Scene {scene_key} completed successfully in {total_time:.2f}s!")
    return metadata


def main():
    parser = argparse.ArgumentParser(description="F.O.R.G.E Sprint 6 All Scenes Reconstruction")
    parser.add_argument("--scenes", nargs="+", default=["campus_0049", "desert_road", "forest_road", "river_bridge"],
                        help="Scenes to reconstruct")
    parser.add_argument("--out-dir", type=str, default="outputs/accuracy_experiment/all_scenes_quality_v1",
                        help="Output experimental directory")
    args = parser.parse_args()

    out_root = PROJECT_ROOT / args.out_dir
    out_root.mkdir(parents=True, exist_ok=True)

    results = {}
    for scene_key in args.scenes:
        if scene_key not in SPRINT6_SCENES:
            print(f"Unknown scene: {scene_key}, skipping.")
            continue
        cfg = SPRINT6_SCENES[scene_key]
        try:
            results[scene_key] = process_scene_sprint6(scene_key, cfg, out_root)
        except Exception as e:
            print(f"[ERROR] Failed processing {scene_key}: {e}")
            import traceback
            traceback.print_exc()

    # Master report JSON
    master_p = out_root / "sprint6_master_report.json"
    master_p.write_text(json.dumps({
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "output_directory": str(out_root.resolve()),
        "scenes_processed": list(results.keys()),
        "scenes": results
    }, indent=2))
    print(f"\n{'='*80}\nSPRINT 6 ALL SCENES COMPLETE — Master Report: {master_p}\n{'='*80}")


if __name__ == "__main__":
    main()
