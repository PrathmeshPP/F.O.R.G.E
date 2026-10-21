"""
F.O.R.G.E — Sprint 1: Geometry & Depth-Edge Filtering Validator.
Evaluates the 4 regression scenes (Campus 0049, Desert Road, Forest Road, River Bridge):
1. Projection & Backprojection round-trip validation (synthetic and real points).
2. Old vs. Corrected rendering orientation comparison.
3. Multi-threshold depth-edge filtering analysis (overlays, rejection masks, retained clouds).
4. Generates quantitative metrics and visual artifacts in outputs/accuracy_experiment/regression_diagnosis_v2/.
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
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.point_cloud import depth_to_points, save_ply
from src.reconstruction.pipeline import render_standard_views


def colorize_depth(depth_map: np.ndarray, vmin: float = None, vmax: float = None) -> Image.Image:
    valid = np.isfinite(depth_map) & (depth_map > 0)
    if not np.any(valid):
        return Image.fromarray(np.zeros((depth_map.shape[0], depth_map.shape[1], 3), dtype=np.uint8))
    _min = vmin if vmin is not None else float(np.percentile(depth_map[valid], 2.0))
    _max = vmax if vmax is not None else float(np.percentile(depth_map[valid], 98.0))
    if _max <= _min:
        _max = _min + 1e-4
    norm = np.clip((depth_map - _min) / (_max - _min), 0.0, 1.0)
    u8 = (norm * 255).astype(np.uint8)
    bgr = cv2.applyColorMap(u8, cv2.COLORMAP_INFERNO)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb[~valid] = [20, 20, 20]
    return Image.fromarray(rgb)


def compute_relative_depth_jump(depth: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    pad = kernel_size // 2
    padded = np.pad(depth, ((pad, pad), (pad, pad)), mode="edge")
    depth_max = np.full_like(depth, -np.inf)
    depth_min = np.full_like(depth, np.inf)

    for y in range(kernel_size):
        for x in range(kernel_size):
            window = padded[y : y + depth.shape[0], x : x + depth.shape[1]]
            depth_max = np.maximum(depth_max, window)
            depth_min = np.minimum(depth_min, window)

    relative_jump = (depth_max - depth_min) / np.maximum(np.abs(depth), 1e-6)
    return relative_jump


def create_rejection_overlay(rgb: np.ndarray, mask_rejected: np.ndarray, color: Tuple[int, int, int] = (255, 30, 100)) -> Image.Image:
    overlay = rgb.copy()
    overlay[mask_rejected] = (
        overlay[mask_rejected].astype(np.float32) * 0.35 + np.array(color, dtype=np.float32) * 0.65
    ).astype(np.uint8)
    return Image.fromarray(overlay)


def run_controlled_projection_tests() -> Dict[str, Any]:
    """Tests projection and backprojection with known synthetic cameras, rotations, and 3D points."""
    print("\n--- Running Controlled Projection / Backprojection Tests ---")
    H, W = 384, 688
    fx, fy = 490.0, 490.0
    cx, cy = W / 2.0, H / 2.0
    intrinsics = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    # Non-identity camera pose: rotated 25 deg around X, 15 deg around Y, translated [0.5, -0.3, 1.2]
    theta_x = np.radians(25.0)
    theta_y = np.radians(15.0)
    Rx = np.array([[1, 0, 0], [0, np.cos(theta_x), -np.sin(theta_x)], [0, np.sin(theta_x), np.cos(theta_x)]])
    Ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)], [0, 1, 0], [-np.sin(theta_y), 0, np.cos(theta_y)]])
    R = Ry @ Rx
    T = np.array([0.5, -0.3, 1.2])
    extrinsics = np.hstack([R, T[:, None]])

    # Generate synthetic depth map (planar ground + step obstacle)
    v, u = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
    # Camera coords: Z = 2.0 + 0.002 * (v - cy)
    depth_synthetic = 2.0 + 0.002 * (v - cy)
    # Add step discontinuity
    depth_synthetic[100:180, 200:300] += 0.8

    # 1. Backproject
    world_pts = depth_to_points(depth_synthetic, extrinsics, intrinsics)
    assert world_pts.shape == (H, W, 3)

    # 2. Camera center verification
    C_computed = -R.T @ T
    # Check that ray from camera center through pixel (u,v) passes through world_pts[v, u]
    cam_pt_00 = (world_pts[192, 344] - C_computed)
    # Project back to camera coordinates
    p_cam_recovered = R @ world_pts.reshape(-1, 3).T + T[:, None]
    X_c, Y_c, Z_c = p_cam_recovered[0], p_cam_recovered[1], p_cam_recovered[2]

    depth_recovered = Z_c.reshape(H, W)
    depth_diff = np.max(np.abs(depth_recovered - depth_synthetic))

    u_proj = (fx * X_c / Z_c + cx).reshape(H, W)
    v_proj = (fy * Y_c / Z_c + cy).reshape(H, W)

    u_diff = np.max(np.abs(u_proj - u))
    v_diff = np.max(np.abs(v_proj - v))

    print(f"  Synthetic Camera Center: {C_computed.round(4).tolist()}")
    print(f"  Depth Recovery Max Error: {depth_diff:.2e} units")
    print(f"  Pixel Reprojection Max Error: u={u_diff:.2e} px, v={v_diff:.2e} px")

    test_passed = bool(depth_diff < 1e-4 and u_diff < 1e-3 and v_diff < 1e-3)
    return {
        "synthetic_test_passed": test_passed,
        "max_depth_error": float(depth_diff),
        "max_pixel_error_u": float(u_diff),
        "max_pixel_error_v": float(v_diff),
        "camera_center": C_computed.tolist(),
        "rotation_det": float(np.linalg.det(R))
    }


def validate_scene(scene_key: str, scene_cfg: Dict[str, Any], exp_root: Path) -> Dict[str, Any]:
    print(f"\n{'='*80}\nVALIDATING SCENE: {scene_key.upper()}\n{'='*80}")
    scene_dir = exp_root / scene_key
    scene_dir.mkdir(parents=True, exist_ok=True)
    filters_dir = scene_dir / "depth_edge_filtering"
    filters_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load Data
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

    rgbs = []
    for f in frame_files:
        im = Image.open(f).convert("RGB")
        if im.size != (W, H):
            im = im.resize((W, H), Image.Resampling.BILINEAR)
        rgbs.append(np.array(im, dtype=np.uint8))
    rgbs = np.stack(rgbs, axis=0)

    # 2. Geometry Inspection & Reprojection Test on Real Predictions
    print(f"  Frames: {num_frames}, Resolution: {W}x{H}")
    world_points = depth_to_points(depth_maps, extrinsics, intrinsics)

    # Check reprojection accuracy for each frame
    reproj_errors = []
    in_bounds_counts = 0
    total_reproj_points = 0

    for i in range(num_frames):
        d_i = depth_maps[i, :, :, 0] if depth_maps.ndim == 4 else depth_maps[i]
        ext_i = extrinsics[i]
        intr_i = intrinsics[i]
        pts_i = world_points[i].reshape(-1, 3)

        R_i = ext_i[:3, :3]
        T_i = ext_i[:3, 3]
        cam_i = R_i @ pts_i.T + T_i[:, None]
        Z_c = cam_i[2]
        valid_z = Z_c > 0.05

        u_rep = intr_i[0, 0] * (cam_i[0, valid_z] / Z_c[valid_z]) + intr_i[0, 2]
        v_rep = intr_i[1, 1] * (cam_i[1, valid_z] / Z_c[valid_z]) + intr_i[1, 2]

        in_bounds = (u_rep >= 0) & (u_rep < W) & (v_rep >= 0) & (v_rep < H)
        in_bounds_counts += np.sum(in_bounds)
        total_reproj_points += len(u_rep)

        # Ground truth grid coordinates
        v_grid, u_grid = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
        u_true = u_grid.reshape(-1)[valid_z]
        v_true = v_grid.reshape(-1)[valid_z]
        err = np.mean(np.sqrt((u_rep - u_true)**2 + (v_rep - v_true)**2))
        reproj_errors.append(float(err))

    mean_reproj_err = float(np.mean(reproj_errors))
    reproj_in_bounds_pct = float(in_bounds_counts / total_reproj_points * 100.0) if total_reproj_points > 0 else 0.0
    print(f"  Reprojection Mean Pixel Error: {mean_reproj_err:.4e} px, In-Bounds: {reproj_in_bounds_pct:.2f}%")

    # Camera centers & baseline
    cam_centers = [-extrinsics[i, :3, :3].T @ extrinsics[i, :3, 3] for i in range(num_frames)]
    cam_centers_np = np.array(cam_centers)
    traj_span = np.ptp(cam_centers_np, axis=0)
    net_trans = float(np.linalg.norm(cam_centers_np[-1] - cam_centers_np[0])) if num_frames > 1 else 0.0
    print(f"  Camera Trajectory: net translation = {net_trans:.4f} units, span = {[round(x, 4) for x in traj_span]}")

    # 3. Old vs. Corrected Rendering Comparison
    # Detect ground plane orientation
    pcd_full = o3d.geometry.PointCloud()
    flat_world = world_points.reshape(-1, 3)
    flat_rgb = rgbs.reshape(-1, 3)
    valid_raw = np.isfinite(flat_world).all(axis=1) & (depth_maps.reshape(-1) > 0.05)
    pcd_full.points = o3d.utility.Vector3dVector(flat_world[valid_raw].astype(np.float64))
    pcd_full.colors = o3d.utility.Vector3dVector(flat_rgb[valid_raw].astype(np.float64) / 255.0)

    # Determine orientation class
    extents = pcd_full.get_axis_aligned_bounding_box().get_extent()
    if extents[2] < min(extents[0], extents[1]):
        orientation_class = "nadir_overhead"
    else:
        orientation_class = "forward_corridor"
    print(f"  Scene Orientation: {orientation_class.upper()} (Extents: {[round(x, 4) for x in extents]})")

    # Corrected views
    ref_box = pcd_full.get_axis_aligned_bounding_box()
    views_corr = render_standard_views(pcd_full, ref_box, is_mesh=False, orientation_class=orientation_class)
    for vname, img in views_corr.items():
        img.save(scene_dir / f"baseline_pcd_corrected_{vname}.png")

    # Old legacy views (hardcoded front=[0, -1, 0], up=[0, 0, 1])
    vis_old = o3d.visualization.Visualizer()
    vis_old.create_window(visible=False, width=800, height=600)
    vis_old.add_geometry(pcd_full)
    c_old = vis_old.get_view_control()
    c_old.set_lookat(ref_box.get_center())
    c_old.set_front([0.0, -1.0, 0.0])  # Old hardcoded top view
    c_old.set_up([0.0, 0.0, 1.0])
    c_old.set_zoom(0.75)
    vis_old.poll_events()
    vis_old.update_renderer()
    buf_old = vis_old.capture_screen_float_buffer(do_render=True)
    vis_old.destroy_window()
    img_old_top = Image.fromarray((np.asarray(buf_old) * 255).astype(np.uint8))
    img_old_top.save(scene_dir / "baseline_pcd_old_top_view.png")

    # Save side-by-side Old vs Corrected Top View
    comp_top = Image.new("RGB", (1600, 600), (14, 17, 23))
    comp_top.paste(img_old_top, (0, 0))
    comp_top.paste(views_corr["top"], (800, 0))
    d_top = ImageDraw.Draw(comp_top)
    d_top.rectangle([10, 10, 360, 45], fill=(0, 0, 0))
    d_top.text((20, 15), "OLD RENDERER (Hardcoded Front [0,-1,0])", fill=(255, 100, 100))
    d_top.rectangle([810, 10, 1180, 45], fill=(0, 0, 0))
    d_top.text((820, 15), f"CORRECTED RENDERER ({orientation_class.upper()})", fill=(100, 255, 180))
    comp_top.save(scene_dir / "comparison_rendering_old_vs_corrected_top.png")

    # 4. Multi-Threshold Depth-Edge Filtering Analysis (Section 3)
    print("\n  --- Depth-Edge Discontinuity Analysis Across Thresholds ---")
    rtols = [0.03, 0.05, 0.08]
    filtering_results = {}

    # Compute jump map for all frames
    all_jumps = []
    for i in range(num_frames):
        d_i = depth_maps[i, :, :, 0] if depth_maps.ndim == 4 else depth_maps[i]
        jump = compute_relative_depth_jump(d_i, kernel_size=3)
        all_jumps.append(jump)
    all_jumps = np.stack(all_jumps, axis=0)

    # Save diagnostics for key frame (Frame 0 or frame with vehicle/bridge)
    key_frame_idx = 0
    d_key = depth_maps[key_frame_idx, :, :, 0] if depth_maps.ndim == 4 else depth_maps[key_frame_idx]
    rgb_key = rgbs[key_frame_idx]
    jump_key = all_jumps[key_frame_idx]

    Image.fromarray(rgb_key).save(filters_dir / "key_frame_rgb.png")
    colorize_depth(d_key).save(filters_dir / "key_frame_depth_colored.png")
    # Colorize jump magnitude (0.0 to 0.2 scale)
    jump_norm = np.clip(jump_key / 0.15, 0.0, 1.0)
    jump_colored = cv2.applyColorMap((jump_norm * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)
    Image.fromarray(cv2.cvtColor(jump_colored, cv2.COLOR_BGR2RGB)).save(filters_dir / "key_frame_depth_jump_magnitude.png")

    for rtol in rtols:
        mask_rejected_3d = all_jumps > rtol
        rejection_ratio = float(np.mean(mask_rejected_3d)) * 100.0
        retained_mask = valid_raw & (~mask_rejected_3d.reshape(-1))
        retained_count = int(np.sum(retained_mask))
        rejected_count = int(np.sum(valid_raw) - retained_count)

        pts_retained = flat_world[retained_mask]
        cols_retained = flat_rgb[retained_mask]

        pcd_retained = o3d.geometry.PointCloud()
        pcd_retained.points = o3d.utility.Vector3dVector(pts_retained.astype(np.float64))
        pcd_retained.colors = o3d.utility.Vector3dVector(cols_retained.astype(np.float64) / 255.0)

        # Save retained point cloud
        ply_name = f"point_cloud_filtered_rtol_{rtol:.2f}.ply"
        save_ply(pts_retained, cols_retained, str(filters_dir / ply_name), binary=True)

        # Key frame overlay
        mask_key_rej = mask_rejected_3d[key_frame_idx]
        overlay_key = create_rejection_overlay(rgb_key, mask_key_rej, color=(255, 30, 80))
        overlay_key.save(filters_dir / f"overlay_rejected_pixels_rtol_{rtol:.2f}.png")
        Image.fromarray((mask_key_rej * 255).astype(np.uint8)).save(filters_dir / f"rejection_mask_rtol_{rtol:.2f}.png")

        # Retained PCD render
        views_ret = render_standard_views(pcd_retained, ref_box, is_mesh=False, orientation_class=orientation_class)
        views_ret["top"].save(filters_dir / f"retained_pcd_top_rtol_{rtol:.2f}.png")
        views_ret["oblique"].save(filters_dir / f"retained_pcd_oblique_rtol_{rtol:.2f}.png")

        pt_bounds = np.ptp(pts_retained, axis=0).round(4).tolist() if len(pts_retained) > 0 else [0, 0, 0]
        filtering_results[f"rtol_{rtol:.2f}"] = {
            "rtol": rtol,
            "rejection_ratio_pct": round(rejection_ratio, 2),
            "retained_points": retained_count,
            "rejected_points": rejected_count,
            "bounds_ptp": pt_bounds,
            "ply_file": ply_name
        }
        print(f"    rtol={rtol:.2f}: {rejection_ratio:.2f}% rejected ({rejected_count:,} pts), retained {retained_count:,} pts, extents: {pt_bounds}")

    # No filtering baseline
    filtering_results["no_filter"] = {
        "rtol": None,
        "rejection_ratio_pct": 0.0,
        "retained_points": int(np.sum(valid_raw)),
        "rejected_points": 0,
        "bounds_ptp": extents.round(4).tolist(),
        "ply_file": "point_cloud_raw_unfused.ply"
    }
    save_ply(flat_world[valid_raw], flat_rgb[valid_raw], str(filters_dir / "point_cloud_raw_unfused.ply"), binary=True)

    # Multi-threshold composite sheet
    w_t, h_t = 384, 216
    comp_sheet = Image.new("RGB", (w_t * 4, h_t * 2 + 50), (14, 17, 23))
    draw_s = ImageDraw.Draw(comp_sheet)
    draw_s.rectangle([0, 0, w_t * 4, 45], fill=(21, 26, 36))
    draw_s.text((20, 12), f"F.O.R.G.E — Depth-Edge Discontinuity Analysis: {scene_key.upper()}", fill=(255, 255, 255))

    comp_sheet.paste(Image.open(filters_dir / "key_frame_rgb.png").resize((w_t, h_t)), (0, 50))
    comp_sheet.paste(Image.open(filters_dir / "key_frame_depth_colored.png").resize((w_t, h_t)), (w_t, 50))
    comp_sheet.paste(Image.open(filters_dir / "key_frame_depth_jump_magnitude.png").resize((w_t, h_t)), (w_t * 2, 50))
    comp_sheet.paste(views_corr["top"].resize((w_t, h_t)), (w_t * 3, 50))

    comp_sheet.paste(Image.open(filters_dir / "overlay_rejected_pixels_rtol_0.03.png").resize((w_t, h_t)), (0, 50 + h_t))
    comp_sheet.paste(Image.open(filters_dir / "overlay_rejected_pixels_rtol_0.05.png").resize((w_t, h_t)), (w_t, 50 + h_t))
    comp_sheet.paste(Image.open(filters_dir / "overlay_rejected_pixels_rtol_0.08.png").resize((w_t, h_t)), (w_t * 2, 50 + h_t))
    comp_sheet.paste(Image.open(filters_dir / "retained_pcd_top_rtol_0.05.png").resize((w_t, h_t)), (w_t * 3, 50 + h_t))

    draw_s.text((10, 55), "Source RGB", fill=(255, 255, 255))
    draw_s.text((w_t + 10, 55), "Predicted Depth", fill=(255, 255, 255))
    draw_s.text((w_t * 2 + 10, 55), "Depth Jump Magnitude", fill=(255, 255, 255))
    draw_s.text((w_t * 3 + 10, 55), "Raw Top View", fill=(255, 255, 255))

    draw_s.text((10, 55 + h_t), "Overlay rtol=0.03", fill=(255, 100, 100))
    draw_s.text((w_t + 10, 55 + h_t), "Overlay rtol=0.05", fill=(255, 200, 100))
    draw_s.text((w_t * 2 + 10, 55 + h_t), "Overlay rtol=0.08", fill=(100, 255, 100))
    draw_s.text((w_t * 3 + 10, 55 + h_t), "Filtered PCD (rtol=0.05)", fill=(100, 200, 255))

    comp_sheet.save(scene_dir / "depth_edge_filtering_composite.png")

    scene_summary = {
        "scene_name": scene_key,
        "num_frames": num_frames,
        "resolution": [W, H],
        "orientation_classification": orientation_class,
        "extents_raw": extents.round(4).tolist(),
        "camera_centers": cam_centers_np.round(4).tolist(),
        "net_translation": round(net_trans, 4),
        "reprojection_mean_pixel_error": mean_reproj_err,
        "reprojection_in_bounds_pct": reproj_in_bounds_pct,
        "filtering_analysis": filtering_results
    }
    with open(scene_dir / "validation_summary.json", "w", encoding="utf-8") as f:
        json.dump(scene_summary, f, indent=2)

    return scene_summary


def main():
    exp_root = Path("outputs/accuracy_experiment/regression_diagnosis_v2")
    exp_root.mkdir(parents=True, exist_ok=True)

    # Step 1: Controlled synthetic test
    synth_results = run_controlled_projection_tests()

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

    all_summaries = {"synthetic_tests": synth_results, "scenes": {}}
    for skey, scfg in scenes.items():
        summary = validate_scene(skey, scfg, exp_root)
        all_summaries["scenes"][skey] = summary

    with open(exp_root / "master_validation_summary.json", "w", encoding="utf-8") as f:
        json.dump(all_summaries, f, indent=2)

    print("\n" + "="*80)
    print("VALIDATION COMPLETE! Master summary saved to:")
    print(exp_root / "master_validation_summary.json")
    print("="*80)


if __name__ == "__main__":
    main()
