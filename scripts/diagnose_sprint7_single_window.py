"""F.O.R.G.E — Sprint 7: Single-Window Diagnostic and Geometric Root-Cause Analysis.

Performs comprehensive diagnostic inspection across all four benchmark scenes:
1. Input and frame integrity: video hashes, frame timestamps, aspect ratios, source contact sheets.
2. Camera poses, intrinsics, and coordinate transforms: pinhole calibration, trajectory metrics, trajectory plots.
3. Predicted depth and confidence distributions and visual preview contact sheets.
4. Per-frame point clouds in common world coordinates and pairwise registration residuals.
5. Scene-specific geometric defect root-cause investigations.
"""

from __future__ import annotations

import os
import sys
import time
import json
import hashlib
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional

import cv2
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.point_cloud import depth_to_points, voxel_grid_fusion
from src.reconstruction.photographic_texture import extract_native_frames, scale_camera_intrinsics

SPRINT7_SCENES = {
    "campus_0049": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4",
        "description": "Campus aerial drone survey: building facades, solar roofs, walkways, and courtyard",
        "orientation_class": "nadir_overhead",
        "num_frames": 8,
        "timestamps": [0.0, 2.302, 4.605, 6.907, 9.193, 11.495, 13.797, 16.099],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/predictions",
        "depth_range": (0.05, 5.0),
        "conf_min": 1.0,
    },
    "desert_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\desert_road.mp4",
        "description": "Desert highway corridor: dark asphalt road, yellow dashed lines, blue Mini Cooper, and sand shoulders",
        "orientation_class": "forward_corridor",
        "num_frames": 8,
        "timestamps": [0.0, 2.836, 5.672, 8.508, 11.345, 14.181, 17.017, 19.853],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions",
        "depth_range": (0.2, 3.5),
        "conf_min": 1.5,
    },
    "forest_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\forest_road.mp4",
        "description": "Forest canopy drone survey: coniferous & deciduous crowns, needle foliage, and asphalt roadway",
        "orientation_class": "nadir_overhead",
        "num_frames": 10,
        "timestamps": [0.0, 3.378, 6.757, 10.135, 13.513, 16.892, 20.270, 23.649, 27.027, 30.405],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/predictions",
        "depth_range": (0.05, 3.5),
        "conf_min": 1.2,
    },
    "river_bridge": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4",
        "description": "Bridge crossing & riverbed corridor: concrete construction, red dirt roadway, riverbank terrain",
        "orientation_class": "forward_corridor",
        "num_frames": 8,
        "timestamps": [17.985, 22.272, 26.560, 30.847, 35.135, 39.423, 43.710, 47.998],
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/predictions",
        "depth_range": (0.2, 4.2),
        "conf_min": 2.0,
    }
}


def compute_file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(65536):
            h.update(chunk)
    return h.hexdigest()


def make_source_frames_contact_sheet(
    frame_paths: List[Path],
    timestamps: List[float],
    scene_name: str,
    output_path: Path
) -> Path:
    N = len(frame_paths)
    cols = 4
    rows = (N + cols - 1) // cols
    w_tile, h_tile = 320, 180
    header_h = 55
    canvas_w = cols * w_tile
    canvas_h = header_h + rows * (h_tile + 25)

    canvas = Image.new("RGB", (canvas_w, canvas_h), (16, 20, 26))
    draw = ImageDraw.Draw(canvas)
    draw.text((15, 14), f"F.O.R.G.E SPRINT 7 SOURCE FRAMES — {scene_name.upper()} ({N} FRAMES)", fill=(255, 255, 255))
    draw.text((15, 34), f"Extracted from source video across flight timeline | Uniform single-window interval", fill=(170, 195, 220))

    for idx, (fp, t) in enumerate(zip(frame_paths, timestamps)):
        r = idx // cols
        c = idx % cols
        x = c * w_tile
        y = header_h + r * (h_tile + 25)

        im = Image.open(fp).resize((w_tile, h_tile), Image.Resampling.LANCZOS)
        canvas.paste(im, (x, y))

        draw.rectangle([x, y + h_tile, x + w_tile, y + h_tile + 22], fill=(10, 14, 20))
        draw.text((x + 8, y + h_tile + 4), f"Frame {idx:02d} | t={t:.2f}s", fill=(220, 230, 240))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, "PNG")
    return output_path


def make_depth_conf_preview(
    depth_maps: np.ndarray,
    conf_maps: np.ndarray,
    timestamps: List[float],
    scene_name: str,
    output_path: Path
) -> Path:
    N, H, W = depth_maps.shape[:3]
    cols = min(N, 4)
    rows = ((N + cols - 1) // cols) * 2  # Depth row then Conf row for each slice
    w_tile, h_tile = 300, int(300 * (H / W))
    header_h = 60
    canvas_w = cols * w_tile
    canvas_h = header_h + (N // cols + (1 if N % cols != 0 else 0)) * (h_tile * 2 + 40)

    canvas = Image.new("RGB", (canvas_w, canvas_h), (16, 20, 26))
    draw = ImageDraw.Draw(canvas)
    draw.text((15, 14), f"F.O.R.G.E SPRINT 7 PREDICTED DEPTH & CONFIDENCE — {scene_name.upper()}", fill=(255, 255, 255))
    draw.text((15, 36), "Upper: Normalized Inverse Depth | Lower: Metric Confidence Map (Infer: 384x688)", fill=(170, 195, 220))

    num_chunks = (N + cols - 1) // cols
    for chunk in range(num_chunks):
        c_start = chunk * cols
        c_end = min(N, c_start + cols)
        base_y = header_h + chunk * (h_tile * 2 + 40)

        for col_idx, i in enumerate(range(c_start, c_end)):
            x = col_idx * w_tile

            # Depth Map (colormap)
            d = depth_maps[i]
            d_val = d[np.isfinite(d) & (d > 0.05)]
            if len(d_val) > 0:
                p5, p95 = np.percentile(d_val, 5), np.percentile(d_val, 95)
                d_norm = np.clip((d - p5) / max(p95 - p5, 1e-4), 0.0, 1.0)
            else:
                d_norm = np.zeros_like(d)
            d_u8 = (d_norm * 255).astype(np.uint8)
            d_color = cv2.applyColorMap(d_u8, cv2.COLORMAP_TURBO)
            d_im = Image.fromarray(cv2.cvtColor(d_color, cv2.COLOR_BGR2RGB)).resize((w_tile, h_tile))
            canvas.paste(d_im, (x, base_y))
            draw.text((x + 8, base_y + 6), f"F{i:02d} Depth (t={timestamps[i]:.2f}s)", fill=(255, 255, 255))

            # Confidence Map (colormap)
            c = conf_maps[i]
            c_val = c[np.isfinite(c)]
            if len(c_val) > 0:
                c_p5, c_p95 = np.percentile(c_val, 5), np.percentile(c_val, 95)
                c_norm = np.clip((c - c_p5) / max(c_p95 - c_p5, 1e-4), 0.0, 1.0)
            else:
                c_norm = np.zeros_like(c)
            c_u8 = (c_norm * 255).astype(np.uint8)
            c_color = cv2.applyColorMap(c_u8, cv2.COLORMAP_VIRIDIS)
            c_im = Image.fromarray(cv2.cvtColor(c_color, cv2.COLOR_BGR2RGB)).resize((w_tile, h_tile))
            canvas.paste(c_im, (x, base_y + h_tile + 4))
            draw.text((x + 8, base_y + h_tile + 10), f"F{i:02d} Conf [min={c.min():.1f}, max={c.max():.1f}]", fill=(255, 255, 255))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, "PNG")
    return output_path


def make_trajectory_diagnostic_plot(
    extrinsics: np.ndarray,
    timestamps: List[float],
    scene_name: str,
    output_path: Path
) -> Path:
    """Renders camera trajectory diagnostic diagram with top-down (X-Z) and side-elevation (Y-Z/X-Y)."""
    W_canvas, H_canvas = 1000, 520
    canvas = Image.new("RGB", (W_canvas, H_canvas), (14, 18, 24))
    draw = ImageDraw.Draw(canvas)

    draw.text((20, 15), f"F.O.R.G.E SPRINT 7 CAMERA TRAJECTORY DIAGNOSTIC — {scene_name.upper()}", fill=(255, 255, 255))
    draw.text((20, 36), "Camera centers and optical forward vectors in world coordinates [meters]", fill=(170, 195, 220))

    N = len(extrinsics)
    cam_centers = []
    cam_fwds = []
    for i in range(N):
        R = extrinsics[i, :3, :3]
        t = extrinsics[i, :3, 3]
        C = -R.T @ t
        fwd = R.T @ np.array([0.0, 0.0, 0.3]) # 30cm visual vector
        cam_centers.append(C)
        cam_fwds.append(fwd)

    centers = np.array(cam_centers)
    fwds = np.array(cam_fwds)

    # Subplot 1: Top-Down X vs Z (Left: x in [40, 480], y in [70, 480])
    draw.rectangle([40, 70, 480, 480], outline=(40, 50, 65), fill=(20, 26, 36))
    draw.text((50, 78), "Top-Down View (X-Z Plane)", fill=(100, 220, 255))
    draw.text((50, 95), "Horiz: X, Vert: Z", fill=(140, 160, 180))

    # Subplot 2: Elevation Profile Z vs Y (Right: x in [520, 960], y in [70, 480])
    draw.rectangle([520, 70, 960, 480], outline=(40, 50, 65), fill=(20, 26, 36))
    draw.text((530, 78), "Elevation Profile (Z-Y Plane)", fill=(100, 240, 160))
    draw.text((530, 95), "Horiz: Z, Vert: Y (Elevation down)", fill=(140, 160, 180))

    # Normalization for Top-Down (X, Z)
    x_min, x_max = centers[:, 0].min(), centers[:, 0].max()
    z_min, z_max = centers[:, 2].min(), centers[:, 2].max()
    y_min, y_max = centers[:, 1].min(), centers[:, 1].max()

    pad_x = max((x_max - x_min) * 0.25, 0.2)
    pad_z = max((z_max - z_min) * 0.25, 0.2)
    pad_y = max((y_max - y_min) * 0.25, 0.2)

    x_lo, x_hi = x_min - pad_x, x_max + pad_x
    z_lo, z_hi = z_min - pad_z, z_max + pad_z
    y_lo, y_hi = y_min - pad_y, y_max + pad_y

    def map_xz(x_val, z_val):
        px = 40 + int(((x_val - x_lo) / max(x_hi - x_lo, 1e-4)) * 400 + 20)
        pz = 480 - int(((z_val - z_lo) / max(z_hi - z_lo, 1e-4)) * 370 + 20)
        return px, pz

    def map_zy(z_val, y_val):
        pz = 520 + int(((z_val - z_lo) / max(z_hi - z_lo, 1e-4)) * 400 + 20)
        py = 70 + int(((y_val - y_lo) / max(y_hi - y_lo, 1e-4)) * 370 + 20)
        return pz, py

    # Draw trajectories
    pts_xz = [map_xz(centers[i, 0], centers[i, 2]) for i in range(N)]
    pts_zy = [map_zy(centers[i, 2], centers[i, 1]) for i in range(N)]

    for i in range(N - 1):
        draw.line([pts_xz[i], pts_xz[i + 1]], fill=(70, 130, 180), width=2)
        draw.line([pts_zy[i], pts_zy[i + 1]], fill=(70, 180, 130), width=2)

    for i in range(N):
        px1, pz1 = pts_xz[i]
        draw.ellipse([px1 - 5, pz1 - 5, px1 + 5, pz1 + 5], fill=(255, 200, 80), outline=(255, 255, 255))
        # Vector arrow
        fx, fz = map_xz(centers[i, 0] + fwds[i, 0], centers[i, 2] + fwds[i, 2])
        draw.line([(px1, pz1), (fx, fz)], fill=(255, 80, 80), width=2)
        draw.text((px1 + 7, pz1 - 6), f"C{i}", fill=(240, 240, 240))

        pz2, py2 = pts_zy[i]
        draw.ellipse([pz2 - 5, py2 - 5, pz2 + 5, py2 + 5], fill=(255, 200, 80), outline=(255, 255, 255))
        fz_e, fy_e = map_zy(centers[i, 2] + fwds[i, 2], centers[i, 1] + fwds[i, 1])
        draw.line([(pz2, py2), (fz_e, fy_e)], fill=(255, 80, 80), width=2)
        draw.text((pz2 + 7, py2 - 6), f"C{i}", fill=(240, 240, 240))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, "PNG")
    return output_path


def compute_pairwise_residuals(
    pcd_list: List[o3d.geometry.PointCloud],
    max_search_dist: float = 0.05
) -> List[Dict[str, Any]]:
    """Calculates nearest-neighbor residual statistics between adjacent frames in overlap regions."""
    residuals = []
    N = len(pcd_list)
    for i in range(N - 1):
        pcd_a = pcd_list[i]
        pcd_b = pcd_list[i + 1]

        if len(pcd_a.points) == 0 or len(pcd_b.points) == 0:
            residuals.append({"pair": f"F{i}-F{i+1}", "status": "empty"})
            continue

        bbox_a = pcd_a.get_axis_aligned_bounding_box()
        bbox_b = pcd_b.get_axis_aligned_bounding_box()

        # Intersection box
        min_inter = np.maximum(bbox_a.get_min_bound(), bbox_b.get_min_bound())
        max_inter = np.minimum(bbox_a.get_max_bound(), bbox_b.get_max_bound())

        if np.any(min_inter >= max_inter):
            residuals.append({
                "pair": f"F{i}-F{i+1}",
                "status": "no_bbox_overlap",
                "median_m": None,
                "rmse_m": None
            })
            continue

        inter_box = o3d.geometry.AxisAlignedBoundingBox(min_inter, max_inter)
        crop_a = pcd_a.crop(inter_box)
        crop_b = pcd_b.crop(inter_box)

        if len(crop_a.points) < 50 or len(crop_b.points) < 50:
            residuals.append({
                "pair": f"F{i}-F{i+1}",
                "status": "sparse_overlap",
                "pts_a": len(crop_a.points),
                "pts_b": len(crop_b.points),
                "median_m": None,
                "rmse_m": None
            })
            continue

        dists = crop_a.compute_point_cloud_distance(crop_b)
        dists_arr = np.asarray(dists)
        # Filter within max search distance to represent true surface correspondences
        inliers = dists_arr[dists_arr <= max_search_dist]
        inlier_ratio = len(inliers) / max(len(dists_arr), 1)

        residuals.append({
            "pair": f"F{i}-F{i+1}",
            "status": "measured",
            "overlap_points_a": len(crop_a.points),
            "overlap_points_b": len(crop_b.points),
            "inlier_ratio": round(inlier_ratio, 3),
            "median_residual_m": round(float(np.median(dists_arr)), 4),
            "mean_residual_m": round(float(np.mean(dists_arr)), 4),
            "rmse_residual_m": round(float(np.sqrt(np.mean(dists_arr ** 2))), 4),
            "inlier_median_m": round(float(np.median(inliers)), 4) if len(inliers) > 0 else None,
            "inlier_rmse_m": round(float(np.sqrt(np.mean(inliers ** 2))), 4) if len(inliers) > 0 else None
        })

    return residuals


def run_single_window_diagnostics():
    out_root = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7_single_window_stabilization"
    out_root.mkdir(parents=True, exist_ok=True)

    master_diag = {}

    for scene_key, cfg in SPRINT7_SCENES.items():
        print(f"\n{'='*80}")
        print(f"RUNNING SPRINT 7 DIAGNOSTIC: {scene_key.upper()}")
        print(f"{'='*80}")

        scene_out = out_root / scene_key
        scene_out.mkdir(parents=True, exist_ok=True)
        per_frame_dir = scene_out / "per_frame_pcds"
        per_frame_dir.mkdir(parents=True, exist_ok=True)

        # 1. Source Video Verification
        vpath = Path(cfg["video_path"])
        v_hash = compute_file_sha256(vpath)
        cap = cv2.VideoCapture(str(vpath))
        w_nat = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h_nat = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        dur_s = total_frames / fps if fps > 0 else 0
        cap.release()

        print(f"  Source Video: {vpath.name}")
        print(f"  SHA256: {v_hash}")
        print(f"  Resolution: {w_nat}x{h_nat} @ {fps:.2f}fps ({total_frames} frames, {dur_s:.2f}s)")

        # Extract frames if not already present
        native_dir = scene_out / "extracted_frames"
        num_frames = cfg["num_frames"]
        extracted = extract_native_frames(str(vpath), cfg["timestamps"][:num_frames], native_dir)
        frame_paths = [e[0] for e in extracted]

        # Contact Sheet
        cs_path = scene_out / "source_frames_contact_sheet.png"
        make_source_frames_contact_sheet(frame_paths, cfg["timestamps"][:num_frames], scene_key, cs_path)
        print(f"  Generated source frames contact sheet: {cs_path.name}")

        # 2. Ingest Predictions
        pred_dir = cfg["predictions_dir"]
        poses_data = np.load(pred_dir / "camera_poses.npz")
        depth_data = np.load(pred_dir / "depth_predictions.npz")

        extrinsics = poses_data["extrinsics"] # (N, 3, 4)
        intrinsics = poses_data["intrinsics"] # (N, 3, 3)
        depth_maps = np.squeeze(depth_data["depth"]) # (N, H, W)
        conf_maps = np.squeeze(depth_data["confidence"]) # (N, H, W)

        N, infer_h, infer_w = depth_maps.shape

        # Camera trajectory visualization
        traj_path = scene_out / "camera_trajectory_diagnostic.png"
        make_trajectory_diagnostic_plot(extrinsics, cfg["timestamps"][:N], scene_key, traj_path)
        print(f"  Generated camera trajectory diagnostic: {traj_path.name}")

        # Depth & confidence previews
        dc_path = scene_out / "depth_confidence_preview.png"
        make_depth_conf_preview(depth_maps, conf_maps, cfg["timestamps"][:N], scene_key, dc_path)
        print(f"  Generated depth & confidence preview: {dc_path.name}")

        # 3. Export Per-Frame Point Clouds and Measure Pairwise Residuals
        pcd_frames = []
        d_min, d_max = cfg["depth_range"]
        c_min = cfg["conf_min"]

        for i in range(N):
            d_i = depth_maps[i]
            c_i = conf_maps[i]
            ext_i = extrinsics[i]
            K_i = intrinsics[i]

            im_native = cv2.imread(str(frame_paths[i]))
            im_rgb = cv2.cvtColor(cv2.resize(im_native, (infer_w, infer_h)), cv2.COLOR_BGR2RGB)

            pts_i = depth_to_points(d_i, ext_i, K_i).reshape(infer_h, infer_w, 3)
            val_i = (
                np.isfinite(pts_i).all(axis=-1)
                & (d_i >= d_min)
                & (d_i <= d_max)
                & (c_i >= c_min)
            )

            pcd_i = o3d.geometry.PointCloud()
            pcd_i.points = o3d.utility.Vector3dVector(pts_i[val_i].astype(np.float64))
            pcd_i.colors = o3d.utility.Vector3dVector(im_rgb[val_i].astype(np.float64) / 255.0)

            # Export per-frame PLY
            frame_ply_path = per_frame_dir / f"frame_{i:04d}_world.ply"
            o3d.io.write_point_cloud(str(frame_ply_path), pcd_i)
            pcd_frames.append(pcd_i)

        residuals = compute_pairwise_residuals(pcd_frames)
        print("  Pairwise Overlap Residuals:")
        for r in residuals:
            print(f"    {r['pair']}: status={r.get('status')} | inlier_ratio={r.get('inlier_ratio')} | median={r.get('median_residual_m')}m | inlier_rmse={r.get('inlier_rmse_m')}m")

        # 4. Scene-Specific Quantitative Diagnostics
        scene_specific_metrics = {}

        if scene_key == "desert_road":
            # Track moving car across all 8 frames
            car_tracks = []
            for i in range(N):
                im_bgr = cv2.imread(str(frame_paths[i]))
                im_infer = cv2.resize(im_bgr, (infer_w, infer_h))
                hsv = cv2.cvtColor(im_infer, cv2.COLOR_BGR2HSV)
                # Defensible road area mask (lower half of image)
                road_y_mask = np.zeros(im_infer.shape[:2], dtype=bool)
                road_y_mask[int(infer_h * 0.45):, :] = True
                car_m = (hsv[:, :, 0] >= 95) & (hsv[:, :, 0] <= 130) & (hsv[:, :, 1] >= 60) & (hsv[:, :, 2] >= 40) & road_y_mask

                pts_i = depth_to_points(depth_maps[i], extrinsics[i], intrinsics[i]).reshape(infer_h, infer_w, 3)
                car_pts = pts_i[car_m]
                ys, xs = np.where(car_m)

                if len(car_pts) > 0:
                    c3d = car_pts.mean(axis=0)
                    car_tracks.append({
                        "frame_idx": i,
                        "timestamp_s": cfg["timestamps"][i],
                        "pixel_center_xy": [float(xs.mean()), float(ys.mean())],
                        "pixel_bbox": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                        "world_center_xyz_m": [round(float(c3d[j]), 4) for j in range(3)],
                        "points_count": len(car_pts)
                    })

            scene_specific_metrics["vehicle_tracking"] = car_tracks
            print(f"  Tracked vehicle in {len(car_tracks)} frames along forward road corridor")

        elif scene_key == "campus_0049":
            # Facade and roof normal distributions
            # Nadir aerial views have predominantly horizontal surfaces
            all_pts = np.vstack([np.asarray(p.points) for p in pcd_frames])
            pcd_merged = o3d.geometry.PointCloud()
            pcd_merged.points = o3d.utility.Vector3dVector(all_pts)
            pcd_merged.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
            normals = np.asarray(pcd_merged.normals)
            vertical_facade_ratio = float(np.mean(np.abs(normals[:, 1]) < 0.3))
            horizontal_roof_ratio = float(np.mean(np.abs(normals[:, 1]) > 0.8))
            scene_specific_metrics["surface_orientations"] = {
                "vertical_facade_point_ratio": round(vertical_facade_ratio, 4),
                "horizontal_roof_ground_ratio": round(horizontal_roof_ratio, 4)
            }
            print(f"  Campus Normal distribution: vertical facades = {vertical_facade_ratio*100:.1f}%, horizontal roof/ground = {horizontal_roof_ratio*100:.1f}%")

        elif scene_key == "forest_road":
            # Canopy density and void analysis
            all_pts = np.vstack([np.asarray(p.points) for p in pcd_frames])
            # Check Z-profile (canopy height variation)
            z_extents = float(np.ptp(all_pts[:, 2]))
            y_extents = float(np.ptp(all_pts[:, 1]))
            scene_specific_metrics["canopy_extents"] = {
                "height_spread_m": round(y_extents, 3),
                "flight_corridor_length_m": round(z_extents, 3),
                "total_canopy_points": len(all_pts)
            }
            print(f"  Forest Canopy: height spread = {y_extents:.2f}m, corridor length = {z_extents:.2f}m, pts = {len(all_pts):,}")

        elif scene_key == "river_bridge":
            # Compare confidence and depth between water and bridge deck
            # Deck is near center, water is in low-elevation regions
            d_all = depth_maps.flatten()
            c_all = conf_maps.flatten()
            val_mask = (d_all >= d_min) & (d_all <= d_max)
            deck_conf_mean = float(np.mean(c_all[val_mask & (c_all >= 2.0)]))
            water_conf_mean = float(np.mean(c_all[val_mask & (c_all < 2.0)]))
            scene_specific_metrics["riverbed_vs_deck"] = {
                "high_conf_deck_mean": round(deck_conf_mean, 2),
                "uncertain_water_mean": round(water_conf_mean, 2)
            }
            print(f"  River Bridge Confidence: Bridge Deck = {deck_conf_mean:.1f}, Water Specular = {water_conf_mean:.1f}")

        # Assemble scene diagnostics record
        diag_record = {
            "scene_name": scene_key,
            "source_video": {
                "path": str(vpath),
                "sha256": v_hash,
                "resolution": [w_nat, h_nat],
                "fps": round(fps, 2),
                "total_frames": total_frames,
                "duration_s": round(dur_s, 2)
            },
            "inference": {
                "frames_ingested": N,
                "timestamps": cfg["timestamps"][:N],
                "inference_resolution": [infer_w, infer_h],
                "aspect_ratio": round(infer_h / infer_w, 4),
                "depth_range_filter": list(cfg["depth_range"]),
                "confidence_threshold": cfg["conf_min"]
            },
            "camera_calibration": {
                "focal_length_x_mean": round(float(np.mean(intrinsics[:, 0, 0])), 2),
                "focal_length_y_mean": round(float(np.mean(intrinsics[:, 1, 1])), 2),
                "principal_point_x": round(float(np.mean(intrinsics[:, 0, 2])), 2),
                "principal_point_y": round(float(np.mean(intrinsics[:, 1, 2])), 2)
            },
            "pairwise_residuals": residuals,
            "scene_specific_metrics": scene_specific_metrics,
            "artifacts": {
                "contact_sheet": str(cs_path.resolve()),
                "camera_trajectory": str(traj_path.resolve()),
                "depth_confidence_preview": str(dc_path.resolve()),
                "per_frame_pcd_dir": str(per_frame_dir.resolve())
            }
        }

        # Save scene diagnostic JSON
        with open(scene_out / "single_window_diagnostics.json", "w") as f:
            json.dump(diag_record, f, indent=2)

        master_diag[scene_key] = diag_record

    # Save Master Diagnostics Report
    master_file = out_root / "sprint7_diagnostics_report.json"
    with open(master_file, "w") as f:
        json.dump(master_diag, f, indent=2)

    print(f"\n{'='*80}")
    print(f"SPRINT 7 DIAGNOSTICS COMPLETE — Master Report: {master_file.name}")
    print(f"{'='*80}")


if __name__ == "__main__":
    run_single_window_diagnostics()
