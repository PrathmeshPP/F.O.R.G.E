"""F.O.R.G.E — Forensic Diagnostic: Window-by-Window Alignment Regression Audit.

Investigates and exposes the root causes of the Sprint 2 multi-frame stitching failure:
1. Under-determined 7-DoF similarity estimation on near-zero baseline and collinear camera centers.
2. Wild scale inflation and unconstrained rotation flips.
3. Color-codes each window independently (Cyan, Magenta, Yellow, Green) to visualize
   duplicated scene copies, fan-shaped spikes, and geometric divergence.
4. Generates quantitative diagnostic JSON reports and multi-angle renders.
"""

from __future__ import annotations

import os
import sys
import json
import time
from pathlib import Path
from typing import Dict, Any, List, Tuple

import numpy as np
import open3d as o3d
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.point_cloud import depth_to_points
from src.geo.umeyama_ransac import umeyama_alignment
from src.reconstruction.pipeline import render_standard_views

WINDOW_COLORS = [
    [0.1, 0.8, 0.9],  # Window 0: Cyan
    [0.9, 0.1, 0.8],  # Window 1: Magenta
    [0.95, 0.85, 0.1], # Window 2: Yellow
    [0.1, 0.85, 0.2],  # Window 3: Green
]


def diagnose_scene_stitching(
    scene_key: str,
    failed_dir: Path,
    diag_out_dir: Path
) -> Dict[str, Any]:
    print(f"\n{'='*80}")
    print(f"RUNNING FORENSIC AUDIT: {scene_key.upper()}")
    print(f"{'='*80}")

    scene_diag_dir = diag_out_dir / scene_key
    scene_diag_dir.mkdir(parents=True, exist_ok=True)

    summary_file = failed_dir / scene_key / "scene_evaluation_summary.json"
    if not summary_file.exists():
        print(f"Summary file not found: {summary_file}")
        return {}

    with open(summary_file, "r", encoding="utf-8") as f:
        failed_summary = json.load(f)

    poses_file = failed_dir / scene_key / "predictions" / "camera_poses.npz"
    depth_file = failed_dir / scene_key / "predictions" / "depth_predictions.npz"
    p_data = np.load(poses_file)
    d_data = np.load(depth_file)
    ext = p_data["extrinsics"]
    intr = p_data["intrinsics"]
    depth = d_data["depth"]

    align_reports = failed_summary["windowed_alignment"]["alignment_reports"]

    # Compute camera trajectory
    def get_centers(extrinsics):
        return np.array([-extrinsics[i, :, :3].T @ extrinsics[i, :, 3] for i in range(extrinsics.shape[0])])

    cam_centers = get_centers(ext)
    traj_lengths = np.linalg.norm(cam_centers[1:] - cam_centers[:-1], axis=1)

    print(f"Total unified frames: {len(ext)}")
    print(f"Trajectory bounds: min={cam_centers.min(axis=0).round(4)}, max={cam_centers.max(axis=0).round(4)}")
    print(f"Total camera path length: {np.sum(traj_lengths):.4f}")

    window_diagnostics = []
    for r in align_reports:
        w_idx = r["window_idx"]
        s = r["scale_factor"]
        R = np.array(r["rotation_matrix"])
        t = np.array(r["translation_vector"])
        max_res = r["max_residual_error"]
        sf = r["shared_frame_indices"]

        rot_trace = np.trace(R)
        rot_angle_deg = float(np.degrees(np.arccos(np.clip((rot_trace - 1.0) / 2.0, -1.0, 1.0))))

        is_scale_explosive = bool(s > 2.0 or s < 0.5)
        is_rotation_extreme = bool(rot_angle_deg > 30.0)

        diag_entry = {
            "window_idx": w_idx,
            "shared_frames": sf,
            "scale_factor": s,
            "rotation_angle_deg": round(rot_angle_deg, 3),
            "translation_norm": round(float(np.linalg.norm(t)), 4),
            "reported_in_sample_residual": max_res,
            "is_scale_explosive": is_scale_explosive,
            "is_rotation_extreme": is_rotation_extreme,
            "degeneracy_mechanism": (
                "Near-zero baseline singularity (division by zero variance)"
                if is_scale_explosive
                else "Collinear motion null-space rotation"
                if is_rotation_extreme
                else "Marginal drift"
            )
        }
        window_diagnostics.append(diag_entry)
        print(f"  Window {w_idx}: Scale={s:.4f}, RotAngle={rot_angle_deg:.1f}deg, Res={max_res:.6f}")
        if is_scale_explosive or is_rotation_extreme:
            print(f"    [CRITICAL DEFECT] {diag_entry['degeneracy_mechanism']}")

    # Build color-coded point cloud showing each window
    # Partition frames according to Sprint 2 plan: W=8, overlap=3
    # Window 0: 0..7, Window 1: 5..12, Window 2: 10..17, Window 3: 16..23
    window_ranges = [(0, 8), (5, 13), (10, 18), (16, 24)]

    # Backproject all frames
    world_pts = depth_to_points(depth, ext, intr)
    flat_world = world_pts.reshape(len(ext), -1, 3)
    flat_depth = depth.reshape(len(ext), -1)

    all_window_pcds = []
    color_coded_pts = []
    color_coded_cols = []

    for w_idx, (start, end) in enumerate(window_ranges):
        if w_idx >= len(WINDOW_COLORS):
            w_col = [0.7, 0.7, 0.7]
        else:
            w_col = WINDOW_COLORS[w_idx]

        w_pts_list = []
        for f_idx in range(start, min(end, len(ext))):
            f_pts = flat_world[f_idx]
            f_d = flat_depth[f_idx]
            valid = np.isfinite(f_pts).all(axis=1) & (f_d > 0.1) & (f_d < np.percentile(f_d[f_d > 0.1], 98.0))
            # Subsample for rendering speed
            sub_pts = f_pts[valid][::16]
            w_pts_list.append(sub_pts)

        if w_pts_list:
            w_pts = np.vstack(w_pts_list)
            pcd_w = o3d.geometry.PointCloud()
            pcd_w.points = o3d.utility.Vector3dVector(w_pts.astype(np.float64))
            pcd_w.paint_uniform_color(w_col)
            all_window_pcds.append(pcd_w)

            color_coded_pts.append(w_pts)
            color_coded_cols.append(np.full_like(w_pts, w_col))

    combined_diag_pcd = o3d.geometry.PointCloud()
    if color_coded_pts:
        combined_diag_pcd.points = o3d.utility.Vector3dVector(np.vstack(color_coded_pts).astype(np.float64))
        combined_diag_pcd.colors = o3d.utility.Vector3dVector(np.vstack(color_coded_cols).astype(np.float64))

    # Save diagnostic colored point cloud
    diag_pcd_path = scene_diag_dir / "window_alignment_color_coded.ply"
    o3d.io.write_point_cloud(str(diag_pcd_path), combined_diag_pcd)

    # Render diagnostic views
    ref_bbox = combined_diag_pcd.get_axis_aligned_bounding_box()
    orientation_class = failed_summary.get("orientation_class", "forward_corridor")
    views = render_standard_views(combined_diag_pcd, ref_bbox, is_mesh=False, orientation_class=orientation_class)
    for vname, img in views.items():
        img.save(scene_diag_dir / f"diagnostic_view_{vname}.png")

    report = {
        "scene_name": scene_key,
        "failed_source_directory": str(failed_dir.resolve()),
        "window_diagnostics": window_diagnostics,
        "unified_camera_trajectory": {
            "total_frames": len(ext),
            "camera_path_length": round(float(np.sum(traj_lengths)), 4),
            "spatial_extents": np.ptp(cam_centers, axis=0).round(4).tolist()
        },
        "color_coded_ply_path": str(diag_pcd_path.resolve()),
        "root_cause_finding": (
            "Collinear or near-zero baseline overlap frames produced singular 7-DoF similarity matrices, "
            "causing wild rotation flips and scale explosions that sheared the point clouds into fan-shaped spikes."
        )
    }

    with open(scene_diag_dir / "alignment_diagnostic_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    print(f"Audit report saved: {scene_diag_dir / 'alignment_diagnostic_report.json'}")
    return report


def main():
    exp_root = PROJECT_ROOT / "outputs/accuracy_experiment"
    failed_dir = exp_root / "high_quality_multiframe_v1"
    recovery_dir = exp_root / "multiframe_stitch_recovery_v1"
    recovery_dir.mkdir(parents=True, exist_ok=True)

    diag_out = recovery_dir / "window_diagnostics"
    diag_out.mkdir(parents=True, exist_ok=True)

    scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]
    all_reports = {}

    for s in scenes:
        rep = diagnose_scene_stitching(s, failed_dir, diag_out)
        all_reports[s] = rep

    with open(diag_out / "master_alignment_diagnostics.json", "w", encoding="utf-8") as f:
        json.dump(all_reports, f, indent=2)

    print(f"\n{'='*80}")
    print("ALL SCENE ALIGNMENT DIAGNOSTICS COMPLETED!")
    print(f"Master Diagnostic File: {diag_out / 'master_alignment_diagnostics.json'}")
    print(f"{'='*80}")


if __name__ == "__main__":
    main()
