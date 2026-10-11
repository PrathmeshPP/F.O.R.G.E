"""F.O.R.G.E — Trajectory & Photogrammetric Quality Analyzer.

Lightweight analysis tool to inspect predicted camera poses, translation baselines,
rotation dynamics, depth distributions, and confidence maps. Evaluates whether a video
sequence provides sufficient parallax and geometric conditions for 3D reconstruction.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Any, List

import numpy as np


def compute_rotation_angle(R1: np.ndarray, R2: np.ndarray) -> float:
    """Computes relative rotation angle in degrees between two 3x3 rotation matrices."""
    rel_R = R2 @ R1.T
    trace_val = np.trace(rel_R)
    cos_angle = np.clip((trace_val - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def analyze_trajectory(
    poses_path: str,
    depth_path: str
) -> Dict[str, Any]:
    """Analyzes camera poses, trajectory geometry, parallax, and depth distributions."""
    poses_npz = np.load(poses_path)
    depth_npz = np.load(depth_path)

    extrinsics = poses_npz["extrinsics"]  # (N, 3, 4)
    intrinsics = poses_npz["intrinsics"]  # (N, 3, 3)
    depth = depth_npz["depth"]            # (N, H, W, 1)
    confidence = depth_npz["confidence"]  # (N, H, W, 1)

    num_frames = len(extrinsics)
    if num_frames < 2:
        raise ValueError(f"Need at least 2 frames for trajectory analysis, found {num_frames}")

    # 1. Camera Translations & Baselines
    # Extrinsics [R | T]: camera center in world coordinates is -R^T * T
    camera_centers = np.zeros((num_frames, 3))
    translations = np.zeros((num_frames, 3))
    rotations = np.zeros((num_frames, 3, 3))

    for i in range(num_frames):
        R = extrinsics[i, :, :3]
        T = extrinsics[i, :, 3]
        rotations[i] = R
        translations[i] = T
        camera_centers[i] = -R.T @ T

    # Frame-to-frame step distances in camera center space
    step_distances = [
        float(np.linalg.norm(camera_centers[i] - camera_centers[i - 1]))
        for i in range(1, num_frames)
    ]
    cumulative_path = float(sum(step_distances))
    net_baseline = float(np.linalg.norm(camera_centers[-1] - camera_centers[0]))

    # 2. Rotations
    step_rotations = [
        compute_rotation_angle(rotations[i - 1], rotations[i])
        for i in range(1, num_frames)
    ]
    cumulative_rotation = float(sum(step_rotations))
    net_rotation = compute_rotation_angle(rotations[0], rotations[-1])

    # 3. Viewing Angles & Optical Axis
    # Optical axis in world coordinates: Z_cam = [0, 0, 1]^T -> Z_world = R^T * [0, 0, 1]^T
    view_dirs = np.zeros((num_frames, 3))
    for i in range(num_frames):
        view_dirs[i] = rotations[i].T @ np.array([0.0, 0.0, 1.0])

    # Mean viewing direction
    mean_view_dir = np.mean(view_dirs, axis=0)
    mean_view_dir /= (np.linalg.norm(mean_view_dir) + 1e-8)

    # In typical world conventions, vertical axis is Y or Z.
    # Check tilt relative to nadir vs horizon.
    # In VGGT-Omega coordinate system: Y is down, Z is forward.
    # For nadir aerials: camera looks down along world direction.
    view_z_alignment = float(np.abs(mean_view_dir[2]))
    view_y_alignment = float(np.abs(mean_view_dir[1]))

    # 4. Depth Distribution & Baseline-to-Depth Ratio (B/D)
    flat_depth = depth.flatten()
    valid_depth = flat_depth[np.isfinite(flat_depth) & (flat_depth > 0)]
    depth_min = float(np.min(valid_depth))
    depth_max = float(np.max(valid_depth))
    depth_median = float(np.median(valid_depth))
    depth_mean = float(np.mean(valid_depth))
    depth_std = float(np.std(valid_depth))
    depth_iqr = float(np.percentile(valid_depth, 75) - np.percentile(valid_depth, 25))
    depth_relief_ratio = (depth_max - depth_min) / (depth_median + 1e-6)

    # Baseline-to-Depth ratio (critical photogrammetric metric)
    bd_ratio = net_baseline / (depth_median + 1e-6)

    # Parallax Metric: Translation vs Rotation ratio
    # If translation is tiny while rotation is huge, motion is purely rotational
    trans_rot_ratio = net_baseline / (np.radians(net_rotation) + 1e-6)

    # 5. Confidence Statistics
    flat_conf = confidence.flatten()
    valid_conf = flat_conf[np.isfinite(flat_conf)]
    conf_min = float(np.min(valid_conf))
    conf_max = float(np.max(valid_conf))
    conf_median = float(np.median(valid_conf))
    conf_mean = float(np.mean(valid_conf))
    conf_pct_gt_1 = float(np.mean(valid_conf > 1.0) * 100.0)
    conf_pct_gt_5 = float(np.mean(valid_conf > 5.0) * 100.0)

    # 6. Quality Checklist & Diagnostic Verdicts
    # Standards:
    # - Good stereo parallax: B/D ratio >= 0.10 (Marginal: 0.05 - 0.10, Degenerate: < 0.05)
    # - Parallax type: Net rotation > 30 deg with B/D < 0.05 indicates rotational hovering
    # - Depth relief: Depth IQR / Median > 0.15 indicates 3D depth variations vs flat planar slice
    bd_verdict = (
        "EXCELLENT" if bd_ratio >= 0.20 else
        "ADEQUATE" if bd_ratio >= 0.08 else
        "WEAK" if bd_ratio >= 0.03 else
        "DEGENERATE (Too low parallax)"
    )

    motion_type = (
        "FORWARD / TRANSLATIONAL FLIGHT" if net_baseline > 0.15 and net_rotation < 45 else
        "PURE ROTATIONAL HOVER (Yaw/Spin)" if net_rotation > 40 and net_baseline < 0.08 else
        "MIXED TRAJECTORY"
    )

    suitable_for_3d = (bd_ratio >= 0.08) and (net_baseline > 0.05) and (depth_iqr / depth_median > 0.08)

    return {
        "num_frames": num_frames,
        "net_baseline": net_baseline,
        "cumulative_path": cumulative_path,
        "step_distances_mean": float(np.mean(step_distances)),
        "step_distances_max": float(np.max(step_distances)),
        "net_rotation_deg": net_rotation,
        "cumulative_rotation_deg": cumulative_rotation,
        "step_rotations_mean": float(np.mean(step_rotations)),
        "depth_min": depth_min,
        "depth_max": depth_max,
        "depth_median": depth_median,
        "depth_mean": depth_mean,
        "depth_std": depth_std,
        "depth_iqr": depth_iqr,
        "depth_relief_ratio": depth_relief_ratio,
        "bd_ratio": bd_ratio,
        "bd_verdict": bd_verdict,
        "trans_rot_ratio": trans_rot_ratio,
        "motion_type": motion_type,
        "conf_min": conf_min,
        "conf_max": conf_max,
        "conf_median": conf_median,
        "conf_mean": conf_mean,
        "conf_pct_gt_1": conf_pct_gt_1,
        "conf_pct_gt_5": conf_pct_gt_5,
        "suitable_for_3d": suitable_for_3d,
    }


def print_report(res: Dict[str, Any], title: str) -> None:
    """Prints a structured photogrammetric quality diagnostic report."""
    print("=" * 72)
    print(f"F.O.R.G.E Trajectory & Parallax Quality Report: {title}")
    print("=" * 72)

    print("\n[1. Motion & Baseline Analysis]")
    print(f"  Frames Analyzed:              {res['num_frames']}")
    print(f"  Net Baseline (Origin->End):   {res['net_baseline']:.4f} m (norm units)")
    print(f"  Cumulative Path Length:       {res['cumulative_path']:.4f} m")
    print(f"  Avg Step Translation:         {res['step_distances_mean']:.4f} m (max: {res['step_distances_max']:.4f} m)")
    print(f"  Net Camera Rotation:          {res['net_rotation_deg']:.2f}°")
    print(f"  Cumulative Rotation:          {res['cumulative_rotation_deg']:.2f}°")
    print(f"  Classified Motion Type:       {res['motion_type']}")

    print("\n[2. Depth & Parallax Distribution]")
    print(f"  Depth Range:                  [{res['depth_min']:.3f} m, {res['depth_max']:.3f} m]")
    print(f"  Depth Median / Mean:          {res['depth_median']:.3f} m / {res['depth_mean']:.3f} m")
    print(f"  Depth Std / IQR:              {res['depth_std']:.3f} m / {res['depth_iqr']:.3f} m")
    print(f"  Depth Relief Dynamic Range:   {res['depth_relief_ratio']:.3f}")
    print(f"  Baseline-to-Depth Ratio (B/D): {res['bd_ratio']:.4f}  --> [{res['bd_verdict']}]")
    print(f"  Translation-to-Rotation:      {res['trans_rot_ratio']:.4f} m/rad")

    print("\n[3. Model Confidence Statistics]")
    print(f"  Confidence Range:             [{res['conf_min']:.2f}, {res['conf_max']:.2f}]")
    print(f"  Confidence Median / Mean:     {res['conf_median']:.2f} / {res['conf_mean']:.2f}")
    print(f"  Pixels with Confidence > 1.0: {res['conf_pct_gt_1']:.1f}%")
    print(f"  Pixels with Confidence > 5.0: {res['conf_pct_gt_5']:.1f}%")

    print("\n[4. Photogrammetric Quality Checklist]")
    c1 = "[PASS]" if res['bd_ratio'] >= 0.08 else "[FAIL]"
    c2 = "[PASS]" if res['net_baseline'] >= 0.05 else "[FAIL]"
    c3 = "[PASS]" if res['motion_type'] != "PURE ROTATIONAL HOVER (Yaw/Spin)" else "[FAIL]"
    c4 = "[PASS]" if (res['depth_iqr'] / res['depth_median']) > 0.08 else "[WARN]"
    c5 = "[PASS]" if res['conf_pct_gt_1'] > 80.0 else "[WARN]"

    print(f"  {c1} Baseline-to-Depth Ratio (B/D >= 0.08)")
    print(f"  {c2} Sufficient Translation (Net Baseline >= 0.05)")
    print(f"  {c3} Non-Degenerate Motion (Parallax vs Pure Rotation)")
    print(f"  {c4} Depth Relief / Structural Variation (IQR/Median > 0.08)")
    print(f"  {c5} High Confidence Depth Predictions (>80% valid)")

    verdict_str = "SUITABLE FOR 3D RECONSTRUCTION" if res['suitable_for_3d'] else "UNSUITABLE FOR 3D RECONSTRUCTION"
    print("\n" + "-" * 72)
    print(f"OVERALL VERDICT: {verdict_str}")
    print("-" * 72)


def main():
    parser = argparse.ArgumentParser(description="Analyze trajectory and parallax quality of F.O.R.G.E predictions")
    parser.add_argument(
        "--poses",
        default="outputs/dji_fly_validation/predictions/camera_poses.npz",
        help="Path to camera_poses.npz"
    )
    parser.add_argument(
        "--depth",
        default="outputs/dji_fly_validation/predictions/depth_predictions.npz",
        help="Path to depth_predictions.npz"
    )
    parser.add_argument(
        "--title",
        default="DJI Drone Validation Run",
        help="Display title for report"
    )
    args = parser.parse_args()

    poses_p = Path(args.poses)
    depth_p = Path(args.depth)

    if not poses_p.is_file():
        print(f"[ERROR] Poses file not found: {poses_p}", file=sys.stderr)
        return 1
    if not depth_p.is_file():
        print(f"[ERROR] Depth file not found: {depth_p}", file=sys.stderr)
        return 1

    res = analyze_trajectory(str(poses_p), str(depth_p))
    print_report(res, args.title)
    return 0


if __name__ == "__main__":
    sys.exit(main())
