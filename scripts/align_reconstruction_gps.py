"""F.O.R.G.E — Dataset-Wide Robust GPS-Aligned 3D Reconstruction Batch Runner.

Executes Umeyama–RANSAC 7-DoF similarity alignment between reconstructed camera trajectories
and real drone onboard GPS telemetry across all eligible videos in the F.O.R.G.E dataset:
1. campus_0049: DJI Air 3S Architectural Courtyard flight (Gandhinagar / Ahmedabad)
2. river_bridge: DJI Air 3S Bridge Span flight segment (Gandhinagar / Ahmedabad)
3. desert_road: Terrestrial forward highway corridor benchmark
4. forest_road: Dense canopy forest road drone flight benchmark
5. lake_speedboat & snow_lift: Third-party example candidates (unprocessed/missing)

Exports metric-scaled, georeferenced 3D models per eligible video:
- surface_mesh_gps_aligned.ply
- surface_mesh_gps_aligned.obj
- point_cloud_gps_aligned.ply
- camera_poses_gps_aligned.npz
- gps_alignment_metadata.json
- alignment_comparison_sheet.png
- Orthographic aligned preview images (top, frontal, oblique)

Dataset-level summary outputs:
- outputs/accuracy_experiment/gps_aligned_v1/batch_alignment_report.json
- outputs/accuracy_experiment/gps_aligned_v1/batch_alignment_summary.csv

Preserves all existing candidate outputs and baseline models completely untouched.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.geo.batch_alignment import (
    VideoAlignmentConfig,
    execute_batch_alignment,
    process_single_video_alignment
)


def discover_forge_dataset(project_root: Path) -> List[VideoAlignmentConfig]:
    """
    Discovers and constructs the authoritative list of all videos in the F.O.R.G.E project.
    Inspects actual workspace directories, input files, and previous reconstruction results.
    """
    configs: List[VideoAlignmentConfig] = []

    # -------------------------------------------------------------------------
    # 1. Campus 0049 (DJI Air 3S Architectural Courtyard - Baseline)
    # -------------------------------------------------------------------------
    campus_video = Path(r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4")
    campus_mesh = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "surface_mesh.ply"
    campus_pcd = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "point_cloud_fused.ply"
    campus_poses = project_root / "outputs" / "dji_0049_validation" / "predictions" / "camera_poses.npz"

    configs.append(
        VideoAlignmentConfig(
            video_id="campus_0049",
            display_name="Campus 0049 (Architectural Courtyard - Baseline)",
            video_path=str(campus_video) if campus_video.is_file() else "dji_0049_video.mp4",
            camera_poses_npz_path=str(campus_poses),
            mesh_ply_path=str(campus_mesh),
            point_cloud_ply_path=str(campus_pcd),
            telemetry_source_path=str(campus_video) if campus_video.is_file() else None,
            video_time_offset_sec=0.0,
            target_frame_timestamps_sec=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
            inlier_threshold_m=2.5,
            min_translation_baseline_m=1.5
        )
    )

    # -------------------------------------------------------------------------
    # 2. Campus 0049 (Architectural Courtyard - Repaired V2 Candidate)
    # -------------------------------------------------------------------------
    campus_mesh_rep = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "candidate_v2_repaired" / "surface_mesh.ply"
    campus_pcd_rep = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "candidate_v2_repaired" / "point_cloud_fused.ply"
    if campus_mesh_rep.is_file():
        configs.append(
            VideoAlignmentConfig(
                video_id="campus_0049_repaired",
                display_name="Campus 0049 (Architectural Courtyard - Repaired V2)",
                video_path=str(campus_video) if campus_video.is_file() else "dji_0049_video.mp4",
                camera_poses_npz_path=str(campus_poses),
                mesh_ply_path=str(campus_mesh_rep),
                point_cloud_ply_path=str(campus_pcd_rep),
                telemetry_source_path=str(campus_video) if campus_video.is_file() else None,
                video_time_offset_sec=0.0,
                target_frame_timestamps_sec=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
                inlier_threshold_m=2.5,
                min_translation_baseline_m=1.5
            )
        )

    # -------------------------------------------------------------------------
    # 3. River Bridge Flight Corridor (84.8m Active Forward Flight - Solved)
    # -------------------------------------------------------------------------
    bridge_source_video = Path(r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4")
    bridge_fc_mesh = project_root / "outputs" / "accuracy_experiment" / "river_bridge_flight_corridor" / "reconstruction" / "surface_mesh.ply"
    bridge_fc_pcd = project_root / "outputs" / "accuracy_experiment" / "river_bridge_flight_corridor" / "reconstruction" / "point_cloud_fused.ply"
    bridge_fc_poses = project_root / "outputs" / "accuracy_experiment" / "river_bridge_flight_corridor" / "predictions" / "camera_poses.npz"

    if bridge_fc_mesh.is_file():
        configs.append(
            VideoAlignmentConfig(
                video_id="river_bridge_flight_corridor",
                display_name="River Bridge (84.8m Flight Corridor - Solved)",
                video_path=str(bridge_source_video),
                camera_poses_npz_path=str(bridge_fc_poses),
                mesh_ply_path=str(bridge_fc_mesh),
                point_cloud_ply_path=str(bridge_fc_pcd),
                telemetry_source_path=str(bridge_source_video) if bridge_source_video.is_file() else None,
                video_time_offset_sec=20.0,  # Active forward flight from 20s to 34s
                target_frame_timestamps_sec=[0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0],
                inlier_threshold_m=2.5,
                min_translation_baseline_m=1.5
            )
        )

    # -------------------------------------------------------------------------
    # 4. River Bridge Hover Segment (52s-64s Stationary Hover - Baseline)
    # -------------------------------------------------------------------------
    bridge_segment_video = project_root / "outputs" / "dji_river_bridge_validation" / "segment_52s_64s.mp4"
    bridge_mesh = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "river_bridge" / "surface_mesh.ply"
    bridge_pcd = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "river_bridge" / "point_cloud_fused.ply"
    bridge_poses = project_root / "outputs" / "dji_river_bridge_validation" / "predictions" / "camera_poses.npz"

    configs.append(
        VideoAlignmentConfig(
            video_id="river_bridge",
            display_name="River Bridge (52s-64s Hover Baseline - Unreliable)",
            video_path=str(bridge_segment_video) if bridge_segment_video.is_file() else str(bridge_source_video),
            camera_poses_npz_path=str(bridge_poses),
            mesh_ply_path=str(bridge_mesh),
            point_cloud_ply_path=str(bridge_pcd),
            telemetry_source_path=str(bridge_source_video) if bridge_source_video.is_file() else None,
            video_time_offset_sec=52.0,  # Sliced from 52s-64s in the 82.8s flight 0050
            target_frame_timestamps_sec=[0.0, 2.002, 4.004, 6.006, 8.008, 10.010],
            inlier_threshold_m=2.5,
            min_translation_baseline_m=1.5
        )
    )

    # -------------------------------------------------------------------------
    # 5. River Bridge Hover Segment (52s-64s Stationary Hover - Repaired V2)
    # -------------------------------------------------------------------------
    bridge_mesh_rep = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "river_bridge" / "candidate_v2_repaired" / "surface_mesh.ply"
    bridge_pcd_rep = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "river_bridge" / "candidate_v2_repaired" / "point_cloud_fused.ply"
    if bridge_mesh_rep.is_file():
        configs.append(
            VideoAlignmentConfig(
                video_id="river_bridge_hover_repaired",
                display_name="River Bridge (52s-64s Hover Repaired - Unreliable)",
                video_path=str(bridge_segment_video) if bridge_segment_video.is_file() else str(bridge_source_video),
                camera_poses_npz_path=str(bridge_poses),
                mesh_ply_path=str(bridge_mesh_rep),
                point_cloud_ply_path=str(bridge_pcd_rep),
                telemetry_source_path=str(bridge_source_video) if bridge_source_video.is_file() else None,
                video_time_offset_sec=52.0,
                target_frame_timestamps_sec=[0.0, 2.002, 4.004, 6.006, 8.008, 10.010],
                inlier_threshold_m=2.5,
                min_translation_baseline_m=1.5
            )
        )

    # -------------------------------------------------------------------------
    # 6. Desert Road (Terrestrial Highway Corridor Benchmark)
    # -------------------------------------------------------------------------
    desert_video = Path(r"C:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega\examples\desert_road.mp4")
    desert_mesh = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "desert_road" / "surface_mesh.ply"
    desert_pcd = project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "desert_road" / "point_cloud_fused.ply"
    desert_poses = project_root / "outputs" / "desert_road_validation" / "predictions" / "camera_poses.npz"

    configs.append(
        VideoAlignmentConfig(
            video_id="desert_road",
            display_name="Desert Road (Highway Corridor Benchmark)",
            video_path=str(desert_video),
            camera_poses_npz_path=str(desert_poses),
            mesh_ply_path=str(desert_mesh),
            point_cloud_ply_path=str(desert_pcd),
            telemetry_source_path=None,  # No GPS in terrestrial dashcam video
            video_time_offset_sec=0.0,
            target_frame_timestamps_sec=[0.0, 1.001, 2.002],
            inlier_threshold_m=2.5
        )
    )

    # -------------------------------------------------------------------------
    # 7. Forest Road (Dense Canopy Drone Corridor)
    # -------------------------------------------------------------------------
    forest_video = Path(r"C:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega\examples\forest_road.mp4")
    forest_mesh = (
        project_root
        / "outputs"
        / "accuracy_experiment"
        / "generalization_disaster_validation_v1"
        / "forest_road"
        / "candidate_uniform_span"
        / "surface_mesh.ply"
    )
    forest_pcd = (
        project_root
        / "outputs"
        / "accuracy_experiment"
        / "generalization_disaster_validation_v1"
        / "forest_road"
        / "candidate_uniform_span"
        / "point_cloud_fused.ply"
    )
    forest_poses = (
        project_root
        / "outputs"
        / "accuracy_experiment"
        / "generalization_disaster_validation_v1"
        / "forest_road"
        / "predictions_uniform_span"
        / "camera_poses.npz"
    )

    configs.append(
        VideoAlignmentConfig(
            video_id="forest_road",
            display_name="Forest Road (Dense Canopy Drone Corridor)",
            video_path=str(forest_video),
            camera_poses_npz_path=str(forest_poses),
            mesh_ply_path=str(forest_mesh),
            point_cloud_ply_path=str(forest_pcd),
            telemetry_source_path=None,  # Public benchmark video without GPS tracks
            video_time_offset_sec=0.0,
            target_frame_timestamps_sec=None,
            inlier_threshold_m=2.5
        )
    )

    # -------------------------------------------------------------------------
    # 8. Lake Speedboat (Third-Party Benchmark - Fully Reconstructed)
    # -------------------------------------------------------------------------
    lake_video = Path(r"C:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega\examples\lake_speedboat.mp4")
    lake_mesh = project_root / "outputs" / "lake_speedboat_validation" / "reconstruction" / "surface_mesh.ply"
    lake_pcd = project_root / "outputs" / "lake_speedboat_validation" / "reconstruction" / "point_cloud_fused.ply"
    lake_poses = project_root / "outputs" / "lake_speedboat_validation" / "predictions" / "camera_poses.npz"

    if lake_video.is_file():
        configs.append(
            VideoAlignmentConfig(
                video_id="lake_speedboat",
                display_name="Lake Speedboat (Third-Party Benchmark)",
                video_path=str(lake_video),
                camera_poses_npz_path=str(lake_poses),
                mesh_ply_path=str(lake_mesh),
                point_cloud_ply_path=str(lake_pcd),
                telemetry_source_path=None  # Public benchmark without GPS telemetry
            )
        )

    # -------------------------------------------------------------------------
    # 9. Snow Lift (Third-Party Benchmark - Fully Reconstructed)
    # -------------------------------------------------------------------------
    snow_video = Path(r"C:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega\examples\snow_lift.mp4")
    snow_mesh = project_root / "outputs" / "snow_lift_validation" / "reconstruction" / "surface_mesh.ply"
    snow_pcd = project_root / "outputs" / "snow_lift_validation" / "reconstruction" / "point_cloud_fused.ply"
    snow_poses = project_root / "outputs" / "snow_lift_validation" / "predictions" / "camera_poses.npz"

    if snow_video.is_file():
        configs.append(
            VideoAlignmentConfig(
                video_id="snow_lift",
                display_name="Snow Lift (Third-Party Benchmark)",
                video_path=str(snow_video),
                camera_poses_npz_path=str(snow_poses),
                mesh_ply_path=str(snow_mesh),
                point_cloud_ply_path=str(snow_pcd),
                telemetry_source_path=None  # Public benchmark without GPS telemetry
            )
        )

    return configs


def print_audit_table(configs: List[VideoAlignmentConfig]):
    """Prints a structured ASCII audit table of all discovered videos in the project."""
    print("\n" + "=" * 120)
    print("F.O.R.G.E DATASET AUDIT & PRELIMINARY PROCESSING STATUS")
    print("=" * 120)
    header = f"{'Video ID':<30} | {'Video File':<12} | {'Poses':<8} | {'Mesh':<8} | {'Telemetry Source':<20} | {'Preliminary Status'}"
    print(header)
    print("-" * 120)

    for cfg in configs:
        v_exists = "YES" if Path(cfg.video_path).is_file() else "NO"
        p_exists = "YES" if cfg.camera_poses_npz_path and Path(cfg.camera_poses_npz_path).is_file() else "NO"
        m_exists = "YES" if cfg.mesh_ply_path and Path(cfg.mesh_ply_path).is_file() else "NO"
        tel_src = Path(cfg.telemetry_source_path).name if cfg.telemetry_source_path and Path(cfg.telemetry_source_path).is_file() else ("None (Dashcam/Public)" if not cfg.telemetry_source_path else "Not Found")

        if p_exists == "NO" or m_exists == "NO":
            prelim_status = "Unprocessed / Missing Reconstruction"
        elif not cfg.telemetry_source_path:
            prelim_status = "Eligible Geometry, Missing Telemetry"
        else:
            prelim_status = "Eligible for GPS Alignment"

        print(f"{cfg.video_id:<30} | {v_exists:<12} | {p_exists:<8} | {m_exists:<8} | {tel_src:<20} | {prelim_status}")

    print("=" * 120 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="F.O.R.G.E Dataset-Wide Robust GPS-Aligned 3D Reconstruction Batch Runner"
    )
    parser.add_argument(
        "--video", "-v",
        type=str,
        default=None,
        help="Specific video ID to align (e.g. 'campus_0049', 'river_bridge', 'desert_road', 'forest_road'). Default is all."
    )
    parser.add_argument(
        "--all", "-a",
        action="store_true",
        help="Process all eligible videos across the dataset (default behavior if --video is omitted)."
    )
    parser.add_argument(
        "--force", "-f",
        action="store_true",
        help="Force re-execution even if valid cached GPS-aligned models already exist."
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=str(PROJECT_ROOT / "outputs" / "accuracy_experiment" / "gps_aligned_v1"),
        help="Base directory for aligned outputs (default: outputs/accuracy_experiment/gps_aligned_v1)."
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="List discovered dataset videos and their preliminary status without executing alignment."
    )

    args = parser.parse_args()
    output_base_dir = Path(args.output_dir)

    # 1. Audit and discover dataset
    dataset_configs = discover_forge_dataset(PROJECT_ROOT)

    # Print the preliminary audit table
    print_audit_table(dataset_configs)

    if args.list:
        return 0

    # Filter target configs
    if args.video:
        target_configs = [c for c in dataset_configs if c.video_id == args.video]
        if not target_configs:
            print(f"[ERROR] Video ID '{args.video}' not found in dataset. Available IDs: {[c.video_id for c in dataset_configs]}", file=sys.stderr)
            return 1
    else:
        target_configs = dataset_configs

    # Execute batch workflow
    report = execute_batch_alignment(target_configs, output_base_dir, force_rerun=args.force)

    # Print summary results table
    print("\n" + "=" * 120)
    print("DATASET-WIDE BATCH ALIGNMENT EXECUTION SUMMARY")
    print("=" * 120)
    print(f"{'Video ID':<16} | {'Status':<26} | {'Scale (m/u)':<11} | {'Inliers':<9} | {'RMSE (m)':<9} | {'Extents (E x N x U) m':<24} | {'Notes / Verdict'}")
    print("-" * 120)

    for r in report["results"]:
        vid = r["video_id"]
        st = r["alignment_status"]
        sc = f"{r['scale_factor_meters_per_unit']:.3f}" if r.get("scale_factor_meters_per_unit") else "N/A"
        inl = f"{r.get('ransac_inliers', 0)}/{r.get('total_correspondences', 0)}" if r.get("total_correspondences") else "N/A"
        rmse = f"{r['residual_rmse_m']:.3f}" if r.get("residual_rmse_m") else "N/A"
        ext = f"{r['aligned_extents_m'][0]}x{r['aligned_extents_m'][1]}x{r['aligned_extents_m'][2]}" if r.get("aligned_extents_m") else "N/A"
        note = r.get("failure_reason") or r.get("conditioning_verdict", "N/A")
        print(f"{vid:<16} | {st:<26} | {sc:<11} | {inl:<9} | {rmse:<9} | {ext:<24} | {note}")

    print("=" * 120)
    print(f"\nBatch processing finished successfully.")
    print(f"Report JSON: {output_base_dir / 'batch_alignment_report.json'}")
    print(f"Summary CSV: {output_base_dir / 'batch_alignment_summary.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
