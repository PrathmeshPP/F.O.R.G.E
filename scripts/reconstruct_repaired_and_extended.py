"""F.O.R.G.E — Reconstruction Repair and Extended Dataset Pipeline.

Executes:
1. river_bridge_flight_corridor: High-parallax 84.8m translational flight corridor (8 frames, t=20s-34s).
2. river_bridge_v2_repaired: Geometry-repaired hover baseline (clamped depth, speck filtering, hole reduction).
3. campus_0049_v2_repaired: Perimeter-cleaned courtyard baseline (cluster filtered, edge manifold cleanup).
4. lake_speedboat: Newly reconstructed third-party video baseline (5 frames).
5. snow_lift: Newly reconstructed third-party video baseline (5 frames).
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.pipeline import GeneralReconstructionPipeline, PipelineConfig


def main():
    targets = [
        {
            "name": "river_bridge_flight_corridor",
            "depth_npz": str(PROJECT_ROOT / "outputs/accuracy_experiment/river_bridge_flight_corridor/predictions/depth_predictions.npz"),
            "poses_npz": str(PROJECT_ROOT / "outputs/accuracy_experiment/river_bridge_flight_corridor/predictions/camera_poses.npz"),
            "frames_dir": str(PROJECT_ROOT / "outputs/accuracy_experiment/river_bridge_flight_corridor/frames"),
            "output_dir": str(PROJECT_ROOT / "outputs/accuracy_experiment/river_bridge_flight_corridor/reconstruction"),
            "config": PipelineConfig(
                min_depth=0.1,
                max_depth=None,
                depth_percentile_cutoff=98.0,
                min_confidence=0.1,
                adaptive_confidence=True,
                target_voxels_per_extent=500,
                density_quantile=0.05,
                min_cluster_faces=100
            )
        },
        {
            "name": "river_bridge_v2_repaired",
            "depth_npz": str(PROJECT_ROOT / "outputs/dji_river_bridge_validation/predictions/depth_predictions.npz"),
            "poses_npz": str(PROJECT_ROOT / "outputs/dji_river_bridge_validation/predictions/camera_poses.npz"),
            "frames_dir": str(PROJECT_ROOT / "outputs/dji_river_bridge_validation/frames"),
            "output_dir": str(PROJECT_ROOT / "outputs/accuracy_experiment/generalization_disaster_validation_v1/river_bridge/candidate_v2_repaired"),
            "config": PipelineConfig(
                min_depth=0.1,
                max_depth=3.2,
                depth_percentile_cutoff=95.0,
                min_confidence=0.1,
                adaptive_confidence=True,
                target_voxels_per_extent=400,
                density_quantile=0.08,
                min_cluster_faces=200
            )
        },
        {
            "name": "campus_0049_v2_repaired",
            "depth_npz": str(PROJECT_ROOT / "outputs/dji_0049_validation/predictions/depth_predictions.npz"),
            "poses_npz": str(PROJECT_ROOT / "outputs/dji_0049_validation/predictions/camera_poses.npz"),
            "frames_dir": str(PROJECT_ROOT / "outputs/dji_0049_validation/frames"),
            "output_dir": str(PROJECT_ROOT / "outputs/accuracy_experiment/generalization_disaster_validation_v1/campus_0049/candidate_v2_repaired"),
            "config": PipelineConfig(
                min_depth=0.1,
                max_depth=None,
                depth_percentile_cutoff=99.0,
                min_confidence=0.1,
                adaptive_confidence=True,
                target_voxels_per_extent=450,
                density_quantile=0.04,
                min_cluster_faces=200
            )
        },
        {
            "name": "lake_speedboat",
            "depth_npz": str(PROJECT_ROOT / "outputs/lake_speedboat_validation/predictions/depth_predictions.npz"),
            "poses_npz": str(PROJECT_ROOT / "outputs/lake_speedboat_validation/predictions/camera_poses.npz"),
            "frames_dir": str(PROJECT_ROOT / "outputs/lake_speedboat_validation/frames"),
            "output_dir": str(PROJECT_ROOT / "outputs/lake_speedboat_validation/reconstruction"),
            "config": PipelineConfig(
                min_depth=0.1,
                max_depth=None,
                depth_percentile_cutoff=98.0,
                min_confidence=0.1,
                adaptive_confidence=True,
                target_voxels_per_extent=400,
                density_quantile=0.04,
                min_cluster_faces=50
            )
        },
        {
            "name": "snow_lift",
            "depth_npz": str(PROJECT_ROOT / "outputs/snow_lift_validation/predictions/depth_predictions.npz"),
            "poses_npz": str(PROJECT_ROOT / "outputs/snow_lift_validation/predictions/camera_poses.npz"),
            "frames_dir": str(PROJECT_ROOT / "outputs/snow_lift_validation/frames"),
            "output_dir": str(PROJECT_ROOT / "outputs/snow_lift_validation/reconstruction"),
            "config": PipelineConfig(
                min_depth=0.1,
                max_depth=None,
                depth_percentile_cutoff=98.0,
                min_confidence=0.1,
                adaptive_confidence=True,
                target_voxels_per_extent=400,
                density_quantile=0.04,
                min_cluster_faces=50
            )
        }
    ]

    for t in targets:
        print("=" * 80)
        print(f"RECONSTRUCTING: {t['name']}")
        print("=" * 80)
        out_p = Path(t["output_dir"])
        out_p.mkdir(parents=True, exist_ok=True)
        pipeline = GeneralReconstructionPipeline(t["config"])
        t0 = time.perf_counter()
        res = pipeline.run(
            depth_npz_path=t["depth_npz"],
            poses_npz_path=t["poses_npz"],
            frames_dir=t["frames_dir"],
            output_dir=t["output_dir"],
            scene_classification=t["name"]
        )
        t_el = time.perf_counter() - t0
        print(f"Done in {t_el:.2f}s | Verts: {res['surface_mesh']['vertex_count']:,} | Tris: {res['surface_mesh']['triangle_count']:,}")

    print("\n[ALL RECONSTRUCTIONS COMPLETED SUCCESSFULLY]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
