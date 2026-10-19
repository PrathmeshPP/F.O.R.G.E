"""F.O.R.G.E — Inference Runner for Forest Road Video.

Runs VGGT-Omega forward passes on the extracted frame sets:
1. Baseline frame set (first 5 seconds, 16.4% trajectory coverage)
2. Uniform span frame set (full 30.4 seconds, 99.9% trajectory coverage)
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.vggt.inference import VGGTInferencePipeline

def run_inference_for_dataset(frames_dir: Path, output_dir: Path, pipeline: VGGTInferencePipeline):
    output_dir.mkdir(parents=True, exist_ok=True)
    depth_npz = output_dir / "depth_predictions.npz"
    poses_npz = output_dir / "camera_poses.npz"

    if depth_npz.is_file() and poses_npz.is_file():
        print(f"[SKIP] Predictions already exist in: {output_dir}")
        return

    frame_files = sorted([str(p.resolve()) for p in frames_dir.glob("*.png")])
    print(f"\n[INFERENCE] Running VGGT-Omega on {len(frame_files)} frames from: {frames_dir.name}")
    t0 = time.perf_counter()
    results = pipeline.run_inference(frame_files)
    t_elapsed = time.perf_counter() - t0
    print(f"  Completed forward pass in {t_elapsed:.2f}s")

    saved_files = pipeline.save_predictions(results, str(output_dir))
    print(f"  Saved predictions: {list(saved_files.keys())}")


def main():
    base_dir = PROJECT_ROOT / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "forest_road"
    base_dir.mkdir(parents=True, exist_ok=True)

    print("Initializing VGGT-Omega Pipeline on cuda:0...")
    pipeline = VGGTInferencePipeline(device="cuda:0", image_resolution=512)
    pipeline.initialize()

    # 1. Baseline Frames
    run_inference_for_dataset(
        frames_dir=base_dir / "frames_baseline",
        output_dir=base_dir / "predictions_baseline",
        pipeline=pipeline
    )

    # 2. Uniform Span Frames
    run_inference_for_dataset(
        frames_dir=base_dir / "frames_uniform_span",
        output_dir=base_dir / "predictions_uniform_span",
        pipeline=pipeline
    )

    # 3. Viewpoint Diversity Frames
    run_inference_for_dataset(
        frames_dir=base_dir / "frames_viewpoint_diversity",
        output_dir=base_dir / "predictions_viewpoint_diversity",
        pipeline=pipeline
    )

    print("\n[ALL INFERENCE COMPLETED SUCCESSFULLY]")


if __name__ == "__main__":
    main()
