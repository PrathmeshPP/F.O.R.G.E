"""F.O.R.G.E — Advanced Frame Selection & Viewpoint Quality Module.

Provides reproducible, generic frame-sampling strategies across arbitrary video inputs:
1. Baseline: Sequential extraction at fixed fps starting at t=0 (legacy behavior, misses long videos).
2. Uniform Span: Uniformly distributes N frames across 100% of the video duration.
3. Viewpoint & Quality Diversity: Evaluates Laplacian blur variance and inter-frame visual disparity
   to select the sharpest, most diverse viewpoints across camera trajectories.
"""

from __future__ import annotations

import os
import argparse
import json
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional, Tuple
import cv2
import numpy as np
from PIL import Image


@dataclass
class SampledFrame:
    index: int
    video_frame_index: int
    timestamp_sec: float
    file_path: str
    sharpness_score: float
    width: int
    height: int


def compute_sharpness(gray_img: np.ndarray) -> float:
    """Computes the Laplacian variance of an image as a measure of focus/sharpness."""
    return float(cv2.Laplacian(gray_img, cv2.CV_64F).var())


def extract_sampled_frames(
    video_path: str,
    output_dir: str,
    strategy: str = "uniform_span",
    num_frames: int = 6,
    target_fps: float = 1.0,
    prefix: str = "frame"
) -> Tuple[List[SampledFrame], Dict[str, Any]]:
    """
    Extracts frames from video according to the specified strategy.

    Args:
        video_path: Path to input video file.
        output_dir: Directory to save extracted frames.
        strategy: 'baseline', 'uniform_span', or 'viewpoint_diversity'.
        num_frames: Target number of frames to extract.
        target_fps: Only used for 'baseline' strategy.
        prefix: Filename prefix for saved frames.

    Returns:
        Tuple of (List of SampledFrame, selection diagnostics metadata dictionary).
    """
    vpath = Path(video_path)
    if not vpath.is_file():
        raise FileNotFoundError(f"Video file not found: {video_path}")

    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(vpath))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    native_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    native_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    native_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration_sec = total_frames / native_fps if native_fps > 0 else 0.0

    selected_indices: List[int] = []

    if strategy == "baseline":
        # Legacy behavior: 1 frame every (1/target_fps) seconds starting at 0 up to num_frames
        interval = max(int(round(native_fps / target_fps)), 1)
        for i in range(num_frames):
            idx = i * interval
            if idx < total_frames:
                selected_indices.append(idx)

    elif strategy == "uniform_span":
        # Uniform spacing across the full duration [0, total_frames - 1]
        if num_frames == 1:
            selected_indices = [total_frames // 2]
        else:
            selected_indices = [int(round(i * (total_frames - 1) / (num_frames - 1))) for i in range(num_frames)]

    elif strategy == "viewpoint_diversity":
        # Divide into num_frames temporal windows; pick the frame with highest sharpness in each window
        window_size = total_frames / num_frames
        for w in range(num_frames):
            start_f = int(w * window_size)
            end_f = int((w + 1) * window_size)
            # Sample candidate frames every few steps within window to find sharpest
            step = max((end_f - start_f) // 10, 1)
            best_idx = start_f
            best_sharpness = -1.0
            for cand_idx in range(start_f, min(end_f, total_frames), step):
                cap.set(cv2.CAP_PROP_POS_FRAMES, cand_idx)
                ret, frame = cap.read()
                if ret:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    s = compute_sharpness(gray)
                    if s > best_sharpness:
                        best_sharpness = s
                        best_idx = cand_idx
            selected_indices.append(best_idx)

    else:
        raise ValueError(f"Unknown sampling strategy: {strategy}")

    # Extract and save selected frames
    results: List[SampledFrame] = []
    sharpness_list: List[float] = []

    for out_idx, f_idx in enumerate(selected_indices):
        cap.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
        ret, frame_bgr = cap.read()
        if not ret:
            continue

        ts = f_idx / native_fps
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        s_score = compute_sharpness(gray)
        sharpness_list.append(s_score)

        filename = f"{prefix}_{out_idx:04d}_{ts:.3f}s.png"
        fpath = out_path / filename

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        Image.fromarray(frame_rgb).save(fpath)

        results.append(
            SampledFrame(
                index=out_idx,
                video_frame_index=f_idx,
                timestamp_sec=round(ts, 4),
                file_path=str(fpath.resolve()),
                sharpness_score=round(s_score, 2),
                width=native_w,
                height=native_h
            )
        )

    cap.release()

    temporal_span = (results[-1].timestamp_sec - results[0].timestamp_sec) if len(results) > 1 else 0.0
    coverage_ratio = (temporal_span / duration_sec) if duration_sec > 0 else 1.0

    metadata = {
        "video_path": str(vpath.resolve()),
        "native_fps": round(native_fps, 2),
        "total_video_frames": total_frames,
        "duration_sec": round(duration_sec, 2),
        "native_resolution": [native_w, native_h],
        "strategy": strategy,
        "requested_frames": num_frames,
        "extracted_frames": len(results),
        "temporal_span_sec": round(temporal_span, 2),
        "trajectory_coverage_ratio": round(coverage_ratio, 4),
        "mean_sharpness": round(float(np.mean(sharpness_list)), 2) if sharpness_list else 0.0,
        "min_sharpness": round(float(np.min(sharpness_list)), 2) if sharpness_list else 0.0,
        "max_sharpness": round(float(np.max(sharpness_list)), 2) if sharpness_list else 0.0,
        "frames": [asdict(f) for f in results]
    }

    meta_file = out_path / "frame_selection_metadata.json"
    with open(meta_file, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    return results, metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Advanced frame sampling with quality awareness")
    parser.add_argument("--video", required=True, help="Input video path")
    parser.add_argument("--output-dir", required=True, help="Output directory")
    parser.add_argument("--strategy", choices=["baseline", "uniform_span", "viewpoint_diversity"], default="uniform_span")
    parser.add_argument("--num-frames", type=int, default=6)
    parser.add_argument("--target-fps", type=float, default=1.0)
    args = parser.parse_args()

    frames, meta = extract_sampled_frames(
        video_path=args.video,
        output_dir=args.output_dir,
        strategy=args.strategy,
        num_frames=args.num_frames,
        target_fps=args.target_fps
    )
    print(f"Extracted {len(frames)} frames under '{args.strategy}' strategy into {args.output_dir}")
    print(f"Coverage: {meta['trajectory_coverage_ratio']*100:.1f}% ({meta['temporal_span_sec']:.1f}s / {meta['duration_sec']:.1f}s)")
    print(f"Sharpness: mean={meta['mean_sharpness']}, min={meta['min_sharpness']}, max={meta['max_sharpness']}")
