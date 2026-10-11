"""
F.O.R.G.E — Frame Extraction Module
Extracts ordered RGB frames from video inputs with configurable FPS sampling and frame caps.
"""

import os
import argparse
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Optional
import cv2
from PIL import Image


@dataclass
class ExtractedFrame:
    frame_index: int
    timestamp_seconds: float
    file_path: str
    width: int
    height: int


def extract_frames(
    video_path: str,
    output_dir: str,
    target_fps: Optional[float] = 1.0,
    max_frames: Optional[int] = 10,
    prefix: str = "frame"
) -> List[ExtractedFrame]:
    """
    Extracts ordered RGB frames from video.

    Args:
        video_path: Path to the input video file (.mp4, .mov, etc.).
        output_dir: Directory where extracted frame images will be stored.
        target_fps: Desired sampling rate in frames per second. If None or <= 0, samples all frames up to max_frames.
        max_frames: Maximum number of frames to extract. If None, extracts all sampled frames.
        prefix: Filename prefix for saved frames.

    Returns:
        List of ExtractedFrame objects containing metadata and file paths.
    """
    video_file = Path(video_path)
    if not video_file.is_file():
        raise FileNotFoundError(f"Video file not found: {video_path}")

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_file))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video file via OpenCV: {video_path}")

    native_fps = cap.get(cv2.CAP_PROP_FPS)
    total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    native_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    native_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration_seconds = total_video_frames / native_fps if native_fps > 0 else 0.0

    if native_fps <= 0:
        native_fps = 30.0

    sample_interval_sec = 1.0 / target_fps if (target_fps is not None and target_fps > 0) else (1.0 / native_fps)

    extracted: List[ExtractedFrame] = []
    current_sample_target_time = 0.0
    frame_counter = 0

    while True:
        ret, frame_bgr = cap.read()
        if not ret:
            break

        current_timestamp = frame_counter / native_fps

        if current_timestamp >= (current_sample_target_time - 1e-4) or target_fps is None:
            frame_idx_str = f"{len(extracted):04d}"
            time_str = f"{current_timestamp:.3f}s"
            filename = f"{prefix}_{frame_idx_str}_{time_str}.png"
            save_path = out_dir / filename

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            Image.fromarray(frame_rgb).save(save_path)

            extracted.append(
                ExtractedFrame(
                    frame_index=len(extracted),
                    timestamp_seconds=round(current_timestamp, 4),
                    file_path=str(save_path.resolve()),
                    width=frame_rgb.shape[1],
                    height=frame_rgb.shape[0]
                )
            )

            current_sample_target_time += sample_interval_sec

            if max_frames is not None and len(extracted) >= max_frames:
                break

        frame_counter += 1

    cap.release()

    if not extracted:
        raise RuntimeError(f"No frames were extracted from video: {video_path}")

    return extracted


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract frames from video")
    parser.add_argument("--video", required=True, help="Path to input video")
    parser.add_argument("--output-dir", required=True, help="Directory to save extracted frames")
    parser.add_argument("--fps", type=float, default=1.0, help="Target FPS to sample (default: 1.0)")
    parser.add_argument("--max-frames", type=int, default=10, help="Maximum frames to extract (default: 10)")
    args = parser.parse_args()

    frames = extract_frames(args.video, args.output_dir, target_fps=args.fps, max_frames=args.max_frames)
    print(f"Successfully extracted {len(frames)} frames into {args.output_dir}")
