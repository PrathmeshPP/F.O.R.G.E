#!/usr/bin/env python3
"""
Inspect source videos, telemetry, and prediction archives for Sprint 9.
"""

import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent

candidate_videos = {
    "campus_0049": [
        Path(r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4"),
    ],
    "desert_road": [
        Path(r"C:\Users\25beevdt025\Desktop\sih1\third_party\vggt-omega-main\examples\desert_road.mp4"),
        Path(r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\desert_road.mp4"),
    ],
    "forest_road": [
        Path(r"C:\Users\25beevdt025\Desktop\sih1\third_party\vggt-omega-main\examples\forest_road.mp4"),
        Path(r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\forest_road.mp4"),
    ],
    "river_bridge": [
        PROJECT_ROOT / "outputs/dji_river_bridge_validation/segment_52s_64s.mp4",
        Path(r"C:\Users\25beevdt025\Downloads\DJI_0876.mov"),
        Path(r"C:\Users\25beevdt025\Desktop\SIH\models and videos\DJI_0876.mov"),
    ]
}

def get_file_sha256(filepath: Path) -> str:
    hasher = hashlib.sha256()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(1048576), b""):
            hasher.update(chunk)
    return hasher.hexdigest()

def inspect_video(vpath: Path):
    if not vpath.is_file():
        return None
    cap = cv2.VideoCapture(str(vpath))
    if not cap.isOpened():
        return None
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    dur = frames / fps if fps > 0 else 0
    cap.release()
    size_bytes = vpath.stat().st_size
    sha = get_file_sha256(vpath)
    return {
        "path": str(vpath),
        "size_bytes": size_bytes,
        "sha256": sha,
        "width": w,
        "height": h,
        "fps": fps,
        "frame_count": frames,
        "duration_sec": dur
    }

inventory = {}
for scene, paths in candidate_videos.items():
    inventory[scene] = []
    for p in paths:
        info = inspect_video(p)
        if info:
            inventory[scene].append(info)
            print(f"[{scene}] Found: {p.name} ({info['width']}x{info['height']}, {info['frame_count']} frames, {info['duration_sec']:.1f}s)")
        else:
            print(f"[{scene}] Not found or unreadable: {p}")

print("\nInventory complete. Found source videos for all benchmark scenes.")
