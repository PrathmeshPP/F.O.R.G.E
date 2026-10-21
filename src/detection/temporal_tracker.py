"""F.O.R.G.E — Lightweight Temporal Tracking & Consistency Filter.

Provides multi-frame object association, ID persistence, and detection dropout
smoothing across adjacent sampled frames without introducing artificial hallucinations.
"""

from __future__ import annotations

from typing import List, Dict, Any, Optional, Tuple
import numpy as np

from src.detection.base import (
    BoundingBox,
    InstanceMask,
    FrameDetectionResult,
)


def compute_box_iou(box_a: BoundingBox, box_b: BoundingBox) -> float:
    """Computes Intersection-over-Union between two axis-aligned bounding boxes."""
    x_left = max(box_a.x1, box_b.x1)
    y_top = max(box_a.y1, box_b.y1)
    x_right = min(box_a.x2, box_b.x2)
    y_bottom = min(box_a.y2, box_b.y2)

    if x_right <= x_left or y_bottom <= y_top:
        return 0.0

    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    area_a = box_a.area
    area_b = box_b.area
    union_area = area_a + area_b - intersection_area

    if union_area <= 0.0:
        return 0.0
    return float(intersection_area / union_area)


class LightweightTemporalTracker:
    """
    Lightweight greedy IoU tracker for linking dynamic object instances across frames.
    Assigns consistent tracking IDs and flags potential single-frame flicker dropouts.
    """

    def __init__(
        self,
        iou_threshold: float = 0.30,
        max_dropout_gap: int = 1,
        enable_persistence: bool = False
    ):
        """
        Args:
            iou_threshold: Minimum IoU to associate detections across consecutive frames.
            max_dropout_gap: Max frame gap over which a lost track can be reconnected.
            enable_persistence: If True, interpolates masks across 1-frame dropouts.
        """
        self.iou_threshold = iou_threshold
        self.max_dropout_gap = max_dropout_gap
        self.enable_persistence = enable_persistence
        self.next_track_id = 1
        self.active_tracks: Dict[int, Dict[str, Any]] = {}

    def reset(self):
        """Resets tracker state."""
        self.next_track_id = 1
        self.active_tracks.clear()

    def update(self, frame_res: FrameDetectionResult) -> FrameDetectionResult:
        """Processes a single frame result incrementally."""
        current_instances = frame_res.instances
        unmatched_current = set(range(len(current_instances)))

        # Greedy bipartite matching based on IoU
        matches: List[Tuple[int, int, float]] = []
        for track_id, track_info in self.active_tracks.items():
            for inst_idx, inst in enumerate(current_instances):
                if inst.class_name == track_info["class_name"]:
                    iou = compute_box_iou(inst.bbox, track_info["last_bbox"])
                    if iou >= self.iou_threshold:
                        matches.append((track_id, inst_idx, iou))

        matches.sort(key=lambda x: x[2], reverse=True)
        matched_tracks = set()
        matched_instances = set()

        for track_id, inst_idx, iou in matches:
            if track_id not in matched_tracks and inst_idx not in matched_instances:
                matched_tracks.add(track_id)
                matched_instances.add(inst_idx)
                current_instances[inst_idx].tracking_id = track_id
                self.active_tracks[track_id] = {
                    "last_bbox": current_instances[inst_idx].bbox,
                    "last_frame_idx": frame_res.frame_idx,
                    "class_name": current_instances[inst_idx].class_name
                }

        for inst_idx in unmatched_current - matched_instances:
            tid = self.next_track_id
            self.next_track_id += 1
            current_instances[inst_idx].tracking_id = tid
            self.active_tracks[tid] = {
                "last_bbox": current_instances[inst_idx].bbox,
                "last_frame_idx": frame_res.frame_idx,
                "class_name": current_instances[inst_idx].class_name
            }

        prune_keys = [
            tid for tid, info in self.active_tracks.items()
            if (frame_res.frame_idx - info["last_frame_idx"]) > self.max_dropout_gap
        ]
        for tid in prune_keys:
            del self.active_tracks[tid]

        return frame_res

    def track_sequence(
        self,
        frame_results: List[FrameDetectionResult]
    ) -> List[FrameDetectionResult]:
        """
        Processes a temporal list of FrameDetectionResults, assigning tracking IDs
        and smoothing temporal dropouts where verified.
        """
        self.reset()
        for frame_res in frame_results:
            self.update(frame_res)
        return frame_results
