"""F.O.R.G.E — Mock Dynamic Object Segmenter for Unit & Integration Testing.

Provides deterministic, zero-dependency instance segmentation results for test suites,
eliminating network weight downloads or GPU requirements.
"""

from __future__ import annotations

import time
from typing import List, Dict, Any, Optional, Tuple
import numpy as np

from src.detection.base import (
    BaseDynamicSegmenter,
    BoundingBox,
    InstanceMask,
    FrameDetectionResult,
)


class MockDynamicSegmenter(BaseDynamicSegmenter):
    """
    Deterministic mock segmenter that generates synthetic vehicle detections
    or empty frames for reproducible testing.
    """

    def __init__(
        self,
        target_classes: Optional[List[str]] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        dilation_pixels: int = 0,
        synthetic_objects: Optional[List[Dict[str, Any]]] = None
    ):
        super().__init__(
            target_classes=target_classes or ["car", "motorcycle", "bus", "truck", "bicycle"],
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            dilation_pixels=dilation_pixels
        )
        # Optional list of synthetic objects to inject:
        # [{"class_name": "car", "class_id": 2, "conf": 0.85, "rel_bbox": [0.4, 0.4, 0.6, 0.6]}]
        self.synthetic_objects = synthetic_objects

    def segment_frame(
        self,
        frame: np.ndarray,
        frame_idx: int = 0,
        timestamp_sec: float = 0.0
    ) -> FrameDetectionResult:
        t0 = time.perf_counter()
        orig_h, orig_w = frame.shape[:2]

        instances: List[InstanceMask] = []
        combined_mask = np.zeros((orig_h, orig_w), dtype=np.uint8)

        if self.synthetic_objects is not None:
            for obj in self.synthetic_objects:
                cls_name = obj.get("class_name", "car")
                cls_id = obj.get("class_id", 2)
                conf = float(obj.get("conf", 0.90))

                if cls_name not in self.target_classes or conf < self.conf_threshold:
                    continue

                rel_bbox = obj.get("rel_bbox", [0.4, 0.4, 0.6, 0.6])
                x1 = int(rel_bbox[0] * orig_w)
                y1 = int(rel_bbox[1] * orig_h)
                x2 = int(rel_bbox[2] * orig_w)
                y2 = int(rel_bbox[3] * orig_h)

                bbox = BoundingBox(
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                    confidence=conf,
                    class_id=cls_id,
                    class_name=cls_name
                )

                inst_mask = np.zeros((orig_h, orig_w), dtype=bool)
                inst_mask[y1:y2, x1:x2] = True

                inst = InstanceMask(
                    mask=inst_mask,
                    bbox=bbox,
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    pixel_count=int(np.sum(inst_mask))
                )
                instances.append(inst)
                combined_mask[inst_mask] = 255

        t1 = time.perf_counter()
        return FrameDetectionResult(
            frame_idx=frame_idx,
            timestamp_sec=timestamp_sec,
            height=orig_h,
            width=orig_w,
            instances=instances,
            combined_mask=combined_mask,
            inference_time_ms=(t1 - t0) * 1000.0
        )
