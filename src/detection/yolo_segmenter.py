"""F.O.R.G.E — YOLO-Based Instance Segmentation for Dynamic Object Masking.

Wraps the Ultralytics YOLO segmentation engine behind the BaseDynamicSegmenter
interface, providing robust road-vehicle and dynamic-obstacle segmentation,
resolution-preserving nearest-neighbor mask rasterization, and configurable
boundary dilation.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple, Set

import numpy as np
import cv2
import torch

from src.detection.base import (
    BaseDynamicSegmenter,
    BoundingBox,
    InstanceMask,
    FrameDetectionResult,
)

DEFAULT_CANDIDATE_PATHS = [
    r"C:\Users\25beevdt025\Desktop\SIH\yolov8n-seg.pt",
    r"C:\Users\25beevdt025\Desktop\SIH\third_party\yolov8n-seg.pt",
    str(Path(__file__).resolve().parent.parent.parent / "models" / "yolov8n-seg.pt"),
    "yolov8n-seg.pt"
]


class YOLOInstanceSegmenter(BaseDynamicSegmenter):
    """
    YOLOv8 instance segmentation backend for dynamic object masking.
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        target_classes: Optional[List[str]] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        dilation_pixels: int = 5,
        device: str = "auto"
    ):
        super().__init__(
            target_classes=target_classes or ["car", "motorcycle", "bus", "truck", "bicycle"],
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            dilation_pixels=dilation_pixels
        )

        self.device = self._resolve_device(device)
        self.model_path = self._resolve_model_path(model_path)
        self.model = self._load_model(self.model_path)

        # Cache class mapping
        self.name_to_id: Dict[str, int] = {v: k for k, v in self.model.names.items()}
        self.target_class_ids: Set[int] = {
            self.name_to_id[c] for c in self.target_classes if c in self.name_to_id
        }

    @staticmethod
    def _resolve_device(device: str) -> str:
        if device == "auto":
            if torch.cuda.is_available():
                return "cuda:0"
            return "cpu"
        return device

    @staticmethod
    def _resolve_model_path(provided_path: Optional[str]) -> str:
        if provided_path and os.path.exists(provided_path):
            return provided_path

        env_path = os.environ.get("FORGE_YOLO_MODEL")
        if env_path and os.path.exists(env_path):
            return env_path

        for candidate in DEFAULT_CANDIDATE_PATHS:
            if os.path.exists(candidate):
                return candidate

        # Return default weight identifier (ultralytics will download if internet is accessible)
        return "yolov8n-seg.pt"

    def _load_model(self, path: str):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise ImportError(
                "The 'ultralytics' package is required for YOLOInstanceSegmenter. "
                "Install it via 'pip install ultralytics'."
            ) from e

        model = YOLO(path)
        return model

    def segment_frame(
        self,
        frame: np.ndarray,
        frame_idx: int = 0,
        timestamp_sec: float = 0.0
    ) -> FrameDetectionResult:
        """
        Executes instance segmentation on a single frame.

        Args:
            frame: (H, W, 3) image in RGB or BGR format (uint8).
            frame_idx: Frame index from the source video sequence.
            timestamp_sec: Frame timestamp in seconds.

        Returns:
            FrameDetectionResult containing binary mask, bounding boxes, and metadata.
        """
        t0 = time.perf_counter()
        orig_h, orig_w = frame.shape[:2]

        # Run inference
        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )

        t1 = time.perf_counter()
        inference_time_ms = (t1 - t0) * 1000.0

        r = results[0]
        instances: List[InstanceMask] = []
        combined_mask = np.zeros((orig_h, orig_w), dtype=np.uint8)

        if r.boxes is not None and len(r.boxes) > 0 and r.masks is not None:
            boxes = r.boxes
            masks_data = r.masks.data.cpu().numpy()  # Shape: (N, mask_h, mask_w) float32 [0.0, 1.0]

            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                cls_name = self.model.names.get(cls_id, str(cls_id))

                # Check if detected class is in target classes
                if cls_id not in self.target_class_ids and cls_name not in self.target_classes:
                    continue

                conf = float(boxes.conf[i].item())
                xyxy = boxes.xyxy[i].cpu().numpy().tolist()

                bbox = BoundingBox(
                    x1=float(xyxy[0]),
                    y1=float(xyxy[1]),
                    x2=float(xyxy[2]),
                    y2=float(xyxy[3]),
                    confidence=conf,
                    class_id=cls_id,
                    class_name=cls_name
                )

                # Rescale instance mask to original frame dimensions using label-safe nearest-neighbor
                raw_mask = masks_data[i]
                if raw_mask.shape != (orig_h, orig_w):
                    resized_mask = cv2.resize(
                        raw_mask,
                        (orig_w, orig_h),
                        interpolation=cv2.INTER_NEAREST
                    )
                else:
                    resized_mask = raw_mask

                binary_mask = (resized_mask > 0.5)

                # Morphological dilation to encompass boundary edges and shadows
                if self.dilation_pixels > 0:
                    kernel_size = 2 * self.dilation_pixels + 1
                    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
                    binary_mask_u8 = (binary_mask.astype(np.uint8) * 255)
                    dilated_u8 = cv2.dilate(binary_mask_u8, kernel)
                    binary_mask = (dilated_u8 > 0)

                inst = InstanceMask(
                    mask=binary_mask,
                    bbox=bbox,
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    pixel_count=int(np.sum(binary_mask))
                )
                instances.append(inst)
                combined_mask[binary_mask] = 255

        return FrameDetectionResult(
            frame_idx=frame_idx,
            timestamp_sec=timestamp_sec,
            height=orig_h,
            width=orig_w,
            instances=instances,
            combined_mask=combined_mask,
            inference_time_ms=inference_time_ms,
            device=self.device
        )
