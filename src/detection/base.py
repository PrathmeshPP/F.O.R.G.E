"""F.O.R.G.E — Dynamic Object Detection & Segmentation Abstraction Layer.

Defines the clean, framework-agnostic interfaces for dynamic object detection,
instance segmentation, and mask generation. Decouples the 3D reconstruction
pipeline from specific detector implementations (YOLO, Mask R-CNN, SAM, etc.).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional, Tuple, Set
import numpy as np


@dataclass
class BoundingBox:
    """Axis-aligned 2D bounding box in pixel coordinates."""
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    class_name: str

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x1": round(self.x1, 2),
            "y1": round(self.y1, 2),
            "x2": round(self.x2, 2),
            "y2": round(self.y2, 2),
            "confidence": round(self.confidence, 4),
            "class_id": self.class_id,
            "class_name": self.class_name,
            "width": round(self.width, 2),
            "height": round(self.height, 2),
            "area_pixels": round(self.area, 2)
        }


@dataclass
class InstanceMask:
    """Binary mask and metadata for a single segmented dynamic object."""
    mask: np.ndarray  # 2D boolean array of shape (H, W)
    bbox: BoundingBox
    class_id: int
    class_name: str
    confidence: float
    pixel_count: int = 0
    tracking_id: Optional[int] = None

    def __post_init__(self):
        if self.pixel_count == 0 and self.mask is not None:
            self.pixel_count = int(np.sum(self.mask))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": round(self.confidence, 4),
            "pixel_count": self.pixel_count,
            "tracking_id": self.tracking_id,
            "bbox": self.bbox.to_dict()
        }


@dataclass
class FrameDetectionResult:
    """Comprehensive detection and segmentation results for a single video frame."""
    frame_idx: int
    timestamp_sec: float
    height: int
    width: int
    instances: List[InstanceMask] = field(default_factory=list)
    combined_mask: np.ndarray = field(default=None)  # (H, W) uint8 binary mask (0=static, 255=dynamic)
    inference_time_ms: float = 0.0
    device: Optional[str] = None

    def __post_init__(self):
        if self.combined_mask is None:
            self.combined_mask = np.zeros((self.height, self.width), dtype=np.uint8)
            for inst in self.instances:
                if inst.mask is not None:
                    # Spatial verification
                    if inst.mask.shape != (self.height, self.width):
                        raise ValueError(
                            f"Instance mask shape {inst.mask.shape} does not match "
                            f"frame dimensions ({self.height}, {self.width})"
                        )
                    self.combined_mask[inst.mask] = 255

    @property
    def total_masked_pixels(self) -> int:
        return int(np.count_nonzero(self.combined_mask))

    @property
    def coverage_ratio(self) -> float:
        total_pixels = self.height * self.width
        if total_pixels == 0:
            return 0.0
        return float(self.total_masked_pixels / total_pixels)

    @property
    def detected_classes(self) -> List[str]:
        return sorted(list({inst.class_name for inst in self.instances}))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame_idx": self.frame_idx,
            "timestamp_sec": round(self.timestamp_sec, 4),
            "resolution": [self.width, self.height],
            "num_instances": len(self.instances),
            "detected_classes": self.detected_classes,
            "total_masked_pixels": self.total_masked_pixels,
            "coverage_ratio": round(self.coverage_ratio, 6),
            "coverage_percentage": round(self.coverage_ratio * 100.0, 3),
            "inference_time_ms": round(self.inference_time_ms, 2),
            "device": self.device,
            "instances": [inst.to_dict() for inst in self.instances]
        }


class BaseDynamicSegmenter(ABC):
    """Abstract base class for dynamic object detection and instance segmentation."""

    def __init__(
        self,
        target_classes: Optional[List[str]] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        dilation_pixels: int = 5
    ):
        """
        Args:
            target_classes: List of class names to detect/segment (e.g., ['car', 'truck']).
            conf_threshold: Minimum detection confidence threshold [0.0, 1.0].
            iou_threshold: NMS IoU threshold [0.0, 1.0].
            dilation_pixels: Morphological dilation radius in pixels to cover boundaries/shadows.
        """
        self.target_classes = target_classes or ["car", "motorcycle", "bus", "truck", "bicycle"]
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.dilation_pixels = dilation_pixels

    @abstractmethod
    def segment_frame(
        self,
        frame: np.ndarray,
        frame_idx: int = 0,
        timestamp_sec: float = 0.0
    ) -> FrameDetectionResult:
        """
        Runs instance segmentation on an RGB or BGR frame.

        Args:
            frame: Input image array of shape (H, W, 3).
            frame_idx: Source video frame index.
            timestamp_sec: Source video timestamp in seconds.

        Returns:
            FrameDetectionResult containing binary mask, bounding boxes, and metadata.
        """
        pass

    def segment_sequence(
        self,
        frames: List[np.ndarray],
        frame_indices: Optional[List[int]] = None,
        timestamps: Optional[List[float]] = None
    ) -> List[FrameDetectionResult]:
        """
        Processes a sequence of frames sequentially or in batches.
        """
        num_frames = len(frames)
        indices = frame_indices or list(range(num_frames))
        times = timestamps or [float(i) for i in range(num_frames)]

        results = []
        for i in range(num_frames):
            res = self.segment_frame(
                frame=frames[i],
                frame_idx=indices[i],
                timestamp_sec=times[i]
            )
            results.append(res)
        return results
