"""F.O.R.G.E — Detection and Dynamic Object Masking Package.

Exports base interfaces, YOLO segmenter, deterministic mock segmenter,
lightweight temporal tracker, and forensic visualizers.
"""

from src.detection.base import (
    BoundingBox,
    InstanceMask,
    FrameDetectionResult,
    BaseDynamicSegmenter,
)
from src.detection.yolo_segmenter import YOLOInstanceSegmenter
from src.detection.mock_segmenter import MockDynamicSegmenter
from src.detection.temporal_tracker import LightweightTemporalTracker
from src.detection.visualizer import (
    render_detection_overlay,
    create_diagnostic_tripanel,
)

__all__ = [
    "BoundingBox",
    "InstanceMask",
    "FrameDetectionResult",
    "BaseDynamicSegmenter",
    "YOLOInstanceSegmenter",
    "MockDynamicSegmenter",
    "LightweightTemporalTracker",
    "render_detection_overlay",
    "create_diagnostic_tripanel",
]
