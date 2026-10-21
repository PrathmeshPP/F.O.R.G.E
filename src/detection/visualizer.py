"""F.O.R.G.E — Dynamic Object Mask & Detection Visualizer.

Generates high-contrast diagnostic visualizations, 3-panel before/after comparisons,
and mask overlays for forensic evaluation of dynamic object segmentation.
"""

from __future__ import annotations

from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

from src.detection.base import (
    FrameDetectionResult,
    InstanceMask,
)

# Curated palette for high-contrast multi-instance visualization
COLOR_PALETTE = [
    (255, 59, 48),    # Red
    (52, 199, 89),    # Green
    (0, 122, 255),    # Blue
    (255, 149, 0),    # Orange
    (175, 82, 222),   # Purple
    (255, 204, 0),    # Yellow
    (88, 86, 214),    # Indigo
    (0, 199, 190),    # Teal
]


def render_detection_overlay(
    frame: np.ndarray,
    result: FrameDetectionResult,
    alpha: float = 0.45
) -> np.ndarray:
    """
    Renders semi-transparent color overlays, contours, and label banners
    onto the frame for all detected instances.

    Args:
        frame: (H, W, 3) uint8 image.
        result: FrameDetectionResult containing instances and metadata.
        alpha: Blending transparency for the instance masks.

    Returns:
        (H, W, 3) uint8 image with overlay annotations.
    """
    canvas = frame.copy()
    overlay = canvas.copy()

    for idx, inst in enumerate(result.instances):
        color = COLOR_PALETTE[idx % len(COLOR_PALETTE)]

        # Apply color overlay on masked pixels
        if inst.mask is not None and np.any(inst.mask):
            overlay[inst.mask] = color

            # Draw outer contour boundary
            mask_u8 = (inst.mask.astype(np.uint8) * 255)
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(canvas, contours, -1, color, 2)

    # Blend overlay with canvas
    cv2.addWeighted(overlay, alpha, canvas, 1 - alpha, 0, canvas)

    # Draw bounding boxes and text badges
    for idx, inst in enumerate(result.instances):
        color = COLOR_PALETTE[idx % len(COLOR_PALETTE)]
        bx = inst.bbox
        x1, y1, x2, y2 = int(bx.x1), int(bx.y1), int(bx.x2), int(bx.y2)

        # Bounding box rectangle
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 1, cv2.LINE_AA)

        # Label text badge
        track_str = f" #{inst.tracking_id}" if inst.tracking_id else ""
        label = f"{inst.class_name}{track_str} {inst.confidence:.2f}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)

        badge_y1 = max(0, y1 - th - baseline - 4)
        badge_y2 = badge_y1 + th + baseline + 4
        badge_x2 = min(canvas.shape[1], x1 + tw + 6)

        cv2.rectangle(canvas, (x1, badge_y1), (badge_x2, badge_y2), color, -1)
        cv2.putText(
            canvas,
            label,
            (x1 + 3, badge_y2 - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    return canvas


def create_diagnostic_tripanel(
    frame: np.ndarray,
    result: FrameDetectionResult
) -> np.ndarray:
    """
    Creates a 3-panel side-by-side diagnostic image:
    [ Panel 1: Original RGB Frame ]
    [ Panel 2: Detection & Mask Overlay ]
    [ Panel 3: Masked-Out Static Background (Dynamic pixels neutralized) ]

    Args:
        frame: (H, W, 3) uint8 image.
        result: FrameDetectionResult.

    Returns:
        (H, 3 * W, 3) composite diagnostic image with section headers.
    """
    h, w = frame.shape[:2]

    # Panel 1: Original frame with header banner
    p1 = frame.copy()
    cv2.putText(p1, f"Frame {result.frame_idx:04d} ({result.timestamp_sec:.2f}s) - Original", (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

    # Panel 2: Overlay
    p2 = render_detection_overlay(frame, result)
    info_txt = f"Detections: {len(result.instances)} | Coverage: {result.coverage_ratio*100:.1f}% | Time: {result.inference_time_ms:.1f}ms"
    cv2.putText(p2, info_txt, (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    # Panel 3: Static background preservation check (dynamic pixels tinted or hatched)
    p3 = frame.copy()
    if result.combined_mask is not None:
        mask_bool = (result.combined_mask > 0)
        # Dim masked dynamic pixels and overlay red hatching/tint to show filtered region
        p3[mask_bool] = (p3[mask_bool].astype(float) * 0.25 + np.array([40, 40, 200]) * 0.75).astype(np.uint8)
    cv2.putText(p3, "Filtered Static Surface (Dynamic Object Removed)", (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2, cv2.LINE_AA)

    # Horizontally stack panels
    tripanel = np.hstack([p1, p2, p3])
    return tripanel
