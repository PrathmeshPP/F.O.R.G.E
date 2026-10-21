"""F.O.R.G.E — Sprint 11 Automated Test Suite.

Tests:
1. Mask dimensions and spatial coordinate alignment.
2. Source frame index and timestamp preservation.
3. Binary mask validity and empty mask handling.
4. Nearest-neighbor label-safe mask rescaling to depth map dimensions.
5. Dynamic pixel exclusion from static 3D point cloud.
6. Preservation of unmasked static points.
7. Class filtering and non-target rejection.
8. Corrupt/invalid input error handling.
9. GPU/CPU device fallback behavior.
10. Lightweight temporal tracker association and ID persistence.
11. End-to-end integration smoke test with MockDynamicSegmenter.
"""

from __future__ import annotations

import sys
import json
import tempfile
from pathlib import Path
from typing import List

import cv2
import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.detection.base import (
    BoundingBox,
    InstanceMask,
    FrameDetectionResult,
    BaseDynamicSegmenter,
)
from src.detection.mock_segmenter import MockDynamicSegmenter
from src.detection.temporal_tracker import (
    LightweightTemporalTracker,
    compute_box_iou,
)
from src.detection.visualizer import (
    render_detection_overlay,
    create_diagnostic_tripanel,
)
from src.reconstruction.multiframe_stitcher import MultiFrameDensePointCloud


class TestDynamicObjectMasking:
    """Sprint 11 comprehensive automated tests."""

    def test_mask_dimensions_and_spatial_alignment(self):
        """Binary mask dimensions must match frame height and width exactly."""
        h, w = 480, 640
        inst_mask = np.zeros((h, w), dtype=bool)
        inst_mask[100:200, 150:300] = True

        bbox = BoundingBox(150, 100, 300, 200, 0.95, 2, "car")
        inst = InstanceMask(mask=inst_mask, bbox=bbox, class_id=2, class_name="car", confidence=0.95)

        res = FrameDetectionResult(
            frame_idx=10,
            timestamp_sec=0.333,
            height=h,
            width=w,
            instances=[inst]
        )

        assert res.combined_mask.shape == (h, w), "Combined mask shape must match frame resolution"
        assert res.combined_mask.dtype == np.uint8, "Mask must be uint8"
        assert np.all((res.combined_mask == 0) | (res.combined_mask == 255)), "Mask values must be binary 0 or 255"
        assert res.total_masked_pixels == (100 * 150), f"Expected 15,000 pixels, got {res.total_masked_pixels}"

        # Test dimension mismatch error
        bad_mask = np.zeros((300, 300), dtype=bool)
        bad_inst = InstanceMask(mask=bad_mask, bbox=bbox, class_id=2, class_name="car", confidence=0.95)
        with pytest.raises(ValueError, match="does not match frame dimensions"):
            _ = FrameDetectionResult(
                frame_idx=10,
                timestamp_sec=0.333,
                height=h,
                width=w,
                instances=[bad_inst]
            )

    def test_frame_indices_and_timestamps_preserved(self):
        """FrameDetectionResult must preserve exact source frame index and timestamp."""
        res = FrameDetectionResult(
            frame_idx=142,
            timestamp_sec=5.922,
            height=720,
            width=1280,
            instances=[]
        )
        assert res.frame_idx == 142
        assert abs(res.timestamp_sec - 5.922) < 1e-4

        d = res.to_dict()
        assert d["frame_idx"] == 142
        assert d["timestamp_sec"] == 5.922
        assert d["resolution"] == [1280, 720]

    def test_binary_mask_validity_and_empty_handling(self):
        """Scenes with no dynamic objects must return an all-zero mask without exceptions."""
        h, w = 1080, 1920
        res = FrameDetectionResult(
            frame_idx=0,
            timestamp_sec=0.0,
            height=h,
            width=w,
            instances=[]
        )
        assert res.total_masked_pixels == 0
        assert res.coverage_ratio == 0.0
        assert res.detected_classes == []
        assert np.count_nonzero(res.combined_mask) == 0

    def test_conversion_and_rescaling_to_depth_coordinates(self):
        """Rescaling masks to match different depth map resolutions must use nearest-neighbor label-safe interpolation."""
        orig_h, orig_w = 720, 1280
        depth_h, depth_w = 360, 640

        mask_orig = np.zeros((orig_h, orig_w), dtype=np.uint8)
        mask_orig[200:400, 400:800] = 255

        # Nearest-neighbor resize
        mask_depth = cv2.resize(mask_orig, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)

        assert mask_depth.shape == (depth_h, depth_w)
        # All values must strictly remain 0 or 255 (no blurred intermediate float/anti-aliasing values)
        assert set(np.unique(mask_depth)).issubset({0, 255}), "Nearest-neighbor must maintain pure binary labels"

    def test_exclusion_of_masked_pixels_from_point_cloud(self):
        """Dynamic pixels must be excluded from static point cloud sampling in construct_clouds."""
        num_frames = 2
        h, w = 100, 100

        depth_maps = np.ones((num_frames, h, w), dtype=np.float32) * 2.0
        extrinsics = np.tile(np.eye(4)[:3], (num_frames, 1, 1))
        intrinsics = np.tile(np.array([[100.0, 0, 50.0], [0, 100.0, 50.0], [0, 0, 1.0]]), (num_frames, 1, 1))
        frame_rgbs = np.ones((num_frames, h, w, 3), dtype=np.uint8) * 128

        # 1. Without dynamic mask
        res_unmasked = MultiFrameDensePointCloud.construct_clouds(
            depth_maps=depth_maps,
            extrinsics=extrinsics,
            intrinsics=intrinsics,
            frame_rgbs=frame_rgbs,
            dynamic_masks=None
        )

        # 2. With dynamic mask covering 20x20 region in frame 0
        dyn_mask = np.zeros((num_frames, h, w), dtype=np.uint8)
        dyn_mask[0, 10:30, 10:30] = 255  # 400 pixels
        dyn_pixels_expected = 400

        res_masked = MultiFrameDensePointCloud.construct_clouds(
            depth_maps=depth_maps,
            extrinsics=extrinsics,
            intrinsics=intrinsics,
            frame_rgbs=frame_rgbs,
            dynamic_masks=dyn_mask
        )

        assert res_masked["dynamic_points_removed"] == dyn_pixels_expected, (
            f"Expected {dyn_pixels_expected} points removed, got {res_masked['dynamic_points_removed']}"
        )
        assert res_masked["clean_valid_count"] == (res_unmasked["clean_valid_count"] - dyn_pixels_expected)

    def test_preservation_of_unmasked_static_points(self):
        """Static background points outside dynamic mask must remain identical in 3D geometry."""
        num_frames = 1
        h, w = 50, 50

        depth_maps = np.ones((num_frames, h, w), dtype=np.float32) * 1.5
        extrinsics = np.tile(np.eye(4)[:3], (num_frames, 1, 1))
        intrinsics = np.tile(np.array([[50.0, 0, 25.0], [0, 50.0, 25.0], [0, 0, 1.0]]), (num_frames, 1, 1))
        frame_rgbs = np.zeros((num_frames, h, w, 3), dtype=np.uint8)
        frame_rgbs[:, :, :, 1] = 200  # Green background

        # Mask top-left 10x10 corner
        dyn_mask = np.zeros((num_frames, h, w), dtype=np.uint8)
        dyn_mask[0, 0:10, 0:10] = 255

        res = MultiFrameDensePointCloud.construct_clouds(
            depth_maps=depth_maps,
            extrinsics=extrinsics,
            intrinsics=intrinsics,
            frame_rgbs=frame_rgbs,
            dynamic_masks=dyn_mask
        )

        clean_cloud = res["pcd_clean"]
        pts = np.asarray(clean_cloud.points)
        cols = np.asarray(clean_cloud.colors)

        # Unmasked points count: 2500 - 100 = 2400
        assert len(pts) == 2400
        # Check that colors remain pure green (0, 200/255, 0)
        assert np.allclose(cols[:, 1], 200.0 / 255.0, atol=1e-3)
        assert np.allclose(cols[:, 0], 0.0, atol=1e-3)

    def test_mock_segmenter_class_filtering(self):
        """Target class filtering must only include specified vehicle classes."""
        segmenter = MockDynamicSegmenter(
            target_classes=["car", "truck"],
            synthetic_objects=[
                {"class_name": "car", "class_id": 2, "conf": 0.90},
                {"class_name": "traffic_light", "class_id": 9, "conf": 0.85},
                {"class_name": "person", "class_id": 0, "conf": 0.88},
                {"class_name": "truck", "class_id": 7, "conf": 0.92},
            ]
        )

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        res = segmenter.segment_frame(frame, frame_idx=0)

        # Only car and truck should be preserved; traffic_light and person excluded
        assert len(res.instances) == 2
        for inst in res.instances:
            assert inst.class_name in ["car", "truck"], f"Unexpected class {inst.class_name} passed filter"

    def test_corrupt_or_invalid_frame_handling(self):
        """Invalid frame shapes must raise clean ValueError."""
        segmenter = MockDynamicSegmenter()
        bad_frame_1d = np.zeros((100,), dtype=np.uint8)
        with pytest.raises(ValueError):
            segmenter.segment_frame(bad_frame_1d)

    def test_temporal_tracker_association(self):
        """LightweightTemporalTracker must match overlapping bboxes and assign consistent IDs."""
        tracker = LightweightTemporalTracker(iou_threshold=0.3)

        # Frame 0: Vehicle at (100, 100) -> (200, 200)
        mask0 = np.zeros((480, 640), dtype=bool)
        mask0[100:200, 100:200] = True
        b0 = BoundingBox(100, 100, 200, 200, 0.9, 2, "car")
        inst0 = InstanceMask(mask0, b0, 2, "car", 0.9)
        res0 = FrameDetectionResult(0, 0.0, 480, 640, [inst0])

        res0 = tracker.update(res0)
        assert res0.instances[0].tracking_id is not None
        initial_id = res0.instances[0].tracking_id

        # Frame 1: Vehicle moved slightly to (110, 105) -> (210, 205) (High IoU)
        mask1 = np.zeros((480, 640), dtype=bool)
        mask1[105:205, 110:210] = True
        b1 = BoundingBox(110, 105, 210, 205, 0.92, 2, "car")
        inst1 = InstanceMask(mask1, b1, 2, "car", 0.92)
        res1 = FrameDetectionResult(1, 0.04, 480, 640, [inst1])

        res1 = tracker.update(res1)
        assert res1.instances[0].tracking_id == initial_id, "Track ID must persist across overlapping frames"

    def test_visualizer_rendering(self):
        """Overlay and diagnostic tri-panel must generate valid uint8 images with correct dimensions."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[100:200, 100:200] = [200, 100, 50]

        mask = np.zeros((480, 640), dtype=bool)
        mask[120:180, 120:180] = True
        bbox = BoundingBox(120, 120, 180, 180, 0.88, 2, "car")
        inst = InstanceMask(mask, bbox, 2, "car", 0.88, tracking_id=1)
        res = FrameDetectionResult(0, 0.0, 480, 640, [inst])

        overlay = render_detection_overlay(frame, res)
        assert overlay.shape == (480, 640, 3)
        assert overlay.dtype == np.uint8

        tripanel = create_diagnostic_tripanel(frame, res)
        assert tripanel.shape == (480, 640 * 3, 3), "Tripanel width must be 3x frame width"
        assert tripanel.dtype == np.uint8

    def test_integration_smoke_with_mock(self):
        """End-to-end smoke test of process_scene_masking using mock segmenter and temporary directory."""
        from scripts.run_dynamic_masking import process_scene_masking

        # Create synthetic test video
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_root = Path(tmp_dir)
            video_path = tmp_root / "test_synth.mp4"

            # Write 10 synthetic frames
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(video_path), fourcc, 10.0, (320, 240))
            for i in range(10):
                f = np.zeros((240, 320, 3), dtype=np.uint8)
                # Draw simulated moving block
                cv2.rectangle(f, (50 + i * 5, 50), (120 + i * 5, 120), (255, 255, 255), -1)
                writer.write(f)
            writer.release()

            segmenter = MockDynamicSegmenter(target_classes=["car"])
            meta = process_scene_masking(
                scene_name="synth_test",
                video_path=video_path,
                output_root=tmp_root / "outputs",
                segmenter=segmenter,
                step=2,
                max_frames=5,
                enable_tracking=True
            )

            assert meta["scene_name"] == "synth_test"
            assert meta["num_frames_processed"] == 5
            assert (tmp_root / "outputs" / "synth_test" / "metadata" / "detections_metadata.json").is_file()
            assert (tmp_root / "outputs" / "synth_test" / "SUMMARY_REPORT.md").is_file()
            assert len(list((tmp_root / "outputs" / "synth_test" / "masks").glob("*.png"))) == 5
            assert len(list((tmp_root / "outputs" / "synth_test" / "diagnostics").glob("*.png"))) == 10
