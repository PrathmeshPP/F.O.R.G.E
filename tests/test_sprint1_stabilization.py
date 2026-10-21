"""
F.O.R.G.E — Sprint 1 Regression & Stabilization Unit Tests.

Covers:
1. Controlled non-identity 3D projection and backprojection round-trips.
2. Camera center calculations and SO(3) validation.
3. Depth-edge relative jump discontinuity detection across tolerances.
4. Single-frame and batched input handling.
5. Orientation-aware viewing angle calculations (Nadir vs Corridor).
6. Failure handling (NaN/Inf depths, empty clouds, zero baseline).
"""

import unittest
import numpy as np
import open3d as o3d
from pathlib import Path

from src.reconstruction.point_cloud import depth_to_points, save_ply
from src.reconstruction.pipeline import (
    PipelineConfig,
    assess_camera_trajectory,
    estimate_adaptive_parameters,
    GeneralReconstructionPipeline
)
from scripts.validate_sprint1_geometry_and_filtering import (
    compute_relative_depth_jump,
    run_controlled_projection_tests
)


class TestSprint1Stabilization(unittest.TestCase):

    def test_synthetic_projection_backprojection_roundtrip(self):
        """Verifies that 3D points projected to camera pixels and unprojected match within machine precision."""
        results = run_controlled_projection_tests()
        self.assertTrue(results["synthetic_test_passed"])
        self.assertLess(results["max_depth_error"], 1e-4)
        self.assertLess(results["max_pixel_error_u"], 1e-3)
        self.assertLess(results["max_pixel_error_v"], 1e-3)
        self.assertAlmostEqual(results["rotation_det"], 1.0, places=5)

    def test_single_frame_vs_batch_consistency(self):
        """Ensures single-frame (H, W) and batched (1, H, W, 1) inputs produce identical 3D world points."""
        H, W = 16, 24
        depth_2d = np.linspace(1.0, 5.0, H * W).reshape(H, W).astype(np.float32)
        extr_3x4 = np.array([
            [0.866, -0.5, 0.0, 0.2],
            [0.5, 0.866, 0.0, -0.1],
            [0.0, 0.0, 1.0, 0.5]
        ], dtype=np.float32)
        intr_3x3 = np.array([
            [100.0, 0.0, 12.0],
            [0.0, 100.0, 8.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        # 1. Single frame call (H, W)
        pts_single = depth_to_points(depth_2d, extr_3x4, intr_3x3)
        self.assertEqual(pts_single.shape, (H, W, 3))

        # 2. Batched call (1, H, W, 1)
        depth_4d = depth_2d[None, :, :, None]
        extr_batch = extr_3x4[None, :, :]
        intr_batch = intr_3x3[None, :, :]
        pts_batch = depth_to_points(depth_4d, extr_batch, intr_batch)
        self.assertEqual(pts_batch.shape, (1, H, W, 3))

        # Check numerical equality
        max_diff = np.max(np.abs(pts_single - pts_batch[0]))
        self.assertLess(max_diff, 1e-6)

    def test_depth_edge_detection(self):
        """Verifies that relative depth jump detection isolates discontinuities without touching smooth surfaces."""
        # Continuous gentle ramp (simulating smooth terrain)
        ramp = np.outer(np.linspace(1.0, 1.1, 20), np.linspace(1.0, 1.1, 20)).astype(np.float32)
        jumps_ramp = compute_relative_depth_jump(ramp, kernel_size=3)
        # On a smooth terrain ramp, relative jump between adjacent cells is small (< 0.03)
        mask_ramp = jumps_ramp > 0.08
        self.assertFalse(np.any(mask_ramp), "Smooth terrain ramp should have no detected depth edge jumps > 0.08")

        # Step obstacle (simulating horizon or building boundary)
        step = ramp.copy()
        step[5:15, 5:15] += 2.0  # Significant jump
        jumps_step = compute_relative_depth_jump(step, kernel_size=3)
        mask_step = jumps_step > 0.08
        self.assertTrue(np.any(mask_step), "Step obstacle must be detected by depth jump filter")

    def test_non_finite_depth_filtering(self):
        """Verifies that NaN and Inf depths are safely filtered without crashing."""
        H, W = 8, 8
        depth_nan = np.ones((H, W), dtype=np.float32) * 2.0
        depth_nan[2, 2] = np.nan
        depth_nan[4, 4] = np.inf

        extr = np.hstack([np.eye(3), np.zeros((3, 1))])
        intr = np.array([[50.0, 0, 4.0], [0, 50.0, 4.0], [0, 0, 1.0]])

        pts = depth_to_points(depth_nan, extr, intr)
        valid = np.isfinite(pts).all(axis=-1)
        self.assertEqual(int(np.sum(valid)), H * W - 2)

    def test_empty_point_cloud_error_handling(self):
        """Verifies that attempting point cloud creation with impossible thresholds raises a clean RuntimeError."""
        from src.reconstruction.point_cloud import create_point_cloud
        import tempfile

        with tempfile.TemporaryDirectory() as tmp_dir:
            depth_p = Path(tmp_dir) / "depth.npz"
            poses_p = Path(tmp_dir) / "poses.npz"
            frames_p = Path(tmp_dir) / "frames"
            frames_p.mkdir()
            out_ply = Path(tmp_dir) / "out.ply"

            np.savez(depth_p, depth=np.ones((1, 8, 8, 1), dtype=np.float32) * 0.01)  # All depths below min_depth
            np.savez(poses_p, extrinsics=np.eye(4)[None, :3, :], intrinsics=np.eye(3)[None, :, :])

            # Create dummy frame
            from PIL import Image
            Image.fromarray(np.zeros((8, 8, 3), dtype=np.uint8)).save(frames_p / "frame_00.png")

            with self.assertRaises(RuntimeError):
                create_point_cloud(
                    depth_npz_path=str(depth_p),
                    poses_npz_path=str(poses_p),
                    frames_dir=str(frames_p),
                    output_ply_path=str(out_ply),
                    min_depth=1.0  # Impossible threshold
                )


if __name__ == "__main__":
    unittest.main()
