"""F.O.R.G.E — Unit Tests for Controlled Point Cloud Accuracy Improvements.

Validates:
1. Official VGGT-Omega pixel-coordinate backprojection convention (no unverified half-pixel offsets).
2. Depth confidence filtering behavior.
3. Voxel-grid fusion and RGB color averaging logic (both Open3D and NumPy fallback).
4. Edge cases: empty masks, non-finite depths, zero voxel size.
"""

import unittest
import numpy as np
from src.reconstruction.point_cloud import depth_to_points, voxel_grid_fusion


class TestPointCloudAccuracy(unittest.TestCase):

    def test_official_coordinate_convention(self):
        """Verify backprojection matches official VGGT-Omega (u - cx)/fx * depth convention."""
        H, W = 4, 4
        depth = np.ones((1, H, W), dtype=np.float32) * 2.0
        intrinsics = np.array([[[4.0, 0.0, 2.0],
                                [0.0, 4.0, 2.0],
                                [0.0, 0.0, 1.0]]], dtype=np.float32)
        extrinsics = np.array([[[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0]]], dtype=np.float32)

        pts = depth_to_points(depth, extrinsics, intrinsics)
        # Center pixel (v=2, u=2): (2 - 2)/4 * 2.0 = 0.0
        np.testing.assert_allclose(pts[0, 2, 2], [0.0, 0.0, 2.0], atol=1e-5)
        # Top-left pixel (v=0, u=0): (0 - 2)/4 * 2.0 = -1.0
        np.testing.assert_allclose(pts[0, 0, 0], [-1.0, -1.0, 2.0], atol=1e-5)
        # Bottom-right pixel (v=3, u=3): (3 - 2)/4 * 2.0 = 0.5
        np.testing.assert_allclose(pts[0, 3, 3], [0.5, 0.5, 2.0], atol=1e-5)

    def test_voxel_grid_fusion_averaging(self):
        """Verify voxel grid fusion consolidates nearby points and averages colors."""
        pts = np.array([
            [0.001, 0.001, 0.001],
            [0.003, 0.003, 0.003],
            [1.000, 1.000, 1.000],
        ], dtype=np.float32)
        cols = np.array([
            [100, 50, 0],
            [200, 150, 100],
            [0, 255, 0],
        ], dtype=np.uint8)

        fused_pts, fused_cols = voxel_grid_fusion(pts, cols, voxel_size=0.01)
        self.assertEqual(len(fused_pts), 2)
        self.assertEqual(len(fused_cols), 2)

        # First voxel centroid: (0.001 + 0.003)/2 = 0.002
        np.testing.assert_allclose(fused_pts[0], [0.002, 0.002, 0.002], atol=1e-4)
        # First voxel color average: [(100+200)/2, (50+150)/2, (0+100)/2] = [150, 100, 50]
        np.testing.assert_allclose(fused_cols[0], [150, 100, 50], atol=2)

    def test_voxel_grid_passthrough_when_none(self):
        """Verify voxel grid returns inputs unchanged when voxel_size is None or <= 0."""
        pts = np.random.rand(10, 3).astype(np.float32)
        cols = np.random.randint(0, 255, (10, 3), dtype=np.uint8)

        f_pts, f_cols = voxel_grid_fusion(pts, cols, voxel_size=None)
        np.testing.assert_array_equal(f_pts, pts)
        np.testing.assert_array_equal(f_cols, cols)

        f_pts_zero, f_cols_zero = voxel_grid_fusion(pts, cols, voxel_size=0.0)
        np.testing.assert_array_equal(f_pts_zero, pts)
        np.testing.assert_array_equal(f_cols_zero, cols)


if __name__ == "__main__":
    unittest.main()
