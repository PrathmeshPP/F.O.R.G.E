"""Unit tests for F.O.R.G.E Multi-Frame Reconstruction & Surface Connectivity.
Tests:
1. Window partitioning logic and overlap constraints.
2. Umeyama 7-DoF SIM3 transformation invariance and camera extrinsics update.
3. Multi-frame dense point cloud backprojection and depth-edge filtering.
4. Surface connectivity mesher and hole-repair topology closure.
"""

from __future__ import annotations

import unittest
import numpy as np
import open3d as o3d

from src.reconstruction.multiframe_stitcher import (
    WindowedInferenceManager,
    MultiFrameDensePointCloud,
    SurfaceConnectivityMesher
)
from src.reconstruction.point_cloud import depth_to_points
from src.geo.umeyama_ransac import umeyama_alignment


class TestMultiFrameReconstruction(unittest.TestCase):

    def test_window_partitioning(self):
        manager = WindowedInferenceManager(pipeline=None, window_size=8, overlap=3)

        # Single window when N <= window_size
        w4 = manager.partition_frames(4)
        self.assertEqual(w4, [(0, 4)])

        w8 = manager.partition_frames(8)
        self.assertEqual(w8, [(0, 8)])

        # Multi-window with N=16
        w16 = manager.partition_frames(16)
        self.assertGreater(len(w16), 1)
        # Verify coverage and overlap
        self.assertEqual(w16[0][0], 0)
        self.assertEqual(w16[-1][1], 16)
        for i in range(len(w16) - 1):
            overlap = w16[i][1] - w16[i + 1][0]
            self.assertGreaterEqual(overlap, 3, f"Window {i} to {i+1} overlap {overlap} < 3")

        # Multi-window with N=24
        w24 = manager.partition_frames(24)
        self.assertEqual(w24[0][0], 0)
        self.assertEqual(w24[-1][1], 24)
        for i in range(len(w24) - 1):
            overlap = w24[i][1] - w24[i + 1][0]
            self.assertGreaterEqual(overlap, 3)

    def test_umeyama_camera_transformation_invariance(self):
        # Synthetic camera centers
        np.random.seed(123)
        c_src = np.random.randn(6, 3)

        true_scale = 1.45
        theta = 0.5
        R_true = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1]
        ])
        t_true = np.array([3.0, -2.0, 1.5])

        c_dst = true_scale * (c_src @ R_true.T) + t_true

        s_est, R_est, t_est = umeyama_alignment(c_src, c_dst)

        self.assertAlmostEqual(s_est, true_scale, places=6)
        np.testing.assert_allclose(R_est, R_true, atol=1e-6)
        np.testing.assert_allclose(t_est, t_true, atol=1e-6)

        # Test camera extrinsics update formula:
        # Camera center: C = -R_cam.T @ t_cam
        # Transformed camera center must equal: s * R_est @ C + t_est
        R_cam = np.eye(3)
        t_cam = np.array([1.0, 2.0, -3.0])
        C_orig = -R_cam.T @ t_cam

        R_cam_new = R_cam @ R_est.T
        t_cam_new = s_est * t_cam - (R_cam @ R_est.T @ t_est)
        C_new = -R_cam_new.T @ t_cam_new

        C_expected = s_est * (R_est @ C_orig) + t_est
        np.testing.assert_allclose(C_new, C_expected, atol=1e-6)

    def test_surface_connectivity_hole_repair(self):
        # Create a simple planar mesh with a hexagonal hole
        # Outer ring of 6 vertices, 6 triangles connected to boundary
        theta = np.linspace(0, 2 * np.pi, 7)[:-1]
        radius = 1.0
        boundary_verts = np.stack([radius * np.cos(theta), radius * np.sin(theta), np.zeros_like(theta)], axis=1)

        # Outer box vertices
        box_verts = np.array([
            [-2.0, -2.0, 0.0],
            [ 2.0, -2.0, 0.0],
            [ 2.0,  2.0, 0.0],
            [-2.0,  2.0, 0.0],
        ])

        all_verts = np.vstack([box_verts, boundary_verts])
        # Build triangles connecting outer box to hexagonal hole
        # For this test, just build a cylinder/cone with open base to test loop closure
        cone = o3d.geometry.TriangleMesh.create_cone(radius=1.0, height=2.0, resolution=8)
        # Remove base triangles to leave an open 8-vertex loop
        triangles = np.asarray(cone.triangles)
        # Keep only side triangles (those not having center base vertex)
        verts = np.asarray(cone.vertices)
        z_vals = verts[triangles, 2]
        side_tris = triangles[np.any(z_vals > 0.01, axis=1)]
        cone_open = o3d.geometry.TriangleMesh()
        cone_open.vertices = cone.vertices
        cone_open.triangles = o3d.utility.Vector3iVector(side_tris)
        cone_open.remove_unreferenced_vertices()

        orig_tri_count = len(cone_open.triangles)
        orig_vert_count = len(cone_open.vertices)

        # Repair surface connectivity
        repaired, meta = SurfaceConnectivityMesher.repair_surface_connectivity(cone_open, max_hole_vertices=12)

        self.assertGreater(meta["holes_filled"], 0)
        self.assertGreater(len(repaired.triangles), orig_tri_count)
        self.assertEqual(len(repaired.vertices), orig_vert_count + 1)  # 1 centroid added

    def test_multiframe_dense_point_cloud(self):
        # Synthetic 2-frame depth and extrinsics
        depth = np.ones((2, 16, 16, 1), dtype=np.float32) * 2.0
        ext = np.zeros((2, 3, 4), dtype=np.float64)
        ext[0, :, :3] = np.eye(3)
        ext[1, :, :3] = np.eye(3)
        ext[1, 0, 3] = 0.2  # 0.2 baseline
        intr = np.zeros((2, 3, 3), dtype=np.float64)
        for i in range(2):
            intr[i] = np.array([[20.0, 0, 8.0], [0, 20.0, 8.0], [0, 0, 1.0]])
        rgbs = np.full((2, 16, 16, 3), 128, dtype=np.uint8)

        cloud_data = MultiFrameDensePointCloud.construct_clouds(
            depth_maps=depth,
            extrinsics=ext,
            intrinsics=intr,
            frame_rgbs=rgbs,
            orientation_class="forward_corridor"
        )

        self.assertEqual(cloud_data["raw_valid_count"], 16 * 16 * 2)
        self.assertGreater(cloud_data["fused_count"], 0)
        self.assertTrue(len(cloud_data["pcd_fused"].points) > 0)
        self.assertTrue(len(cloud_data["pcd_fused"].normals) > 0)


if __name__ == "__main__":
    unittest.main()
