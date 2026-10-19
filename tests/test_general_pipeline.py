"""Unit tests for F.O.R.G.E GeneralReconstructionPipeline and Adaptive Parameter Estimation."""

import unittest
import numpy as np
from src.reconstruction.pipeline import (
    PipelineConfig,
    assess_camera_trajectory,
    estimate_adaptive_parameters,
    GeneralReconstructionPipeline
)


class TestGeneralPipeline(unittest.TestCase):

    def test_trajectory_assessment(self):
        """Verify camera trajectory diagnostics and classification."""
        # 1. Low-baseline hover trajectory
        ext_hover = np.zeros((5, 4, 4), dtype=np.float64)
        for i in range(5):
            ext_hover[i, :3, :3] = np.eye(3)
            ext_hover[i, :3, 3] = [0.001 * i, 0.0, 0.0]
            ext_hover[i, 3, 3] = 1.0

        diag_hover = assess_camera_trajectory(ext_hover)
        self.assertEqual(diag_hover["motion_type"], "low_baseline_hover")
        self.assertTrue(len(diag_hover["warnings"]) > 0)

        # 2. Forward axial motion (vehicle driving)
        ext_forward = np.zeros((4, 4, 4), dtype=np.float64)
        for i in range(4):
            ext_forward[i, :3, :3] = np.eye(3)
            ext_forward[i, :3, 3] = [0.0, 0.0, -0.2 * i]
            ext_forward[i, 3, 3] = 1.0

        diag_forward = assess_camera_trajectory(ext_forward)
        self.assertEqual(diag_forward["motion_type"], "forward_axial_motion")

    def test_adaptive_parameter_estimation(self):
        """Verify scale-relative parameter calculation."""
        config = PipelineConfig(
            min_depth=0.1,
            max_depth=None,
            depth_percentile_cutoff=99.0,
            voxel_size="auto",
            target_voxels_per_extent=100
        )

        H, W = 8, 8
        depth_maps = np.linspace(1.0, 10.0, 2 * H * W).reshape((2, H, W)).astype(np.float32)
        conf = np.ones((2, H, W), dtype=np.float32) * 5.0
        extrinsics = np.tile(np.eye(4)[None, :, :], (2, 1, 1))
        intrinsics = np.array([[[100.0, 0.0, 4.0], [0.0, 100.0, 4.0], [0.0, 0.0, 1.0]]]*2)

        params = estimate_adaptive_parameters(depth_maps, conf, extrinsics, intrinsics, config)
        self.assertAlmostEqual(params["resolved_min_depth"], 0.1)
        self.assertGreater(params["resolved_max_depth"], 9.0)
        self.assertIsNotNone(params["resolved_voxel_size"])
        self.assertGreater(params["resolved_voxel_size"], 0.0)

    def test_conservative_confidence_threshold_preservation(self):
        """Verify that adaptive confidence NEVER exceeds 0.5 and does not amputate distant depth."""
        config = PipelineConfig(adaptive_confidence=True)
        H, W = 8, 8
        depth_maps = np.linspace(1.0, 10.0, 2 * H * W).reshape((2, H, W)).astype(np.float32)
        # Even with high median confidence of 10.0, the threshold should remain conservative (<= 0.5)
        conf = np.ones((2, H, W), dtype=np.float32) * 10.0
        extrinsics = np.tile(np.eye(4)[None, :, :], (2, 1, 1))
        intrinsics = np.array([[[100.0, 0.0, 4.0], [0.0, 100.0, 4.0], [0.0, 0.0, 1.0]]]*2)

        params = estimate_adaptive_parameters(depth_maps, conf, extrinsics, intrinsics, config)
        self.assertLessEqual(params["resolved_min_confidence"], 0.5)
        self.assertGreaterEqual(params["resolved_min_confidence"], 0.1)

    def test_disaster_readiness_evaluation(self):
        """Verify disaster-readiness assessment produces required warnings and unverified scale flag."""
        from src.reconstruction.pipeline import evaluate_disaster_readiness
        import open3d as o3d

        active_pts = np.array([[0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]], dtype=np.float64)
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(active_pts)
        mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2], [1, 3, 2]])

        traj = {"motion_type": "low_baseline_hover", "warnings": ["Low baseline"], "trajectory_extent": 0.05}
        depths = np.ones((1, 4, 4))
        conf = np.ones((1, 4, 4)) * 5.0

        eval_result = evaluate_disaster_readiness(
            active_points=active_pts,
            mesh=mesh,
            traj_diagnostics=traj,
            depth_maps=depths,
            confidences=conf,
            scene_classification="forest_canopy"
        )

        self.assertFalse(eval_result["scale_verification_status"]["is_metric_ground_truth_calibrated"])
        self.assertEqual(eval_result["uncertainty_and_occlusion"]["unobserved_ground_status"], "OCCLUDED_BY_CANOPY")
        self.assertEqual(eval_result["operational_readiness_verdict"]["metric_engineering_and_clearance"], "NOT_PERMITTED_WITHOUT_GCP")


if __name__ == "__main__":
    unittest.main()
