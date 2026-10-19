"""F.O.R.G.E — Automated Test Suite for Geospatial Telemetry & Umeyama–RANSAC 7-DoF Alignment.

Validates:
1. Exact recovery of known synthetic 7-DoF transformation parameters (scale, rotation, translation).
2. RANSAC gross outlier rejection on contaminated correspondence sets.
3. Degenerate and edge-case handling (insufficient points, collinear points, non-finite inputs).
4. Geodetic WGS-84 to local ENU coordinate conversion and millimeter-level round-trip accuracy.
5. Strict consistency of transformed camera extrinsics and 3D geometry (projection ray invariance).
6. Real DJI video telemetry extraction and frame timestamp synchronization.
"""

import math
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d

from src.geo import (
    GPSTelemetryRecord,
    extract_dji_telemetry,
    match_telemetry_to_frame_timestamps,
    geodetic_to_enu,
    enu_to_geodetic,
    umeyama_alignment,
    umeyama_ransac_alignment,
    apply_similarity_transform,
    transform_camera_poses,
    transform_mesh
)


class TestGeoAlignment(unittest.TestCase):

    def setUp(self):
        # Set deterministic seed for repeatable testing
        np.random.seed(42)

    def _create_random_rotation(self) -> np.ndarray:
        """Generates a valid orthonormal 3D rotation matrix in SO(3)."""
        axis = np.random.randn(3)
        axis /= np.linalg.norm(axis)
        angle = np.random.uniform(0.1, math.pi - 0.1)
        K = np.array([
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)
        return R

    def test_synthetic_7dof_transformation_recovery(self):
        """Verify exact recovery of known scale, rotation, and translation within numerical tolerance."""
        true_scale = 87.5
        true_R = self._create_random_rotation()
        true_t = np.array([12.345, -67.890, 45.123], dtype=np.float64)

        # 10 non-collinear 3D points
        src_pts = np.random.uniform(-5.0, 5.0, size=(10, 3)).astype(np.float64)
        # Transform: dst = s * R * src + t
        dst_pts = true_scale * (src_pts @ true_R.T) + true_t

        est_scale, est_R, est_t = umeyama_alignment(src_pts, dst_pts)

        # Verify numerical tolerances (1e-6)
        self.assertAlmostEqual(est_scale, true_scale, places=6)
        np.testing.assert_allclose(est_R, true_R, atol=1e-6)
        np.testing.assert_allclose(est_t, true_t, atol=1e-6)

        # Residuals must be practically zero
        pred_dst = est_scale * (src_pts @ est_R.T) + est_t
        res = np.linalg.norm(dst_pts - pred_dst, axis=1)
        self.assertLess(float(np.max(res)), 1e-6)

    def test_ransac_outlier_rejection(self):
        """Verify that RANSAC rejects gross outliers and computes accurate fit from inliers."""
        true_scale = 50.0
        true_R = self._create_random_rotation()
        true_t = np.array([-10.0, 20.0, 5.0], dtype=np.float64)

        n_inliers = 15
        n_outliers = 5
        n_total = n_inliers + n_outliers

        src_pts = np.random.uniform(-10.0, 10.0, size=(n_total, 3)).astype(np.float64)
        dst_pts = true_scale * (src_pts @ true_R.T) + true_t

        # Add mild noise (0.1m) to inliers
        dst_pts[:n_inliers] += np.random.normal(0.0, 0.1, size=(n_inliers, 3))

        # Add gross outliers (50 - 100m error) to the last 5 points
        outlier_offsets = np.random.uniform(50.0, 100.0, size=(n_outliers, 3))
        dst_pts[n_inliers:] += outlier_offsets

        res = umeyama_ransac_alignment(
            src_pts=src_pts,
            dst_pts=dst_pts,
            inlier_threshold_m=2.0,
            max_iterations=2000,
            random_seed=42
        )

        self.assertTrue(res.success)
        self.assertGreaterEqual(res.num_inliers, n_inliers)
        # All 5 gross outliers must be marked False in inlier mask
        self.assertFalse(any(res.inlier_mask[n_inliers:]))
        # Recovered scale should be within 1% of true scale
        self.assertAlmostEqual(res.scale, true_scale, delta=0.5)
        self.assertLess(res.rmse_m, 1.0)

    def test_insufficient_and_degenerate_correspondences(self):
        """Verify graceful failure on <3 points, collinear points, or NaN inputs."""
        # 1. Less than 3 points
        pts_2 = np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float64)
        res_insufficient = umeyama_ransac_alignment(pts_2, pts_2)
        self.assertFalse(res_insufficient.success)
        self.assertIn("Insufficient", res_insufficient.status_message)

        # 2. Degenerate collinear points (all lying on the same line)
        collinear_src = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3]], dtype=np.float64)
        collinear_dst = np.array([[0, 0, 0], [2, 2, 2], [4, 4, 4], [6, 6, 6]], dtype=np.float64)
        # Should not crash; handles rank-deficiency safely
        res_collinear = umeyama_ransac_alignment(collinear_src, collinear_dst)
        self.assertIsNotNone(res_collinear)

        # 3. Non-finite values
        nan_src = np.array([[0, 0, np.nan], [1, 2, 3], [4, 5, 6]], dtype=np.float64)
        res_nan = umeyama_ransac_alignment(nan_src, collinear_dst[:3])
        self.assertFalse(res_nan.success)

    def test_geodetic_to_enu_roundtrip(self):
        """Verify WGS-84 geodetic to local ENU conversion and millimeter-level round-trip closure."""
        ref_lat = 23.1518528
        ref_lon = 72.8823588
        ref_alt = 307.854

        # Test target 250m East, 180m North, -15m Down
        tgt_lat = 23.153478
        tgt_lon = 72.884801
        tgt_alt = 292.854

        e, n, u = geodetic_to_enu(tgt_lat, tgt_lon, tgt_alt, ref_lat, ref_lon, ref_alt)
        # Origin must map to [0, 0, 0]
        e0, n0, u0 = geodetic_to_enu(ref_lat, ref_lon, ref_alt, ref_lat, ref_lon, ref_alt)
        self.assertAlmostEqual(e0, 0.0, places=4)
        self.assertAlmostEqual(n0, 0.0, places=4)
        self.assertAlmostEqual(u0, 0.0, places=4)

        # Round-trip back to geodetic
        lat_back, lon_back, alt_back = enu_to_geodetic(e, n, u, ref_lat, ref_lon, ref_alt)
        self.assertAlmostEqual(lat_back, tgt_lat, places=6)
        self.assertAlmostEqual(lon_back, tgt_lon, places=6)
        self.assertAlmostEqual(alt_back, tgt_alt, delta=0.001)  # 1mm tolerance

    def test_camera_pose_and_mesh_transformation_consistency(self):
        """
        Verify that transforming camera extrinsics preserves ray projection:
        A 3D world point projected through the original camera must project to the
        EXACT same pixel coordinates through the transformed camera!
        """
        scale = 25.0
        R_sim = self._create_random_rotation()
        t_sim = np.array([5.0, -15.0, 30.0], dtype=np.float64)

        # Original camera
        R_cam = self._create_random_rotation()
        t_cam = np.array([0.5, 1.2, -3.0], dtype=np.float64)
        ext_orig = np.eye(4)
        ext_orig[:3, :3] = R_cam
        ext_orig[:3, 3] = t_cam

        intr = np.array([
            [500.0, 0.0, 256.0],
            [0.0, 500.0, 256.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Original world point
        C_w_orig = -R_cam.T @ t_cam
        p_w_orig = C_w_orig + R_cam.T @ np.array([0.2, -0.3, 2.5])  # In front of camera (Z=2.5)

        # Transform camera and point
        ext_aligned = transform_camera_poses(ext_orig[None, ...], scale, R_sim, t_sim)[0]
        p_w_aligned = apply_similarity_transform(p_w_orig[None, :], scale, R_sim, t_sim)[0]

        # Verify new camera center matches transformed camera center
        C_w_aligned_expected = scale * (R_sim @ C_w_orig) + t_sim
        C_w_aligned_actual = -ext_aligned[:3, :3].T @ ext_aligned[:3, 3]
        np.testing.assert_allclose(C_w_aligned_actual, C_w_aligned_expected, atol=1e-5)

        # Project original point into original camera
        p_c_orig = ext_orig[:3, :3] @ p_w_orig + ext_orig[:3, 3]
        u_orig = intr[0, 0] * p_c_orig[0] / p_c_orig[2] + intr[0, 2]
        v_orig = intr[1, 1] * p_c_orig[1] / p_c_orig[2] + intr[1, 2]

        # Project aligned point into aligned camera
        p_c_aligned = ext_aligned[:3, :3] @ p_w_aligned + ext_aligned[:3, 3]
        u_aligned = intr[0, 0] * p_c_aligned[0] / p_c_aligned[2] + intr[0, 2]
        v_aligned = intr[1, 1] * p_c_aligned[1] / p_c_aligned[2] + intr[1, 2]

        # Pixel projections MUST be identical
        self.assertAlmostEqual(u_orig, u_aligned, places=5)
        self.assertAlmostEqual(v_orig, v_aligned, places=5)

    def test_dji_real_telemetry_extraction(self):
        """Verify telemetry extraction from the real Campus 0049 DJI Air 3S video."""
        video_path = Path(r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4")
        if not video_path.is_file():
            self.skipTest(f"Video file not present at {video_path}")

        records = extract_dji_telemetry(str(video_path))
        self.assertGreater(len(records), 900)

        # Verify GPS coordinates are in reasonable Gandhinagar/Ahmedabad range
        rec0 = records[0]
        self.assertAlmostEqual(rec0.latitude_deg, 23.1518, delta=0.01)
        self.assertAlmostEqual(rec0.longitude_deg, 72.8823, delta=0.01)
        self.assertGreater(rec0.altitude_m, 200.0)

        # Match with frame timestamps
        target_timestamps = [0.0, 1.001, 2.002, 3.003, 4.004, 5.005, 6.006, 7.007, 8.008, 9.009]
        matched = match_telemetry_to_frame_timestamps(records, target_timestamps)
        self.assertEqual(len(matched), len(target_timestamps))
        self.assertTrue(all(m is not None for m in matched))


    def test_sidecar_srt_telemetry_extraction(self):
        """Verify parsing of DJI companion .SRT sidecar subtitle records."""
        import tempfile
        srt_content = (
            "1\n00:00:01,000 --> 00:00:01,033\n"
            "F/2.8, 1/120, ISO 100\n"
            "[iso : 100] [shutter : 1/120.0] [fnum : 280] [ev : 0] [ct : 5400] [color_md : default] "
            "[focal_len : 240] [dzoom_ratio: 100%]\n"
            "GPS (72.882358, 23.151852, 307.85) BAROMETER: 308.2\n\n"
            "2\n00:00:02,000 --> 00:00:02,033\n"
            "GPS (72.882410, 23.151910, 308.10) BAROMETER: 308.5\n\n"
            "3\n00:00:03,000 --> 00:00:03,033\n"
            "GPS (72.882480, 23.151980, 308.45) BAROMETER: 308.9\n"
        )
        with tempfile.NamedTemporaryFile("w", suffix=".SRT", delete=False, encoding="utf-8") as f:
            f.write(srt_content)
            tmp_srt = f.name

        try:
            records = extract_dji_telemetry(tmp_srt)
            self.assertEqual(len(records), 3)
            self.assertAlmostEqual(records[0].timestamp_sec, 1.0)
            self.assertAlmostEqual(records[0].latitude_deg, 23.151852)
            self.assertAlmostEqual(records[0].longitude_deg, 72.882358)
            self.assertAlmostEqual(records[0].altitude_m, 307.85)
            self.assertEqual(records[0].source_type, "dji_srt")
            self.assertAlmostEqual(records[1].timestamp_sec, 2.0)
            self.assertAlmostEqual(records[2].timestamp_sec, 3.0)
        finally:
            Path(tmp_srt).unlink(missing_ok=True)

    def test_trajectory_conditioning_classification(self):
        """Verify accurate conditioning classification for zero-motion, hover, and translational flights."""
        from src.geo.batch_alignment import assess_trajectory_conditioning

        # 1. Zero motion (stationary hover)
        src_zero = np.zeros((5, 3))
        dst_zero = np.zeros((5, 3))
        cond_zero = assess_trajectory_conditioning(src_zero, dst_zero)
        self.assertEqual(cond_zero.conditioning_verdict, "DEGENERATE_ZERO_MOTION")
        self.assertGreater(len(cond_zero.warnings), 0)

        # 2. Vertical hover descent (horizontal span 0.05m < 1.5m noise floor, vertical span 4.0m)
        src_hover = np.array([[0, 0, -i * 0.1] for i in range(10)], dtype=np.float64)
        dst_hover = np.array([[0.01 * np.sin(i), 0.01 * np.cos(i), -i * 0.4] for i in range(10)], dtype=np.float64)
        cond_hover = assess_trajectory_conditioning(src_hover, dst_hover, min_horizontal_baseline_m=1.5)
        self.assertEqual(cond_hover.conditioning_verdict, "WEAK_HORIZONTAL_BASELINE_HOVER")
        self.assertEqual(cond_hover.motion_type, "vertical_hover_descent")

        # 3. Well-constrained translational flight (span 25m)
        src_trans = np.array([[i * 1.0, i * 0.5, 0] for i in range(10)], dtype=np.float64)
        dst_trans = np.array([[i * 10.0, i * 5.0, 0] for i in range(10)], dtype=np.float64)
        cond_trans = assess_trajectory_conditioning(src_trans, dst_trans, min_horizontal_baseline_m=1.5)
        self.assertEqual(cond_trans.conditioning_verdict, "WELL_CONSTRAINED_TRANSLATION")
        self.assertEqual(len(cond_trans.warnings), 0)

    def test_batch_alignment_multi_video_isolation(self):
        """Verify multi-video batch execution, failure isolation, and report generation."""
        import tempfile
        from src.geo.batch_alignment import VideoAlignmentConfig, execute_batch_alignment

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir)

            # Create mock geometry
            mock_mesh_path = tmp_path / "mock_mesh.ply"
            mock_poses_path = tmp_path / "mock_poses.npz"
            mesh = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
            o3d.io.write_triangle_mesh(str(mock_mesh_path), mesh)

            extrinsics = np.stack([np.eye(4) for _ in range(5)], axis=0)
            for i in range(5):
                extrinsics[i, 0, 3] = -i * 1.0  # Centers at [i, 0, 0]
            np.savez_compressed(str(mock_poses_path), extrinsics=extrinsics, intrinsics=np.eye(3))

            # Case A: Missing telemetry file
            cfg_missing = VideoAlignmentConfig(
                video_id="test_missing_tel",
                display_name="Test Missing Telemetry",
                video_path=str(tmp_path / "nonexistent.mp4"),
                camera_poses_npz_path=str(mock_poses_path),
                mesh_ply_path=str(mock_mesh_path)
            )

            # Case B: Missing camera poses NPZ
            cfg_no_poses = VideoAlignmentConfig(
                video_id="test_no_poses",
                display_name="Test Missing Poses",
                video_path=str(tmp_path / "nonexistent.mp4"),
                camera_poses_npz_path=str(tmp_path / "nonexistent_poses.npz"),
                mesh_ply_path=str(mock_mesh_path)
            )

            # Execute batch runner
            batch_out = tmp_path / "batch_out"
            report = execute_batch_alignment([cfg_missing, cfg_no_poses], batch_out)

            # Verify batch completed without crashing
            self.assertEqual(report["total_videos_audited"], 2)
            self.assertEqual(report["total_aligned_successfully"], 0)
            self.assertTrue((batch_out / "batch_alignment_report.json").is_file())
            self.assertTrue((batch_out / "batch_alignment_summary.csv").is_file())

            res_a = report["results"][0]
            self.assertEqual(res_a["status_code"], "telemetry_source_missing")

            res_b = report["results"][1]
            self.assertEqual(res_b["status_code"], "poses_missing")

    def test_quality_gates_evaluation(self):
        """Verify multi-criteria quality gate evaluation against geometric and geospatial defects."""
        import tempfile
        from src.geo.quality_gates import evaluate_reconstruction_quality_gate

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir)

            # 1. Deficient mesh (< 5000 vertices) should FAIL geometry gate
            small_mesh_path = tmp_path / "small_mesh.ply"
            small_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=10)
            o3d.io.write_triangle_mesh(str(small_mesh_path), small_mesh)

            qg_small = evaluate_reconstruction_quality_gate(
                target_id="test_small",
                mesh_path=small_mesh_path
            )
            self.assertFalse(qg_small.passed)
            self.assertEqual(qg_small.overall_verdict, "FAIL")
            self.assertTrue(any("Insufficient vertex count" in f for f in qg_small.hard_failures))

            # 2. Geospatial orientation tilt defect check (R[2,2] > 0.85 indicates optical axis pointing into sky)
            dense_mesh_path = tmp_path / "dense_mesh.ply"
            dense_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=10.0, resolution=100)
            o3d.io.write_triangle_mesh(str(dense_mesh_path), dense_mesh)

            # Metadata with hover defect: R[2,2] = 0.956 (pointing straight up)
            meta_tilted = {
                "alignment_status": "UNRELIABLE_POORLY_CONSTRAINED",
                "trajectory_conditioning": {
                    "conditioning_verdict": "WEAK_HORIZONTAL_BASELINE_HOVER",
                    "horizontal_span_gps_m": 0.08
                },
                "similarity_transformation": {
                    "scale_factor_meters_per_unit": 63.7,
                    "rotation_matrix": [
                        [0.505, -0.815, -0.283],
                        [0.859, 0.505, 0.079],
                        [0.079, -0.283, 0.956]
                    ]
                },
                "fit_statistics": {
                    "residual_rmse_m": 0.18,
                    "inlier_ratio_pct": 100.0
                }
            }

            qg_tilted = evaluate_reconstruction_quality_gate(
                target_id="test_tilted",
                mesh_path=dense_mesh_path,
                alignment_metadata=meta_tilted
            )
            self.assertFalse(qg_tilted.passed)
            self.assertEqual(qg_tilted.overall_verdict, "FAIL")
            self.assertTrue(any("Collinear hover orientation defect" in f for f in qg_tilted.hard_failures))

            # 3. Clean flight corridor metadata
            meta_clean = {
                "alignment_status": "SUCCESS_METRIC_ALIGNED",
                "trajectory_conditioning": {
                    "conditioning_verdict": "WELL_CONSTRAINED_TRANSLATION",
                    "horizontal_span_gps_m": 76.5
                },
                "similarity_transformation": {
                    "scale_factor_meters_per_unit": 357.5,
                    "rotation_matrix": [
                        [-0.587, -0.019, 0.809],
                        [-0.768, 0.329, -0.549],
                        [-0.256, -0.944, -0.208]
                    ]
                },
                "fit_statistics": {
                    "residual_rmse_m": 0.236,
                    "inlier_ratio_pct": 100.0
                }
            }

            qg_clean = evaluate_reconstruction_quality_gate(
                target_id="test_clean",
                mesh_path=dense_mesh_path,
                alignment_metadata=meta_clean
            )
            self.assertTrue(qg_clean.passed)
            self.assertIn(qg_clean.overall_verdict, ["PASS", "PASS_WITH_WARNINGS"])


if __name__ == "__main__":
    unittest.main()

