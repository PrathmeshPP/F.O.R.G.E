"""F.O.R.G.E — Sprint 7: Single-Window Reconstruction Stabilization Unit Tests.

Covers:
1. Deliverable Completeness: Verifies that every scene has unified PCDs, surface meshes, textured OBJs, MTLs, atlases, and metadata.
2. Desert Road Dual Candidates: Verifies existence and validity of both static clean highway and vehicle-stabilized candidates.
3. Diagnostic Artifacts: Verifies trajectory plots, depth previews, per-frame PCDs, and diagnostic JSONs.
4. Mesh Topology and Validity: Verifies all meshes parse cleanly in Open3D with expected geometric complexity.
5. Master Report Integrity: Verifies sprint7_master_report.json and sprint7_diagnostics_report.json.
"""

import unittest
import json
from pathlib import Path
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSprint7SingleWindowStabilization(unittest.TestCase):

    def setUp(self):
        self.out_root = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7_single_window_stabilization"
        self.scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]

    def test_all_four_scene_directories_exist(self):
        """Verifies that all 4 scene directories exist and are populated."""
        self.assertTrue(self.out_root.is_dir(), "sprint7_single_window_stabilization directory must exist")
        for s in self.scenes:
            s_dir = self.out_root / s
            self.assertTrue(s_dir.is_dir(), f"Scene directory for {s} must exist")

    def test_scene_deliverables_complete(self):
        """Verifies that each scene contains all required 3D model and texture deliverables."""
        required_files = [
            "point_cloud_unfused.ply",
            "point_cloud_fused.ply",
            "surface_mesh_repaired.ply",
            "surface_mesh_repaired.obj",
            "multiview_textured_mesh.obj",
            "multiview_textured_mesh.mtl",
            "textured_mesh.obj",
            "textured_mesh.mtl",
            "texture_atlas.png",
            "surface_mesh_photographic.ply",
            "candidate_metadata.json",
            "comparison_contact_sheet.png",
            "source_frames_contact_sheet.png",
            "camera_trajectory_diagnostic.png",
            "depth_confidence_preview.png"
        ]

        for s in self.scenes:
            s_dir = self.out_root / s
            for fn in required_files:
                f_path = s_dir / fn
                self.assertTrue(f_path.is_file(), f"Missing required file: {s}/{fn}")
                self.assertGreater(f_path.stat().st_size, 0, f"File {s}/{fn} is empty")

    def test_desert_road_dual_candidates(self):
        """Verifies that Desert Road has both static clean and vehicle-stabilized candidates."""
        dr_dir = self.out_root / "desert_road"
        for fn in [
            "point_cloud_static_clean.ply",
            "point_cloud_vehicle_stabilized.ply",
            "surface_mesh_static_clean.ply",
            "surface_mesh_static_clean.obj",
            "surface_mesh_vehicle_stabilized.ply",
            "surface_mesh_vehicle_stabilized.obj"
        ]:
            f_path = dr_dir / fn
            self.assertTrue(f_path.is_file(), f"Missing Desert Road candidate file: {fn}")
            self.assertGreater(f_path.stat().st_size, 0, f"Desert Road file {fn} is empty")

    def test_meshes_parse_and_valid_in_open3d(self):
        """Verifies that all 4 meshes parse in Open3D with high geometric fidelity."""
        min_verts = {
            "campus_0049": 100000,
            "desert_road": 70000,
            "forest_road": 300000,
            "river_bridge": 100000
        }
        for s, min_v in min_verts.items():
            mesh_p = self.out_root / s / "surface_mesh_repaired.ply"
            mesh = o3d.io.read_triangle_mesh(str(mesh_p))
            self.assertGreater(len(mesh.vertices), min_v, f"Mesh {s} has too few vertices: {len(mesh.vertices)}")
            self.assertGreater(len(mesh.triangles), min_v * 1.5, f"Mesh {s} has too few triangles")
            bbox = mesh.get_axis_aligned_bounding_box()
            extent = bbox.get_max_bound() - bbox.get_min_bound()
            self.assertTrue((extent > 0.1).all(), f"Mesh {s} has degenerate extent: {extent}")

    def test_texture_atlases_valid_on_disk(self):
        """Verifies that texture atlases exist and are valid PNG images."""
        expected_resolutions = {
            "campus_0049": (4096, 4096),
            "desert_road": (2048, 2048),
            "forest_road": (4096, 4096),
            "river_bridge": (4096, 4096)
        }
        from PIL import Image
        for s, (exp_w, exp_h) in expected_resolutions.items():
            png_p = self.out_root / s / "texture_atlas.png"
            self.assertTrue(png_p.is_file(), f"Missing texture atlas: {s}")
            with Image.open(png_p) as im:
                self.assertEqual(im.size, (exp_w, exp_h), f"Atlas {s} has unexpected size: {im.size}")

    def test_master_report_and_diagnostics_integrity(self):
        """Verifies that sprint7_master_report.json and sprint7_diagnostics_report.json document all 4 scenes."""
        master_p = self.out_root / "sprint7_master_report.json"
        diag_p = self.out_root / "sprint7_diagnostics_report.json"

        self.assertTrue(master_p.is_file(), "sprint7_master_report.json must exist")
        self.assertTrue(diag_p.is_file(), "sprint7_diagnostics_report.json must exist")

        with open(master_p) as f:
            data = json.load(f)
        self.assertEqual(len(data["scenes_processed"]), 4)
        for s in self.scenes:
            self.assertIn(s, data["scenes"])
            scene_info = data["scenes"][s]
            self.assertIn("point_clouds", scene_info)
            self.assertIn("surface_mesh", scene_info)
            self.assertIn("texture_deliverables", scene_info)
            self.assertIn("preview_renders", scene_info)

        with open(diag_p) as f:
            diag_data = json.load(f)
        for s in self.scenes:
            self.assertIn(s, diag_data)
            self.assertIn("pairwise_residuals", diag_data[s])
            self.assertIn("camera_calibration", diag_data[s])

    def test_desert_road_multiview_support_filter(self):
        """Verifies that 100% of points in Desert Road fused point cloud have multi-view support >= 2."""
        import numpy as np
        from src.reconstruction.point_cloud import filter_by_multiview_support

        pcd_p = self.out_root / "desert_road" / "point_cloud_fused.ply"
        pcd = o3d.io.read_point_cloud(str(pcd_p))
        pts = np.asarray(pcd.points)
        self.assertGreater(len(pts), 50000, "Fused PCD must have substantial point count")

        pred_dir = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions"
        depth_data = np.load(pred_dir / "depth_predictions.npz")
        poses_data = np.load(pred_dir / "camera_poses.npz")
        d_maps = np.squeeze(depth_data["depth"])
        c_maps = np.squeeze(depth_data["confidence"])
        ext = poses_data["extrinsics"]
        intr = poses_data["intrinsics"]

        _, _, support_counts = filter_by_multiview_support(
            pts, np.zeros_like(pts), ext, intr, d_maps, c_maps,
            min_camera_support=1, rtol=0.08, min_confidence=1.5, return_support_counts=True
        )

        unsupported_count = int(np.sum(support_counts < 2))
        self.assertEqual(unsupported_count, 0, f"Found {unsupported_count} unsupported single-camera fringe points in fused cloud")

    def test_desert_road_elevation_continuity_and_step_discontinuity(self):
        """Numerically validates that road/shoulder elevation is smooth with no stepped terraces (>10mm)."""
        import numpy as np
        pcd_p = self.out_root / "desert_road" / "point_cloud_fused.ply"
        pcd = o3d.io.read_point_cloud(str(pcd_p))
        pts = np.asarray(pcd.points)

        # Shoulder plane fit on left shoulder: X in [-0.6, -0.2], Z in [-1.0, 1.0]
        mask_left = (pts[:, 0] >= -0.6) & (pts[:, 0] <= -0.2) & (pts[:, 2] >= -1.0) & (pts[:, 2] <= 1.0)
        left_pts = pts[mask_left]
        self.assertGreater(len(left_pts), 10000)

        A = np.c_[left_pts[:, 0], left_pts[:, 2], np.ones(len(left_pts))]
        coeff, _, _, _ = np.linalg.lstsq(A, left_pts[:, 1], rcond=None)
        pred_y = A @ coeff
        diff_mm = np.abs(left_pts[:, 1] - pred_y) * 1000.0

        mean_err = float(np.mean(diff_mm))
        p90_err = float(np.percentile(diff_mm, 90))
        self.assertLess(mean_err, 4.0, f"Mean shoulder plane error {mean_err:.2f}mm exceeds 4mm")
        self.assertLess(p90_err, 8.0, f"90th percentile shoulder plane error {p90_err:.2f}mm exceeds 8mm")

        # Test step discontinuities between adjacent slices along Z
        z_slices = np.linspace(-1.0, 1.0, 11)
        mean_residuals = []
        for zi in range(len(z_slices) - 1):
            sub = (left_pts[:, 2] >= z_slices[zi]) & (left_pts[:, 2] < z_slices[zi + 1])
            if np.sum(sub) > 50:
                mean_residuals.append(float(np.mean(left_pts[sub, 1] - pred_y[sub]) * 1000.0))

        step_jumps = np.abs(np.diff(mean_residuals))
        max_step = float(np.max(step_jumps)) if len(step_jumps) > 0 else 0.0
        self.assertLess(max_step, 10.0, f"Step discontinuity between adjacent corridor slices {max_step:.2f}mm exceeds 10mm")

    def test_desert_road_candidate_a_dynamic_exclusion(self):
        """Verifies Candidate A has zero blue vehicle points situated on highway."""
        import numpy as np
        mesh_p = self.out_root / "desert_road" / "surface_mesh_photographic.ply"
        mesh = o3d.io.read_triangle_mesh(str(mesh_p))
        verts = np.asarray(mesh.vertices)
        cols = np.asarray(mesh.vertex_colors)

        # Vehicle search zone: Z in [0.1, 0.8], X in [-0.2, 0.4], Y > 0.03
        veh_zone = (verts[:, 1] > 0.03) & (verts[:, 0] > -0.2) & (verts[:, 0] < 0.4) & (verts[:, 2] > 0.1) & (verts[:, 2] < 0.8)
        veh_cols = cols[veh_zone]

        if len(veh_cols) > 0:
            is_blue = (veh_cols[:, 2] > veh_cols[:, 0] + 0.15) & (veh_cols[:, 2] > veh_cols[:, 1] + 0.10)
            blue_count = int(is_blue.sum())
            self.assertEqual(blue_count, 0, f"Found {blue_count} blue vehicle vertices in Candidate A static highway")

    def test_desert_road_candidate_b_vehicle_stabilization(self):
        """Verifies Candidate B contains vehicle at Frame 0 reference position with no duplicate ghosts."""
        import numpy as np
        mesh_p = self.out_root / "desert_road" / "surface_mesh_vehicle_stabilized_photographic.ply"
        mesh = o3d.io.read_triangle_mesh(str(mesh_p))
        verts = np.asarray(mesh.vertices)
        cols = np.asarray(mesh.vertex_colors)

        # Vehicle at reference Frame 0 position (Z in [0.1, 0.8], X in [-0.2, 0.4], Y > 0.03)
        ref_zone = (verts[:, 1] > 0.03) & (verts[:, 0] > -0.2) & (verts[:, 0] < 0.4) & (verts[:, 2] > 0.1) & (verts[:, 2] < 0.8)
        self.assertGreater(np.sum(ref_zone), 500, "Candidate B must contain reconstructed vehicle at Frame 0 position")

        # Highway forward corridor: Z < 0.0. There must be ZERO duplicate vehicle instances
        fwd_corridor = (verts[:, 1] > 0.03) & (verts[:, 0] > -0.2) & (verts[:, 0] < 0.4) & (verts[:, 2] < 0.0)
        fwd_cols = cols[fwd_corridor]
        if len(fwd_cols) > 0:
            is_ghost_blue = (fwd_cols[:, 2] > fwd_cols[:, 0] + 0.15) & (fwd_cols[:, 2] > fwd_cols[:, 1] + 0.10)
            ghost_count = int(is_ghost_blue.sum())
            self.assertEqual(ghost_count, 0, f"Found {ghost_count} duplicate ghost vehicle vertices ahead on highway")

    def test_acceptance_test_artifacts_exist(self):
        """Verifies that all Sprint 7B audit and acceptance artifacts exist on disk."""
        acc_dir = self.out_root / "desert_road" / "acceptance_tests"
        self.assertTrue(acc_dir.is_dir())
        for fn in [
            "matched_before_after_acceptance_comparison.png",
            "point_cloud_layer_forensics.png",
            "cross_section_shoulder_analysis.png",
            "separate_geometry_vs_textured_oblique.png"
        ]:
            f = acc_dir / fn
            self.assertTrue(f.is_file(), f"Missing acceptance artifact: {fn}")
            self.assertGreater(f.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()

