"""F.O.R.G.E — Sprint 7D: All-Scene Geometry Stabilization Verification Tests.

Verifies:
1. Deliverable Completeness across all 4 benchmark scenes (campus_0049, desert_road, forest_road, river_bridge).
2. Desert Road Dual Candidates: Candidate A static clean highway and Candidate B single-vehicle stabilized models.
3. Geometric Integrity: Valid non-degenerate bounds, vertex/triangle counts, Open3D topology.
4. Texture Deliverables: Multi-material OBJ, unified atlas OBJ, atlas PNG (4096x4096), photographic PLY.
5. Preview Perspectives: Top, oblique, closeup, candidate B views, and bridge-focused renders.
6. Diagnostic Artifacts: Trajectory diagnostics, depth confidence previews, per-frame PCDs, diagnostics JSON.
7. Master Reports: all_scenes_comparison_contact_sheet.png, all_scenes_defect_comparison.png, sprint7d_master_report.json.
"""

import json
import unittest
from pathlib import Path
from PIL import Image
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSprint7DAllSceneGeometryStabilization(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.out_root = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7d_all_scene_geometry_stabilization"
        cls.scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]

    def test_output_root_and_all_four_directories_exist(self):
        """Verifies that the output directory and all four scene subdirectories exist."""
        self.assertTrue(self.out_root.is_dir(), f"Missing output root: {self.out_root}")
        for s in self.scenes:
            s_dir = self.out_root / s
            self.assertTrue(s_dir.is_dir(), f"Missing scene directory for {s}")

    def test_scene_deliverables_complete(self):
        """Verifies that each scene contains all required 3D model, mesh, and texture files."""
        required_files = [
            "point_cloud_unfused.ply",
            "point_cloud_fused.ply",
            "surface_mesh_raw.ply",
            "surface_mesh_raw.obj",
            "surface_mesh_repaired.ply",
            "surface_mesh_repaired.obj",
            "multiview_textured_mesh.obj",
            "multiview_textured_mesh.mtl",
            "textured_mesh.obj",
            "textured_mesh.mtl",
            "texture_atlas.png",
            "surface_mesh_photographic.ply",
            "candidate_metadata.json",
            "scene_diagnostics.json",
            "processing.log",
            "source_frames_contact_sheet.png",
            "depth_confidence_preview.png",
            "camera_trajectory_diagnostic.png",
            "comparison_contact_sheet.png",
            "render_view_top.png",
            "render_view_oblique.png",
            "render_view_closeup.png"
        ]

        for s in self.scenes:
            s_dir = self.out_root / s
            for fn in required_files:
                f_path = s_dir / fn
                self.assertTrue(f_path.is_file(), f"Missing required file: {s}/{fn}")
                self.assertGreater(f_path.stat().st_size, 0, f"File is empty: {s}/{fn}")

    def test_desert_road_dual_candidates(self):
        """Verifies Desert Road has both static clean and vehicle-stabilized candidates."""
        dr_dir = self.out_root / "desert_road"
        for fn in [
            "point_cloud_static_clean.ply",
            "point_cloud_vehicle_stabilized.ply",
            "surface_mesh_static_clean.ply",
            "surface_mesh_static_clean.obj",
            "surface_mesh_vehicle_stabilized.ply",
            "surface_mesh_vehicle_stabilized.obj",
            "surface_mesh_vehicle_stabilized_photographic.ply",
            "render_view_candidate_b_top.png",
            "render_view_candidate_b_oblique.png",
            "render_view_candidate_b_closeup.png"
        ]:
            f_path = dr_dir / fn
            self.assertTrue(f_path.is_file(), f"Missing Desert Road deliverable: {fn}")
            self.assertGreater(f_path.stat().st_size, 0)

    def test_river_bridge_bridge_focused_view(self):
        """Verifies river_bridge has bridge-focused render view."""
        bridge_render = self.out_root / "river_bridge" / "render_view_bridge_focused.png"
        self.assertTrue(bridge_render.is_file(), "Missing render_view_bridge_focused.png for river_bridge")
        self.assertGreater(bridge_render.stat().st_size, 0)

    def test_per_frame_pcds_exist_and_rendered(self):
        """Verifies that per_frame_pcds directory contains PLYs and preview images for all frames."""
        expected_frames = {
            "campus_0049": 8,
            "desert_road": 8,
            "forest_road": 10,
            "river_bridge": 8
        }
        for s, n_frames in expected_frames.items():
            pcd_dir = self.out_root / s / "per_frame_pcds"
            self.assertTrue(pcd_dir.is_dir(), f"Missing per_frame_pcds dir for {s}")
            for i in range(n_frames):
                ply_f = pcd_dir / f"frame_{i:02d}.ply"
                png_f = pcd_dir / f"frame_{i:02d}_preview.png"
                self.assertTrue(ply_f.is_file(), f"Missing frame PLY: {s}/frame_{i:02d}.ply")
                self.assertTrue(png_f.is_file(), f"Missing frame preview: {s}/frame_{i:02d}_preview.png")

    def test_meshes_load_and_have_high_fidelity(self):
        """Verifies that repaired and photographic meshes parse in Open3D with expected topology."""
        min_verts = {
            "campus_0049": 100000,
            "desert_road": 40000,
            "forest_road": 250000,
            "river_bridge": 100000
        }
        for s, min_v in min_verts.items():
            for m_name in ["surface_mesh_repaired.ply", "surface_mesh_photographic.ply"]:
                mesh_p = self.out_root / s / m_name
                mesh = o3d.io.read_triangle_mesh(str(mesh_p))
                self.assertGreater(len(mesh.vertices), min_v, f"{s}/{m_name} has too few vertices: {len(mesh.vertices)}")
                self.assertGreater(len(mesh.triangles), min_v * 1.5, f"{s}/{m_name} has too few triangles")
                bbox = mesh.get_axis_aligned_bounding_box()
                extent = bbox.get_max_bound() - bbox.get_min_bound()
                self.assertTrue((extent > 0.05).all(), f"{s}/{m_name} has degenerate extents: {extent}")

    def test_texture_atlases_valid_png(self):
        """Verifies texture atlas resolution is 4096x4096 and readable."""
        for s in self.scenes:
            atlas_p = self.out_root / s / "texture_atlas.png"
            with Image.open(atlas_p) as img:
                self.assertEqual(img.size, (4096, 4096), f"Unexpected atlas size for {s}: {img.size}")

    def test_master_reports_and_comparison_sheets(self):
        """Verifies that master contact sheets and sprint7d_master_report.json exist and validate."""
        sheet_p = self.out_root / "all_scenes_comparison_contact_sheet.png"
        self.assertTrue(sheet_p.is_file(), "Missing all_scenes_comparison_contact_sheet.png")
        self.assertGreater(sheet_p.stat().st_size, 10000)

        defect_sheet_p = self.out_root / "all_scenes_defect_comparison.png"
        self.assertTrue(defect_sheet_p.is_file(), "Missing all_scenes_defect_comparison.png")
        self.assertGreater(defect_sheet_p.stat().st_size, 10000)

        master_json_p = self.out_root / "sprint7d_master_report.json"
        self.assertTrue(master_json_p.is_file(), "Missing sprint7d_master_report.json")
        with open(master_json_p, "r") as f:
            data = json.load(f)
        self.assertIn("scenes", data)
        for s in self.scenes:
            self.assertIn(s, data["scenes"])
        self.assertIn("acceptance_gates", data)
        for s in self.scenes:
            self.assertIn(s, data["acceptance_gates"])


if __name__ == "__main__":
    unittest.main()
