"""F.O.R.G.E — Sprint 7C: Remaining Scenes Reconstruction Verification Tests.

Verifies:
1. Deliverable Completeness for campus_0049, forest_road, river_bridge.
2. Geometric Integrity: Valid non-degenerate bounds, vertex/triangle counts.
3. Texture Deliverables: Multi-material OBJ, unified atlas OBJ, atlas PNG (4096x4096), photographic PLY.
4. Preview Perspectives: Top, oblique, closeup, and bridge-focused renders.
5. Diagnostic Artifacts: Trajectory diagnostic, depth confidence preview, per-frame PCDs, diagnostics JSON.
6. Acceptance Gates: All 4 gates evaluated, Desert Road marked NOT ACCEPTED.
"""

import json
import unittest
from pathlib import Path
from PIL import Image
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSprint7CRemainingScenes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.out_root = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7c_remaining_scenes"
        cls.scenes = ["campus_0049", "forest_road", "river_bridge"]

    def test_output_root_and_scene_directories_exist(self):
        """Verifies that the output directory and all three scene subdirectories exist."""
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

    def test_river_bridge_additional_deliverables(self):
        """Verifies river_bridge has bridge-focused render view."""
        bridge_render = self.out_root / "river_bridge" / "render_view_bridge_focused.png"
        self.assertTrue(bridge_render.is_file(), "Missing render_view_bridge_focused.png for river_bridge")
        self.assertGreater(bridge_render.stat().st_size, 0)

    def test_per_frame_pcds_exist_and_rendered(self):
        """Verifies that per_frame_pcds directory contains PLYs and preview images for all frames."""
        expected_frames = {
            "campus_0049": 8,
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
            "forest_road": 250000,
            "river_bridge": 100000
        }
        for s, min_v in min_verts.items():
            for m_name in ["surface_mesh_repaired.ply", "surface_mesh_photographic.ply"]:
                mesh_p = self.out_root / s / m_name
                mesh = o3d.io.read_triangle_mesh(str(mesh_p))
                self.assertGreater(len(mesh.vertices), min_v, f"{s}/{m_name} has too few vertices: {len(mesh.vertices)}")
                self.assertGreater(len(mesh.triangles), min_v, f"{s}/{m_name} has too few triangles: {len(mesh.triangles)}")
                bbox = mesh.get_axis_aligned_bounding_box()
                extent = bbox.get_max_bound() - bbox.get_min_bound()
                self.assertTrue((extent > 0.05).all(), f"{s}/{m_name} has degenerate extents: {extent}")

    def test_texture_atlases_valid_png(self):
        """Verifies texture atlas resolution is 4096x4096 and readable."""
        for s in self.scenes:
            atlas_p = self.out_root / s / "texture_atlas.png"
            with Image.open(atlas_p) as img:
                self.assertEqual(img.size, (4096, 4096), f"Unexpected atlas size for {s}: {img.size}")

    def test_master_report_and_combined_sheet(self):
        """Verifies that all_scenes_comparison_contact_sheet.png and sprint7c_master_report.json exist and validate."""
        sheet_p = self.out_root / "all_scenes_comparison_contact_sheet.png"
        self.assertTrue(sheet_p.is_file(), "Missing all_scenes_comparison_contact_sheet.png")
        self.assertGreater(sheet_p.stat().st_size, 10000)

        master_json_p = self.out_root / "sprint7c_master_report.json"
        self.assertTrue(master_json_p.is_file(), "Missing sprint7c_master_report.json")
        with open(master_json_p, "r") as f:
            data = json.load(f)
        self.assertIn("scenes", data)
        for s in self.scenes:
            self.assertIn(s, data["scenes"])
        self.assertEqual(data["desert_road_status"]["status"], "NOT ACCEPTED")
        self.assertIn("acceptance_gates", data)
        self.assertEqual(data["acceptance_gates"]["desert_road"]["gate_c_visual"], "FAIL (fan-shaped stepped shoulder surfaces)")


if __name__ == "__main__":
    unittest.main()
