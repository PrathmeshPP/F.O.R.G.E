"""F.O.R.G.E — Sprint 6 All Scenes Reconstruction & Quality Verification Unit Tests.

Covers:
1. Deliverable Completeness: Verifies that every scene has unified PCDs, surface meshes, textured OBJs, MTLs, atlases, and metadata.
2. Mesh Topology and Validity: Verifies that all 4 newly generated meshes parse cleanly in Open3D with expected geometric complexity.
3. Photographic Texture Assets: Verifies that texture atlases and material libraries are valid on disk.
4. Master Report Integrity: Verifies that sprint6_master_report.json documents all four scenes.
"""

import unittest
import json
from pathlib import Path
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSprint6AllScenesReconstruction(unittest.TestCase):

    def setUp(self):
        self.out_root = PROJECT_ROOT / "outputs/accuracy_experiment/all_scenes_quality_v1"
        self.scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]

    def test_all_four_scene_directories_exist(self):
        """Verifies that all 4 scene directories were created and populated."""
        self.assertTrue(self.out_root.is_dir(), "all_scenes_quality_v1 directory must exist")
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
            "comparison_contact_sheet.png"
        ]

        for s in self.scenes:
            s_dir = self.out_root / s
            for fn in required_files:
                f_path = s_dir / fn
                self.assertTrue(f_path.is_file(), f"Missing required file: {s}/{fn}")
                self.assertGreater(f_path.stat().st_size, 0, f"File {s}/{fn} is empty")

    def test_meshes_parse_and_valid_in_open3d(self):
        """Verifies that all 4 meshes parse in Open3D with high geometric fidelity."""
        min_verts = {
            "campus_0049": 100000,
            "desert_road": 70000,
            "forest_road": 300000,
            "river_bridge": 100000
        }

        for s in self.scenes:
            mesh_p = self.out_root / s / "surface_mesh_repaired.ply"
            mesh = o3d.io.read_triangle_mesh(str(mesh_p))
            self.assertGreater(len(mesh.vertices), min_verts[s], f"Scene {s} mesh vertices too low: {len(mesh.vertices)}")
            self.assertGreater(len(mesh.triangles), len(mesh.vertices), f"Scene {s} must have more triangles than vertices")

            bbox = mesh.get_axis_aligned_bounding_box()
            extents = bbox.get_extent()
            self.assertTrue(all(e > 0.1 for e in extents), f"Scene {s} bounding box extents invalid: {extents}")

    def test_sprint6_master_report_valid(self):
        """Verifies that sprint6_master_report.json contains all 4 scenes."""
        report_p = self.out_root / "sprint6_master_report.json"
        self.assertTrue(report_p.is_file(), "sprint6_master_report.json must exist")

        data = json.loads(report_p.read_text())
        self.assertEqual(len(data["scenes_processed"]), 4)
        for s in self.scenes:
            self.assertIn(s, data["scenes"])
            scene_data = data["scenes"][s]
            self.assertIn("point_clouds", scene_data)
            self.assertIn("surface_mesh", scene_data)
            self.assertIn("texture_deliverables", scene_data)
            self.assertIn("preview_renders", scene_data)


if __name__ == "__main__":
    unittest.main()
