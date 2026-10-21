#!/usr/bin/env python3
"""
Unit and Regression Test Suite for Sprint 7E: Visual Acceptance Audit and Targeted Recovery
========================================================================================
Validates deliverable completeness, mesh integrity, texture atlas dimensions,
boundary jump statistics, and master reporting artifacts for Sprint 7E.
"""

import json
import unittest
from pathlib import Path
import numpy as np
import open3d as o3d
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_ROOT = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7e_visual_acceptance_audit"


class TestSprint7EVisualAcceptanceAudit(unittest.TestCase):

    def test_01_output_directories_exist(self):
        """Verify all four benchmark scene subdirectories exist."""
        self.assertTrue(OUTPUT_ROOT.is_dir(), f"Missing output root: {OUTPUT_ROOT}")
        for scene in ["campus_0049", "desert_road", "forest_road", "river_bridge"]:
            s_dir = OUTPUT_ROOT / scene
            self.assertTrue(s_dir.is_dir(), f"Missing scene directory: {s_dir}")

    def test_02_desert_road_dual_candidates(self):
        """Verify Candidate A and Candidate B files exist for Desert Road."""
        dr_dir = OUTPUT_ROOT / "desert_road"
        required = [
            "surface_mesh_static_clean.ply",
            "surface_mesh_vehicle_stabilized.ply",
            "surface_mesh_vehicle_stabilized_photographic.ply",
            "candidate_a_top.png",
            "candidate_a_oblique.png",
            "candidate_a_shoulder_closeup.png",
            "candidate_a_untextured.png",
            "candidate_b_top.png",
            "candidate_b_oblique.png",
            "candidate_b_vehicle_closeup.png",
            "candidate_b_untextured.png"
        ]
        for fname in required:
            fp = dr_dir / fname
            self.assertTrue(fp.is_file(), f"Missing Desert Road deliverable: {fname}")
            self.assertGreater(fp.stat().st_size, 1000, f"File too small: {fname}")

    def test_03_mesh_geometry_integrity(self):
        """Verify all reconstructed meshes load cleanly with valid vertices, triangles, and colors."""
        for scene in ["campus_0049", "desert_road", "forest_road", "river_bridge"]:
            ply_path = OUTPUT_ROOT / scene / "surface_mesh_photographic.ply"
            self.assertTrue(ply_path.is_file(), f"Missing photographic mesh: {ply_path}")
            mesh = o3d.io.read_triangle_mesh(str(ply_path))
            verts = np.asarray(mesh.vertices)
            tris = np.asarray(mesh.triangles)
            cols = np.asarray(mesh.vertex_colors)

            self.assertGreater(len(verts), 30000, f"{scene} vertex count too low: {len(verts)}")
            self.assertGreater(len(tris), 50000, f"{scene} triangle count too low: {len(tris)}")
            self.assertTrue(np.isfinite(verts).all(), f"{scene} has non-finite vertices")
            self.assertGreater(len(cols), 0, f"{scene} missing vertex colors")
            self.assertTrue((cols >= 0.0).all() and (cols <= 1.0).all(), f"{scene} invalid colors")

    def test_04_texture_atlas_properties(self):
        """Verify packed texture atlas PNG exists and has valid 4096x4096 resolution."""
        for scene in ["campus_0049", "desert_road", "forest_road", "river_bridge"]:
            atlas_p = OUTPUT_ROOT / scene / "texture_atlas.png"
            self.assertTrue(atlas_p.is_file(), f"Missing texture atlas: {atlas_p}")
            with Image.open(atlas_p) as im:
                self.assertEqual(im.size, (4096, 4096), f"{scene} unexpected atlas size: {im.size}")

    def test_05_desert_road_boundary_jump(self):
        """Verify Candidate A highway boundary jump metric is constrained (no severe saw-teeth)."""
        meta_p = OUTPUT_ROOT / "desert_road/candidate_metadata.json"
        self.assertTrue(meta_p.is_file(), "Missing desert road metadata")
        data = json.loads(meta_p.read_text())
        b_stats = data.get("boundary_jump_statistics", {})
        self.assertIn("mean_jump_mm", b_stats, "Missing mean_jump_mm in boundary stats")
        self.assertLess(b_stats["mean_jump_mm"], 50.0, f"Mean boundary jump too high: {b_stats['mean_jump_mm']} mm")

    def test_06_master_deliverables_exist(self):
        """Verify master contact sheets and JSON report exist and are non-empty."""
        contact_sheet = OUTPUT_ROOT / "all_scenes_acceptance_contact_sheet.png"
        defect_sheet = OUTPUT_ROOT / "all_scenes_defect_comparison.png"
        report_json = OUTPUT_ROOT / "sprint7e_master_report.json"

        self.assertTrue(contact_sheet.is_file(), "Missing all_scenes_acceptance_contact_sheet.png")
        self.assertTrue(defect_sheet.is_file(), "Missing all_scenes_defect_comparison.png")
        self.assertTrue(report_json.is_file(), "Missing sprint7e_master_report.json")

        self.assertGreater(contact_sheet.stat().st_size, 50000, "Contact sheet too small")
        self.assertGreater(defect_sheet.stat().st_size, 50000, "Defect sheet too small")

        data = json.loads(report_json.read_text())
        self.assertIn("acceptance_matrix", data, "Missing acceptance_matrix in master JSON")
        self.assertEqual(len(data["acceptance_matrix"]), 5, "Expected 5 entries in acceptance matrix")


if __name__ == "__main__":
    unittest.main()
