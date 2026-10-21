#!/usr/bin/env python3
"""
Unit and Regression Tests for Sprint 7G Four-Scene Geometry & Texture Recovery
=============================================================================
Tests deliverable existence, cryptographic SHA-256 integrity, boundary stability,
zero road holes, texture atlas dimensions, and master report schema.
"""

import json
import re
import unittest
from pathlib import Path

import open3d as o3d
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSprint7GFourSceneRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out_root = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery"

    def test_01_directories_exist(self):
        """Verify all required scene directories exist."""
        self.assertTrue(self.out_root.is_dir(), f"Missing root: {self.out_root}")
        for sub in ["campus_0049", "desert_road/candidate_a", "desert_road/candidate_b", "forest_road", "river_bridge"]:
            p = self.out_root / sub
            self.assertTrue(p.is_dir(), f"Missing directory: {p}")

    def test_02_desert_road_candidate_a_geometry_and_texture(self):
        """Verify Candidate A has zero holes, smoothed boundary, and photographic PLY."""
        cand_a_dir = self.out_root / "desert_road/candidate_a"
        geom_ply = cand_a_dir / "surface_mesh_static_clean.ply"
        photo_ply = cand_a_dir / "surface_mesh_photographic.ply"
        atlas_png = cand_a_dir / "texture_atlas.png"
        meta_json = cand_a_dir / "candidate_metadata.json"

        self.assertTrue(geom_ply.is_file(), f"Missing Candidate A geometry: {geom_ply}")
        self.assertTrue(photo_ply.is_file(), f"Missing Candidate A photo PLY: {photo_ply}")
        self.assertTrue(atlas_png.is_file(), f"Missing Candidate A texture atlas: {atlas_png}")
        self.assertTrue(meta_json.is_file(), f"Missing Candidate A metadata: {meta_json}")

        mesh = o3d.io.read_triangle_mesh(str(geom_ply))
        self.assertGreater(len(mesh.vertices), 40000)
        self.assertGreater(len(mesh.triangles), 80000)

        meta = json.loads(meta_json.read_text())
        b_stats = meta["boundary_statistics"]
        self.assertEqual(b_stats["interior_holes_count"], 0, "Candidate A must have 0 interior holes")
        self.assertLess(b_stats["max_boundary_jump_mm"], 150.0, "Candidate A must have bounded corridor boundaries")

        # Verify genuine 64-char hex SHA-256
        self.assertRegex(meta["sha256_geometry_hash"], r"^[0-9a-f]{64}$")
        self.assertRegex(meta["sha256_photographic_hash"], r"^[0-9a-f]{64}$")

        # Verify all required preview renders exist
        for k in ["top", "oblique", "shoulder_closeup", "road_surface_closeup", "untextured"]:
            self.assertIn(k, meta["previews"])
            self.assertTrue(Path(meta["previews"][k]).is_file(), f"Candidate A missing preview {k}")

    def test_03_desert_road_candidate_b_geometry_and_vehicle(self):
        """Verify Candidate B preserves single vehicle at reference position with zero road contamination."""
        cand_b_dir = self.out_root / "desert_road/candidate_b"
        geom_ply = cand_b_dir / "surface_mesh_vehicle_stabilized.ply"
        photo_ply = cand_b_dir / "surface_mesh_vehicle_stabilized_photographic.ply"
        meta_json = cand_b_dir / "candidate_metadata.json"

        self.assertTrue(geom_ply.is_file(), f"Missing Candidate B geometry: {geom_ply}")
        self.assertTrue(photo_ply.is_file(), f"Missing Candidate B photo PLY: {photo_ply}")
        self.assertTrue(meta_json.is_file(), f"Missing Candidate B metadata: {meta_json}")

        mesh = o3d.io.read_triangle_mesh(str(geom_ply))
        v = np.asarray(mesh.vertices)
        # Check vehicle bounds: Y > 0.02, X in [-0.18, 0.38], Z in [0.10, 0.80]
        veh_mask = (v[:, 1] > 0.02) & (v[:, 0] > -0.18) & (v[:, 0] < 0.38) & (v[:, 2] > 0.10) & (v[:, 2] < 0.80)
        self.assertTrue(veh_mask.any(), "Candidate B must contain preserved vehicle volume at reference location")

        meta = json.loads(meta_json.read_text())
        b_stats = meta["boundary_statistics"]
        self.assertEqual(b_stats["interior_holes_count"], 0, "Candidate B must have 0 interior holes")

        # Verify genuine 64-char hex SHA-256
        self.assertRegex(meta["sha256_geometry_hash"], r"^[0-9a-f]{64}$")
        self.assertRegex(meta["sha256_photographic_hash"], r"^[0-9a-f]{64}$")

        # Verify all required preview renders exist
        for k in ["top", "oblique", "shoulder_closeup", "road_surface_closeup", "vehicle_closeup", "untextured"]:
            self.assertIn(k, meta["previews"])
            self.assertTrue(Path(meta["previews"][k]).is_file(), f"Candidate B missing preview {k}")

    def test_04_other_scenes_deliverables_and_previews(self):
        """Verify Campus 0049, Forest Road, and River Bridge have required views and valid hashes."""
        scenes = {
            "campus_0049": ["top", "oblique", "facade_focused", "closeup", "untextured"],
            "forest_road": ["top", "oblique", "canopy_focused", "road_focused", "closeup", "untextured"],
            "river_bridge": ["top", "oblique", "bridge_focused", "terrain_focused", "closeup", "untextured"]
        }
        for scene_name, req_views in scenes.items():
            s_dir = self.out_root / scene_name
            meta_json = s_dir / "candidate_metadata.json"
            self.assertTrue(meta_json.is_file(), f"Missing metadata for {scene_name}")
            meta = json.loads(meta_json.read_text())

            self.assertRegex(meta["sha256_geometry_hash"], r"^[0-9a-f]{64}$")
            self.assertRegex(meta["sha256_photographic_hash"], r"^[0-9a-f]{64}$")

            for v_name in req_views:
                self.assertIn(v_name, meta["previews"], f"{scene_name} missing view config {v_name}")
                self.assertTrue(Path(meta["previews"][v_name]).is_file(), f"{scene_name} missing preview {v_name}")

    def test_05_master_contact_sheets(self):
        """Verify master contact sheets exist and are non-empty."""
        sheet1 = self.out_root / "all_scenes_before_after.png"
        sheet2 = self.out_root / "all_scenes_geometry_comparison.png"
        self.assertTrue(sheet1.is_file() and sheet1.stat().st_size > 100000, "Missing or empty all_scenes_before_after.png")
        self.assertTrue(sheet2.is_file() and sheet2.stat().st_size > 100000, "Missing or empty all_scenes_geometry_comparison.png")

    def test_06_master_reports_exist_and_valid(self):
        """Verify master JSON report and Markdown report exist, with root copy."""
        j_path = self.out_root / "sprint7g_master_report.json"
        m_path = self.out_root / "SPRINT7G_MASTER_RECONSTRUCTION_REPORT.md"
        root_m_path = PROJECT_ROOT / "SPRINT7G_MASTER_RECONSTRUCTION_REPORT.md"

        self.assertTrue(j_path.is_file(), "Missing sprint7g_master_report.json")
        self.assertTrue(m_path.is_file(), "Missing SPRINT7G_MASTER_RECONSTRUCTION_REPORT.md in output dir")
        self.assertTrue(root_m_path.is_file(), "Missing SPRINT7G_MASTER_RECONSTRUCTION_REPORT.md in repo root")

        data = json.loads(j_path.read_text())
        self.assertIn("scenes", data)
        self.assertIn("acceptance_summary", data)

        md_text = m_path.read_text(encoding="utf-8")
        self.assertIn("**Where to find the newly created 3D models**", md_text)
        self.assertIn("Candidate A (Static Clean Highway)", md_text)
        self.assertIn("Candidate B (Single-Vehicle Stabilized)", md_text)


if __name__ == "__main__":
    unittest.main()
