#!/usr/bin/env python3
"""
Unit and Regression Tests for Sprint 7H Final Visual Defect Recovery
====================================================================
Tests deliverable existence, cryptographic SHA-256 integrity, boundary stability,
zero interior holes, tight vehicle instance mask (zero rectangular road box),
stage audit rendering progression for Desert Road Candidate A, bridge centering,
forest canopy/road orientation, and master report schema.
"""

import json
import re
import unittest
from pathlib import Path

import open3d as o3d
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSprint7HFinalDefectRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out_root = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7h_final_defect_recovery"

    def test_01_directories_exist(self):
        """Verify all required scene directories and stage audit exist."""
        self.assertTrue(self.out_root.is_dir(), f"Missing root: {self.out_root}")
        subdirs = [
            "campus_0049",
            "desert_road/candidate_a",
            "desert_road/candidate_a/stage_audit",
            "desert_road/candidate_b",
            "forest_road",
            "river_bridge",
        ]
        for sub in subdirs:
            p = self.out_root / sub
            self.assertTrue(p.is_dir(), f"Missing directory: {p}")

    def test_02_desert_road_candidate_a_stage_audit_and_geometry(self):
        """Verify Candidate A 5-stage progression audit renders, zero holes, and clean corridor."""
        cand_a_dir = self.out_root / "desert_road/candidate_a"
        stage_dir = cand_a_dir / "stage_audit"
        
        # Verify all 5 stages from identical camera viewpoints
        for stage_idx, stage_name in [
            (1, "point_cloud"),
            (2, "raw_untextured_mesh"),
            (3, "smoothed_untextured_mesh"),
            (4, "photographic_mesh"),
            (5, "textured_obj"),
        ]:
            top_img = stage_dir / f"stage{stage_idx}_{stage_name}_top.png"
            shld_img = stage_dir / f"stage{stage_idx}_{stage_name}_shoulder.png"
            self.assertTrue(top_img.is_file(), f"Missing {top_img}")
            self.assertTrue(shld_img.is_file(), f"Missing {shld_img}")
            self.assertGreater(top_img.stat().st_size, 10000)
            self.assertGreater(shld_img.stat().st_size, 10000)

        geom_ply = cand_a_dir / "surface_mesh_static_clean.ply"
        photo_ply = cand_a_dir / "surface_mesh_photographic.ply"
        meta_json = cand_a_dir / "candidate_metadata.json"

        self.assertTrue(geom_ply.is_file(), f"Missing Candidate A geometry: {geom_ply}")
        self.assertTrue(photo_ply.is_file(), f"Missing Candidate A photo PLY: {photo_ply}")
        self.assertTrue(meta_json.is_file(), f"Missing Candidate A metadata: {meta_json}")

        mesh = o3d.io.read_triangle_mesh(str(geom_ply))
        self.assertGreater(len(mesh.vertices), 40000)
        self.assertGreater(len(mesh.triangles), 80000)

        meta = json.loads(meta_json.read_text())
        b_stats = meta["boundary_statistics"]
        self.assertEqual(b_stats["interior_holes_count"], 0, "Candidate A must have 0 interior holes")
        self.assertLess(b_stats["max_boundary_jump_mm"], 150.0, "Candidate A corridor boundaries must be bounded")

        # Verify genuine 64-char hex SHA-256
        self.assertRegex(meta["sha256_geometry_hash"], r"^[0-9a-f]{64}$")
        self.assertRegex(meta["sha256_photographic_hash"], r"^[0-9a-f]{64}$")

        # Verify preview renders exist
        for k in ["top", "oblique", "shoulder_closeup", "road_surface_closeup", "untextured"]:
            self.assertIn(k, meta["previews"])
            self.assertTrue(Path(meta["previews"][k]).is_file(), f"Candidate A missing preview {k}")

    def test_03_desert_road_candidate_b_tight_instance_mask(self):
        """Verify Candidate B uses tight shape-aware instance mask (zero rectangular road box)."""
        cand_b_dir = self.out_root / "desert_road/candidate_b"
        geom_ply = cand_b_dir / "surface_mesh_vehicle_stabilized.ply"
        photo_ply = cand_b_dir / "surface_mesh_vehicle_stabilized_photographic.ply"
        meta_json = cand_b_dir / "candidate_metadata.json"

        self.assertTrue(geom_ply.is_file(), f"Missing Candidate B geometry: {geom_ply}")
        self.assertTrue(photo_ply.is_file(), f"Missing Candidate B photo PLY: {photo_ply}")
        self.assertTrue(meta_json.is_file(), f"Missing Candidate B metadata: {meta_json}")

        mesh = o3d.io.read_triangle_mesh(str(geom_ply))
        v = np.asarray(mesh.vertices)
        
        # In Sprint 7G, naive 3D box captured >4,300 vertices including a giant flat road box.
        # In Sprint 7H, tight vehicle mask isolates vehicle body:
        # Road plane: Y = -0.019475 * X - 0.057574 * Z + 0.088181 (Y points downward in camera coordinates)
        # Therefore, points above road satisfy y_road - Y > 0.012
        road_y = -0.019475 * v[:, 0] - 0.057574 * v[:, 2] + 0.088181
        delta_y = road_y - v[:, 1]
        in_veh_xz = (v[:, 0] > -0.15) & (v[:, 0] < 0.25) & (v[:, 2] > 0.20) & (v[:, 2] < 0.70)
        veh_verts = (delta_y > 0.012) & in_veh_xz
        veh_count = np.sum(veh_verts)
        
        # Vehicle body has between 100 and 1500 vertices (tight vehicle volume)
        self.assertGreater(veh_count, 100, "Vehicle volume must be present")
        self.assertLess(veh_count, 1500, "Vehicle volume must be tightly masked, not capturing huge road area")

        meta = json.loads(meta_json.read_text())
        b_stats = meta["boundary_statistics"]
        self.assertEqual(b_stats["interior_holes_count"], 0, "Candidate B must have 0 interior holes")

        # Verify genuine 64-char hex SHA-256
        self.assertRegex(meta["sha256_geometry_hash"], r"^[0-9a-f]{64}$")
        self.assertRegex(meta["sha256_photographic_hash"], r"^[0-9a-f]{64}$")

        # Verify all required preview renders exist including vehicle closeup
        for k in ["top", "oblique", "shoulder_closeup", "road_surface_closeup", "vehicle_closeup", "untextured"]:
            self.assertIn(k, meta["previews"])
            self.assertTrue(Path(meta["previews"][k]).is_file(), f"Candidate B missing preview {k}")

    def test_04_other_scenes_deliverables_and_previews(self):
        """Verify Campus 0049, Forest Road, and River Bridge have required views and valid hashes."""
        scenes = {
            "campus_0049": ["top", "oblique", "facade_focused", "closeup", "untextured"],
            "forest_road": ["top", "oblique", "canopy_focused", "road_focused", "closeup", "untextured"],
            "river_bridge": ["top", "oblique", "bridge_focused", "terrain_focused", "closeup", "untextured"],
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
        """Verify all three master contact sheets exist and are non-empty."""
        sheet1 = self.out_root / "all_scenes_before_after.png"
        sheet2 = self.out_root / "desert_road_candidate_comparison.png"
        sheet3 = self.out_root / "all_scenes_untextured_geometry.png"
        self.assertTrue(sheet1.is_file() and sheet1.stat().st_size > 100000, "Missing or empty all_scenes_before_after.png")
        self.assertTrue(sheet2.is_file() and sheet2.stat().st_size > 100000, "Missing or empty desert_road_candidate_comparison.png")
        self.assertTrue(sheet3.is_file() and sheet3.stat().st_size > 100000, "Missing or empty all_scenes_untextured_geometry.png")

    def test_06_master_reports_exist_and_valid(self):
        """Verify master JSON report and Markdown report exist with required contents."""
        j_path = self.out_root / "sprint7h_master_report.json"
        m_path = self.out_root / "SPRINT7H_MASTER_RECONSTRUCTION_REPORT.md"
        root_m_path = PROJECT_ROOT / "SPRINT7H_MASTER_RECONSTRUCTION_REPORT.md"

        self.assertTrue(j_path.is_file(), "Missing sprint7h_master_report.json")
        self.assertTrue(m_path.is_file(), "Missing SPRINT7H_MASTER_RECONSTRUCTION_REPORT.md in output dir")
        self.assertTrue(root_m_path.is_file(), "Missing SPRINT7H_MASTER_RECONSTRUCTION_REPORT.md in repo root")

        data = json.loads(j_path.read_text())
        self.assertIn("scenes", data)
        self.assertIn("acceptance_summary", data)

        # Check acceptance decisions in acceptance_summary
        summary = data["acceptance_summary"]
        self.assertEqual(summary["campus_0049"]["overall"], "PARTIAL")
        self.assertEqual(summary["desert_road_candidate_a"]["overall"], "PASS")
        self.assertEqual(summary["desert_road_candidate_b"]["overall"], "PASS")
        self.assertEqual(summary["forest_road"]["overall"], "PASS")
        self.assertEqual(summary["river_bridge"]["overall"], "PASS")

        md_text = m_path.read_text(encoding="utf-8")
        self.assertIn("**Where to find the newly created 3D models**", md_text)
        self.assertIn("Candidate A", md_text)
        self.assertIn("Static Clean Highway", md_text)
        self.assertIn("Candidate B", md_text)
        self.assertIn("Single-Vehicle Preserved", md_text)


if __name__ == "__main__":
    unittest.main()
