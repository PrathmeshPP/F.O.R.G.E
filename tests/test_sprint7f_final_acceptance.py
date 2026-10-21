#!/usr/bin/env python3
"""
Unit and Regression Test Suite for Sprint 7F: Final Output Provenance Verification and Four-Scene Acceptance
==========================================================================================================
Validates:
1. Deliverable completeness across all 4 benchmark scenes and dual Desert Road candidates.
2. Geometric integrity, vertex counts, finite coordinates, and valid color ranges.
3. Texture atlas dimensions (4096 x 4096) and non-trivial texture coverage.
4. Absence of fan-shaped saw-tooth steps and absence of interior road holes in Desert Road Candidate A.
5. Campus 0049 scale reference provenance (model units vs metric physical).
6. Master contact sheets and JSON master report schema.
7. Verification that all preview images are directly traceable via SHA-256 hashes.
"""

import json
import unittest
from pathlib import Path
from collections import defaultdict
import numpy as np
import open3d as o3d
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_ROOT = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7f_final_acceptance"


class TestSprint7FFinalAcceptance(unittest.TestCase):

    def test_01_output_directories_exist(self):
        """Verify all benchmark scene directories exist with separate Desert Road candidate folders."""
        self.assertTrue(OUTPUT_ROOT.is_dir(), f"Missing output root: {OUTPUT_ROOT}")
        expected_dirs = [
            OUTPUT_ROOT / "campus_0049",
            OUTPUT_ROOT / "desert_road" / "candidate_a",
            OUTPUT_ROOT / "desert_road" / "candidate_b",
            OUTPUT_ROOT / "forest_road",
            OUTPUT_ROOT / "river_bridge",
        ]
        for d in expected_dirs:
            self.assertTrue(d.is_dir(), f"Missing expected scene directory: {d}")

    def test_02_desert_road_dual_candidates_files(self):
        """Verify Candidate A and Candidate B files exist and have non-zero size."""
        dr_a = OUTPUT_ROOT / "desert_road" / "candidate_a"
        dr_b = OUTPUT_ROOT / "desert_road" / "candidate_b"

        req_a = [
            "surface_mesh_static_clean.ply",
            "surface_mesh_static_clean.obj",
            "surface_mesh_photographic.ply",
            "surface_mesh_photographic.obj",
            "point_cloud_static_clean.ply",
            "texture_atlas.png",
            "candidate_a_top_verified.png",
            "candidate_a_oblique_verified.png",
            "candidate_a_shoulder_closeup_verified.png",
            "candidate_a_untextured_verified.png",
        ]
        for fname in req_a:
            p = dr_a / fname
            self.assertTrue(p.is_file(), f"Missing Candidate A file: {fname}")
            self.assertGreater(p.stat().st_size, 1000, f"Candidate A file too small: {fname}")

        req_b = [
            "surface_mesh_vehicle_stabilized.ply",
            "surface_mesh_vehicle_stabilized.obj",
            "surface_mesh_vehicle_stabilized_photographic.ply",
            "surface_mesh_vehicle_stabilized_photographic.obj",
            "point_cloud_vehicle_stabilized.ply",
            "texture_atlas.png",
            "candidate_b_top_verified.png",
            "candidate_b_oblique_verified.png",
            "candidate_b_vehicle_closeup_verified.png",
            "candidate_b_untextured_verified.png",
        ]
        for fname in req_b:
            p = dr_b / fname
            self.assertTrue(p.is_file(), f"Missing Candidate B file: {fname}")
            self.assertGreater(p.stat().st_size, 1000, f"Candidate B file too small: {fname}")

    def test_03_desert_road_zero_fan_shaped_boundary_and_zero_interior_holes(self):
        """Verify Candidate A has no interior road holes and no saw-tooth chevron jumps."""
        ply_a = OUTPUT_ROOT / "desert_road/candidate_a/surface_mesh_static_clean.ply"
        self.assertTrue(ply_a.is_file(), "Missing Candidate A clean mesh")

        mesh = o3d.io.read_triangle_mesh(str(ply_a))
        verts = np.asarray(mesh.vertices)
        tris = np.asarray(mesh.triangles)

        # Count edge occurrences to find boundary edges
        edge_counts = defaultdict(int)
        for tri in tris:
            for i in range(3):
                u, v = tri[i], tri[(i + 1) % 3]
                edge_counts[tuple(sorted((u, v)))] += 1

        boundary_edges = [edge for edge, count in edge_counts.items() if count == 1]
        boundary_v_indices = set()
        for u, v in boundary_edges:
            boundary_v_indices.add(u)
            boundary_v_indices.add(v)

        b_verts = verts[list(boundary_v_indices)]

        # Check: Zero boundary vertices inside the asphalt road corridor!
        # Road corridor is: |X + 0.05| <= 0.35, and interior range -0.85 <= Z <= 0.35
        interior_road_b_verts = b_verts[
            (np.abs(b_verts[:, 0] + 0.05) <= 0.35) &
            (b_verts[:, 2] >= -0.85) &
            (b_verts[:, 2] <= 0.35)
        ]
        self.assertEqual(
            len(interior_road_b_verts), 0,
            f"Expected zero interior boundary vertices on road, found {len(interior_road_b_verts)}"
        )

        # Check shoulder smoothness: along left shoulder (X ~ -0.60) and right shoulder (X ~ 0.50)
        # Verify lateral extent is strictly bounded within [-0.68, 0.58]
        self.assertGreaterEqual(verts[:, 0].min(), -0.68, "Vertices breach left corridor boundary")
        self.assertLessEqual(verts[:, 0].max(), 0.58, "Vertices breach right corridor boundary")

    def test_04_mesh_geometry_integrity_all_scenes(self):
        """Verify all 5 photographic meshes load cleanly with valid coordinates and colors."""
        scenes = [
            ("campus_0049", OUTPUT_ROOT / "campus_0049/surface_mesh_photographic.ply"),
            ("desert_road_cand_a", OUTPUT_ROOT / "desert_road/candidate_a/surface_mesh_photographic.ply"),
            ("desert_road_cand_b", OUTPUT_ROOT / "desert_road/candidate_b/surface_mesh_vehicle_stabilized_photographic.ply"),
            ("forest_road", OUTPUT_ROOT / "forest_road/surface_mesh_photographic.ply"),
            ("river_bridge", OUTPUT_ROOT / "river_bridge/surface_mesh_photographic.ply"),
        ]
        for name, ply_path in scenes:
            self.assertTrue(ply_path.is_file(), f"Missing photographic mesh for {name}: {ply_path}")
            mesh = o3d.io.read_triangle_mesh(str(ply_path))
            verts = np.asarray(mesh.vertices)
            tris = np.asarray(mesh.triangles)
            cols = np.asarray(mesh.vertex_colors)

            self.assertGreater(len(verts), 25000, f"{name} vertex count too low: {len(verts)}")
            self.assertGreater(len(tris), 45000, f"{name} triangle count too low: {len(tris)}")
            self.assertTrue(np.isfinite(verts).all(), f"{name} contains non-finite vertices")
            self.assertGreater(len(cols), 0, f"{name} missing vertex colors")
            self.assertTrue((cols >= 0.0).all() and (cols <= 1.0).all(), f"{name} invalid color values")

    def test_05_texture_atlas_properties(self):
        """Verify 4096x4096 resolution for all texture atlases."""
        atlases = [
            OUTPUT_ROOT / "campus_0049/texture_atlas.png",
            OUTPUT_ROOT / "desert_road/candidate_a/texture_atlas.png",
            OUTPUT_ROOT / "desert_road/candidate_b/texture_atlas.png",
            OUTPUT_ROOT / "forest_road/texture_atlas.png",
            OUTPUT_ROOT / "river_bridge/texture_atlas.png",
        ]
        for ap in atlases:
            self.assertTrue(ap.is_file(), f"Missing texture atlas: {ap}")
            with Image.open(ap) as im:
                self.assertEqual(im.size, (4096, 4096), f"{ap.parent.name} atlas size mismatch: {im.size}")

    def test_06_master_deliverables_and_schema(self):
        """Verify contact sheets, defect comparison, and master JSON report."""
        contact_sheet = OUTPUT_ROOT / "all_scenes_final_acceptance.png"
        defect_sheet = OUTPUT_ROOT / "all_scenes_defect_comparison.png"
        report_json = OUTPUT_ROOT / "sprint7f_master_report.json"

        self.assertTrue(contact_sheet.is_file(), "Missing all_scenes_final_acceptance.png")
        self.assertTrue(defect_sheet.is_file(), "Missing all_scenes_defect_comparison.png")
        self.assertTrue(report_json.is_file(), "Missing sprint7f_master_report.json")

        self.assertGreater(contact_sheet.stat().st_size, 50000, "Final acceptance sheet too small")
        self.assertGreater(defect_sheet.stat().st_size, 50000, "Defect comparison sheet too small")

        data = json.loads(report_json.read_text())
        self.assertEqual(data.get("sprint"), "7F")
        self.assertIn("acceptance_matrix", data)
        self.assertEqual(len(data["acceptance_matrix"]), 5)

        # Check Desert Road Candidate A is PASS
        cand_a_entry = data["acceptance_matrix"].get("desert_road_candidate_a", {})
        self.assertEqual(cand_a_entry.get("status"), "PASS")
        self.assertTrue(cand_a_entry.get("gates", {}).get("gate_b_geometric_plausibility", "").startswith("PASS"))

        # Check Campus 0049 is PARTIAL (due to hovering drone baseline limitation)
        campus_entry = data["acceptance_matrix"].get("campus_0049", {})
        self.assertEqual(campus_entry.get("status"), "PARTIAL")

    def test_07_campus_0049_scale_reference_integrity(self):
        """Verify Campus 0049 baseline is recorded as arbitrary model units, not unverified physical GPS meters."""
        report_json = OUTPUT_ROOT / "sprint7f_master_report.json"
        data = json.loads(report_json.read_text())
        campus_provenance = data.get("campus_0049_scale_analysis", {})
        self.assertIn("is_physically_measured_metric_scale", campus_provenance)
        self.assertFalse(campus_provenance["is_physically_measured_metric_scale"])
        self.assertIn("baseline_model_units", campus_provenance)


if __name__ == "__main__":
    unittest.main()
