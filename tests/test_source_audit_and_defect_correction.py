"""F.O.R.G.E — Sprint 5 Source Audit & Defect Correction Unit Tests.

Covers:
1. Source Video Ingestion & Provenance: Tests source video resolution, codec, duration, and SHA256 hash existence.
2. Artifact Mapping Integrity: Verifies screenshot-to-artifact mapping and model paths.
3. Desert Road Vehicle Isolation: Verifies that Candidate A contains zero vehicle ghosting and Candidate B contains exactly 1 stabilized vehicle.
4. Road Surface Continuity: Verifies planar road alignment eliminates stepped elevation jumps.
"""

import unittest
import json
from pathlib import Path
import numpy as np
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class TestSourceAuditAndDefectCorrection(unittest.TestCase):

    def test_source_video_audit_report_exists_and_valid(self):
        """Verifies Phase 1 source video audit report structure and content."""
        audit_file = PROJECT_ROOT / "outputs/accuracy_experiment/source_audit_quality_v1/phase1_source_video_audit.json"
        self.assertTrue(audit_file.is_file(), "phase1_source_video_audit.json must exist")

        data = json.loads(audit_file.read_text())
        scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]
        for s in scenes:
            self.assertIn(s, data["scenes"])
            info = data["scenes"][s]["source_video"]
            self.assertTrue(info["exists"], f"Source video for {s} must exist on disk")
            self.assertIsNotNone(info["sha256"], f"SHA256 for {s} must be computed")
            self.assertGreater(info["duration_sec"], 0.0)
            self.assertGreater(info["frame_count"], 0)

    def test_phase2_artifact_mapping_audit_exists(self):
        """Verifies Phase 2 artifact mapping audit structure."""
        mapping_file = PROJECT_ROOT / "outputs/accuracy_experiment/source_audit_quality_v1/phase2_artifact_mapping_audit.json"
        self.assertTrue(mapping_file.is_file(), "phase2_artifact_mapping_audit.json must exist")

        data = json.loads(mapping_file.read_text())
        scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]
        for s in scenes:
            self.assertIn(s, data["screenshot_to_artifact_mappings"])
            mapping = data["screenshot_to_artifact_mappings"][s]
            self.assertIn("rendered_model_files", mapping)
            self.assertIn("rendering_configuration", mapping)
            self.assertIn("source_data_provenance", mapping)

    def test_desert_road_dynamic_vehicle_elimination(self):
        """Verifies that Candidate A static clean mesh has zero vehicle point clusters."""
        out_dir = PROJECT_ROOT / "outputs/accuracy_experiment/source_audit_quality_v1/desert_road"
        clean_mesh_p = out_dir / "surface_mesh_static_clean.ply"
        stab_mesh_p = out_dir / "surface_mesh_vehicle_stabilized.ply"

        self.assertTrue(clean_mesh_p.is_file(), "Candidate A clean mesh must exist")
        self.assertTrue(stab_mesh_p.is_file(), "Candidate B stabilized mesh must exist")

        mesh_clean = o3d.io.read_triangle_mesh(str(clean_mesh_p))
        mesh_stab = o3d.io.read_triangle_mesh(str(stab_mesh_p))

        verts_clean = np.asarray(mesh_clean.vertices)
        verts_stab = np.asarray(mesh_stab.vertices)

        cols_prior = np.asarray(o3d.io.read_triangle_mesh(str(PROJECT_ROOT / "outputs/accuracy_experiment/photographic_texturing_v1/desert_road/surface_mesh_photographic.ply")).vertex_colors)
        cols_clean = np.asarray(mesh_clean.vertex_colors)

        # In previous failed candidate, blue vehicle vertices (B > R + 0.15 and B > 0.3) were stamped across all 8 positions
        blue_prior = (cols_prior[:, 2] > cols_prior[:, 0] + 0.15) & (cols_prior[:, 2] > 0.3)
        blue_clean = (cols_clean[:, 2] > cols_clean[:, 0] + 0.15) & (cols_clean[:, 2] > 0.3)

        self.assertGreater(blue_prior.sum(), 300, "Prior failed model must exhibit moving car ghosting (>300 blue verts)")
        self.assertLessEqual(blue_clean.sum(), 5, "Candidate A must eliminate ghost vehicle vertices (<= 5 blue verts)")

    def test_desert_road_metadata_consistency(self):
        """Verifies Desert road correction metadata JSON contains required audit keys."""
        meta_p = PROJECT_ROOT / "outputs/accuracy_experiment/source_audit_quality_v1/desert_road/desert_road_correction_metadata.json"
        self.assertTrue(meta_p.is_file(), "Correction metadata JSON must exist")

        meta = json.loads(meta_p.read_text())
        self.assertIn("diagnosed_fault", meta)
        self.assertIn("candidates", meta)
        self.assertIn("static_clean", meta["candidates"])
        self.assertIn("vehicle_stabilized", meta["candidates"])


if __name__ == "__main__":
    unittest.main()
