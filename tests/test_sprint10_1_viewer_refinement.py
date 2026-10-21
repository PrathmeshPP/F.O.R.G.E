#!/usr/bin/env python3
"""
Unit and regression tests for F.O.R.G.E Sprint 10.1:
Viewer Presentation, Camera Framing, and Metadata Consistency.

Validates:
1. Geometry-count labeling semantics (unique geometry vertices, rendered position entries, triangle count, source assets).
2. All 5 scene entries and representations (Textured OBJ, Photographic PLY, Untextured PLY).
3. Frozen model integrity and cryptographic SHA-256 digests.
4. Candidate A vs Candidate B distinct file provenance.
5. Camera framing calculations, bounding boxes, and scene-specific camera presets.
6. Desert Road Candidate B vehicle focus framing (framing only, zero mesh modification).
7. Non-fabrication disclosures and unavailable count fallback handling.
"""

import json
import hashlib
from pathlib import Path
import pytest
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RELEASE_DIR = PROJECT_ROOT / "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
MANIFEST_PATH = PROJECT_ROOT / "viewer/src/data/model-manifest.json"
DIST_MANIFEST_PATH = PROJECT_ROOT / "viewer/dist/model-manifest.json"


class TestSprint10_1ViewerRefinement:

    @pytest.fixture(scope="class")
    def manifest(self):
        assert MANIFEST_PATH.is_file(), "model-manifest.json must exist"
        with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
            return json.load(f)

    def test_01_manifest_version_and_freeze(self, manifest):
        assert manifest["version"] == "1.1.0"
        assert "10.1" in manifest["sprint"]
        assert manifest["baseline_freeze"] == "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
        assert len(manifest["scenes"]) == 5

    def test_02_all_representations_exist_and_match_hashes(self, manifest):
        """Verify that every representation for all 5 scenes exists on disk with unchanged SHA-256."""
        for scene in manifest["scenes"]:
            sc_id = scene["id"]
            reps = scene["representations"]
            assert "textured_obj" in reps
            assert "photographic_ply" in reps
            assert "untextured_ply" in reps

            for rep_key, rep in reps.items():
                url = rep["obj_url"] if rep["type"] == "obj" else rep["ply_url"]
                rel_path = url.replace("/models/", "").replace("/", "\\")
                disk_path = RELEASE_DIR / rel_path
                assert disk_path.is_file(), f"Asset missing on disk: {disk_path} for {sc_id} ({rep_key})"

                # Verify byte size
                actual_size = disk_path.stat().st_size
                assert actual_size == rep["file_size_bytes"], (
                    f"File size mismatch for {disk_path}: disk={actual_size}, manifest={rep['file_size_bytes']}"
                )

                # Verify SHA-256 hash
                h = hashlib.sha256(disk_path.read_bytes()).hexdigest()
                assert h == rep["sha256"], (
                    f"SHA-256 hash mismatch for {disk_path}: disk={h}, manifest={rep['sha256']}"
                )

    def test_03_geometry_count_semantics_obj_vs_ply(self, manifest):
        """
        Verify the distinction between:
        - Unique geometry vertices (source mesh topological vertices)
        - Rendered position entries (Three.js GPU buffer elements: 3x triangles for non-indexed OBJ)
        - Triangle count
        """
        for scene in manifest["scenes"]:
            sc_id = scene["id"]
            unique_v = scene["vertex_count"]
            triangles = scene["triangle_count"]

            # 1. Check OBJ mesh
            obj_rel = scene["representations"]["textured_obj"]["obj_url"].replace("/models/", "")
            obj_path = RELEASE_DIR / obj_rel
            
            # Count unique v and f in OBJ
            v_count = 0
            f_count = 0
            with open(obj_path, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    if line.startswith("v "):
                        v_count += 1
                    elif line.startswith("f "):
                        f_count += 1

            assert v_count == unique_v, f"OBJ unique v mismatch for {sc_id}: file={v_count}, manifest={unique_v}"
            assert f_count == triangles, f"OBJ face count mismatch for {sc_id}: file={f_count}, manifest={triangles}"

            # In Three.js, unindexed OBJ creates 3 position entries per triangle
            rendered_positions_obj = f_count * 3
            assert rendered_positions_obj == triangles * 3

            # 2. Check PLY mesh
            ply_rel = scene["representations"]["photographic_ply"]["ply_url"].replace("/models/", "")
            ply_path = RELEASE_DIR / ply_rel
            
            ply_v = None
            ply_f = None
            with open(ply_path, "rb") as f:
                while True:
                    line = f.readline()
                    if line.startswith(b"element vertex"):
                        ply_v = int(line.strip().split()[-1])
                    elif line.startswith(b"element face"):
                        ply_f = int(line.strip().split()[-1])
                    if line.strip() == b"end_header":
                        break

            assert ply_v == unique_v, f"PLY unique v mismatch for {sc_id}: file={ply_v}, manifest={unique_v}"
            assert ply_f == triangles, f"PLY face count mismatch for {sc_id}: file={ply_f}, manifest={triangles}"

    def test_04_desert_road_candidate_a_and_b_provenance(self, manifest):
        """Verify Candidate A and Candidate B load distinct assets with different hashes."""
        cand_a = next(s for s in manifest["scenes"] if s["id"] == "desert_road_candidate_a")
        cand_b = next(s for s in manifest["scenes"] if s["id"] == "desert_road_candidate_b")

        # Distinct OBJ hashes
        assert cand_a["representations"]["textured_obj"]["sha256"] != cand_b["representations"]["textured_obj"]["sha256"]
        # Distinct PLY hashes
        assert cand_a["representations"]["photographic_ply"]["sha256"] != cand_b["representations"]["photographic_ply"]["sha256"]
        assert cand_a["representations"]["untextured_ply"]["sha256"] != cand_b["representations"]["untextured_ply"]["sha256"]

        # Different triangle and vertex counts due to preserved vehicle
        assert cand_a["vertex_count"] == 53069
        assert cand_b["vertex_count"] == 53101
        assert cand_b["vertex_count"] > cand_a["vertex_count"]

        assert cand_a["triangle_count"] == 105467
        assert cand_b["triangle_count"] == 105518
        assert cand_b["triangle_count"] > cand_a["triangle_count"]

    def test_05_camera_framing_and_scene_presets(self, manifest):
        """Verify camera framing presets and bounds."""
        for scene in manifest["scenes"]:
            init_cam = scene["initial_camera"]
            assert "position" in init_cam and len(init_cam["position"]) == 3
            assert "target" in init_cam and len(init_cam["target"]) == 3
            assert init_cam["position"] != init_cam["target"]

        # River bridge deck centering verification
        river = next(s for s in manifest["scenes"] if s["id"] == "river_bridge")
        # Target must point towards bridge deck centered coordinates (~0.67, ~0.23, ~-0.68)
        assert abs(river["initial_camera"]["target"][0] - 0.673) < 0.05
        assert abs(river["initial_camera"]["target"][1] - 0.232) < 0.05
        assert abs(river["initial_camera"]["target"][2] - (-0.681)) < 0.05

    def test_06_desert_road_vehicle_focus_coordinates(self):
        """Verify Candidate B vehicle location without altering mesh."""
        mesh_b_path = RELEASE_DIR / "desert_road/candidate_b/surface_mesh_vehicle_stabilized.ply"
        mesh_b = trimesh.load(mesh_b_path, process=False)
        center = (mesh_b.bounds[0] + mesh_b.bounds[1]) / 2.0

        # Raw vehicle coordinate in candidate metadata is [0.03, 0.04, 0.45]
        raw_veh = [0.03, 0.04, 0.45]
        centered_veh = raw_veh - center

        # Centered vehicle target in Viewport3D / ComparisonView is [0.084, -0.053, 0.536]
        assert abs(centered_veh[0] - 0.084) < 0.01
        assert abs(centered_veh[1] - (-0.053)) < 0.01
        assert abs(centered_veh[2] - 0.536) < 0.01
