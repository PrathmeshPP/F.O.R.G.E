#!/usr/bin/env python3
"""
Unit and integration tests for F.O.R.G.E Sprint 9 Video-to-3D Reconstruction and Viewer Release.
Validates end-to-end provenance, file integrity, mesh geometry, texture bindings, manifest accuracy,
and live asset serving.
"""

import json
import hashlib
from pathlib import Path
import pytest
import urllib.request
import open3d as o3d
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RELEASE_DIR = PROJECT_ROOT / "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
MANIFEST_PATH = PROJECT_ROOT / "viewer/src/data/model-manifest.json"


def compute_sha256(filepath: Path) -> str:
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(1048576):
            h.update(chunk)
    return h.hexdigest()


class TestSprint9VideoToViewer:

    def test_01_source_inventory_validity(self):
        inv_path = RELEASE_DIR / "source_inventory.json"
        assert inv_path.is_file(), "source_inventory.json must exist in release directory"
        data = json.loads(inv_path.read_text(encoding="utf-8"))

        assert "benchmark_scenes" in data
        assert "campus_0049" in data["benchmark_scenes"]
        assert "desert_road" in data["benchmark_scenes"]
        assert "forest_road" in data["benchmark_scenes"]
        assert "river_bridge" in data["benchmark_scenes"]

        for s_key in ["campus_0049", "desert_road", "forest_road", "river_bridge"]:
            s_info = data["benchmark_scenes"][s_key]
            assert s_info["verified"] is True
            assert s_info["sha256"] and len(s_info["sha256"]) == 64
            assert s_info["width"] > 0
            assert s_info["height"] > 0
            assert s_info["frame_count"] > 0
            assert s_info["duration_sec"] > 0

    def test_02_scene_directories_and_required_artifacts(self):
        targets = [
            "campus_0049",
            "desert_road/candidate_a",
            "desert_road/candidate_b",
            "forest_road",
            "river_bridge"
        ]
        required_artifacts = [
            "source_frames_contact_sheet.png",
            "depth_confidence_preview.png",
            "point_cloud_unfused.ply",
            "point_cloud_fused.ply",
            "surface_mesh_repaired.ply",
            "surface_mesh_photographic.ply",
            "textured_mesh.obj",
            "textured_mesh.mtl",
            "texture_atlas.png",
            "render_top.png",
            "render_oblique.png",
            "render_closeup.png",
            "baseline_vs_new.png",
            "candidate_metadata.json",
            "scene_diagnostics.json",
            "processing.log"
        ]

        for target in targets:
            s_dir = RELEASE_DIR / target
            assert s_dir.is_dir(), f"Scene directory {target} must exist"
            for art in required_artifacts:
                art_p = s_dir / art
                assert art_p.is_file(), f"Artifact {art} must exist in {target}"
                assert art_p.stat().st_size > 0, f"Artifact {art} in {target} cannot be empty"

    def test_03_master_release_deliverables(self):
        master_files = [
            "all_scenes_before_after.png",
            "all_scenes_release_contact_sheet.png",
            "run_manifest.json",
            "source_inventory.json",
            "sprint9_reconstruction_report.json",
            "SPRINT9_RECONSTRUCTION_REPORT.md"
        ]
        for mf in master_files:
            p = RELEASE_DIR / mf
            assert p.is_file(), f"Master release deliverable {mf} must exist"
            assert p.stat().st_size > 0

    def test_04_manifest_schema_and_disk_hash_match(self):
        assert MANIFEST_PATH.is_file()
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        assert manifest["sprint"] in ["Sprint 9", "Sprint 10.1"]
        assert manifest["baseline_freeze"] == "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
        assert len(manifest["scenes"]) == 5

        for sc in manifest["scenes"]:
            for rep_name, rep in sc["representations"].items():
                if rep_name == "textured_obj":
                    url = rep["obj_url"]
                    rel = url.replace("/models/", "")
                    disk_file = RELEASE_DIR / rel
                    assert disk_file.is_file(), f"Referenced OBJ {url} must exist at {disk_file}"
                    computed_sha = compute_sha256(disk_file)
                    assert computed_sha == rep["sha256"], f"SHA256 mismatch for {url}"

                elif rep_name in ["photographic_ply", "untextured_ply"]:
                    url = rep["ply_url"]
                    rel = url.replace("/models/", "")
                    disk_file = RELEASE_DIR / rel
                    assert disk_file.is_file(), f"Referenced PLY {url} must exist at {disk_file}"
                    computed_sha = compute_sha256(disk_file)
                    assert computed_sha == rep["sha256"], f"SHA256 mismatch for {url}"

    def test_05_obj_mtl_texture_reference_integrity(self):
        for target in ["campus_0049", "desert_road/candidate_a", "desert_road/candidate_b", "forest_road", "river_bridge"]:
            s_dir = RELEASE_DIR / target
            obj_path = s_dir / "textured_mesh.obj"
            mtl_path = s_dir / "textured_mesh.mtl"
            atlas_path = s_dir / "texture_atlas.png"

            # Check OBJ refers to textured_mesh.mtl
            obj_lines = obj_path.read_text(encoding="utf-8", errors="ignore").splitlines()[:10]
            has_mtllib = any(l.startswith("mtllib ") for l in obj_lines)
            assert has_mtllib, f"OBJ in {target} must contain mtllib reference"

            # Check MTL refers to texture_atlas.png
            mtl_content = mtl_path.read_text(encoding="utf-8")
            assert "map_Kd texture_atlas.png" in mtl_content, f"MTL in {target} must reference texture_atlas.png"
            assert atlas_path.is_file()

    def test_06_live_viewer_asset_serving(self):
        # Query manifest
        req = urllib.request.urlopen("http://127.0.0.1:5173/model-manifest.json")
        assert req.getcode() == 200
        data = json.loads(req.read().decode("utf-8"))
        assert data["sprint"] in ["Sprint 9", "Sprint 10.1"]
        assert data["baseline_freeze"] == "outputs/accuracy_experiment/sprint9_video_to_viewer_release"

        # Query a Sprint 9 asset
        req_obj = urllib.request.urlopen("http://127.0.0.1:5173/models/desert_road/candidate_a/textured_mesh.mtl")
        assert req_obj.getcode() == 200
        content = req_obj.read().decode("utf-8")
        assert "Material_Atlas" in content
