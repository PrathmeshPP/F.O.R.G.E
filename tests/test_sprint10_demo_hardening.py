#!/usr/bin/env python3
"""
Unit and integration tests for F.O.R.G.E Sprint 10 National Hackathon Demonstration Hardening.
Validates the one-command launch script, live demo guide, demo release artifacts,
manifest integrity, and live asset serving.
"""

import json
from pathlib import Path
import urllib.request
import pytest

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEMO_DIR = PROJECT_ROOT / "outputs/accuracy_experiment/sprint10_demo_release"
SPRINT9_DIR = PROJECT_ROOT / "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
MANIFEST_PATH = PROJECT_ROOT / "viewer/src/data/model-manifest.json"


class TestSprint10DemoHardening:

    def test_01_launcher_script_and_demo_guide(self):
        bat_path = PROJECT_ROOT / "start_forge_explorer.bat"
        assert bat_path.is_file(), "start_forge_explorer.bat must exist in repository root"
        content = bat_path.read_text(encoding="utf-8", errors="ignore")
        assert "serve_viewer.py" in content
        assert "5173" in content
        assert "sprint9_video_to_viewer_release" in content

        guide_path = PROJECT_ROOT / "docs/SPRINT10_LIVE_DEMO_GUIDE.md"
        assert guide_path.is_file(), "SPRINT10_LIVE_DEMO_GUIDE.md must exist in docs/"
        guide_text = guide_path.read_text(encoding="utf-8")
        assert "Five-Minute Judge Demonstration Sequence" in guide_text
        assert "Technical Defense & Judge Q&A Guide" in guide_text
        assert "Campus 0049" in guide_text
        assert "Desert Road" in guide_text

    def test_02_demo_release_deliverables_and_screenshots(self):
        assert DEMO_DIR.is_dir(), "sprint10_demo_release directory must exist"
        
        required_files = [
            "verified_manifest_snapshot.json",
            "model_texture_integrity_report.json",
            "test_and_build_report.json",
            "sprint10_demo_readiness.json",
            "SPRINT10_DEMO_READINESS_REPORT.md"
        ]
        for rf in required_files:
            p = DEMO_DIR / rf
            assert p.is_file(), f"Missing deliverable: {rf}"
            assert p.stat().st_size > 0

        # Screenshots
        screenshots_dir = DEMO_DIR / "screenshots"
        assert screenshots_dir.is_dir()
        expected_shots = [
            "demo_campus_0049.png",
            "demo_desert_candidate_a.png",
            "demo_desert_candidate_b.png",
            "demo_desert_comparison.png",
            "demo_forest_road.png",
            "demo_river_bridge.png"
        ]
        for shot in expected_shots:
            shot_p = screenshots_dir / shot
            assert shot_p.is_file(), f"Missing screenshot: {shot}"
            assert shot_p.stat().st_size > 50000, f"Screenshot {shot} appears too small or corrupt"

    def test_03_model_texture_integrity(self):
        rep_path = DEMO_DIR / "model_texture_integrity_report.json"
        assert rep_path.is_file()
        rep = json.loads(rep_path.read_text(encoding="utf-8"))
        assert rep["all_passed"] is True
        assert len(rep["targets"]) == 5

        for target in rep["targets"]:
            assert target["obj_file"]["has_mtllib_reference"] is True
            assert target["mtl_file"]["references_texture_atlas"] is True
            assert target["texture_atlas"]["is_readable"] is True
            assert target["bundle_integrity_status"] == "PASS"

    def test_04_live_viewer_and_asset_endpoints(self):
        # Query root
        req_root = urllib.request.urlopen("http://127.0.0.1:5173/")
        assert req_root.getcode() == 200
        html = req_root.read().decode("utf-8")
        assert "F.O.R.G.E Explorer" in html or "html" in html.lower()

        # Query manifest
        req_man = urllib.request.urlopen("http://127.0.0.1:5173/model-manifest.json")
        assert req_man.getcode() == 200
        man = json.loads(req_man.read().decode("utf-8"))
        assert len(man["scenes"]) == 5

        # Query a Sprint 9 model asset
        req_obj = urllib.request.urlopen("http://127.0.0.1:5173/models/desert_road/candidate_b/textured_mesh.obj")
        assert req_obj.getcode() == 200
        first_chunk = req_obj.read(500).decode("utf-8")
        assert "mtllib textured_mesh.mtl" in first_chunk
