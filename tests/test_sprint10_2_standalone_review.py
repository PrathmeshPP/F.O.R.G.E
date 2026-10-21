#!/usr/bin/env python3
"""
Unit and regression tests for F.O.R.G.E Sprint 10.2:
Standalone 3D Model Deliverables and Manual Inspection.

Validates:
1. Model file existence and nonzero size for all 5 scenes.
2. Independent 3D mesh loading (trimesh and open3d).
3. Nonempty vertices and faces for all triangle meshes.
4. Finite coordinates across all vertices (0 NaNs, 0 Infs).
5. Valid texture references and resolving MTL paths.
6. Correct relative paths inside the export package.
7. Successful reload of saved GLB and OBJ exports.
8. Correct model catalog entries and matching SHA-256 hashes.
9. Master review ZIP package existence, nonzero size, and valid structure.
"""

import json
import hashlib
import zipfile
from pathlib import Path
import pytest
import numpy as np
import trimesh
import open3d as o3d
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REVIEW_DIR = PROJECT_ROOT / "outputs/manual_model_review"
ZIP_PATH = REVIEW_DIR / "forge_models_for_manual_inspection.zip"
CATALOG_PATH = REVIEW_DIR / "MODEL_CATALOG.md"
INTEGRITY_REPORT_PATH = REVIEW_DIR / "model_integrity_report.json"

SCENE_KEYS = [
    "campus_0049",
    "desert_road_candidate_a",
    "desert_road_candidate_b",
    "forest_road",
    "river_bridge",
]

EXPECTED_ARTIFACT_NAMES = [
    "model_geometry.ply",
    "model_geometry.obj",
    "model_photographic.ply",
    "model_textured.obj",
    "model_textured.mtl",
    "texture_atlas.png",
    "model_preview.glb",
    "render_overview.png",
    "render_oblique.png",
    "metadata.json",
]


class TestSprint10_2StandaloneReview:

    def test_01_all_scene_directories_and_files_exist_with_nonzero_size(self):
        """Verify that every scene folder exists and contains all required artifacts with nonzero size."""
        assert REVIEW_DIR.is_dir(), "outputs/manual_model_review directory must exist"

        for sc in SCENE_KEYS:
            sc_dir = REVIEW_DIR / sc
            assert sc_dir.is_dir(), f"Scene directory missing: {sc_dir}"

            for art_name in EXPECTED_ARTIFACT_NAMES:
                art_p = sc_dir / art_name
                assert art_p.is_file(), f"Artifact missing: {art_p}"
                sz = art_p.stat().st_size
                assert sz > 0, f"Artifact is empty (0 bytes): {art_p}"

    def test_02_mesh_loading_nonempty_and_positive_extents(self):
        """Verify that geometry, photographic, and textured meshes load with vertices, faces, and positive 3D dimensions."""
        for sc in SCENE_KEYS:
            sc_dir = REVIEW_DIR / sc
            
            # 1. Untextured geometric PLY
            p_geom = sc_dir / "model_geometry.ply"
            m_geom = trimesh.load(str(p_geom), process=False)
            assert len(m_geom.vertices) >= 50000, f"Too few vertices in {p_geom}: {len(m_geom.vertices)}"
            assert len(m_geom.faces) >= 100000, f"Too few faces in {p_geom}: {len(m_geom.faces)}"
            ext_geom = m_geom.extents
            assert np.all(ext_geom > 0.2), f"Non-positive bounding box in {p_geom}: {ext_geom}"

            # 2. Untextured geometric OBJ
            p_obj = sc_dir / "model_geometry.obj"
            m_obj = o3d.io.read_triangle_mesh(str(p_obj))
            assert len(m_obj.vertices) == len(m_geom.vertices), f"Vertex count mismatch between PLY and OBJ for {sc}"
            assert len(m_obj.triangles) == len(m_geom.faces), f"Face count mismatch between PLY and OBJ for {sc}"

            # 3. Photographic PLY (vertex colors)
            p_photo = sc_dir / "model_photographic.ply"
            m_photo = o3d.io.read_triangle_mesh(str(p_photo))
            assert len(m_photo.vertices) == len(m_geom.vertices)
            assert m_photo.has_vertex_colors(), f"{p_photo} must contain vertex colors"

            # 4. Self-contained GLB
            p_glb = sc_dir / "model_preview.glb"
            m_glb = trimesh.load(str(p_glb), file_type="glb")
            assert isinstance(m_glb, (trimesh.Trimesh, trimesh.Scene))
            if isinstance(m_glb, trimesh.Scene):
                assert len(m_glb.geometry) >= 1
                first_geom = list(m_glb.geometry.values())[0]
                assert len(first_geom.vertices) > 0
                assert len(first_geom.faces) > 0

    def test_03_all_coordinates_finite(self):
        """Verify that no vertex coordinates contain NaNs or Infs."""
        for sc in SCENE_KEYS:
            p_geom = REVIEW_DIR / sc / "model_geometry.ply"
            m = trimesh.load(str(p_geom), process=False)
            assert np.all(np.isfinite(m.vertices)), f"Non-finite vertex coordinates found in {p_geom}"

    def test_04_texture_references_and_materials_valid(self):
        """Verify that model_textured.obj points to model_textured.mtl and texture_atlas.png is readable."""
        for sc in SCENE_KEYS:
            sc_dir = REVIEW_DIR / sc

            # Check OBJ mtllib line
            obj_p = sc_dir / "model_textured.obj"
            found_mtllib = False
            with open(obj_p, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    if line.strip().startswith("mtllib model_textured.mtl"):
                        found_mtllib = True
                        break
            assert found_mtllib, f"{obj_p} must reference 'mtllib model_textured.mtl'"

            # Check MTL map_Kd line
            mtl_p = sc_dir / "model_textured.mtl"
            found_tex = False
            with open(mtl_p, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    if "map_Kd texture_atlas.png" in line:
                        found_tex = True
                        break
            assert found_tex, f"{mtl_p} must reference 'map_Kd texture_atlas.png'"

            # Check texture atlas image validity
            tex_p = sc_dir / "texture_atlas.png"
            assert tex_p.is_file()
            with Image.open(tex_p) as img:
                assert img.width == 4096 and img.height == 4096, f"Expected 4096x4096 atlas, got {img.size}"

    def test_05_zip_package_integrity(self):
        """Verify that the master inspection ZIP exists and contains all 5 scenes with valid files."""
        assert ZIP_PATH.is_file(), f"ZIP package missing: {ZIP_PATH}"
        assert ZIP_PATH.stat().st_size > 50 * 1024 * 1024, "ZIP package appears too small (< 50 MB)"

        with zipfile.ZipFile(ZIP_PATH, "r") as zf:
            namelist = zf.namelist()
            assert "README.txt" in namelist

            for sc in SCENE_KEYS:
                for art in EXPECTED_ARTIFACT_NAMES:
                    expected_arc = f"{sc}/{art}"
                    assert expected_arc in namelist, f"ZIP missing expected file: {expected_arc}"

            # Test ZIP file integrity
            bad_file = zf.testzip()
            assert bad_file is None, f"Corrupted file in ZIP: {bad_file}"

    def test_06_model_catalog_and_integrity_report(self):
        """Verify that MODEL_CATALOG.md and model_integrity_report.json match actual disk files."""
        assert CATALOG_PATH.is_file(), "MODEL_CATALOG.md must exist"
        cat_text = CATALOG_PATH.read_text(encoding="utf-8")
        assert "Windows 3D Viewer" in cat_text
        assert "Blender" in cat_text
        assert "MeshLab" in cat_text
        assert "CloudCompare" in cat_text

        assert INTEGRITY_REPORT_PATH.is_file(), "model_integrity_report.json must exist"
        with open(INTEGRITY_REPORT_PATH, "r", encoding="utf-8") as f:
            rep = json.load(f)

        assert rep["all_scenes_passed"] is True
        assert len(rep["scenes"]) == 5

        # Verify hashes recorded in report match files on disk
        for sc_data in rep["scenes"]:
            for art in sc_data["artifacts"]:
                disk_p = Path(art["absolute_path"])
                assert disk_p.is_file(), f"Report references missing file: {disk_p}"
                
                h = hashlib.sha256(disk_p.read_bytes()).hexdigest()
                assert h == art["sha256"], f"Hash mismatch for {disk_p}: report={art['sha256']}, disk={h}"

    def test_07_source_video_traceability(self):
        """Verify that every scene references a valid existing source video dataset."""
        with open(INTEGRITY_REPORT_PATH, "r", encoding="utf-8") as f:
            rep = json.load(f)

        for sc_data in rep["scenes"]:
            vinfo = sc_data["source_video_provenance"]
            assert vinfo["name"], "Source video name must be recorded"
            assert vinfo["resolution"] in ["1280x720", "3840x2160"]
            assert vinfo["frame_count"] > 0
            assert len(vinfo["sha256"]) == 64
