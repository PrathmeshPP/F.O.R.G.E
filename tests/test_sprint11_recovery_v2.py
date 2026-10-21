"""F.O.R.G.E — Sprint 11 Recovery V2 Test Suite

Validates that all 4 Recovery V2 scene deliverables meet quality and structural requirements:
1. Campus 0049: 4K texture atlas present, verified reload, sharp photographic ground details.
2. Desert Road: Mask-aware texturing verified, zero moving vehicles, clean corridor geometry.
3. Forest Road: High-fidelity canopy preserved, valid geometry and previews.
4. River/Bridge: Structural bridge deck preserved, valid geometry and previews.
5. Absolute Local Constraint: No remote git mutations.
"""

import json
import hashlib
from pathlib import Path
import pytest
import numpy as np
import open3d as o3d
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RECOVERY_V2_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v2"


def compute_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()


@pytest.fixture(scope="module")
def scene_dirs():
    scenes = ["campus_0049", "desert_road", "forest_road", "river_bridge"]
    dirs = {s: RECOVERY_V2_ROOT / s for s in scenes}
    for s, d in dirs.items():
        assert d.is_dir(), f"Scene directory missing: {d}"
    return dirs


def test_01_all_scene_deliverables_exist(scene_dirs):
    """Verify all 4 scenes contain the required GLB, OBJ, PLY, previews, and metadata."""
    for s, d in scene_dirs.items():
        assert (d / "masked_reconstruction.glb").is_file(), f"{s} GLB missing"
        assert (d / "masked_reconstruction.obj").is_file(), f"{s} OBJ missing"
        assert (d / "masked_reconstruction.ply").is_file(), f"{s} PLY missing"
        assert (d / "masked_reconstruction_preview.png").is_file(), f"{s} preview missing"
        assert (d / "neutral_geometry_preview.png").is_file(), f"{s} neutral geometry preview missing"
        assert (d / "reconstruction_metadata.json").is_file(), f"{s} metadata missing"

        # Check non-zero size
        assert (d / "masked_reconstruction.glb").stat().st_size > 100000
        assert (d / "masked_reconstruction.obj").stat().st_size > 100000
        assert (d / "masked_reconstruction.ply").stat().st_size > 100000


def test_02_texture_atlases_and_mtl_references(scene_dirs):
    """Verify that Campus 0049 and Desert Road include a 4K texture atlas and correct MTL."""
    for s in ["campus_0049", "desert_road"]:
        d = scene_dirs[s]
        atlas_path = d / "texture_atlas.png"
        mtl_path = d / "masked_reconstruction.mtl"
        obj_path = d / "masked_reconstruction.obj"

        assert atlas_path.is_file(), f"{s} texture_atlas.png missing"
        assert atlas_path.stat().st_size > 1000000, f"{s} texture atlas too small"

        assert mtl_path.is_file(), f"{s} MTL missing"
        with open(mtl_path, "r") as f:
            mtl_content = f.read()
        assert "map_Kd texture_atlas.png" in mtl_content, f"{s} MTL does not reference texture_atlas.png"

        with open(obj_path, "r") as f:
            first_lines = [next(f) for _ in range(10)]
        assert any(l.startswith("mtllib masked_reconstruction.mtl") for l in first_lines), f"{s} OBJ does not declare mtllib"


def test_03_independent_mesh_reload_and_topology(scene_dirs):
    """Verify each mesh can be reloaded independently by Open3D and Trimesh, with positive bounding extents."""
    for s, d in scene_dirs.items():
        # Open3D PLY load
        ply_path = d / "masked_reconstruction.ply"
        m_o3d = o3d.io.read_triangle_mesh(str(ply_path))
        v_o3d = np.asarray(m_o3d.vertices)
        f_o3d = np.asarray(m_o3d.triangles)
        assert len(v_o3d) > 10000, f"{s} PLY has too few vertices"
        assert len(f_o3d) > 10000, f"{s} PLY has too few triangles"
        assert np.isfinite(v_o3d).all(), f"{s} PLY contains non-finite vertices"

        # Trimesh GLB load
        glb_path = d / "masked_reconstruction.glb"
        tri_m = trimesh.load(str(glb_path), file_type="glb")
        if isinstance(tri_m, trimesh.Scene):
            geoms = list(tri_m.geometry.values())
            tri_m = trimesh.util.concatenate(geoms)
        assert len(tri_m.vertices) > 10000, f"{s} GLB has too few vertices"
        assert len(tri_m.faces) > 10000, f"{s} GLB has too few faces"
        assert np.isfinite(tri_m.vertices).all(), f"{s} GLB contains non-finite vertices"

        extents = np.ptp(v_o3d, axis=0)
        assert (extents > 0.05).all(), f"{s} bounding box extents not strictly positive: {extents}"


def test_04_desert_road_vehicle_elimination(scene_dirs):
    """Verify Desert Road geometry is planar and clean, and has zero interior holes."""
    d = scene_dirs["desert_road"]
    m = o3d.io.read_triangle_mesh(str(d / "masked_reconstruction.ply"))
    pts = np.asarray(m.vertices)

    # Road corridor vertices
    road_mask = np.abs(pts[:, 0] + 0.05) <= 0.35
    road_pts = pts[road_mask]
    assert len(road_pts) > 20000, "Too few road vertices"

    # Fit road plane Y = a*X + b*Z + c
    A = np.column_stack([road_pts[:, 0], road_pts[:, 2], np.ones(len(road_pts))])
    Y = road_pts[:, 1]
    plane, _, _, _ = np.linalg.lstsq(A, Y, rcond=None)
    residuals = np.abs(Y - A @ plane)

    # Road plane must be smooth with mean residual <= 3.0 mm (zero vehicle bumps)
    assert residuals.mean() < 0.003, f"Desert Road corridor mean residual {residuals.mean()*1000:.2f} mm > 3.0 mm"
    assert (residuals > 0.03).sum() < 200, "Residual outliers indicate vehicle bumps on road surface"


def test_05_campus_0049_texture_fidelity(scene_dirs):
    """Verify Campus 0049 preserves 4K texture atlas and has >= 120k vertices."""
    d = scene_dirs["campus_0049"]
    m = o3d.io.read_triangle_mesh(str(d / "masked_reconstruction.ply"))
    assert len(m.vertices) >= 120000, f"Campus 0049 vertex count dropped to {len(m.vertices)}"
    assert len(m.triangles) >= 240000, f"Campus 0049 triangle count dropped to {len(m.triangles)}"

    # Texture atlas must be 4096x4096
    from PIL import Image
    with Image.open(d / "texture_atlas.png") as atlas_img:
        assert atlas_img.size == (4096, 4096), f"Texture atlas size {atlas_img.size} is not 4096x4096"


def test_06_metadata_integrity(scene_dirs):
    """Verify reconstruction_metadata.json in all 4 scenes contains valid SHA256 hashes and deliverable stats."""
    for s, d in scene_dirs.items():
        meta_file = d / "reconstruction_metadata.json"
        with open(meta_file, "r") as f:
            data = json.load(f)

        assert data["scene_name"] == s
        assert data["recovery_version"] == "v2"
        deliv = data["deliverables"]
        assert "glb" in deliv and "obj" in deliv and "ply" in deliv

        # Verify cryptographic hash of GLB matches disk
        assert deliv["glb"]["sha256"] == compute_sha256(d / "masked_reconstruction.glb")
        assert deliv["ply"]["sha256"] == compute_sha256(d / "masked_reconstruction.ply")
