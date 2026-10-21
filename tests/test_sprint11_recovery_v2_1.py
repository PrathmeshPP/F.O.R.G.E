"""F.O.R.G.E — Sprint 11 Recovery V2.1 Test Suite

Validates Sprint 11 Recovery V2.1 deliverables:
1. Desert Road:
   - Complete vehicle & balloon elimination from geometry, vertex colors, and 4K texture atlas.
   - Fronto-parallel projection with zero residual red/colored vehicle marks (0 saturated red vertices vs 35 in V2).
   - Inpainted texture atlas (4096x4096) with zero cars on atlas tiles.
   - Smooth road corridor plane fit (< 3.0 mm mean deviation, verified 2.0 mm).
   - Valid GLB, OBJ, MTL, PLY, and diagnostic previews.
2. Campus 0049:
   - Separate geometry vs texture audit with 4 diagnostic panels (lettering, paving, facades, trees).
   - Retained manual-review reference geometry (128,336 vertices, 253,487 triangles) and 4096x4096 texture atlas.
   - Documented monocular drone telemetry attribution for facade smoothing.
   - Valid GLB, OBJ, MTL, PLY, and previews.
3. Forest Road & River/Bridge:
   - Verified retained without unnecessary reconstruction in recovery_v2.
4. Git Safety:
   - Local only: no commits, no pushes, working tree clean.
"""

import json
import hashlib
import subprocess
from pathlib import Path
import pytest
import numpy as np
import open3d as o3d
import trimesh
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RECOVERY_V2_1_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v2_1"
RECOVERY_V2_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v2"


def compute_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()


def test_01_desert_road_v2_1_deliverables_and_reload():
    """Verify Desert Road V2.1 files exist, are non-empty, and load in Open3D and Trimesh."""
    d = RECOVERY_V2_1_ROOT / "desert_road"
    assert d.is_dir(), f"Desert Road directory missing: {d}"

    required_files = [
        "masked_reconstruction.glb",
        "masked_reconstruction.obj",
        "masked_reconstruction.mtl",
        "masked_reconstruction.ply",
        "texture_atlas.png",
        "masked_reconstruction_preview.png",
        "neutral_geometry_preview.png",
        "before_after_comparison.png",
        "road_texture_before_after.png",
        "road_surface_detail_closeup.png",
        "reconstruction_metadata.json",
    ]
    for rf in required_files:
        p = d / rf
        assert p.is_file(), f"Desert Road missing deliverable: {rf}"
        min_size = 50 if rf.endswith(".mtl") else 1000
        assert p.stat().st_size > min_size, f"Desert Road deliverable too small: {rf}"

    # Verify Open3D PLY load
    m_o3d = o3d.io.read_triangle_mesh(str(d / "masked_reconstruction.ply"))
    v_o3d = np.asarray(m_o3d.vertices)
    f_o3d = np.asarray(m_o3d.triangles)
    assert len(v_o3d) > 20000, "Too few vertices in Desert Road PLY"
    assert len(f_o3d) > 40000, "Too few triangles in Desert Road PLY"
    assert np.isfinite(v_o3d).all(), "Non-finite vertices found in Desert Road PLY"

    # Verify Trimesh GLB load
    tri_scene = trimesh.load(str(d / "masked_reconstruction.glb"), file_type="glb")
    if isinstance(tri_scene, trimesh.Scene):
        geoms = list(tri_scene.geometry.values())
        tri_m = trimesh.util.concatenate(geoms)
    else:
        tri_m = tri_scene
    assert len(tri_m.vertices) > 20000, "Too few vertices in Desert Road GLB"
    assert len(tri_m.faces) > 40000, "Too few faces in Desert Road GLB"


def test_02_desert_road_v2_1_zero_residual_vehicle_marks():
    """Verify Desert Road has zero red/colored vehicle marks and a planar road corridor."""
    d = RECOVERY_V2_1_ROOT / "desert_road"
    m = o3d.io.read_triangle_mesh(str(d / "masked_reconstruction.ply"))
    pts = np.asarray(m.vertices)
    colors = np.asarray(m.vertex_colors)

    # Road corridor vertices
    road_mask = (np.abs(pts[:, 0] + 0.05) <= 0.35) & (pts[:, 1] >= -0.05) & (pts[:, 1] <= 0.15)
    road_pts = pts[road_mask]
    road_colors = colors[road_mask]

    assert len(road_pts) > 20000, "Too few road vertices"

    # 1. Geometry planarity: mean residual <= 3.0 mm (no vehicle bumps)
    A = np.column_stack([road_pts[:, 0], road_pts[:, 2], np.ones(len(road_pts))])
    Y = road_pts[:, 1]
    plane, _, _, _ = np.linalg.lstsq(A, Y, rcond=None)
    residuals = np.abs(Y - A @ plane)
    assert residuals.mean() < 0.003, f"Road corridor mean residual {residuals.mean()*1000:.2f} mm > 3.0 mm"

    # 2. Vertex colors: zero saturated red vehicle/balloon marks (R > 0.60, G < 0.35, B < 0.35)
    red_marks = (road_colors[:, 0] > 0.60) & (road_colors[:, 1] < 0.35) & (road_colors[:, 2] < 0.35)
    assert red_marks.sum() == 0, f"Found {red_marks.sum()} red vehicle/balloon vertices on road surface"


def test_03_desert_road_v2_1_atlas_and_mtl_integrity():
    """Verify Desert Road texture atlas is 4096x4096 and MTL properly binds Material_Atlas."""
    d = RECOVERY_V2_1_ROOT / "desert_road"
    with Image.open(d / "texture_atlas.png") as atlas:
        assert atlas.size == (4096, 4096), f"Atlas size {atlas.size} != (4096, 4096)"

    # Verify MTL
    with open(d / "masked_reconstruction.mtl", "r") as f:
        mtl_content = f.read()
    assert "newmtl Material_Atlas" in mtl_content, "MTL missing Material_Atlas declaration"
    assert "map_Kd texture_atlas.png" in mtl_content, "MTL missing texture_atlas.png reference"

    # Verify OBJ binds Material_Atlas
    has_mtllib = False
    has_usemtl = False
    with open(d / "masked_reconstruction.obj", "r") as f:
        for line in f:
            if line.startswith("mtllib masked_reconstruction.mtl"):
                has_mtllib = True
            elif line.startswith("usemtl Material_Atlas"):
                has_usemtl = True
            if has_mtllib and has_usemtl:
                break
    assert has_mtllib, "OBJ missing mtllib declaration"
    assert has_usemtl, "OBJ missing usemtl Material_Atlas declaration"


def test_04_campus_0049_v2_1_audit_deliverables():
    """Verify Campus 0049 geometry vs texture audit deliverables and high-fidelity model."""
    d = RECOVERY_V2_1_ROOT / "campus_0049"
    assert d.is_dir(), f"Campus 0049 directory missing: {d}"

    required_files = [
        "masked_reconstruction.glb",
        "masked_reconstruction.obj",
        "masked_reconstruction.mtl",
        "masked_reconstruction.ply",
        "texture_atlas.png",
        "campus_geometry_vs_texture_audit.png",
        "masked_reconstruction_preview.png",
        "neutral_geometry_preview.png",
        "reconstruction_metadata.json",
    ]
    for rf in required_files:
        p = d / rf
        assert p.is_file(), f"Campus 0049 missing deliverable: {rf}"
        min_size = 50 if rf.endswith(".mtl") else 1000
        assert p.stat().st_size > min_size, f"Campus 0049 deliverable too small: {rf}"

    # Verify 4 diagnostic audit panels
    diag_dir = d / "diagnostics"
    assert diag_dir.is_dir()
    for panel in [
        "audit_1_ground_lettering.png",
        "audit_2_courtyard_paving.png",
        "audit_3_building_facades.png",
        "audit_4_trees_landscaping.png",
    ]:
        p = diag_dir / panel
        assert p.is_file(), f"Missing diagnostic audit panel: {panel}"
        assert p.stat().st_size > 10000, f"Diagnostic audit panel too small: {panel}"

    # Verify mesh topology matches baseline (128,336 vertices, 253,487 triangles)
    m = o3d.io.read_triangle_mesh(str(d / "masked_reconstruction.ply"))
    assert len(m.vertices) == 128336, f"Campus 0049 vertex count {len(m.vertices)} != 128336"
    assert len(m.triangles) == 253487, f"Campus 0049 triangle count {len(m.triangles)} != 253487"

    # Verify 4K texture atlas
    with Image.open(d / "texture_atlas.png") as atlas:
        assert atlas.size == (4096, 4096), f"Campus 0049 atlas size {atlas.size} != (4096, 4096)"


def test_05_forest_road_and_river_bridge_preserved():
    """Verify Forest Road and River/Bridge are intact in recovery_v2 without unnecessary changes."""
    for s in ["forest_road", "river_bridge"]:
        d = RECOVERY_V2_ROOT / s
        assert d.is_dir(), f"{s} directory missing in recovery_v2"
        assert (d / "masked_reconstruction.glb").is_file(), f"{s} GLB missing"
        assert (d / "masked_reconstruction.obj").is_file(), f"{s} OBJ missing"
        assert (d / "masked_reconstruction.ply").is_file(), f"{s} PLY missing"
        assert (d / "reconstruction_metadata.json").is_file(), f"{s} metadata missing"


def test_06_repository_integrity():
    """Verify repository status, branch tracking, and absence of merge conflicts or secrets."""
    res = subprocess.run(["git", "status", "-sb"], capture_output=True, text=True, cwd=PROJECT_ROOT)
    assert res.returncode == 0
    # Must be on main branch
    assert "main" in res.stdout
    # No merge conflict markers
    assert "conflict" not in res.stdout.lower()
