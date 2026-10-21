"""
F.O.R.G.E — Sprint 11: Quantitative & Visual Regression Analysis
Audit Sprint 9 vs Sprint 11 (Unmasked and Masked) across all four scenes.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Any

import numpy as np
import open3d as o3d
import trimesh
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parent.parent

SPRINT9_ROOT = PROJECT_ROOT / "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
SPRINT11_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11"
OUT_AUDIT_DIR = SPRINT11_ROOT / "regression_audit"
OUT_AUDIT_DIR.mkdir(parents=True, exist_ok=True)


def analyze_mesh(mesh_path: Path) -> Dict[str, Any]:
    if not mesh_path.is_file():
        return {"error": f"File not found: {mesh_path}"}

    size_bytes = mesh_path.stat().st_size
    m = o3d.io.read_triangle_mesh(str(mesh_path))
    v = np.asarray(m.vertices)
    f = np.asarray(m.triangles)

    if len(v) == 0:
        return {"error": "Mesh has zero vertices", "size_bytes": size_bytes}

    extents = np.ptp(v, axis=0) if len(v) > 0 else np.zeros(3)
    bbox_min = v.min(axis=0) if len(v) > 0 else np.zeros(3)
    bbox_max = v.max(axis=0) if len(v) > 0 else np.zeros(3)

    # Connected components
    clusters, num_faces, _ = m.cluster_connected_triangles()
    num_components = len(num_faces) if len(num_faces) > 0 else 0

    # Degenerate triangles (duplicate vertex indices)
    degen_indices = (f[:, 0] == f[:, 1]) | (f[:, 1] == f[:, 2]) | (f[:, 2] == f[:, 0])
    degen_tris = int(np.count_nonzero(degen_indices))
    
    # Non-manifold edges
    non_manifold_e = len(m.get_non_manifold_edges(allow_boundary_edges=False))
    
    # Edge manifold
    is_edge_m = m.is_edge_manifold()
    is_vert_m = m.is_vertex_manifold()

    return {
        "file_name": mesh_path.name,
        "path": str(mesh_path.resolve()),
        "size_bytes": size_bytes,
        "size_mb": round(size_bytes / (1024 * 1024), 3),
        "vertex_count": len(v),
        "triangle_count": len(f),
        "extents": extents.round(4).tolist(),
        "bbox_min": bbox_min.round(4).tolist(),
        "bbox_max": bbox_max.round(4).tolist(),
        "num_connected_components": num_components,
        "degenerate_triangles": degen_tris,
        "non_manifold_edges": non_manifold_e,
        "is_edge_manifold": bool(is_edge_m),
        "is_vertex_manifold": bool(is_vert_m),
        "has_vertex_colors": bool(m.has_vertex_colors())
    }


def render_mesh_to_image(
    mesh_path: Path,
    view_front: list,
    view_up: list,
    lookat: list,
    zoom: float = 0.7,
    width: int = 800,
    height: int = 600
) -> Image.Image:
    if not mesh_path.is_file():
        img = Image.new("RGB", (width, height), (30, 30, 30))
        d = ImageDraw.Draw(img)
        d.text((width // 4, height // 2), f"Missing: {mesh_path.name}", fill=(255, 100, 100))
        return img

    m = o3d.io.read_triangle_mesh(str(mesh_path))
    m.compute_vertex_normals()

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    vis.add_geometry(m)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(lookat)
    ctrl.set_front(view_front)
    ctrl.set_up(view_up)
    ctrl.set_zoom(zoom)

    vis.poll_events()
    vis.update_renderer()
    buf = vis.capture_screen_float_buffer(do_render=True)
    vis.destroy_window()

    return Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))


def main():
    print("=" * 80)
    print("F.O.R.G.E Sprint 11 — Quantitative & Visual Regression Analysis")
    print("=" * 80)

    scenes = {
        "desert_road": {
            "sprint9_mesh": SPRINT9_ROOT / "desert_road/candidate_a/surface_mesh_repaired.ply",
            "sprint9_cand_b": SPRINT9_ROOT / "desert_road/candidate_b/surface_mesh_repaired.ply",
            "sprint11_unmasked": SPRINT11_ROOT / "desert_road/unmasked_reconstruction.ply",
            "sprint11_masked": SPRINT11_ROOT / "desert_road/masked_reconstruction.ply",
            "view_front": [0.0, -0.707, -0.707],
            "view_up": [0.0, -0.707, 0.707],
            "zoom": 0.65
        },
        "campus_0049": {
            "sprint9_mesh": SPRINT9_ROOT / "campus_0049/surface_mesh_repaired.ply",
            "sprint11_unmasked": SPRINT11_ROOT / "campus_0049/unmasked_reconstruction.ply",
            "sprint11_masked": SPRINT11_ROOT / "campus_0049/masked_reconstruction.ply",
            "view_front": [0.25, -0.45, -0.85],
            "view_up": [0.0, -1.0, 0.0],
            "zoom": 0.70
        },
        "forest_road": {
            "sprint9_mesh": SPRINT9_ROOT / "forest_road/surface_mesh_repaired.ply",
            "sprint11_unmasked": SPRINT11_ROOT / "forest_road/unmasked_reconstruction.ply",
            "sprint11_masked": SPRINT11_ROOT / "forest_road/masked_reconstruction.ply",
            "view_front": [0.25, -0.45, -0.85],
            "view_up": [0.0, -1.0, 0.0],
            "zoom": 0.70
        },
        "river_bridge": {
            "sprint9_mesh": SPRINT9_ROOT / "river_bridge/surface_mesh_repaired.ply",
            "sprint11_unmasked": SPRINT11_ROOT / "river_bridge/unmasked_reconstruction.ply",
            "sprint11_masked": SPRINT11_ROOT / "river_bridge/masked_reconstruction.ply",
            "view_front": [0.0, -0.707, -0.707],
            "view_up": [0.0, -0.707, 0.707],
            "zoom": 0.65
        }
    }

    full_audit = {}

    for sname, scfg in scenes.items():
        print(f"\n[*] Auditing Scene: {sname}")
        s9_stats = analyze_mesh(scfg["sprint9_mesh"])
        s11_u_stats = analyze_mesh(scfg["sprint11_unmasked"])
        s11_m_stats = analyze_mesh(scfg["sprint11_masked"])

        full_audit[sname] = {
            "sprint9": s9_stats,
            "sprint11_unmasked": s11_u_stats,
            "sprint11_masked": s11_m_stats
        }

        print(f"    Sprint 9:          {s9_stats.get('vertex_count', 0):,} verts, {s9_stats.get('triangle_count', 0):,} tris | extents: {s9_stats.get('extents', [])} | components: {s9_stats.get('num_connected_components', 0)}")
        print(f"    Sprint 11 Unmasked:{s11_u_stats.get('vertex_count', 0):,} verts, {s11_u_stats.get('triangle_count', 0):,} tris | extents: {s11_u_stats.get('extents', [])} | components: {s11_u_stats.get('num_connected_components', 0)}")
        print(f"    Sprint 11 Masked:  {s11_m_stats.get('vertex_count', 0):,} verts, {s11_m_stats.get('triangle_count', 0):,} tris | extents: {s11_m_stats.get('extents', [])} | components: {s11_m_stats.get('num_connected_components', 0)}")

        # Load Sprint 9 mesh center to align camera lookat
        m_s9 = o3d.io.read_triangle_mesh(str(scfg["sprint9_mesh"]))
        ctr_s9 = m_s9.get_axis_aligned_bounding_box().get_center()

        # Render 3-way matched comparison
        w, h = 800, 600
        im_s9 = render_mesh_to_image(scfg["sprint9_mesh"], scfg["view_front"], scfg["view_up"], ctr_s9, scfg["zoom"], w, h)
        im_s11_u = render_mesh_to_image(scfg["sprint11_unmasked"], scfg["view_front"], scfg["view_up"], ctr_s9, scfg["zoom"], w, h)
        im_s11_m = render_mesh_to_image(scfg["sprint11_masked"], scfg["view_front"], scfg["view_up"], ctr_s9, scfg["zoom"], w, h)

        # Assemble side-by-side banner
        banner_w = w * 3
        banner_h = h + 80
        banner = Image.new("RGB", (banner_w, banner_h), (18, 22, 28))
        draw = ImageDraw.Draw(banner)

        banner.paste(im_s9, (0, 80))
        banner.paste(im_s11_u, (w, 80))
        banner.paste(im_s11_m, (w * 2, 80))

        title = f"F.O.R.G.E Sprint 11 Regression Audit — {sname.upper()} [Matched View Comparison]"
        draw.text((25, 14), title, fill=(255, 255, 255))
        draw.text((25, 42), f"Left: Sprint 9 Verified Baseline | Center: Sprint 11 Unmasked Initial Run | Right: Sprint 11 Masked Initial Run", fill=(180, 210, 240))

        # Column tags
        draw.rectangle([15, 90, 360, 125], fill=(10, 14, 20))
        draw.text((25, 98), f"1. Sprint 9 Baseline ({s9_stats.get('vertex_count', 0):,} verts)", fill=(100, 255, 160))

        draw.rectangle([w + 15, 90, w + 400, 125], fill=(10, 14, 20))
        draw.text((w + 25, 98), f"2. Sprint 11 Unmasked ({s11_u_stats.get('vertex_count', 0):,} verts)", fill=(255, 180, 80))

        draw.rectangle([w * 2 + 15, 90, w * 2 + 400, 125], fill=(10, 14, 20))
        draw.text((w * 2 + 25, 98), f"3. Sprint 11 Masked ({s11_m_stats.get('vertex_count', 0):,} verts)", fill=(255, 100, 100))

        comp_img_path = OUT_AUDIT_DIR / f"{sname}_comparison_baseline_vs_sprint11.png"
        banner.save(comp_img_path)
        print(f"    [+] Saved comparison image: {comp_img_path}")

    # Write audit JSON
    audit_json_path = OUT_AUDIT_DIR / "regression_audit_metrics.json"
    with open(audit_json_path, "w", encoding="utf-8") as f:
        json.dump(full_audit, f, indent=2)
    print(f"\n[+] Saved regression audit report: {audit_json_path}")


if __name__ == "__main__":
    main()
