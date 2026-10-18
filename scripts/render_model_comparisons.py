"""Renders consistent top, frontal, and oblique screenshots of original vs corrected models.

Ensures identical camera framing, bounding box centers, and visual styles for side-by-side inspection.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Tuple, List, Dict, Any

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont


def capture_mesh_or_pcd_views(
    geom_path: str,
    bbox_ref: o3d.geometry.AxisAlignedBoundingBox,
    is_mesh: bool = False,
    width: int = 800,
    height: int = 600,
) -> Dict[str, Image.Image]:
    """Captures Top, Frontal, and Oblique views of a geometry using a fixed reference bounding box."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)

    if is_mesh:
        geom = o3d.io.read_triangle_mesh(geom_path)
        geom.compute_vertex_normals()
    else:
        geom = o3d.io.read_point_cloud(geom_path)

    vis.add_geometry(geom)

    # Use reference bounding box for consistent camera placement
    ctr = bbox_ref.get_center()
    ext = bbox_ref.get_extent()
    max_dim = max(ext)

    views = {}
    view_configs = [
        ("top", [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),       # Looking down along +Y
        ("frontal", [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]),   # Looking along +Z
        ("oblique", [0.6, -0.7, -0.6], [0.0, -1.0, 0.0]),  # 45-deg oblique
    ]

    for v_name, front, up in view_configs:
        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr)
        ctrl.set_front(front)
        ctrl.set_up(up)
        ctrl.set_zoom(0.7)

        vis.poll_events()
        vis.update_renderer()

        img_buf = vis.capture_screen_float_buffer(do_render=True)
        img_np = (np.asarray(img_buf) * 255).astype(np.uint8)
        views[v_name] = Image.fromarray(img_np)

    vis.destroy_window()
    return views


def render_comparative_inspection_sheet(
    orig_pcd_path: str,
    corr_pcd_path: str,
    orig_mesh_path: str,
    corr_mesh_path: str,
    output_png: str,
    title: str,
) -> None:
    """Renders a 4-row x 3-column inspection sheet comparing Top, Frontal, and Oblique views."""
    # 1. Compute common reference bounding box from the corrected point cloud
    corr_pcd = o3d.io.read_point_cloud(corr_pcd_path)
    ref_bbox = corr_pcd.get_axis_aligned_bounding_box()

    print(f"Rendering views for: {title}...")
    print("  Capturing original PCD views...")
    views_orig_pcd = capture_mesh_or_pcd_views(orig_pcd_path, ref_bbox, is_mesh=False)
    print("  Capturing corrected PCD views...")
    views_corr_pcd = capture_mesh_or_pcd_views(corr_pcd_path, ref_bbox, is_mesh=False)

    print("  Capturing original Mesh views...")
    views_orig_mesh = capture_mesh_or_pcd_views(orig_mesh_path, ref_bbox, is_mesh=True)
    print("  Capturing corrected Mesh views...")
    views_corr_mesh = capture_mesh_or_pcd_views(corr_mesh_path, ref_bbox, is_mesh=True)

    # Compose sheet
    w_tile, h_tile = 800, 600
    sheet_w = w_tile * 3
    sheet_h = h_tile * 4 + 100

    sheet = Image.new("RGB", (sheet_w, sheet_h), (15, 18, 25))
    draw = ImageDraw.Draw(sheet)

    # Title header
    draw.rectangle([0, 0, sheet_w, 90], fill=(20, 26, 36))
    draw.text((30, 20), f"F.O.R.G.E — Visual Model Inspection & Comparison Sheet", fill=(255, 255, 255))
    draw.text((30, 50), f"Dataset: {title} | Consistent Framing (Top, Frontal, Oblique)", fill=(120, 220, 255))

    rows = [
        ("A. ORIGINAL POINT CLOUD (Baseline)", views_orig_pcd, (255, 200, 100)),
        ("B. ACCURACY-CORRECTED POINT CLOUD", views_corr_pcd, (100, 255, 180)),
        ("C. ORIGINAL SURFACE MESH (Baseline)", views_orig_mesh, (255, 180, 120)),
        ("D. ACCURACY-CORRECTED SURFACE MESH", views_corr_mesh, (120, 220, 255)),
    ]

    for r_idx, (r_label, v_dict, col) in enumerate(rows):
        y_top = 95 + r_idx * h_tile
        for c_idx, v_name in enumerate(["top", "frontal", "oblique"]):
            x_left = c_idx * w_tile
            tile = v_dict[v_name]
            sheet.paste(tile, (x_left, y_top))

            # Tile border & label
            t_draw = ImageDraw.Draw(sheet)
            t_draw.rectangle([x_left + 2, y_top + 2, x_left + w_tile - 2, y_top + h_tile - 2], outline=(35, 45, 60))
            t_draw.rectangle([x_left + 15, y_top + 15, x_left + 420, y_top + 55], fill=(0, 0, 0))
            t_draw.text((x_left + 25, y_top + 20), f"{r_label}", fill=col)
            t_draw.text((x_left + 25, y_top + 36), f"View: {v_name.capitalize()}", fill=(200, 210, 225))

    sheet.save(output_png, format="PNG")
    print(f"  Saved visual inspection sheet: {output_png}")


def main() -> None:
    out_dir = Path("outputs/accuracy_experiment")
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Candidate 0050 (Bridge)
    render_comparative_inspection_sheet(
        orig_pcd_path="outputs/dji_river_bridge_validation/reconstruction/point_cloud.ply",
        corr_pcd_path="outputs/accuracy_experiment/bridge_0050/point_cloud_corrected.ply",
        orig_mesh_path="outputs/dji_river_bridge_validation/reconstruction/mesh_experiment_v2/surface_mesh.ply",
        corr_mesh_path="outputs/accuracy_experiment/bridge_0050/surface_mesh_corrected.ply",
        output_png=str(out_dir / "bridge_0050_visual_inspection.png"),
        title="Candidate 0050 (River/Bridge Corridor)"
    )

    # 2. Candidate 0049 (Campus)
    render_comparative_inspection_sheet(
        orig_pcd_path="outputs/dji_0049_validation/reconstruction/point_cloud.ply",
        corr_pcd_path="outputs/accuracy_experiment/campus_0049/point_cloud_corrected.ply",
        orig_mesh_path="outputs/dji_0049_validation/reconstruction/surface_mesh.ply",
        corr_mesh_path="outputs/accuracy_experiment/campus_0049/surface_mesh_corrected.ply",
        output_png=str(out_dir / "campus_0049_visual_inspection.png"),
        title="Candidate 0049 (Campus Oblique Area)"
    )


if __name__ == "__main__":
    main()
