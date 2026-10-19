"""F.O.R.G.E — Production Validation Visual & Metric Inspection Script.

Renders matched Top, Frontal, and Oblique views comparing:
1. Original Baseline Point Cloud
2. Confidence-Filtered Point Cloud (Unfused, min_confidence=1.5)
3. Fused Point Cloud (voxel_size=0.005, min_confidence=1.5)
4. Reconstructed Surface Mesh

Computes quantitative metrics on spatial coverage, hole presence, point counts,
and colour statistics.
"""

from __future__ import annotations

import os
import sys
import json
from pathlib import Path
from typing import Tuple, List, Dict, Any

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw


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

    ctr = bbox_ref.get_center()

    views = {}
    view_configs = [
        ("top", [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),        # Looking down (+Y down)
        ("frontal", [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]),    # Looking along +Z
        ("oblique", [0.6, -0.7, -0.6], [0.0, -1.0, 0.0]),   # 45-deg oblique
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


def render_4row_inspection_sheet(
    orig_pcd_path: str,
    conf_pcd_path: str,
    fused_pcd_path: str,
    mesh_path: str,
    output_png: str,
    title: str,
    pcd_counts: Dict[str, int],
    mesh_stats: Dict[str, int],
) -> None:
    """Renders a 4-row x 3-column inspection sheet comparing all four stages."""
    # Use fused point cloud for bounding box reference to ensure consistent framing
    fused_pcd = o3d.io.read_point_cloud(fused_pcd_path)
    ref_bbox = fused_pcd.get_axis_aligned_bounding_box()

    print(f"Rendering views for: {title}...")
    print("  Capturing Row 1: Original PCD...")
    v1 = capture_mesh_or_pcd_views(orig_pcd_path, ref_bbox, is_mesh=False)
    print("  Capturing Row 2: Confidence-Filtered PCD...")
    v2 = capture_mesh_or_pcd_views(conf_pcd_path, ref_bbox, is_mesh=False)
    print("  Capturing Row 3: Fused PCD (0.005)...")
    v3 = capture_mesh_or_pcd_views(fused_pcd_path, ref_bbox, is_mesh=False)
    print("  Capturing Row 4: Reconstructed Mesh...")
    v4 = capture_mesh_or_pcd_views(mesh_path, ref_bbox, is_mesh=True)

    w_tile, h_tile = 800, 600
    sheet_w = w_tile * 3
    sheet_h = h_tile * 4 + 100

    sheet = Image.new("RGB", (sheet_w, sheet_h), (14, 17, 24))
    draw = ImageDraw.Draw(sheet)

    # Title header
    draw.rectangle([0, 0, sheet_w, 90], fill=(22, 28, 40))
    draw.text((30, 18), "F.O.R.G.E — Controlled Point-Cloud Accuracy & Mesh Validation", fill=(255, 255, 255))
    draw.text((30, 48), f"Dataset: {title} | Consistent Framing (Top, Frontal, Oblique) | VGGT-Omega Official Convention", fill=(100, 210, 255))

    rows = [
        (f"A. ORIGINAL POINT CLOUD (Baseline) — {pcd_counts['orig']:,} pts", v1, (255, 205, 110)),
        (f"B. CONFIDENCE FILTERED (Unfused, conf>=1.5) — {pcd_counts['conf']:,} pts", v2, (120, 240, 170)),
        (f"C. VOXEL-GRID FUSED (voxel=0.005, conf>=1.5) — {pcd_counts['fused']:,} pts", v3, (90, 210, 255)),
        (f"D. RECONSTRUCTED SURFACE MESH — {mesh_stats['verts']:,} verts, {mesh_stats['faces']:,} faces", v4, (220, 170, 255)),
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
            t_draw.rectangle([x_left + 15, y_top + 15, x_left + 520, y_top + 58], fill=(0, 0, 0))
            t_draw.text((x_left + 25, y_top + 20), f"{r_label}", fill=col)
            t_draw.text((x_left + 25, y_top + 38), f"View Angle: {v_name.capitalize()}", fill=(200, 215, 230))

    sheet.save(output_png, format="PNG")
    print(f"  Saved inspection sheet: {output_png}")


def analyze_model_metrics(
    orig_pcd_path: str,
    conf_pcd_path: str,
    fused_pcd_path: str,
    mesh_path: str,
) -> Dict[str, Any]:
    """Computes comprehensive quantitative metrics on point clouds and mesh."""
    # Point clouds
    pcd_orig = o3d.io.read_point_cloud(orig_pcd_path)
    pts_orig = np.asarray(pcd_orig.points)
    cols_orig = np.asarray(pcd_orig.colors) * 255.0

    pcd_conf = o3d.io.read_point_cloud(conf_pcd_path)
    pts_conf = np.asarray(pcd_conf.points)
    cols_conf = np.asarray(pcd_conf.colors) * 255.0

    pcd_fused = o3d.io.read_point_cloud(fused_pcd_path)
    pts_fused = np.asarray(pcd_fused.points)
    cols_fused = np.asarray(pcd_fused.colors) * 255.0

    # Mesh
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh_verts = np.asarray(mesh.vertices)
    mesh_triangles = np.asarray(mesh.triangles)

    # Edge lengths
    v0 = mesh_verts[mesh_triangles[:, 0]]
    v1 = mesh_verts[mesh_triangles[:, 1]]
    v2 = mesh_verts[mesh_triangles[:, 2]]
    e1 = np.linalg.norm(v1 - v0, axis=1)
    e2 = np.linalg.norm(v2 - v1, axis=1)
    e3 = np.linalg.norm(v0 - v2, axis=1)
    all_edges = np.concatenate([e1, e2, e3])

    return {
        "point_clouds": {
            "original": {
                "count": len(pts_orig),
                "min": pts_orig.min(axis=0).round(4).tolist(),
                "max": pts_orig.max(axis=0).round(4).tolist(),
                "extents": (pts_orig.max(axis=0) - pts_orig.min(axis=0)).round(4).tolist(),
                "mean_rgb": cols_orig.mean(axis=0).round(2).tolist(),
                "std_rgb": cols_orig.std(axis=0).round(2).tolist(),
            },
            "confidence_filtered": {
                "count": len(pts_conf),
                "min": pts_conf.min(axis=0).round(4).tolist(),
                "max": pts_conf.max(axis=0).round(4).tolist(),
                "extents": (pts_conf.max(axis=0) - pts_conf.min(axis=0)).round(4).tolist(),
                "mean_rgb": cols_conf.mean(axis=0).round(2).tolist(),
                "std_rgb": cols_conf.std(axis=0).round(2).tolist(),
                "retention_vs_orig_pct": round(len(pts_conf) / len(pts_orig) * 100, 2),
            },
            "fused": {
                "count": len(pts_fused),
                "min": pts_fused.min(axis=0).round(4).tolist(),
                "max": pts_fused.max(axis=0).round(4).tolist(),
                "extents": (pts_fused.max(axis=0) - pts_fused.min(axis=0)).round(4).tolist(),
                "mean_rgb": cols_fused.mean(axis=0).round(2).tolist(),
                "std_rgb": cols_fused.std(axis=0).round(2).tolist(),
                "reduction_ratio_vs_filtered": round(len(pts_conf) / len(pts_fused), 2),
            },
        },
        "mesh": {
            "vertex_count": len(mesh_verts),
            "face_count": len(mesh_triangles),
            "min": mesh_verts.min(axis=0).round(4).tolist(),
            "max": mesh_verts.max(axis=0).round(4).tolist(),
            "extents": (mesh_verts.max(axis=0) - mesh_verts.min(axis=0)).round(4).tolist(),
            "is_watertight": mesh.is_watertight(),
            "edge_lengths": {
                "mean": round(float(all_edges.mean()), 4),
                "std": round(float(all_edges.std()), 4),
                "min": round(float(all_edges.min()), 4),
                "max": round(float(all_edges.max()), 4),
                "median": round(float(np.median(all_edges)), 4),
            },
        },
    }


def main():
    root = Path(".")
    val_dir = root / "outputs" / "accuracy_experiment" / "production_validation"
    val_dir.mkdir(parents=True, exist_ok=True)

    # 1. Candidate 0050 (River/Bridge)
    b_orig_pcd = str(root / "outputs" / "dji_river_bridge_validation" / "reconstruction" / "point_cloud.ply")
    b_conf_pcd = str(val_dir / "bridge_0050" / "point_cloud_conf_filtered.ply")
    b_fused_pcd = str(val_dir / "bridge_0050" / "point_cloud_fused_005.ply")
    b_mesh = str(val_dir / "bridge_0050" / "mesh" / "surface_mesh.ply")

    b_metrics = analyze_model_metrics(b_orig_pcd, b_conf_pcd, b_fused_pcd, b_mesh)

    render_4row_inspection_sheet(
        orig_pcd_path=b_orig_pcd,
        conf_pcd_path=b_conf_pcd,
        fused_pcd_path=b_fused_pcd,
        mesh_path=b_mesh,
        output_png=str(val_dir / "bridge_0050_production_validation.png"),
        title="Candidate 0050 (River/Bridge 52s–64s)",
        pcd_counts={
            "orig": b_metrics["point_clouds"]["original"]["count"],
            "conf": b_metrics["point_clouds"]["confidence_filtered"]["count"],
            "fused": b_metrics["point_clouds"]["fused"]["count"],
        },
        mesh_stats={
            "verts": b_metrics["mesh"]["vertex_count"],
            "faces": b_metrics["mesh"]["face_count"],
        },
    )

    # 2. Candidate 0049 (Campus)
    c_orig_pcd = str(root / "outputs" / "dji_0049_validation" / "reconstruction" / "point_cloud.ply")
    c_conf_pcd = str(val_dir / "campus_0049" / "point_cloud_conf_filtered.ply")
    c_fused_pcd = str(val_dir / "campus_0049" / "point_cloud_fused_005.ply")
    c_mesh = str(val_dir / "campus_0049" / "mesh" / "surface_mesh.ply")

    c_metrics = analyze_model_metrics(c_orig_pcd, c_conf_pcd, c_fused_pcd, c_mesh)

    render_4row_inspection_sheet(
        orig_pcd_path=c_orig_pcd,
        conf_pcd_path=c_conf_pcd,
        fused_pcd_path=c_fused_pcd,
        mesh_path=c_mesh,
        output_png=str(val_dir / "campus_0049_production_validation.png"),
        title="Candidate 0049 (Campus Oblique Area)",
        pcd_counts={
            "orig": c_metrics["point_clouds"]["original"]["count"],
            "conf": c_metrics["point_clouds"]["confidence_filtered"]["count"],
            "fused": c_metrics["point_clouds"]["fused"]["count"],
        },
        mesh_stats={
            "verts": c_metrics["mesh"]["vertex_count"],
            "faces": c_metrics["mesh"]["face_count"],
        },
    )

    # Save all metrics
    metrics_summary = {
        "title": "F.O.R.G.E Controlled Point-Cloud Accuracy Validation Metrics",
        "units_disclaimer": "All coordinate units are in model coordinate space (nominally meters in VGGT-Omega convention). Not independently verified against ground truth survey/GCPs.",
        "settings": {
            "min_confidence": 1.5,
            "voxel_size": 0.005,
            "pixel_offset": 0.0,
            "min_depth": 0.1,
            "max_depth": 100.0,
        },
        "candidate_0050_bridge": b_metrics,
        "candidate_0049_campus": c_metrics,
    }

    metrics_json_path = val_dir / "production_validation_metrics.json"
    with open(metrics_json_path, "w", encoding="utf-8") as f:
        json.dump(metrics_summary, f, indent=2)

    print(f"\nSaved production validation metrics JSON: {metrics_json_path}")


if __name__ == "__main__":
    main()
