"""F.O.R.G.E — Diagnostic Comparison Contact Sheet Generator.

Generates visual diagnostic contact sheets with consistent camera views and labeling:
1. river_bridge_repair_comparison_sheet.png:
   Compares Original Hover Baseline vs. Repaired Hover Candidate vs. 84.8m Flight Corridor.
2. campus_0049_repair_comparison_sheet.png:
   Compares Original Baseline vs. Repaired Candidate V2.
3. dataset_overview_contact_sheet.png:
   Comprehensive overview of all 6 in-scope scenes in the F.O.R.G.E dataset.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont


def render_mesh_views(mesh: o3d.geometry.TriangleMesh, width: int = 640, height: int = 360) -> Dict[str, Image.Image]:
    """Renders top, frontal, and oblique views of a mesh."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    mesh.compute_vertex_normals()
    vis.add_geometry(mesh)

    ctr = mesh.get_axis_aligned_bounding_box().get_center()
    views = {}
    view_configs = [
        ("top", [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
        ("frontal", [0.0, -1.0, 0.2], [0.0, 0.0, 1.0]),
        ("oblique", [0.6, -0.7, 0.5], [0.0, 0.0, 1.0])
    ]

    for vname, front, up in view_configs:
        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr)
        ctrl.set_front(front)
        ctrl.set_up(up)
        ctrl.set_zoom(0.75)
        vis.poll_events()
        vis.update_renderer()
        buf = vis.capture_screen_float_buffer(do_render=True)
        views[vname] = Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))

    vis.destroy_window()
    return views


def create_river_bridge_comparison_sheet(project_root: Path, out_path: Path):
    """Creates 3-row diagnostic sheet comparing River Bridge models."""
    print("Generating River Bridge comparison sheet...")
    targets = [
        (
            "Original Hover Baseline (Stationary Hover 52s-64s)",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "river_bridge" / "surface_mesh.ply",
            "Vertices: 255,497 | Tris: 507,399 | Boundary Edges: 5,465 | Fragments: 73 | Vertical Extent: 400.44m (Ill-Conditioned Tilt)"
        ),
        (
            "Repaired Hover Candidate (Clamped Depth & Fragment Pruning)",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "river_bridge" / "candidate_v2_repaired" / "surface_mesh.ply",
            "Vertices: 404,965 | Tris: 803,536 | Depth Clamped: 3.2u | Fragments: 39 (46% reduction) | Vertical Extent: 189.42m"
        ),
        (
            "84.8m Flight Corridor (Active Translation 20s-34s - Solved)",
            project_root / "outputs" / "accuracy_experiment" / "river_bridge_flight_corridor" / "reconstruction" / "surface_mesh.ply",
            "Vertices: 323,203 | Tris: 637,405 | Flight Baseline: 84.8m | Inliers: 8/8 (100%) | RMSE: 0.236m | Ground-Oriented"
        )
    ]

    cw, ch = 640, 360
    header_h = 100
    row_header_h = 45
    total_w = cw * 3
    total_h = header_h + len(targets) * (ch + row_header_h) + 20

    sheet = Image.new("RGB", (total_w, total_h), (16, 20, 28))
    draw = ImageDraw.Draw(sheet)

    # Main Header
    draw.rectangle([0, 0, total_w, header_h], fill=(24, 32, 48))
    draw.text((30, 20), "F.O.R.G.E — River Bridge Reconstruction & Alignment Comparison", fill=(255, 255, 255))
    draw.text((30, 55), "Evolution from Stationary Hover Defect (400m Vertical Spike) to Repaired Model and 84.8m Metric Flight Corridor", fill=(100, 220, 255))

    curr_y = header_h
    vnames = ["top", "frontal", "oblique"]
    vlabels = ["Top (Plan / Overhead)", "Frontal (Elevation)", "Oblique (Perspective)"]

    for row_idx, (title, mesh_path, metrics) in enumerate(targets):
        # Row Header
        draw.rectangle([0, curr_y, total_w, curr_y + row_header_h], fill=(30, 40, 56))
        draw.text((30, curr_y + 8), f"Model {row_idx+1}: {title}", fill=(255, 215, 0))
        draw.text((30, curr_y + 26), metrics, fill=(180, 200, 220))
        curr_y += row_header_h

        mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        views = render_mesh_views(mesh, width=cw, height=ch)

        for col_idx, vn in enumerate(vnames):
            x = col_idx * cw
            sheet.paste(views[vn], (x, curr_y))
            draw.rectangle([x + 10, curr_y + 10, x + 300, curr_y + 35], fill=(0, 0, 0))
            draw.text((x + 18, curr_y + 14), vlabels[col_idx], fill=(255, 255, 255))

        curr_y += ch

    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)
    print(f"Saved: {out_path} ({out_path.stat().st_size/1024:.1f} KB)")


def create_campus_comparison_sheet(project_root: Path, out_path: Path):
    """Creates 2-row diagnostic sheet comparing Campus 0049 models."""
    print("Generating Campus 0049 comparison sheet...")
    targets = [
        (
            "Original Baseline (General Pipeline)",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "surface_mesh.ply",
            "Vertices: 279,375 | Triangles: 557,796 | Isolated Perimeter Clusters: 17 | Extents: 181.1m x 185.3m x 38.0m"
        ),
        (
            "Repaired Candidate V2 (Filtered Flying Pixels & Cleaned Boundaries)",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "candidate_v2_repaired" / "surface_mesh.ply",
            "Vertices: 264,676 | Triangles: 526,742 | 99.8% Main Component Dominance | Isolated Clusters Reduced to 6 (65% reduction)"
        )
    ]

    cw, ch = 640, 360
    header_h = 100
    row_header_h = 45
    total_w = cw * 3
    total_h = header_h + len(targets) * (ch + row_header_h) + 20

    sheet = Image.new("RGB", (total_w, total_h), (16, 20, 28))
    draw = ImageDraw.Draw(sheet)

    # Main Header
    draw.rectangle([0, 0, total_w, header_h], fill=(24, 32, 48))
    draw.text((30, 20), "F.O.R.G.E — Campus 0049 Geometry Repair Comparison", fill=(255, 255, 255))
    draw.text((30, 55), "Baseline vs. Repaired Candidate V2 with Perimeter Fragment Filtering and Architectural Integrity", fill=(100, 220, 255))

    curr_y = header_h
    vnames = ["top", "frontal", "oblique"]
    vlabels = ["Top (Plan / Overhead)", "Frontal (Elevation)", "Oblique (Perspective)"]

    for row_idx, (title, mesh_path, metrics) in enumerate(targets):
        draw.rectangle([0, curr_y, total_w, curr_y + row_header_h], fill=(30, 40, 56))
        draw.text((30, curr_y + 8), f"Model {row_idx+1}: {title}", fill=(255, 215, 0))
        draw.text((30, curr_y + 26), metrics, fill=(180, 200, 220))
        curr_y += row_header_h

        mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        views = render_mesh_views(mesh, width=cw, height=ch)

        for col_idx, vn in enumerate(vnames):
            x = col_idx * cw
            sheet.paste(views[vn], (x, curr_y))
            draw.rectangle([x + 10, curr_y + 10, x + 300, curr_y + 35], fill=(0, 0, 0))
            draw.text((x + 18, curr_y + 14), vlabels[col_idx], fill=(255, 255, 255))

        curr_y += ch

    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)
    print(f"Saved: {out_path} ({out_path.stat().st_size/1024:.1f} KB)")


def create_dataset_overview_sheet(project_root: Path, out_path: Path):
    """Creates a comprehensive 6-scene overview sheet of the entire dataset."""
    print("Generating Complete Dataset Overview sheet...")
    scenes = [
        (
            "Campus 0049",
            "DJI Air 3S Architectural Courtyard (Gandhinagar)",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "campus_0049" / "candidate_v2_repaired" / "surface_mesh.ply",
            "Status: Repaired V2 | Verts: 264k | GPS: Weak Baseline Hover"
        ),
        (
            "River Bridge (Flight Corridor)",
            "DJI Air 3S 84.8m Forward Flight Corridor",
            project_root / "outputs" / "accuracy_experiment" / "river_bridge_flight_corridor" / "reconstruction" / "surface_mesh.ply",
            "Status: Metric Aligned | Verts: 323k | GPS: 100% Inliers (0.236m RMSE)"
        ),
        (
            "Desert Road",
            "Terrestrial Forward Highway Corridor Benchmark",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "desert_road" / "surface_mesh.ply",
            "Status: Native Coordinates | Verts: 205k | GPS: Blocked Missing Telemetry"
        ),
        (
            "Forest Road",
            "Dense Canopy Drone Corridor Benchmark",
            project_root / "outputs" / "accuracy_experiment" / "generalization_disaster_validation_v1" / "forest_road" / "candidate_uniform_span" / "surface_mesh.ply",
            "Status: Native Coordinates | Verts: 395k | GPS: Blocked Missing Telemetry"
        ),
        (
            "Lake Speedboat",
            "Third-Party Dynamic Water Corridor Benchmark",
            project_root / "outputs" / "lake_speedboat_validation" / "reconstruction" / "surface_mesh.ply",
            "Status: Native Coordinates | Verts: 147k | GPS: Blocked Missing Telemetry"
        ),
        (
            "Snow Lift",
            "Third-Party Alpine Cableway Benchmark",
            project_root / "outputs" / "snow_lift_validation" / "reconstruction" / "surface_mesh.ply",
            "Status: Native Coordinates | Verts: 331k | GPS: Blocked Missing Telemetry"
        )
    ]

    cw, ch = 640, 360
    header_h = 100
    card_header_h = 45
    cols = 3
    rows = 2
    total_w = cw * cols
    total_h = header_h + rows * (ch + card_header_h) + 30

    sheet = Image.new("RGB", (total_w, total_h), (16, 20, 28))
    draw = ImageDraw.Draw(sheet)

    # Main Header
    draw.rectangle([0, 0, total_w, header_h], fill=(24, 32, 48))
    draw.text((30, 20), "F.O.R.G.E — Complete Established Dataset Overview (All 6 Confirmed Scenes)", fill=(255, 255, 255))
    draw.text((30, 55), "Multi-Video Reconstruction & Geospatial Alignment Status Across Aerial, Terrestrial, Canopy & Benchmark Scenes", fill=(100, 220, 255))

    for idx, (title, subtitle, mesh_path, status_line) in enumerate(scenes):
        r = idx // cols
        c = idx % cols
        x = c * cw
        y = header_h + r * (ch + card_header_h)

        # Card Header
        draw.rectangle([x, y, x + cw, y + card_header_h], fill=(30, 40, 56))
        draw.text((x + 15, y + 6), f"Scene {idx+1}: {title}", fill=(255, 215, 0))
        draw.text((x + 15, y + 25), status_line, fill=(180, 220, 240))

        if mesh_path.is_file():
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            views = render_mesh_views(mesh, width=cw, height=ch)
            sheet.paste(views["oblique"], (x, y + card_header_h))
            draw.rectangle([x + 15, y + card_header_h + 10, x + 250, y + card_header_h + 35], fill=(0, 0, 0))
            draw.text((x + 22, y + card_header_h + 14), subtitle[:32], fill=(200, 255, 200))
        else:
            draw.rectangle([x, y + card_header_h, x + cw, y + card_header_h + ch], fill=(40, 20, 20))
            draw.text((x + 50, y + card_header_h + 150), "Mesh Not Available", fill=(255, 100, 100))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)
    print(f"Saved: {out_path} ({out_path.stat().st_size/1024:.1f} KB)")


def main():
    project_root = Path(__file__).resolve().parent.parent
    out_dir = project_root / "outputs" / "accuracy_experiment" / "gps_aligned_v1"

    create_river_bridge_comparison_sheet(
        project_root,
        out_dir / "river_bridge_repair_comparison_sheet.png"
    )
    create_campus_comparison_sheet(
        project_root,
        out_dir / "campus_0049_repair_comparison_sheet.png"
    )
    create_dataset_overview_sheet(
        project_root,
        out_dir / "dataset_overview_contact_sheet.png"
    )


if __name__ == "__main__":
    main()
