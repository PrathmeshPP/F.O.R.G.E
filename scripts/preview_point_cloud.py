"""
F.O.R.G.E — Point Cloud Visual Quality Assurance (Phase 2C)
Renders multi-view orthographic projections (X-Z Top-Down, X-Y Frontal, Z-Y Side Profile)
from binary PLY point clouds using only native NumPy and Pillow.
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Tuple, List, Dict, Any
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def parse_ply(ply_path: str) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """
    Parses binary PLY file into vertex coordinates and RGB colors.
    """
    with open(ply_path, "rb") as f:
        header_lines = []
        num_vertices = 0
        is_binary = False
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line.startswith("element vertex"):
                num_vertices = int(line.split()[-1])
            if "format binary_little_endian" in line:
                is_binary = True
            if line == "end_header":
                break

        if not is_binary:
            raise ValueError(f"Expected binary_little_endian PLY format in: {ply_path}")

        dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('r', 'u1'), ('g', 'u1'), ('b', 'u1')
        ])
        vertex_data = np.fromfile(f, dtype=dtype, count=num_vertices)

    points = np.stack([vertex_data['x'], vertex_data['y'], vertex_data['z']], axis=-1)
    colors = np.stack([vertex_data['r'], vertex_data['g'], vertex_data['b']], axis=-1)
    return points, colors, header_lines


def render_orthographic_panel(
    c_x: np.ndarray,
    c_y: np.ndarray,
    colors: np.ndarray,
    title: str,
    x_label: str,
    y_label: str,
    panel_size: Tuple[int, int] = (640, 600),
    point_size: int = 2
) -> Image.Image:
    """
    Renders an orthographic projection of 2D coordinates into a styled Image panel.
    Preserves true metric aspect ratio.
    """
    panel_w, panel_h = panel_size
    margin_left, margin_right = 70, 30
    margin_top, margin_bottom = 60, 60

    plot_w = panel_w - margin_left - margin_right
    plot_h = panel_h - margin_top - margin_bottom

    # Determine metric coordinate bounds
    min_x, max_x = float(c_x.min()), float(c_x.max())
    min_y, max_y = float(c_y.min()), float(c_y.max())

    span_x = max(max_x - min_x, 1e-4)
    span_y = max(max_y - min_y, 1e-4)

    # Maintain true metric aspect ratio inside plot area
    aspect_data = span_x / span_y
    aspect_plot = plot_w / plot_h

    if aspect_data > aspect_plot:
        # Data is wider than plot box; fit width
        fit_w = plot_w
        fit_h = int(plot_w / aspect_data)
        off_x = margin_left
        off_y = margin_top + (plot_h - fit_h) // 2
    else:
        # Data is taller than plot box; fit height
        fit_h = plot_h
        fit_w = int(plot_h * aspect_data)
        off_x = margin_left + (plot_w - fit_w) // 2
        off_y = margin_top

    # Map coordinates to raster pixels
    # X increases to the right; Y increases upwards (so top pixel is max_y)
    px = off_x + ((c_x - min_x) / span_x * (fit_w - 1)).astype(np.int32)
    py = off_y + ((max_y - c_y) / span_y * (fit_h - 1)).astype(np.int32)

    # Initialize canvas (dark theme: #151922)
    canvas = np.full((panel_h, panel_w, 3), (21, 25, 34), dtype=np.uint8)

    # Plot border area (#1c212d)
    canvas[margin_top:margin_top + plot_h, margin_left:margin_left + plot_w] = (28, 33, 45)

    # Splat points into canvas
    valid_pts = (px >= 0) & (px < panel_w) & (py >= 0) & (py < panel_h)
    px_v = px[valid_pts]
    py_v = py[valid_pts]
    col_v = colors[valid_pts]

    canvas[py_v, px_v] = col_v
    if point_size >= 2:
        # 2x2 splat for visual prominence
        canvas[np.clip(py_v + 1, 0, panel_h - 1), px_v] = col_v
        canvas[py_v, np.clip(px_v + 1, 0, panel_w - 1)] = col_v
        canvas[np.clip(py_v + 1, 0, panel_h - 1), np.clip(px_v + 1, 0, panel_w - 1)] = col_v

    img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(img)

    # Draw frame boundary
    draw.rectangle(
        [margin_left, margin_top, margin_left + plot_w, margin_top + plot_h],
        outline=(60, 70, 90),
        width=1
    )

    # Text overlays
    draw.text((margin_left, 16), title, fill=(240, 245, 255))
    draw.text((margin_left, 36), f"Span: {span_x:.2f}m x {span_y:.2f}m | Points: {len(c_x):,}", fill=(140, 160, 185))

    # Axis bounds labels
    draw.text((margin_left, panel_h - margin_bottom + 10), f"{min_x:.1f}m", fill=(160, 175, 195))
    draw.text((margin_left + plot_w - 40, panel_h - margin_bottom + 10), f"{max_x:.1f}m", fill=(160, 175, 195))
    draw.text((margin_left + plot_w // 2 - 40, panel_h - margin_bottom + 28), x_label, fill=(200, 215, 240))

    # Y-axis bounds labels
    draw.text((10, margin_top + plot_h - 15), f"{min_y:.1f}m", fill=(160, 175, 195))
    draw.text((10, margin_top), f"{max_y:.1f}m", fill=(160, 175, 195))
    draw.text((10, margin_top + plot_h // 2 - 10), y_label, fill=(200, 215, 240))

    return img


def generate_qa_preview(
    ply_path: str,
    output_png_path: str,
    max_render_points: int = 100_000
) -> Dict[str, Any]:
    """
    Generates multi-view visual QA orthographic projections composite image.
    """
    points, colors, header_lines = parse_ply(ply_path)
    total_vertices = len(points)

    # Subsample if exceeds max_render_points
    if total_vertices > max_render_points:
        indices = np.linspace(0, total_vertices - 1, max_render_points, dtype=np.int64)
        sample_pts = points[indices]
        sample_cols = colors[indices]
        rendered_points_count = max_render_points
    else:
        sample_pts = points
        sample_cols = colors
        rendered_points_count = total_vertices

    X = sample_pts[:, 0]
    Y = sample_pts[:, 1]
    Z = sample_pts[:, 2]

    # Coordinate mapping:
    # 1. Top-Down Bird's-Eye (X-Z): Horizontal=X, Vertical=Z (Forward direction up)
    panel_xz = render_orthographic_panel(
        c_x=X,
        c_y=Z,
        colors=sample_cols,
        title="Top-Down Bird's Eye View (X vs Z)",
        x_label="Lateral X (m)",
        y_label="Forward Z (m)"
    )

    # 2. Frontal View (X-Y): Horizontal=X, Vertical=-Y (Elevation up, ground down)
    panel_xy = render_orthographic_panel(
        c_x=X,
        c_y=-Y,
        colors=sample_cols,
        title="Frontal / Camera Elevation (X vs -Y)",
        x_label="Lateral X (m)",
        y_label="Elevation -Y (m)"
    )

    # 3. Side Profile (Z-Y): Horizontal=Z, Vertical=-Y (Elevation up, depth right)
    panel_zy = render_orthographic_panel(
        c_x=Z,
        c_y=-Y,
        colors=sample_cols,
        title="Side Profile View (Z vs -Y)",
        x_label="Forward Depth Z (m)",
        y_label="Elevation -Y (m)"
    )

    # Combine into 3-panel horizontal layout with header
    panel_w, panel_h = panel_xz.size
    header_h = 70
    composite_w = panel_w * 3
    composite_h = panel_h + header_h

    composite = Image.new("RGB", (composite_w, composite_h), color=(14, 17, 23))
    draw = ImageDraw.Draw(composite)

    # Header section
    draw.rectangle([0, 0, composite_w, header_h], fill=(21, 25, 34))
    draw.line([0, header_h, composite_w, header_h], fill=(45, 55, 75), width=2)

    draw.text((25, 14), "F.O.R.G.E — Point Cloud Visual Quality Assurance (Phase 2C)", fill=(255, 255, 255))

    bbox_min = [round(float(v), 2) for v in points.min(axis=0)]
    bbox_max = [round(float(v), 2) for v in points.max(axis=0)]
    extents = [round(float(v), 2) for v in (points.max(axis=0) - points.min(axis=0))]

    sub_info = (
        f"Source: {Path(ply_path).name} | Total Points: {total_vertices:,} | "
        f"Rendered Sample: {rendered_points_count:,} | "
        f"Bounding Box: X=[{bbox_min[0]}, {bbox_max[0]}]m (dX={extents[0]}m), "
        f"Y=[{bbox_min[1]}, {bbox_max[1]}]m (dY={extents[1]}m), "
        f"Z=[{bbox_min[2]}, {bbox_max[2]}]m (dZ={extents[2]}m)"
    )
    draw.text((25, 38), sub_info, fill=(160, 180, 205))

    # Paste panels
    composite.paste(panel_xz, (0, header_h))
    composite.paste(panel_xy, (panel_w, header_h))
    composite.paste(panel_zy, (panel_w * 2, header_h))

    out_file = Path(output_png_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    composite.save(out_file, format="PNG")

    return {
        "ply_path": str(Path(ply_path).resolve()),
        "output_png_path": str(out_file.resolve()),
        "header_lines": header_lines,
        "total_vertices": total_vertices,
        "rendered_points": rendered_points_count,
        "bounding_box_min": bbox_min,
        "bounding_box_max": bbox_max,
        "bounding_box_extents": extents,
        "png_dimensions": (composite_w, composite_h),
        "png_file_size_bytes": out_file.stat().st_size
    }


def main():
    parser = argparse.ArgumentParser(description="Render orthographic visual QA preview for PLY point cloud")
    parser.add_argument(
        "--input-ply",
        default="outputs/desert_road_validation/reconstruction/point_cloud.ply",
        help="Path to input .ply file"
    )
    parser.add_argument(
        "--output-png",
        default="outputs/desert_road_validation/reconstruction/point_cloud_preview.png",
        help="Path for output preview image"
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=100_000,
        help="Maximum points to render (default: 100000)"
    )

    args = parser.parse_args()

    print("=" * 65)
    print("F.O.R.G.E Phase 2C — Point Cloud Visual QA Preview")
    print("=" * 65)
    print(f"Input PLY:   {args.input_ply}")
    print(f"Output PNG:  {args.output_png}")
    print(f"Max Points:  {args.max_points:,}")
    print("-" * 65)

    stats = generate_qa_preview(
        ply_path=args.input_ply,
        output_png_path=args.output_png,
        max_render_points=args.max_points
    )

    print("\n[Visual QA Results]")
    print(f"  Header elements:       {len(stats['header_lines'])} lines")
    print(f"  Total Vertices:        {stats['total_vertices']:,}")
    print(f"  Sampled for Render:    {stats['rendered_points']:,}")
    print(f"  Bounding Box Min:      {stats['bounding_box_min']}")
    print(f"  Bounding Box Max:      {stats['bounding_box_max']}")
    print(f"  Bounding Box Extents:  {stats['bounding_box_extents']}")
    print(f"  Preview Image Size:    {stats['png_dimensions'][0]}x{stats['png_dimensions'][1]} px")
    print(f"  PNG File Size:         {stats['png_file_size_bytes']:,} bytes ({stats['png_file_size_bytes'] / (1024**2):.2f} MB)")
    print(f"  Saved Image Path:      {stats['output_png_path']}")
    print("=" * 65)
    print("VISUAL QA COMPLETE: SUCCESS")
    print("=" * 65)


if __name__ == "__main__":
    main()
