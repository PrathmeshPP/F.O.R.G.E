"""F.O.R.G.E Interactive 3D Point Cloud Viewer.

Phase 2D: Loads a colored PLY point cloud and displays it in an interactive
Open3D 3D window with RGB vertex colors, coordinate frame axes, and mouse controls
(orbit, pan, zoom). Supports optional non-destructive in-memory downsampling.
Never overwrites or modifies the input file.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Tuple

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="F.O.R.G.E Interactive 3D Point Cloud Viewer (Open3D)"
    )
    parser.add_argument(
        "--input",
        type=str,
        default="outputs/desert_road_validation/reconstruction/point_cloud.ply",
        help="Path to colored PLY point cloud file.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Optional voxel size (meters) for in-memory downsampling (0.0 = no downsampling).",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Optional maximum number of points to display via uniform downsampling (0 = all points).",
    )
    parser.add_argument(
        "--axis-size",
        type=float,
        default=1.0,
        help="Size (meters) of the 3D coordinate frame axes [X=Red, Y=Green, Z=Blue].",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Viewer window width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Viewer window height in pixels.",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Validate point cloud integrity (vertex count, finiteness, colors, extents) without launching GUI.",
    )
    return parser.parse_args()


def verify_point_cloud(pcd: o3d.geometry.PointCloud) -> Tuple[bool, dict]:
    """Verify point cloud vertex counts, finite coordinates, and colors."""
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    vertex_count = len(points)
    has_points = vertex_count > 0
    finite_coords = bool(np.isfinite(points).all()) if has_points else False
    has_colors = bool(len(colors) == vertex_count and vertex_count > 0)
    finite_colors = bool(np.isfinite(colors).all()) if has_colors else False

    min_bound = points.min(axis=0) if has_points else np.zeros(3)
    max_bound = points.max(axis=0) if has_points else np.zeros(3)
    extents = max_bound - min_bound

    stats = {
        "vertex_count": vertex_count,
        "finite_coords": finite_coords,
        "has_colors": has_colors,
        "finite_colors": finite_colors,
        "min_bound": min_bound.tolist(),
        "max_bound": max_bound.tolist(),
        "extents": extents.tolist(),
    }

    is_valid = has_points and finite_coords and has_colors and finite_colors
    return is_valid, stats


def main() -> int:
    args = parse_args()
    ply_path = Path(args.input).resolve()

    if not ply_path.is_file():
        print(f"[ERROR] Point cloud file not found: {ply_path}", file=sys.stderr)
        return 1

    print(f"[INFO] Loading point cloud from: {ply_path}")
    pcd = o3d.io.read_point_cloud(str(ply_path))

    is_valid, stats = verify_point_cloud(pcd)
    print(f"[INFO] Point cloud loaded successfully:")
    print(f"       - Vertex count:     {stats['vertex_count']:,}")
    print(f"       - Finite coords:    {stats['finite_coords']}")
    print(f"       - Colored vertices: {stats['has_colors']} (finite: {stats['finite_colors']})")
    print(f"       - Bounding box min: [{stats['min_bound'][0]:.3f}, {stats['min_bound'][1]:.3f}, {stats['min_bound'][2]:.3f}] m")
    print(f"       - Bounding box max: [{stats['max_bound'][0]:.3f}, {stats['max_bound'][1]:.3f}, {stats['max_bound'][2]:.3f}] m")
    print(f"       - Box extents:      [{stats['extents'][0]:.3f}, {stats['extents'][1]:.3f}, {stats['extents'][2]:.3f}] m")

    if not is_valid:
        print("[ERROR] Point cloud failed validation checks!", file=sys.stderr)
        return 1

    if args.verify_only:
        print("[INFO] Verification passed (--verify-only specified). Exiting without opening window.")
        return 0

    # In-memory downsampling for display responsiveness if requested (never modifies the file)
    display_pcd = pcd
    if args.voxel_size > 0.0:
        print(f"[INFO] Applying in-memory voxel downsampling (voxel_size = {args.voxel_size} m)...")
        display_pcd = pcd.voxel_down_sample(voxel_size=args.voxel_size)
        print(f"[INFO] Display points after voxel downsampling: {len(display_pcd.points):,}")
    elif args.max_points > 0 and len(pcd.points) > args.max_points:
        every_k = max(1, len(pcd.points) // args.max_points)
        print(f"[INFO] Applying in-memory uniform downsampling (every {every_k}th point)...")
        display_pcd = pcd.uniform_down_sample(every_k)
        print(f"[INFO] Display points after uniform downsampling: {len(display_pcd.points):,}")

    geometries = [display_pcd]

    # Add 3D coordinate frame (X=Red, Y=Green, Z=Blue)
    if args.axis_size > 0.0:
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=args.axis_size,
            origin=[0.0, 0.0, 0.0],
        )
        geometries.append(coord_frame)
        print(f"[INFO] Coordinate frame added at origin (size = {args.axis_size} m). [X=Red, Y=Green, Z=Blue]")

    window_title = f"F.O.R.G.E 3D Viewer — {ply_path.name} ({len(display_pcd.points):,} points)"
    print("[INFO] Launching Open3D interactive viewer...")
    print("       Controls:")
    print("         - Left Click + Drag: Orbit / Rotate")
    print("         - Right Click + Drag / Shift + Left Click: Pan")
    print("         - Scroll Wheel / Ctrl + Left Click + Drag: Zoom")
    print("         - R: Reset camera view")
    print("         - Q / Esc: Close viewer")

    o3d.visualization.draw_geometries(
        geometries,
        window_name=window_title,
        width=args.width,
        height=args.height,
        left=50,
        top=50,
    )
    print("[INFO] Viewer window closed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
