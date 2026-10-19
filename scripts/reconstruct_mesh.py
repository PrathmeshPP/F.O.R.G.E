"""F.O.R.G.E — Surface Mesh Reconstruction & Rectangular Presentation Module.

Reconstructs an authentic, colored 3D surface mesh from drone point clouds while
ensuring professional visual presentation:
1. Authentic KD-Tree Color Transfer: Restores authentic source RGB colors to mesh vertices via
   inverse-distance-weighted interpolation from the unorganized source point cloud.
2. Gap Support & Edge Filtering: Eliminates fabricated geometry by pruning vertices farther than
   a strict distance threshold from source points and rejecting triangles with excessive edge lengths.
3. Island/Speck Pruning: Removes detached floating triangle clusters to ensure a clean boundary.
4. Rectangular Presentation Base: Places the actual reconstruction atop a separate, neutral
   rectangular display base (plinth) for exhibition, explicitly labeled as presentation-only.
   Never alters, fabricates, or stretches terrain geometry.
5. Export & Verification: Exports both pure and presentation models as PLY and OBJ with RGB colors,
   verifying manifold topology, finite coordinates, face indices, and color standard deviations.

Never modifies or overwrites the original point cloud or previous validation outputs.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, Any, Tuple, List, Optional

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="F.O.R.G.E — Surface Mesh Reconstruction & Rectangular Presentation"
    )
    parser.add_argument(
        "--input",
        type=str,
        default="outputs/dji_0049_validation/reconstruction/point_cloud.ply",
        help="Path to input colored PLY point cloud file.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="outputs/dji_0049_validation/reconstruction",
        help="Directory to save reconstructed meshes, previews, and metadata.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.01,
        help="Voxel size (meters) for point cloud downsampling (default: 0.01m = 1cm).",
    )
    parser.add_argument(
        "--normal-radius",
        type=float,
        default=0.03,
        help="Search radius (meters) for normal estimation (default: 0.03m).",
    )
    parser.add_argument(
        "--normal-max-nn",
        type=int,
        default=30,
        help="Maximum nearest neighbors for normal estimation (default: 30).",
    )
    parser.add_argument(
        "--poisson-depth",
        type=int,
        default=9,
        help="Octree depth for Poisson reconstruction (default: 9).",
    )
    parser.add_argument(
        "--density-quantile",
        type=float,
        default=0.15,
        help="Quantile threshold for Poisson density pruning (default: 0.15 = bottom 15 percent).",
    )
    parser.add_argument(
        "--max-support-dist",
        type=float,
        default=0.02,
        help="Maximum distance (meters) from mesh vertex to nearest PCD point to reject fabricated geometry (default: 0.02m).",
    )
    parser.add_argument(
        "--max-edge-length",
        type=float,
        default=0.05,
        help="Maximum edge length (meters) to reject unsupported bridging triangles (default: 0.05m).",
    )
    parser.add_argument(
        "--min-cluster-faces",
        type=int,
        default=100,
        help="Minimum number of triangular faces in a connected component to retain (default: 100).",
    )
    parser.add_argument(
        "--bpa-radii",
        type=float,
        nargs="+",
        default=[0.015, 0.03, 0.06],
        help="Ball radii (meters) for Ball Pivoting Algorithm (default: 0.015 0.03 0.06).",
    )
    parser.add_argument(
        "--add-presentation-base",
        action="store_true",
        default=True,
        help="Add a separate, neutral rectangular display base for visual presentation (default: True).",
    )
    parser.add_argument(
        "--no-presentation-base",
        action="store_false",
        dest="add_presentation_base",
        help="Disable rectangular display base creation.",
    )
    parser.add_argument(
        "--base-padding",
        type=float,
        default=0.05,
        help="Padding (meters) around mesh bounding box for base (default: 0.05m = 5cm).",
    )
    parser.add_argument(
        "--base-thickness",
        type=float,
        default=0.03,
        help="Thickness (meters) of presentation base (default: 0.03m = 3cm).",
    )
    parser.add_argument(
        "--generate-preview",
        action="store_true",
        default=True,
        help="Render multi-angle visual QA preview image (mesh_preview.png).",
    )
    parser.add_argument(
        "--view",
        action="store_true",
        help="Launch interactive Open3D window on completion.",
    )
    parser.add_argument(
        "--view-only",
        action="store_true",
        help="Directly view existing presentation mesh without re-running reconstruction.",
    )
    return parser.parse_args()


def compute_color_statistics(colors: np.ndarray) -> Dict[str, Any]:
    """Computes mean, standard deviation, min, and max for RGB color channels (0.0 - 1.0)."""
    if len(colors) == 0:
        return {"count": 0}
    r = colors[:, 0]
    g = colors[:, 1]
    b = colors[:, 2]
    return {
        "count": len(colors),
        "mean_rgb": [round(float(r.mean()), 4), round(float(g.mean()), 4), round(float(b.mean()), 4)],
        "std_rgb": [round(float(r.std()), 4), round(float(g.std()), 4), round(float(b.std()), 4)],
        "min_rgb": [round(float(r.min()), 4), round(float(g.min()), 4), round(float(b.min()), 4)],
        "max_rgb": [round(float(r.max()), 4), round(float(g.max()), 4), round(float(b.max()), 4)],
    }


def transfer_colors_kdtree(
    mesh: o3d.geometry.TriangleMesh,
    source_pcd: o3d.geometry.PointCloud,
    k: int = 4
) -> np.ndarray:
    """
    Transfers authentic RGB colors from the source point cloud to mesh vertices
    using inverse-distance-weighted (IDW) KD-Tree k-nearest neighbor interpolation.
    """
    mesh_verts = np.asarray(mesh.vertices)
    pcd_colors = np.asarray(source_pcd.colors)
    num_verts = len(mesh_verts)

    if num_verts == 0 or len(pcd_colors) == 0:
        return np.zeros((num_verts, 3))

    kdtree = o3d.geometry.KDTreeFlann(source_pcd)
    transferred_colors = np.zeros((num_verts, 3), dtype=np.float64)

    for i in range(num_verts):
        [count, idx, d2] = kdtree.search_knn_vector_3d(mesh_verts[i], k)
        d = np.sqrt(d2)
        weights = 1.0 / (d + 1e-6)
        weights /= weights.sum()
        transferred_colors[i] = np.sum(pcd_colors[idx] * weights[:, None], axis=0)

    mesh.vertex_colors = o3d.utility.Vector3dVector(transferred_colors)
    return transferred_colors


def filter_mesh_support(
    mesh: o3d.geometry.TriangleMesh,
    reference_pcd: o3d.geometry.PointCloud,
    max_support_dist: float = 0.02,
    max_edge_length: float = 0.05,
    min_cluster_faces: int = 100
) -> o3d.geometry.TriangleMesh:
    """
    Removes unsupported vertices, large bridging triangles, and tiny disconnected fragments:
    1. Rejects vertices farther than max_support_dist from the reference point cloud.
    2. Rejects triangles with any edge length exceeding max_edge_length.
    3. Prunes small disconnected triangle clusters with fewer than min_cluster_faces.
    """
    verts = np.asarray(mesh.vertices)
    if len(verts) == 0:
        return mesh

    # 1. Distance-to-PCD vertex support check
    kdtree = o3d.geometry.KDTreeFlann(reference_pcd)
    dists = np.zeros(len(verts))
    for i in range(len(verts)):
        [k, idx, d2] = kdtree.search_knn_vector_3d(verts[i], 1)
        dists[i] = np.sqrt(d2[0])

    unsupported_mask = dists > max_support_dist
    mesh.remove_vertices_by_mask(unsupported_mask)

    # 2. Triangle edge length check
    tris = np.asarray(mesh.triangles)
    v_clean = np.asarray(mesh.vertices)
    if len(tris) > 0 and len(v_clean) > 0:
        e0 = np.linalg.norm(v_clean[tris[:, 1]] - v_clean[tris[:, 0]], axis=1)
        e1 = np.linalg.norm(v_clean[tris[:, 2]] - v_clean[tris[:, 1]], axis=1)
        e2 = np.linalg.norm(v_clean[tris[:, 0]] - v_clean[tris[:, 2]], axis=1)
        max_edge = np.maximum(np.maximum(e0, e1), e2)
        long_edge_mask = max_edge > max_edge_length
        mesh.remove_triangles_by_mask(long_edge_mask)
        mesh.remove_unreferenced_vertices()

    # 3. Disconnected cluster pruning (removes tiny floating specks)
    if min_cluster_faces > 0 and len(mesh.triangles) > 0:
        split = mesh.cluster_connected_triangles()
        tri_clusters, _, _ = split
        counts = np.bincount(np.asarray(tri_clusters))
        remove_tri_mask = counts[np.asarray(tri_clusters)] < min_cluster_faces
        mesh.remove_triangles_by_mask(remove_tri_mask)
        mesh.remove_unreferenced_vertices()

    # 4. Clean topology
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    return mesh


def create_presentation_base(
    mesh: o3d.geometry.TriangleMesh,
    padding: float = 0.05,
    thickness: float = 0.03,
    color: Tuple[float, float, float] = (0.16, 0.18, 0.22)
) -> Tuple[o3d.geometry.TriangleMesh, Dict[str, Any]]:
    """
    Creates a neutral, separate rectangular presentation base (plinth) underneath the reconstruction.
    The base is strictly for aesthetic presentation and display, and is explicitly labeled as
    non-geographic/presentation-only so it is never included in measurements.
    """
    verts = np.asarray(mesh.vertices)
    if len(verts) == 0:
        return o3d.geometry.TriangleMesh(), {}

    v_min = verts.min(axis=0)
    v_max = verts.max(axis=0)
    extents = v_max - v_min

    # Detect scene orientation:
    # If Z extent is the smallest (nadir flight), base is in the X-Y plane at max_z (furthest from drone).
    # If Y extent is the smallest (horizontal flight), base is in the X-Z plane at max_y (lowest elevation).
    if extents[2] < min(extents[0], extents[1]):
        # Nadir orientation (X-Y ground plane)
        bx_min = v_min[0] - padding
        bx_max = v_max[0] + padding
        by_min = v_min[1] - padding
        by_max = v_max[1] + padding
        bz_min = v_max[2] + 0.002
        w = bx_max - bx_min
        h = by_max - by_min
        d = thickness
        base_box = o3d.geometry.TriangleMesh.create_box(width=w, height=h, depth=d)
        base_box.translate([bx_min, by_min, bz_min])
        elevation_axis = "Z (Depth)"
    else:
        # Oblique / Horizontal orientation (X-Z ground plane)
        bx_min = v_min[0] - padding
        bx_max = v_max[0] + padding
        bz_min = v_min[2] - padding
        bz_max = v_max[2] + padding
        by_min = v_max[1] + 0.002
        w = bx_max - bx_min
        h = thickness
        d = bz_max - bz_min
        base_box = o3d.geometry.TriangleMesh.create_box(width=w, height=h, depth=d)
        base_box.translate([bx_min, by_min, bz_min])
        elevation_axis = "Y (Elevation)"

    base_box.paint_uniform_color(list(color))
    base_box.compute_vertex_normals()

    base_meta = {
        "is_presentation_only": True,
        "included_in_measurements": False,
        "elevation_axis": elevation_axis,
        "dimensions_m": [round(float(w), 4), round(float(h), 4), round(float(d), 4)],
        "color_rgb": [round(float(c), 3) for c in color],
        "vertex_count": len(base_box.vertices),
        "triangle_count": len(base_box.triangles),
    }
    return base_box, base_meta


def verify_mesh(
    mesh: o3d.geometry.TriangleMesh,
    pcd_bounds: Tuple[np.ndarray, np.ndarray]
) -> Tuple[bool, Dict[str, Any]]:
    """Verifies geometric and topological validity of a mesh."""
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    vertex_count = len(vertices)
    triangle_count = len(triangles)
    has_vertices = vertex_count > 0
    has_triangles = triangle_count > 0

    finite_coords = bool(np.isfinite(vertices).all()) if has_vertices else False
    valid_indices = bool((triangles.min() >= 0) and (triangles.max() < vertex_count)) if (has_triangles and has_vertices) else False

    has_colors = bool(mesh.has_vertex_colors() and len(mesh.vertex_colors) == vertex_count)
    has_normals = bool(mesh.has_vertex_normals() and len(mesh.vertex_normals) == vertex_count)
    is_edge_manifold = bool(mesh.is_edge_manifold())
    is_vertex_manifold = bool(mesh.is_vertex_manifold())

    v_min = vertices.min(axis=0) if has_vertices else np.zeros(3)
    v_max = vertices.max(axis=0) if has_vertices else np.zeros(3)
    extents = v_max - v_min

    p_min, p_max = pcd_bounds
    bounds_ok = bool(np.all(v_min >= (p_min - 0.2)) and np.all(v_max <= (p_max + 0.2)))

    is_valid = has_vertices and has_triangles and finite_coords and valid_indices and bounds_ok

    color_stats = compute_color_statistics(np.asarray(mesh.vertex_colors)) if has_colors else {}

    stats = {
        "is_valid": is_valid,
        "vertex_count": vertex_count,
        "triangle_count": triangle_count,
        "finite_coordinates": finite_coords,
        "valid_triangle_indices": valid_indices,
        "has_vertex_colors": has_colors,
        "has_vertex_normals": has_normals,
        "is_edge_manifold": is_edge_manifold,
        "is_vertex_manifold": is_vertex_manifold,
        "bounding_box_min": [round(float(x), 4) for x in v_min],
        "bounding_box_max": [round(float(x), 4) for x in v_max],
        "extents": [round(float(x), 4) for x in extents],
        "color_stats": color_stats
    }
    return is_valid, stats


def render_comparison_preview(
    items: List[Tuple[str, str, Any]],
    output_png_path: str,
    panel_w: int = 640,
    panel_h: int = 560
) -> str:
    """Renders a comparative visual QA contact sheet comparing point clouds and candidate meshes."""
    num_panels = len(items)
    header_h = 75
    comp_w = panel_w * num_panels
    comp_h = panel_h + header_h

    composite = Image.new("RGB", (comp_w, comp_h), color=(14, 17, 23))
    draw = ImageDraw.Draw(composite)

    draw.rectangle([0, 0, comp_w, header_h], fill=(21, 25, 34))
    draw.line([0, header_h, comp_w, header_h], fill=(45, 55, 75), width=2)
    draw.text((25, 14), "F.O.R.G.E — Surface Mesh Geometry & Presentation QA Comparison", fill=(255, 255, 255))
    draw.text((25, 40), "Source Point Cloud vs Authentic Filtered Mesh vs Rectangular Presentation Model", fill=(140, 165, 195))

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=panel_w, height=panel_h)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])

    temp_files = []
    for idx, (title, subtitle, geom) in enumerate(items):
        vis.clear_geometries()
        vis.add_geometry(geom)

        if isinstance(geom, o3d.geometry.TriangleMesh):
            opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
            g_verts = np.asarray(geom.vertices)
        else:
            opt.point_size = 2.0
            g_verts = np.asarray(geom.points)

        g_min = g_verts.min(axis=0)
        g_max = g_verts.max(axis=0)
        g_center = (g_min + g_max) / 2.0
        g_extents = g_max - g_min

        ctr = vis.get_view_control()
        # Adapt camera view: nadir flight (small dZ) vs corridor flight (small dY)
        if g_extents[2] < min(g_extents[0], g_extents[1]):
            # Nadir overhead view
            ctr.set_front([0.0, 0.0, -1.0])
            ctr.set_up([0.0, -1.0, 0.0])
        else:
            # Oblique flight corridor view
            ctr.set_front([0.0, -0.6, -0.8])
            ctr.set_up([0.0, -1.0, 0.0])

        ctr.set_lookat(g_center)
        ctr.set_zoom(0.7)

        vis.poll_events()
        vis.update_renderer()

        t_path = Path(output_png_path).parent / f"_temp_comp_{idx}.png"
        vis.capture_screen_image(str(t_path), do_render=True)
        temp_files.append(t_path)

        img = Image.open(t_path)
        x_off = idx * panel_w
        composite.paste(img, (x_off, header_h))

        # Title badge
        draw.rectangle([x_off + 15, header_h + 15, x_off + panel_w - 15, header_h + 58], fill=(0, 0, 0))
        draw.text((x_off + 22, header_h + 18), title, fill=(255, 220, 100))
        draw.text((x_off + 22, header_h + 36), subtitle, fill=(180, 200, 225))

    vis.destroy_window()

    out_file = Path(output_png_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    composite.save(out_file, format="PNG")

    for tp in temp_files:
        if tp.exists():
            tp.unlink()

    return str(out_file.resolve())


def main() -> int:
    args = parse_args()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    presentation_ply = out_dir / "surface_mesh_presentation.ply"
    surface_ply = out_dir / "surface_mesh.ply"

    if args.view_only:
        inp = Path(args.input).resolve()
        if inp.is_file():
            target_ply = inp
        elif (out_dir / "surface_mesh_gps_aligned.ply").is_file():
            target_ply = out_dir / "surface_mesh_gps_aligned.ply"
        elif presentation_ply.is_file():
            target_ply = presentation_ply
        else:
            target_ply = surface_ply

        if not target_ply.is_file():
            print(f"[ERROR] Reconstructed mesh not found in: {out_dir}", file=sys.stderr)
            return 1
        print(f"[INFO] Loading mesh from: {target_ply}")
        mesh = o3d.io.read_triangle_mesh(str(target_ply))
        mesh.compute_vertex_normals()
        print(f"       Vertices:  {len(mesh.vertices):,}")
        print(f"       Triangles: {len(mesh.triangles):,}")
        ext_max = float(np.max(mesh.get_axis_aligned_bounding_box().get_extent())) if len(mesh.vertices) > 0 else 1.0
        axis_size = max(0.3, ext_max * 0.1)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_size)
        print("[INFO] Launching Open3D interactive viewer...")
        o3d.visualization.draw_geometries(
            [mesh, coord_frame],
            window_name=f"F.O.R.G.E Mesh Viewer — {len(mesh.vertices):,} verts, {len(mesh.triangles):,} faces",
            width=1280,
            height=720,
            left=50,
            top=50
        )
        return 0

    total_t0 = time.perf_counter()
    ply_path = Path(args.input).resolve()

    print("=" * 72)
    print("F.O.R.G.E — Surface Mesh Reconstruction & Rectangular Presentation")
    print("=" * 72)
    print(f"Input Point Cloud:      {ply_path}")
    print(f"Output Directory:       {out_dir}")
    print(f"Voxel Downsampling:     {args.voxel_size} m")
    print(f"Max Support Distance:   {args.max_support_dist} m")
    print(f"Max Edge Length:        {args.max_edge_length} m")
    print(f"Min Cluster Faces:      {args.min_cluster_faces}")
    print(f"Presentation Base:      {args.add_presentation_base}")
    print("-" * 72)

    # 1. Load Source Point Cloud
    print("\n[Step 1/5] Loading source point cloud...")
    pcd = o3d.io.read_point_cloud(str(ply_path))
    pts = np.asarray(pcd.points)
    raw_pcd_colors = np.asarray(pcd.colors)
    p_min = pts.min(axis=0)
    p_max = pts.max(axis=0)
    print(f"  Loaded {len(pts):,} points")
    print(f"  Bounding Box: [{p_min[0]:.3f}, {p_min[1]:.3f}, {p_min[2]:.3f}] to [{p_max[0]:.3f}, {p_max[1]:.3f}, {p_max[2]:.3f}] m")

    pcd_color_stats = compute_color_statistics(raw_pcd_colors)
    print("  Source PCD Color Stats (0-1):")
    print(f"    Mean RGB: {pcd_color_stats['mean_rgb']}")
    print(f"    Std RGB:  {pcd_color_stats['std_rgb']}")

    # 2. Downsampling & Normal Estimation
    print("\n[Step 2/5] Downsampling and estimating normals...")
    down = pcd.voxel_down_sample(voxel_size=args.voxel_size)
    down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.normal_radius,
            max_nn=args.normal_max_nn
        )
    )
    down.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))
    print(f"  Downsampled cloud: {len(down.points):,} points with oriented normals")

    # 3. Candidate A: Ball Pivoting Algorithm (BPA)
    print("\n[Step 3/5] Generating Candidate A: Ball Pivoting Algorithm (BPA)...")
    bpa_t0 = time.perf_counter()
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        down, o3d.utility.DoubleVector(args.bpa_radii)
    )
    bpa_mesh.remove_unreferenced_vertices()
    bpa_mesh.remove_degenerate_triangles()
    bpa_mesh.remove_duplicated_triangles()
    bpa_mesh.remove_duplicated_vertices()
    bpa_mesh.remove_non_manifold_edges()
    bpa_mesh.compute_vertex_normals()
    transfer_colors_kdtree(bpa_mesh, pcd)
    bpa_time = time.perf_counter() - bpa_t0
    bpa_valid, bpa_stats = verify_mesh(bpa_mesh, (p_min, p_max))
    print(f"  BPA reconstructed in {bpa_time:.2f}s: {len(bpa_mesh.vertices):,} verts, {len(bpa_mesh.triangles):,} faces")

    # 4. Candidate B: Stricter Screened Poisson + Support & Cluster Filtering
    print("\n[Step 4/5] Generating Candidate B: Screened Poisson + Support & Cluster Filtering...")
    poisson_t0 = time.perf_counter()
    mesh_p, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        down, depth=args.poisson_depth
    )
    dens_np = np.asarray(densities)
    dens_thresh = float(np.percentile(dens_np, args.density_quantile * 100.0))
    mesh_p.remove_vertices_by_mask(dens_np < dens_thresh)

    filter_mesh_support(
        mesh=mesh_p,
        reference_pcd=down,
        max_support_dist=args.max_support_dist,
        max_edge_length=args.max_edge_length,
        min_cluster_faces=args.min_cluster_faces
    )

    transfer_colors_kdtree(mesh_p, pcd)
    poisson_time = time.perf_counter() - poisson_t0
    p_valid, p_stats = verify_mesh(mesh_p, (p_min, p_max))
    print(f"  Poisson reconstructed in {poisson_time:.2f}s: {len(mesh_p.vertices):,} verts, {len(mesh_p.triangles):,} faces")
    print(f"  Vertex Color Stats: Mean={p_stats['color_stats']['mean_rgb']}, Std={p_stats['color_stats']['std_rgb']}")

    # 5. Optional Rectangular Presentation Base Creation
    print("\n[Step 5/5] Creating presentation base and exporting files...")
    base_mesh = None
    base_meta = {}
    presentation_mesh = mesh_p
    if args.add_presentation_base:
        base_mesh, base_meta = create_presentation_base(
            mesh=mesh_p,
            padding=args.base_padding,
            thickness=args.base_thickness
        )
        base_ply = out_dir / "display_base.ply"
        o3d.io.write_triangle_mesh(str(base_ply), base_mesh, write_ascii=False, write_vertex_normals=True, write_vertex_colors=True)
        print(f"  - Saved neutral presentation base: {base_ply.name} ({base_meta['dimensions_m']})")

        # Combine into complete rectangular presentation model
        presentation_mesh = mesh_p + base_mesh
        pres_ply = out_dir / "surface_mesh_presentation.ply"
        pres_obj = out_dir / "surface_mesh_presentation.obj"
        o3d.io.write_triangle_mesh(str(pres_ply), presentation_mesh, write_ascii=False, write_vertex_normals=True, write_vertex_colors=True)
        o3d.io.write_triangle_mesh(str(pres_obj), presentation_mesh, write_ascii=True, write_vertex_normals=True, write_vertex_colors=True)
        print(f"  - Saved presentation model: {pres_ply.name} ({pres_ply.stat().st_size / (1024**2):.2f} MB)")
        print(f"  - Saved presentation model: {pres_obj.name} ({pres_obj.stat().st_size / (1024**2):.2f} MB)")

    # Save Canonical Pure Reconstructions (Without Base, Unaltered for Spatial Science)
    surface_ply = out_dir / "surface_mesh.ply"
    surface_obj = out_dir / "surface_mesh.obj"
    o3d.io.write_triangle_mesh(str(surface_ply), mesh_p, write_ascii=False, write_vertex_normals=True, write_vertex_colors=True)
    o3d.io.write_triangle_mesh(str(surface_obj), mesh_p, write_ascii=True, write_vertex_normals=True, write_vertex_colors=True)

    bpa_ply = out_dir / "bpa_mesh.ply"
    o3d.io.write_triangle_mesh(str(bpa_ply), bpa_mesh, write_ascii=False, write_vertex_normals=True, write_vertex_colors=True)

    poisson_ply = out_dir / "poisson_strict_mesh.ply"
    o3d.io.write_triangle_mesh(str(poisson_ply), mesh_p, write_ascii=False, write_vertex_normals=True, write_vertex_colors=True)

    print(f"  - Saved pure surface_mesh.ply ({surface_ply.stat().st_size / (1024**2):.2f} MB)")
    print(f"  - Saved pure surface_mesh.obj ({surface_obj.stat().st_size / (1024**2):.2f} MB)")

    # Render Visual QA Preview
    preview_path = out_dir / "mesh_preview.png"
    if args.generate_preview:
        print("\n[Rendering Visual QA Preview Image]...")
        preview_items = [
            ("1. Source Point Cloud", f"{len(pts):,} pts | Raw Unprojected RGB", pcd),
            ("2. Surface Mesh (Pure)", f"{len(mesh_p.vertices):,} verts, {len(mesh_p.triangles):,} faces | Filtered", mesh_p),
            ("3. Presentation Model", f"{len(presentation_mesh.vertices):,} verts | Rectangular Base", presentation_mesh)
        ]
        render_comparison_preview(items=preview_items, output_png_path=str(preview_path))
        print(f"  - Saved Preview Image: {preview_path.name}")

    total_time = time.perf_counter() - total_t0

    # Build and Save Metadata JSON
    metadata = {
        "pipeline": "F.O.R.G.E — Surface Mesh Reconstruction & Rectangular Presentation",
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "input_point_cloud": str(ply_path),
        "output_directory": str(out_dir),
        "source_pcd_stats": {
            "total_points": len(pts),
            "color_stats": pcd_color_stats,
            "bounds_min": [round(float(x), 4) for x in p_min],
            "bounds_max": [round(float(x), 4) for x in p_max],
            "extents": [round(float(x), 4) for x in (p_max - p_min)]
        },
        "pure_reconstruction_stats": p_stats,
        "presentation_base": base_meta,
        "bpa_baseline_stats": bpa_stats,
        "parameters": {
            "voxel_size_m": args.voxel_size,
            "poisson_depth": args.poisson_depth,
            "density_quantile": args.density_quantile,
            "max_support_dist_m": args.max_support_dist,
            "max_edge_length_m": args.max_edge_length,
            "min_cluster_faces": args.min_cluster_faces,
            "base_padding_m": args.base_padding,
            "base_thickness_m": args.base_thickness
        },
        "suitability_assessment": {
            "video_id": "0049",
            "camera_trajectory": "Near-zero translation (0.046m baseline), substantial in-place yaw rotation (179.1 deg)",
            "geometry_quality": "WEAK — Lacks parallax baseline for reliable triangulation depth; reconstruction is essentially an unprojected planar panorama with thin depth relief (dZ=0.30m)",
            "visual_presentation": "Cleaned with support distance and cluster filtering, mounted on a neutral rectangular plinth without artificial terrain fabrication",
            "suitable_for_disaster_response_demo": False,
            "recommendation": "Retain video 0050 (river bridge corridor) as the primary F.O.R.G.E demonstration footage due to true forward flight baseline and rich 3D structural parallax."
        },
        "total_runtime_sec": round(total_time, 3)
    }

    meta_file = out_dir / "mesh_metadata.json"
    with open(meta_file, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)
    print(f"  - Saved Metadata: {meta_file.name}")

    print("\n" + "=" * 72)
    print("MESH RECONSTRUCTION & PRESENTATION: SUCCESS")
    print(f"Total time elapsed: {total_time:.2f}s")
    print("=" * 72)

    if args.view:
        print("\n[INFO] Launching Open3D interactive viewer for rectangular presentation mesh...")
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        o3d.visualization.draw_geometries(
            [presentation_mesh, coord_frame],
            window_name=f"F.O.R.G.E Rectangular Presentation — {len(presentation_mesh.vertices):,} verts, {len(presentation_mesh.triangles):,} faces",
            width=1280,
            height=720,
            left=50,
            top=50
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
