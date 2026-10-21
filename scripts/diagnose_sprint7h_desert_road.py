import sys
import numpy as np
import open3d as o3d
from pathlib import Path
import cv2

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")

# Load candidate A from 7g
p_7g_a = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/candidate_a"
p_7f_a = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7f_final_acceptance/desert_road/candidate_a"

pcd = o3d.io.read_point_cloud(str(p_7g_a / "point_cloud_static_clean.ply"))
raw_mesh = o3d.io.read_triangle_mesh(str(p_7f_a / "surface_mesh_static_clean.ply"))
smooth_mesh = o3d.io.read_triangle_mesh(str(p_7g_a / "surface_mesh_static_clean.ply"))
photo_mesh = o3d.io.read_triangle_mesh(str(p_7g_a / "surface_mesh_photographic.ply"))

print(f"PCD points: {len(pcd.points)}")
print(f"Raw mesh vertices: {len(raw_mesh.vertices)}, triangles: {len(raw_mesh.triangles)}")
print(f"Smooth mesh vertices: {len(smooth_mesh.vertices)}, triangles: {len(smooth_mesh.triangles)}")
print(f"Photo mesh vertices: {len(photo_mesh.vertices)}")

# Let's inspect cross-sections across the sandy shoulders at several Z positions along the road:
# Z goes from ~ -2.5 to ~ 1.0
v = np.asarray(smooth_mesh.vertices)
n = np.asarray(smooth_mesh.vertex_normals)
c = np.asarray(photo_mesh.vertex_colors)

# Sample z slices
for z_val in [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5]:
    mask = (v[:, 2] >= z_val - 0.05) & (v[:, 2] <= z_val + 0.05)
    pts = v[mask]
    cols = c[mask]
    normals = n[mask]
    # Shoulder left (X < -0.2), road (-0.2 <= X <= 0.2), shoulder right (X > 0.2)
    left_sand = pts[:, 0] < -0.2
    right_sand = pts[:, 0] > 0.2
    print(f"Slice Z={z_val:.1f}: total {len(pts)} pts | Left sand: {left_sand.sum()} pts, Right sand: {right_sand.sum()} pts")
    if left_sand.sum() > 0:
        y_left = pts[left_sand, 1]
        c_left = cols[left_sand]
        mean_c = np.round(c_left.mean(axis=0), 3)
        std_c = np.round(c_left.std(axis=0), 3)
        print(f"   Left sand Y: min={y_left.min():.4f}, max={y_left.max():.4f}, std={y_left.std():.4f} | Color RGB mean={mean_c}, std={std_c}")
    if right_sand.sum() > 0:
        y_right = pts[right_sand, 1]
        c_right = cols[right_sand]
        mean_c = np.round(c_right.mean(axis=0), 3)
        std_c = np.round(c_right.std(axis=0), 3)
        print(f"   Right sand Y: min={y_right.min():.4f}, max={y_right.max():.4f}, std={y_right.std():.4f} | Color RGB mean={mean_c}, std={std_c}")

# Check the reddish area
# In 7g: where are vertices with high red / low green/blue or distinct reddish hue?
# Typical sand RGB: [0.65, 0.52, 0.38]
r_ratio = c[:, 0] / np.maximum(c[:, 1], 1e-4)
reddish_mask = (r_ratio > 1.45) & (v[:, 0] > 0.1) & (v[:, 0] < 0.4)
print(f"\nReddish vertices count: {reddish_mask.sum()}")
if reddish_mask.sum() > 0:
    red_pts = v[reddish_mask]
    print(f"Reddish pts X range: [{red_pts[:, 0].min():.3f}, {red_pts[:, 0].max():.3f}]")
    print(f"Reddish pts Y range: [{red_pts[:, 1].min():.3f}, {red_pts[:, 1].max():.3f}]")
    print(f"Reddish pts Z range: [{red_pts[:, 2].min():.3f}, {red_pts[:, 2].max():.3f}]")
