import open3d as o3d
import numpy as np
import cv2

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/forest_road/surface_mesh_photographic.ply")
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# Check colors: road is grey asphalt with yellow/white lines.
# Canopy is green foliage.
# Road vertices:
grey_mask = (np.std(c, axis=1) < 0.05) & (np.mean(c, axis=1) > 0.25) & (np.mean(c, axis=1) < 0.6)
v_road = v[grey_mask]
print(f"FR Road vertices: {len(v_road)}")
print(f"FR Road X: [{v_road[:,0].min():.3f}, {v_road[:,0].max():.3f}]")
print(f"FR Road Y: [{v_road[:,1].min():.3f}, {v_road[:,1].max():.3f}]")
print(f"FR Road Z: [{v_road[:,2].min():.3f}, {v_road[:,2].max():.3f}]")

# Tree canopy vertices: green foliage
green_mask = (c[:, 1] > c[:, 0] * 1.1) & (c[:, 1] > c[:, 2] * 1.1) & (c[:, 1] > 0.25)
v_tree = v[green_mask]
print(f"FR Tree vertices: {len(v_tree)}")
print(f"FR Tree X: [{v_tree[:,0].min():.3f}, {v_tree[:,0].max():.3f}]")
print(f"FR Tree Y: [{v_tree[:,1].min():.3f}, {v_tree[:,1].max():.3f}]")
print(f"FR Tree Z: [{v_tree[:,2].min():.3f}, {v_tree[:,2].max():.3f}]")

# Road mean Y vs Tree mean Y:
print(f"Road mean Y: {v_road[:,1].mean():.3f}, Tree mean Y: {v_tree[:,1].mean():.3f}")
# Road mean Z vs Tree mean Z:
print(f"Road mean Z: {v_road[:,2].mean():.3f}, Tree mean Z: {v_tree[:,2].mean():.3f}")
