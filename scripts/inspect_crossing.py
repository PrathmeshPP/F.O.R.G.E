import open3d as o3d
import numpy as np
import cv2

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# Check around X in [0.4, 1.0], Z in [0.6, 1.0]
mask = (v[:, 0] > 0.4) & (v[:, 0] < 1.0) & (v[:, 2] > 0.6) & (v[:, 2] < 1.0)
pts = v[mask]
cols = c[mask]
print(f"Points in region: {len(pts)}")

# Render ortho image of this region
img = np.zeros((600, 600, 3), dtype=np.uint8)
u = np.clip(((pts[:, 0] - 0.4) / 0.6 * 599).astype(int), 0, 599)
v_idx = np.clip(((pts[:, 2] - 0.6) / 0.4 * 599).astype(int), 0, 599)
img[v_idx, u] = (cols[:, [2, 1, 0]] * 255).astype(np.uint8)
cv2.imwrite("scratch_bridge_stream_crossing.png", img)
print("Saved scratch_bridge_stream_crossing.png")
