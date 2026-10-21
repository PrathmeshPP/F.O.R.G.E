import open3d as o3d
import numpy as np
import cv2

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# Render top-down orthographic 2D image of the vertices by projecting (X, Z) onto an image
# Bounding box of mesh:
min_x, max_x = v[:, 0].min(), v[:, 0].max()
min_z, max_z = v[:, 2].min(), v[:, 2].max()

img_w, img_h = 1000, 1000
canvas = np.zeros((img_h, img_w, 3), dtype=np.uint8)

# Map X to col, Z to row
cols = np.clip(((v[:, 0] - min_x) / (max_x - min_x) * (img_w - 1)).astype(int), 0, img_w - 1)
rows = np.clip(((v[:, 2] - min_z) / (max_z - min_z) * (img_h - 1)).astype(int), 0, img_h - 1)
colors_bgr = (c[:, [2, 1, 0]] * 255).astype(np.uint8)

canvas[rows, cols] = colors_bgr

# Draw a coordinate grid with labels
for x_val in np.linspace(min_x, max_x, 9):
    col = int((x_val - min_x) / (max_x - min_x) * (img_w - 1))
    cv2.line(canvas, (col, 0), (col, img_h), (50, 50, 50), 1)
    cv2.putText(canvas, f"X={x_val:.1f}", (col + 2, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

for z_val in np.linspace(min_z, max_z, 9):
    row = int((z_val - min_z) / (max_z - min_z) * (img_h - 1))
    cv2.line(canvas, (0, row), (img_w, row), (50, 50, 50), 1)
    cv2.putText(canvas, f"Z={z_val:.1f}", (5, row - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

cv2.imwrite("scratch_rb_ortho_grid.png", canvas)
print("Saved scratch_rb_ortho_grid.png")
