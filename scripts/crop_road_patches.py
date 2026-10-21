import open3d as o3d
import numpy as np
import cv2

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# Check points along road
points = [
    ("pt0", [2.14, 0.30, -0.33]),
    ("pt25", [-0.60, 0.03, 0.90]),
    ("pt50", [0.66, -0.15, 1.65]),
    ("pt75", [0.30, -0.27, 2.07]),
    ("pt100", [-0.69, -0.43, 2.75]),
    ("bridge_guess1", [0.20, -0.05, 0.75]),
    ("bridge_guess2", [0.35, -0.10, 0.95]),
    ("bridge_guess3", [0.45, -0.12, 1.15]),
    ("bridge_guess4", [0.55, -0.15, 1.35]),
]

for name, pt in points:
    # 0.6m box
    mask = (np.abs(v[:, 0] - pt[0]) < 0.35) & (np.abs(v[:, 2] - pt[2]) < 0.35)
    pts = v[mask]
    cols = c[mask]
    if len(pts) > 100:
        # render 2D ortho patch
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        u = np.clip(((pts[:, 0] - (pt[0] - 0.35)) / 0.70 * 299).astype(int), 0, 299)
        v_idx = np.clip(((pts[:, 2] - (pt[2] - 0.35)) / 0.70 * 299).astype(int), 0, 299)
        img[v_idx, u] = (cols[:, [2, 1, 0]] * 255).astype(np.uint8)
        cv2.putText(img, f"{name}: X={pt[0]:.2f}, Z={pt[2]:.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imwrite(f"scratch_patch_{name}.png", img)
        print(f"Saved scratch_patch_{name}.png ({len(pts)} pts)")
