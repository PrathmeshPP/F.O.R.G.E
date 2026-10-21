import open3d as o3d
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")

# Let's test a few lookat positions along the road near the river crossing
# The river runs across the road. Let's find the bridge location.
# Let's render a grid of views along X from 0.0 to 1.5, Z from 0.5 to 2.0
test_lookats = [
    ("bridge_A", [0.45, -0.05, 1.15], 0.20),
    ("bridge_B", [0.80, -0.15, 1.60], 0.20),
    ("bridge_C", [1.10, -0.20, 1.80], 0.20),
    ("bridge_D", [0.60, -0.10, 1.40], 0.15),
    ("bridge_E", [0.75, -0.12, 1.50], 0.12),
]

for name, lookat, zoom in test_lookats:
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    vis.add_geometry(m)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(lookat)
    ctrl.set_front([0.15, -0.65, -0.74])
    ctrl.set_up([0.0, -0.74, 0.65])
    ctrl.set_zoom(zoom)

    vis.poll_events()
    vis.update_renderer()
    p = f"scratch_bridge_test_{name}.png"
    vis.capture_screen_image(p, do_render=True)
    vis.destroy_window()
    print(f"Rendered {p}")
