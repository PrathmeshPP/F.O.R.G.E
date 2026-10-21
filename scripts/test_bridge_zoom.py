import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")

# Lookat centered right on the bridge crossing
lookat = [0.95, -0.18, 1.55]

for zoom in [0.55, 0.60, 0.65]:
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    vis.add_geometry(m)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(lookat)
    ctrl.set_front([0.15, -0.75, -0.64])
    ctrl.set_up([0.0, -0.64, 0.75])
    ctrl.set_zoom(zoom)

    vis.poll_events()
    vis.update_renderer()
    p = f"scratch_test_bridge_zoom_{zoom}.png"
    vis.capture_screen_image(p, do_render=True)
    vis.destroy_window()
    print(f"Rendered {p}")
