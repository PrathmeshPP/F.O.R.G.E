import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")

test_cfgs = [
    ("test1", [0.70, -0.14, 1.30], 0.45),
    ("test2", [0.60, -0.12, 1.15], 0.45),
    ("test3", [0.75, -0.15, 1.45], 0.35),
    ("test4", [0.55, -0.10, 1.25], 0.35),
]

for name, lookat, zoom in test_cfgs:
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    vis.add_geometry(m)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(lookat)
    ctrl.set_front([0.20, -0.85, -0.48])
    ctrl.set_up([0.0, -0.48, 0.88])
    ctrl.set_zoom(zoom)

    vis.poll_events()
    vis.update_renderer()
    p = f"scratch_test_bridge_{name}.png"
    vis.capture_screen_image(p, do_render=True)
    vis.destroy_window()
    print(f"Rendered {p}")
