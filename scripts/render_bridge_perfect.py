import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=1024, height=768)
vis.add_geometry(m)
opt = vis.get_render_option()
opt.background_color = np.asarray([0.08, 0.10, 0.14])
opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
opt.light_on = False

ctrl = vis.get_view_control()
# In scratch_test_bridge_test1, lookat was [0.70, -0.14, 1.30].
# Let's shift lookat down and right
ctrl.set_lookat([0.62, -0.16, 1.42])
ctrl.set_front([0.20, -0.85, -0.48])
ctrl.set_up([0.0, -0.48, 0.88])
ctrl.set_zoom(0.40)

vis.poll_events()
vis.update_renderer()
p = "scratch_bridge_framed_perfect.png"
vis.capture_screen_image(p, do_render=True)
vis.destroy_window()
print(f"Rendered {p}")
