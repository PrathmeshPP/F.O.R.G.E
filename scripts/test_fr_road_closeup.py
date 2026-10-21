import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/forest_road/surface_mesh_photographic.ply")

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=1024, height=768)
vis.add_geometry(m)
opt = vis.get_render_option()
opt.background_color = np.asarray([0.08, 0.10, 0.14])
opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
opt.light_on = False

ctrl = vis.get_view_control()
# Road bend is at X ~ -0.25, Y ~ 0.05, Z ~ 0.95
ctrl.set_lookat([-0.25, 0.05, 0.95])
ctrl.set_front([0.0, -0.35, -0.94])
ctrl.set_up([0.0, -0.94, 0.35])
ctrl.set_zoom(0.22)

vis.poll_events()
vis.update_renderer()
p = "scratch_fr_road_closeup.png"
vis.capture_screen_image(p, do_render=True)
vis.destroy_window()
print(f"Rendered {p}")
