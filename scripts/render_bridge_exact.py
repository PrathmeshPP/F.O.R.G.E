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
pinhole = ctrl.convert_to_pinhole_camera_parameters()

target = np.array([1.131, 0.166, 0.526]) # Exact bridge location
eye = target + np.array([-0.3, -0.7, -0.5]) # close camera viewing bridge
up = np.array([0.0, -0.5, 0.85])

z_axis = target - eye
z_axis /= np.linalg.norm(z_axis)
x_axis = np.cross(up, z_axis)
x_axis /= np.linalg.norm(x_axis)
y_axis = np.cross(z_axis, x_axis)

R = np.vstack([x_axis, y_axis, z_axis])
t = -R @ eye

extrinsic = np.eye(4)
extrinsic[:3, :3] = R
extrinsic[:3, 3] = t

pinhole.extrinsic = extrinsic
ctrl.convert_from_pinhole_camera_parameters(pinhole, allow_arbitrary=True)

vis.poll_events()
vis.update_renderer()
p = "scratch_bridge_centered_exact.png"
vis.capture_screen_image(p, do_render=True)
vis.destroy_window()
print(f"Rendered {p}")
