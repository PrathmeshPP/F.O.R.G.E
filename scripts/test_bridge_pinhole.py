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
cam_params = ctrl.convert_to_pinhole_camera_parameters()

# In pinhole parameters:
# Extrinsics is 4x4 matrix [R | t] where P_cam = R @ P_world + t
# Let bridge be at P_bridge = [0.95, -0.16, 1.55]
# Let camera be at C = P_bridge + [-0.3, -0.8, -0.8] (looking forward and down at the bridge)
# Camera distance ~ 1.1 meters!
target = np.array([0.95, -0.16, 1.55])
eye = target + np.array([-0.2, -0.7, -0.6]) # 0.95m away
up = np.array([0.0, -0.6, 0.8])

# Build camera matrix
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

cam_params.extrinsic = extrinsic
ctrl.convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)

vis.poll_events()
vis.update_renderer()
p = "scratch_bridge_pinhole_closeup.png"
vis.capture_screen_image(p, do_render=True)
vis.destroy_window()
print(f"Rendered {p}")
