import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=1024, height=768)
vis.add_geometry(m)

ctrl = vis.get_view_control()
ctrl.set_lookat([0.62, -0.16, 1.42])
ctrl.set_front([0.20, -0.85, -0.48])
ctrl.set_up([0.0, -0.48, 0.88])
ctrl.set_zoom(0.40)

# Capture depth buffer
vis.poll_events()
vis.update_renderer()
depth = vis.capture_depth_float_buffer(do_render=True)
depth_arr = np.asarray(depth)

# Pinhole params
pinhole = ctrl.convert_to_pinhole_camera_parameters()
K = pinhole.intrinsic.intrinsic_matrix
extrinsic = pinhole.extrinsic
R = extrinsic[:3, :3]
t = extrinsic[:3, 3]

# The bridge is around pixel (u=640, v=630)
u_pix, v_pix = 640, 630
z_cam = depth_arr[v_pix, u_pix]
print(f"Depth at ({u_pix}, {v_pix}): {z_cam}")

# Unproject to 3D world coordinates
x_cam = (u_pix - K[0, 2]) * z_cam / K[0, 0]
y_cam = (v_pix - K[1, 2]) * z_cam / K[1, 1]
P_cam = np.array([x_cam, y_cam, z_cam])
P_world = R.T @ (P_cam - t)
print(f"Exact bridge world coordinates: {np.round(P_world, 3)}")

vis.destroy_window()
