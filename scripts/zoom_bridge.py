import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# In scratch_bridge_from_cam7.png, the bridge is in the distance where the road crosses the river.
# Let's inspect vertices where the river meets the road:
# The river has dark/greenish/muddy texture or water channel.
# Let's find road vertices near the river:
# Look at road vertices around X in [0.8, 1.8], Z in [1.2, 2.2]
mask = (v[:, 0] > 0.6) & (v[:, 0] < 1.8) & (v[:, 2] > 1.2) & (v[:, 2] < 2.2)
sub_v = v[mask]
print(f"Candidate bridge vertices count: {len(sub_v)}")
center_sub = sub_v.mean(axis=0)
print(f"Center of candidate bridge: {np.round(center_sub, 3)}")

# Let's test 3 close-up camera angles targeting this center
test_angles = [
    ("bridge_angle1", center_sub, [-0.6, -0.6, -0.5], 0.15),
    ("bridge_angle2", center_sub, [0.0, -0.7, -0.7], 0.12),
    ("bridge_angle3", center_sub, [0.7, -0.5, -0.5], 0.12),
    ("bridge_overhead", center_sub, [0.0, -1.0, 0.0], 0.18),
]

for name, lookat, front, zoom in test_angles:
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    vis.add_geometry(m)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(lookat)
    ctrl.set_front(front)
    ctrl.set_up([0.0, 0.0, 1.0] if abs(front[1]) > 0.9 else [0.0, -1.0, 0.0])
    ctrl.set_zoom(zoom)

    vis.poll_events()
    vis.update_renderer()
    p = f"scratch_bridge_zoom_{name}.png"
    vis.capture_screen_image(p, do_render=True)
    vis.destroy_window()
    print(f"Rendered {p}")
