import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
ctr_pt = m.get_axis_aligned_bounding_box().get_center()

# In River Bridge:
# BBox: X in [-1.49, 2.41], Y in [-0.44, 0.30], Z in [-0.34, 2.75]
# Let's find where the bridge railing/structure is:
# Look at the bridge location in 2D top view:
# In top view (scratch_rb_test1_top.png or river_bridge/render_view_top_verified.png):
# The road curves from (0.0, 0.0) down-right to (1.2, 1.6).
# The river runs across around Z=1.3 to 1.7, X=0.8 to 1.3.

test_cfgs = [
    ("zoom_bridge", {"lookat": [0.95, -0.15, 1.55], "front": [0.0, -0.80, -0.60], "up": [0.0, -0.60, 0.80], "zoom": 0.30}),
    ("deck_closeup", {"lookat": [0.95, -0.15, 1.55], "front": [0.30, -0.75, -0.58], "up": [0.0, -0.61, 0.79], "zoom": 0.22}),
    ("side_piers", {"lookat": [0.95, -0.15, 1.55], "front": [-0.60, -0.60, -0.53], "up": [0.0, -0.66, 0.75], "zoom": 0.25}),
]

for name, cfg in test_cfgs:
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    vis.add_geometry(m)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(cfg["lookat"])
    ctrl.set_front(cfg["front"])
    ctrl.set_up(cfg["up"])
    ctrl.set_zoom(cfg["zoom"])

    vis.poll_events()
    vis.update_renderer()
    p = f"scratch_bridge_framing_{name}.png"
    vis.capture_screen_image(p, do_render=True)
    vis.destroy_window()
    print(f"Rendered {p}")
