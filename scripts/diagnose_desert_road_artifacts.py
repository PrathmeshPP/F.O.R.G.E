"""
F.O.R.G.E — Diagnostic script for Desert Road:
Renders:
1. Neutral grey material (geometry only, vertex colors disabled)
2. Vertex-color rendering
3. Full textured rendering (if texture atlas exists)
"""

from pathlib import Path
import numpy as np
import open3d as o3d
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REC_V1_DESERT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v1/desert_road"
DIAG_DIR = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v2/diagnostics"
DIAG_DIR.mkdir(parents=True, exist_ok=True)

mesh_path = REC_V1_DESERT / "masked_reconstruction.ply"
mesh = o3d.io.read_triangle_mesh(str(mesh_path))
mesh.compute_vertex_normals()

ctr = mesh.get_axis_aligned_bounding_box().get_center()
front = [0.0, -0.707, -0.707]
up = [0.0, -0.707, 0.707]
zoom = 0.65

w, h = 800, 600

# 1. Neutral grey render
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=w, height=h)
m_grey = o3d.geometry.TriangleMesh(mesh)
m_grey.vertex_colors = o3d.utility.Vector3dVector(np.full((len(m_grey.vertices), 3), 0.72))
vis.add_geometry(m_grey)
opt = vis.get_render_option()
opt.background_color = np.asarray([0.08, 0.10, 0.14])
opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
opt.light_on = True

ctrl = vis.get_view_control()
ctrl.set_lookat(ctr)
ctrl.set_front(front)
ctrl.set_up(up)
ctrl.set_zoom(zoom)
vis.poll_events()
vis.update_renderer()
buf_grey = vis.capture_screen_float_buffer(do_render=True)
vis.destroy_window()
img_grey = Image.fromarray((np.asarray(buf_grey) * 255).astype(np.uint8))
img_grey.save(DIAG_DIR / "desert_road_diag_neutral_grey.png")

# 2. Vertex-color render
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=w, height=h)
vis.add_geometry(mesh)
opt = vis.get_render_option()
opt.background_color = np.asarray([0.08, 0.10, 0.14])
opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
opt.light_on = False

ctrl = vis.get_view_control()
ctrl.set_lookat(ctr)
ctrl.set_front(front)
ctrl.set_up(up)
ctrl.set_zoom(zoom)
vis.poll_events()
vis.update_renderer()
buf_vc = vis.capture_screen_float_buffer(do_render=True)
vis.destroy_window()
img_vc = Image.fromarray((np.asarray(buf_vc) * 255).astype(np.uint8))
img_vc.save(DIAG_DIR / "desert_road_diag_vertex_colors.png")

print(f"[+] Diagnostic renders saved to {DIAG_DIR}")
