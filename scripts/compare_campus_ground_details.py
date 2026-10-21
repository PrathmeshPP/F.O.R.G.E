"""
Render close-up comparison of ground details in Campus 0049:
1. Manual review model (with 4K texture atlas)
2. Recovery v1 model (per-vertex color)
"""

from pathlib import Path
import numpy as np
import open3d as o3d
import trimesh
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DIAG_DIR = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v2/diagnostics"
DIAG_DIR.mkdir(parents=True, exist_ok=True)

# 1. Manual model review (Textured OBJ with Atlas)
manual_obj = PROJECT_ROOT / "outputs/manual_model_review/campus_0049/model_textured.obj"
m_manual = o3d.io.read_triangle_model(str(manual_obj))

# Let's render offscreen with Open3D visualization or PyVista/Trimesh
# In open3d, read_triangle_mesh with texture:
m_manual_mesh = o3d.io.read_triangle_mesh(str(manual_obj), enable_post_processing=True)
print("Manual mesh has textures:", len(m_manual_mesh.textures))

# Let's check close-up coordinates around courtyard / ground markings
# Center: [0.15, 1.25, 0.75], zoom=0.25
ctr = [0.15, 1.25, 0.75]
front = [0.20, -0.40, -0.89]
up = [0.0, -1.0, 0.0]
zoom = 0.25
w, h = 1000, 750

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=w, height=h)
vis.add_geometry(m_manual_mesh)
ctrl = vis.get_view_control()
ctrl.set_lookat(ctr)
ctrl.set_front(front)
ctrl.set_up(up)
ctrl.set_zoom(zoom)
vis.poll_events()
vis.update_renderer()
buf = vis.capture_screen_float_buffer(do_render=True)
vis.destroy_window()
img_manual = Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))
img_manual.save(DIAG_DIR / "campus_manual_review_ground_detail.png")

# 2. Recovery v1 model (vertex color)
rec_ply = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v1/campus_0049/masked_reconstruction.ply"
m_rec = o3d.io.read_triangle_mesh(str(rec_ply))
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=w, height=h)
vis.add_geometry(m_rec)
ctrl = vis.get_view_control()
ctrl.set_lookat(ctr)
ctrl.set_front(front)
ctrl.set_up(up)
ctrl.set_zoom(zoom)
vis.poll_events()
vis.update_renderer()
buf = vis.capture_screen_float_buffer(do_render=True)
vis.destroy_window()
img_rec = Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))
img_rec.save(DIAG_DIR / "campus_recovery_v1_ground_detail.png")

print(f"[+] Saved close-up renders to {DIAG_DIR}")
