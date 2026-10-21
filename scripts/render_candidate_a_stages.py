import sys
import numpy as np
import open3d as o3d
from pathlib import Path

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
out_dir = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7h_final_defect_recovery/desert_road/candidate_a/stage_audit"
out_dir.mkdir(parents=True, exist_ok=True)

# Sources from 7f and 7g
p_7f_a = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7f_final_acceptance/desert_road/candidate_a"
p_7g_a = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/candidate_a"

# 1. Fused point cloud
pcd = o3d.io.read_point_cloud(str(p_7g_a / "point_cloud_static_clean.ply"))

# 2. Raw untextured mesh
raw_mesh = o3d.io.read_triangle_mesh(str(p_7f_a / "surface_mesh_static_clean.ply"))
raw_mesh.compute_vertex_normals()

# 3. Smoothed untextured mesh
smooth_mesh = o3d.io.read_triangle_mesh(str(p_7g_a / "surface_mesh_static_clean.ply"))
smooth_mesh.compute_vertex_normals()

# 4. Photographic vertex-colored mesh
photo_mesh = o3d.io.read_triangle_mesh(str(p_7g_a / "surface_mesh_photographic.ply"))

# 5. Final textured OBJ
textured_mesh = o3d.io.read_triangle_mesh(str(p_7g_a / "textured_mesh.obj"), True)

def render_geom(geom, out_path, is_untextured=False, is_pcd=False):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    
    if is_untextured:
        m = o3d.geometry.TriangleMesh(geom)
        m.compute_vertex_normals()
        gray = np.full((len(m.vertices), 3), 0.72)
        m.vertex_colors = o3d.utility.Vector3dVector(gray)
        vis.add_geometry(m)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.10, 0.14])
        opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
        opt.mesh_shade_option = o3d.visualization.MeshShadeOption.Color
        opt.light_on = True
    elif is_pcd:
        vis.add_geometry(geom)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.10, 0.14])
        opt.point_size = 3.0
    else:
        vis.add_geometry(geom)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.10, 0.14])
        opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
        opt.light_on = False

    ctrl = vis.get_view_control()
    # Let's use the shoulder closeup view and top view
    ctr = [0.0, 0.08, -0.6]
    ctrl.set_lookat(ctr)
    ctrl.set_front([0.0, -1.0, 0.0])
    ctrl.set_up([0.0, 0.0, 1.0])
    ctrl.set_zoom(0.70)

    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(str(out_path), do_render=True)
    vis.destroy_window()

print("Rendering Stage 1: Point Cloud...")
render_geom(pcd, out_dir / "stage1_point_cloud.png", is_pcd=True)

print("Rendering Stage 2: Raw Mesh (Untextured)...")
render_geom(raw_mesh, out_dir / "stage2_raw_untextured_mesh.png", is_untextured=True)

print("Rendering Stage 3: Smoothed Mesh (Untextured)...")
render_geom(smooth_mesh, out_dir / "stage3_smoothed_untextured_mesh.png", is_untextured=True)

print("Rendering Stage 4: Photographic Vertex-Colored Mesh...")
render_geom(photo_mesh, out_dir / "stage4_photo_mesh.png")

print("Rendering Stage 5: Final Textured OBJ...")
render_geom(textured_mesh, out_dir / "stage5_textured_obj.png")

print("All 5 stages rendered successfully!")
