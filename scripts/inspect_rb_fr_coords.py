import open3d as o3d
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")

# River bridge
rb_mesh = o3d.io.read_triangle_mesh(str(PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply"))
v_rb = np.asarray(rb_mesh.vertices)
c_rb = np.asarray(rb_mesh.vertex_colors)

print(f"River bridge vertices: {len(v_rb)}")
print(f"RB BBox Min: {np.round(v_rb.min(axis=0), 3)}, Max: {np.round(v_rb.max(axis=0), 3)}")

# Where is the bridge deck?
# Road/deck is typically grey asphalt/concrete: c[:, 0] ~ c[:, 1] ~ c[:, 2] ~ 0.4-0.6
# Let's inspect vertices along the road line
# Forest road
fr_mesh = o3d.io.read_triangle_mesh(str(PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/forest_road/surface_mesh_photographic.ply"))
v_fr = np.asarray(fr_mesh.vertices)
print(f"\nForest road vertices: {len(v_fr)}")
print(f"FR BBox Min: {np.round(v_fr.min(axis=0), 3)}, Max: {np.round(v_fr.max(axis=0), 3)}")

# In forest road, where is the road surface vs canopy?
# In FR, Y is up or Z is up? Let's check min/max in each axis.
