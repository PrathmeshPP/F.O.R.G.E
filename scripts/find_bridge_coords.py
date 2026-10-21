import open3d as o3d
import numpy as np

rb_mesh = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
v = np.asarray(rb_mesh.vertices)
c = np.asarray(rb_mesh.vertex_colors)

# In River Bridge:
# Water is typically green/blue or low elevation.
# The bridge is the structure spanning across the river channel.
# Let's find vertices with road-like color:
# Grey road: std of RGB is small, brightness > 0.3
color_std = np.std(c, axis=1)
brightness = np.mean(c, axis=1)
is_road = (color_std < 0.05) & (brightness > 0.35) & (brightness < 0.65)
v_road = v[is_road]
print(f"Road-like vertices: {len(v_road)}")
print(f"Road X: [{v_road[:,0].min():.2f}, {v_road[:,0].max():.2f}]")
print(f"Road Y: [{v_road[:,1].min():.2f}, {v_road[:,1].max():.2f}]")
print(f"Road Z: [{v_road[:,2].min():.2f}, {v_road[:,2].max():.2f}]")

# Let's check the bridge section specifically (across river)
# Look at the bridge center
print("Center of road points:", np.round(np.mean(v_road, axis=0), 3))
