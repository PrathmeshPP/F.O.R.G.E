import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/river_bridge/surface_mesh_photographic.ply")
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# Let's inspect vertices along that long dirt road/paved road
# In the image scratch_bridge_framing_zoom_bridge.png:
# Top end of road is at high Z or low Z?
# In Open3D, with front=[0, -0.8, -0.6], up=[0, -0.6, 0.8]:
# Camera looks in +Y, +Z. Up is -Y, +Z.
# Let's find vertices with road colors and inspect their (X, Y, Z)
print(f"Mesh center: {m.get_axis_aligned_bounding_box().get_center()}")
print(f"Mesh extent: {m.get_axis_aligned_bounding_box().get_extent()}")

# Let's sample 5 points along the road:
# Look at vertices with low saturation and mid brightness
sat = np.max(c, axis=1) - np.min(c, axis=1)
val = np.max(c, axis=1)
road_idx = np.where((sat < 0.15) & (val > 0.40) & (val < 0.70))[0]
v_road = v[road_idx]

# Let's print min and max coordinates
print(f"Road X: {v_road[:,0].min():.3f} to {v_road[:,0].max():.3f}")
print(f"Road Y: {v_road[:,1].min():.3f} to {v_road[:,1].max():.3f}")
print(f"Road Z: {v_road[:,2].min():.3f} to {v_road[:,2].max():.3f}")

# Sort along Z
idx_sort = np.argsort(v_road[:, 2])
for p in [0.0, 0.25, 0.5, 0.75, 1.0]:
    i = int(p * (len(idx_sort) - 1))
    pt = v_road[idx_sort[i]]
    print(f"Road percentile {p*100:.0f}%: X={pt[0]:.3f}, Y={pt[1]:.3f}, Z={pt[2]:.3f}")
