import open3d as o3d
import numpy as np

m = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/candidate_b/surface_mesh_vehicle_stabilized.ply")
v = np.asarray(m.vertices)

# Fit plane: Y = -0.019475 * X - 0.057574 * Z + 0.088181
y_road = -0.019475 * v[:, 0] - 0.057574 * v[:, 2] + 0.088181
delta_y = y_road - v[:, 1] # Positive if above road plane (since -Y is UP)

# Look in vehicle corridor: X in [-0.15, 0.25], Z in [0.2, 0.7]
veh_box = (v[:, 0] > -0.15) & (v[:, 0] < 0.25) & (v[:, 2] > 0.2) & (v[:, 2] < 0.7)
print(f"Points in vehicle bounding box: {veh_box.sum()}")
print(f"Delta Y (height above road plane) for box points: min={delta_y[veh_box].min():.4f}, max={delta_y[veh_box].max():.4f}, mean={delta_y[veh_box].mean():.4f}")

elevated_car = veh_box & (delta_y > 0.015) # More than 1.5 cm above road
print(f"Elevated car vertices (body/roof/balloons): {elevated_car.sum()}")
road_around_car = veh_box & (~elevated_car)
print(f"Road surface vertices under/around car: {road_around_car.sum()}")
