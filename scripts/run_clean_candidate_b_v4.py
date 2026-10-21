import sys
import cv2
import numpy as np
import open3d as o3d
from pathlib import Path

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
sys.path.insert(0, str(PROJECT_ROOT))
from src.reconstruction.photographic_texture import scale_camera_intrinsics

pred_dir = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions"
native_dir = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/native_frames"

poses_data = np.load(pred_dir / "camera_poses.npz")
extrinsics = poses_data["extrinsics"]
intrinsics = poses_data["intrinsics"]
scaled_intrinsics = scale_camera_intrinsics(intrinsics, (688, 384), (1280, 720))

native_paths = sorted(list(native_dir.glob("frame_*.png")))
frames = [cv2.imread(str(p)) for p in native_paths]
H_nat, W_nat = frames[0].shape[:2]

scales = [
    np.array([1.0, 1.0, 1.0]),
    np.array([0.961, 0.968, 0.977]),
    np.array([0.954, 0.965, 0.974]),
    np.array([0.974, 0.981, 0.993]),
    np.array([0.985, 1.001, 1.007]),
    np.array([1.011, 1.029, 1.037]),
    np.array([1.054, 1.063, 1.076]),
    np.array([1.068, 1.077, 1.083])
]
comp_frames = [np.clip(im.astype(np.float64) * scales[i][None, None, :], 0, 255).astype(np.uint8) for i, im in enumerate(frames)]

# 1. GrabCut shape-aware mask for vehicle in Frame 0
im0 = frames[0]
mask_gc = np.zeros((H_nat, W_nat), np.uint8)
bgdModel = np.zeros((1, 65), np.float64)
fgdModel = np.zeros((1, 65), np.float64)
rect = (475, 370, 245, 195)
cv2.grabCut(im0, mask_gc, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
veh_mask0 = np.where((mask_gc == 2) | (mask_gc == 0), 0, 1).astype(bool)

hsv0 = cv2.cvtColor(im0, cv2.COLOR_BGR2HSV)
shadow0 = np.zeros((H_nat, W_nat), dtype=bool)
shadow0[460:555, 660:715] = hsv0[460:555, 660:715, 2] < 68
veh_instance_mask0 = veh_mask0 | shadow0

# Moving vehicle masks for all frames
car_masks = []
for i, im in enumerate(frames):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    teal = (hsv[:, :, 0] >= 85) & (hsv[:, :, 0] <= 115) & (hsv[:, :, 1] >= 60) & (hsv[:, :, 2] >= 50)
    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(teal.astype(np.uint8))
    m = np.zeros((H_nat, W_nat), dtype=bool)
    if n_labels > 1:
        best = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        x, y, w, h, area = stats[best]
        x1 = max(0, x - 45)
        x2 = min(W_nat - 1, x + w + 85)
        y1 = max(0, y - 180)
        y2 = min(H_nat - 1, y + h + 50)
        m[y1:y2, x1:x2] = True
    car_masks.append(m)

# 2. Candidate B Geometry
src_mesh_b = o3d.io.read_triangle_mesh(str(PROJECT_ROOT / "outputs/accuracy_experiment/sprint7f_final_acceptance/desert_road/candidate_b/surface_mesh_vehicle_stabilized.ply"))
v_orig = np.asarray(src_mesh_b.vertices)
N_v = len(v_orig)

# Road plane fit
y_road = -0.019475 * v_orig[:, 0] - 0.057574 * v_orig[:, 2] + 0.088181
delta_y = y_road - v_orig[:, 1]
is_elevated_car = (v_orig[:, 0] > -0.15) & (v_orig[:, 0] < 0.25) & (v_orig[:, 2] > 0.20) & (v_orig[:, 2] < 0.70) & (delta_y > 0.012)

mesh_smooth = src_mesh_b.filter_smooth_taubin(number_of_iterations=30)
v_smoothed = np.asarray(mesh_smooth.vertices)
v_smoothed[is_elevated_car] = v_orig[is_elevated_car]
mesh_smooth.vertices = o3d.utility.Vector3dVector(v_smoothed)
mesh_smooth.compute_vertex_normals()

# 3. Candidate B Texturing:
# Determine vehicle vertices by projecting into Frame 0
R0 = extrinsics[0, :3, :3]
t0 = extrinsics[0, :3, 3]
K0 = scaled_intrinsics[0]
P_c0 = (R0 @ v_smoothed.T + t0[:, None]).T
z0 = P_c0[:, 2]
u0 = K0[0, 0] * (P_c0[:, 0] / np.maximum(z0, 1e-4)) + K0[0, 2]
v0 = K0[1, 1] * (P_c0[:, 1] / np.maximum(z0, 1e-4)) + K0[1, 2]
u0_int = np.clip(u0.astype(int), 0, W_nat - 1)
v0_int = np.clip(v0.astype(int), 0, H_nat - 1)

# In Frame 0, exactly which 3D vertices project into the shape-aware vehicle mask:
is_veh_vert = (z0 > 0.05) & (u0 >= 0) & (u0 < W_nat) & (v0 >= 0) & (v0 < H_nat) & veh_instance_mask0[v0_int, u0_int] & (v_smoothed[:, 0] > -0.15) & (v_smoothed[:, 0] < 0.30) & (v_smoothed[:, 2] > 0.15) & (v_smoothed[:, 2] < 0.75)
print(f"Vehicle vertices in Candidate B: {is_veh_vert.sum()}")

blend_cols = np.zeros((N_v, 3), dtype=np.float64)
weights = np.zeros(N_v, dtype=np.float64)

for i in range(len(comp_frames)):
    R = extrinsics[i, :3, :3]
    t = extrinsics[i, :3, 3]
    K = scaled_intrinsics[i]
    rgb = cv2.cvtColor(comp_frames[i], cv2.COLOR_BGR2RGB)

    P_c = (R @ v_smoothed.T + t[:, None]).T
    Z_c = P_c[:, 2]
    valid_z = Z_c > 0.05
    u = K[0, 0] * (P_c[:, 0] / np.maximum(Z_c, 1e-4)) + K[0, 2]
    v = K[1, 1] * (P_c[:, 1] / np.maximum(Z_c, 1e-4)) + K[1, 2]

    in_bounds = valid_z & (u >= 0) & (u < W_nat) & (v >= 0) & (v < H_nat)
    u_int = np.clip(u.astype(np.int32), 0, W_nat - 1)
    v_int = np.clip(v.astype(np.int32), 0, H_nat - 1)

    if i == 0:
        # In frame 0: vehicle vertices receive texture from Frame 0.
        # Static road/sand vertices also receive texture from Frame 0.
        # Moving car in later frames is not here.
        in_bounds = in_bounds
    else:
        # In frames 1..7: exclude moving vehicle in frame i
        in_car = car_masks[i][v_int, u_int]
        # And do NOT overwrite the preserved vehicle with road texture
        in_bounds = in_bounds & (~in_car) & (~is_veh_vert)

    # Raised-cosine windowing
    fb = np.clip((H_nat - 1.0 - v) / 270.0, 0.0, 1.0)
    fade_b = 0.5 - 0.5 * np.cos(fb * np.pi)
    ft = np.clip(v / 200.0, 0.0, 1.0)
    fade_t = 0.5 - 0.5 * np.cos(ft * np.pi)
    fl = np.clip(u / 200.0, 0.0, 1.0)
    fade_l = 0.5 - 0.5 * np.cos(fl * np.pi)
    fr = np.clip((W_nat - 1.0 - u) / 200.0, 0.0, 1.0)
    fade_r = 0.5 - 0.5 * np.cos(fr * np.pi)

    feather = fade_b * fade_t * fade_l * fade_r
    if i == 0:
        feather = np.where(is_veh_vert, 1.0, feather)

    valid_pts = in_bounds & (feather > 1e-5)
    if valid_pts.any():
        sampled = rgb[v_int[valid_pts], u_int[valid_pts]].astype(np.float64) / 255.0
        w = feather[valid_pts]
        blend_cols[valid_pts] += sampled * w[:, None]
        weights[valid_pts] += w

mask = weights > 1e-6
print(f"Masked vertices: {mask.sum()} / {N_v} ({mask.sum()/N_v*100:.1f}%)")
final_cols = np.full((N_v, 3), 0.5, dtype=np.float64)
final_cols[mask] = blend_cols[mask] / weights[mask, None]

unweighted = ~mask
if unweighted.any():
    kdtree = o3d.geometry.KDTreeFlann(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(v_smoothed[mask])))
    for idx in np.where(unweighted)[0]:
        _, nearest_idx, _ = kdtree.search_knn_vector_3d(v_smoothed[idx], 1)
        final_cols[idx] = final_cols[mask][nearest_idx[0]]

print(f"Candidate B color mean: {np.round(final_cols.mean(axis=0), 3)}")

m_photo_b = o3d.geometry.TriangleMesh(mesh_smooth)
m_photo_b.vertex_colors = o3d.utility.Vector3dVector(final_cols)

def render_view(mesh, cfg, out_p):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    vis.add_geometry(mesh)
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
    vis.capture_screen_image(str(out_p), do_render=True)
    vis.destroy_window()

ctr = m_photo_b.get_axis_aligned_bounding_box().get_center()
render_view(m_photo_b, {"front": [0.0, -1.0, 0.0], "up": [0.0, 0.0, 1.0], "zoom": 0.75, "lookat": ctr}, "scratch_cand_b_clean_v4_top.png")
render_view(m_photo_b, {"front": [0.20, -0.60, -0.77], "up": [0.0, -0.78, 0.62], "zoom": 0.35, "lookat": [-0.05, 0.06, 0.52]}, "scratch_cand_b_clean_v4_veh.png")
print("Rendered scratch_cand_b_clean_v4_top.png and scratch_cand_b_clean_v4_veh.png successfully!")
