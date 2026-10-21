import sys
import numpy as np
import open3d as o3d
from pathlib import Path
import cv2
from PIL import Image

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

# Load smoothed mesh
mesh = o3d.io.read_triangle_mesh("outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/candidate_a/surface_mesh_static_clean.ply")
verts = np.asarray(mesh.vertices)
tris = np.asarray(mesh.triangles)
tri_centers = verts[tris].mean(axis=1)
mesh.compute_triangle_normals()
tri_normals = np.asarray(mesh.triangle_normals)

face_cam = np.zeros(len(tris), dtype=int)
face_best_score = np.full(len(tris), -1e9)

for i in range(8):
    R = extrinsics[i, :3, :3]
    t = extrinsics[i, :3, 3]
    K = scaled_intrinsics[i]
    P_c = (R @ tri_centers.T + t[:, None]).T
    dist = np.linalg.norm(P_c, axis=1) + 1e-6
    rays = P_c / dist[:, None]
    N_c = (R @ tri_normals.T).T
    cos_t = -np.sum(rays * N_c, axis=1)
    
    # Project tri centers to camera i
    Z_c = np.maximum(P_c[:, 2], 1e-4)
    u = K[0, 0] * (P_c[:, 0] / Z_c) + K[0, 2]
    v = K[1, 1] * (P_c[:, 1] / Z_c) + K[1, 2]
    in_b = (P_c[:, 2] > 0.05) & (u >= 0) & (u < W_nat) & (v >= 0) & (v < H_nat)
    u_int = np.clip(u.astype(int), 0, W_nat - 1)
    v_int = np.clip(v.astype(int), 0, H_nat - 1)
    
    # Exclude faces that hit the car mask in camera i!
    in_car = in_b & car_masks[i][v_int, u_int]
    
    score = np.where(in_b & (~in_car), cos_t / np.maximum(dist, 0.3), -1e9)
    better = score > face_best_score
    face_cam[better] = i
    face_best_score[better] = score[better]

print(f"Face assignments across 8 cameras:")
for i in range(8):
    print(f"  Cam {i}: {(face_cam == i).sum()} faces")
