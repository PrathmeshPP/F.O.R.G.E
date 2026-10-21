"""
Test mask-aware texturing for Desert Road:
Verify that dynamic YOLO masks prevent vehicle pixels from being projected onto the road surface.
"""

from pathlib import Path
import cv2
import numpy as np
import open3d as o3d
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PRED_DIR = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions"
FRAMES_DIR = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/frames"

poses_data = np.load(PRED_DIR / "camera_poses.npz")
depth_data = np.load(PRED_DIR / "depth_predictions.npz")
extrinsics = poses_data["extrinsics"]
intrinsics = poses_data["intrinsics"]
d_maps = np.squeeze(depth_data["depth"])
H_inf, W_inf = d_maps.shape[1], d_maps.shape[2]

frames_files = sorted(list(FRAMES_DIR.glob("*.png")))
num_cams = len(frames_files)

import sys
sys.path.insert(0, str(PROJECT_ROOT))
from src.detection.yolo_segmenter import YOLOInstanceSegmenter
seg = YOLOInstanceSegmenter(
    model_path=r"C:\Users\25beevdt025\Desktop\SIH\yolov8n-seg.pt",
    target_classes=["car", "truck", "bus", "motorcycle", "bicycle"],
    dilation_pixels=12,
    device="cpu"
)

frame_rgbs = []
car_masks_inf = []
for idx, f in enumerate(frames_files):
    im_bgr = cv2.imread(str(f))
    rgb = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2RGB)
    res = seg.segment_frame(rgb, frame_idx=idx, timestamp_sec=idx*2.836)
    m_inf = cv2.resize(res.combined_mask, (W_inf, H_inf), interpolation=cv2.INTER_NEAREST) > 0
    car_masks_inf.append(m_inf)
    
    im_inf = cv2.resize(rgb, (W_inf, H_inf), interpolation=cv2.INTER_LINEAR)
    frame_rgbs.append(im_inf)

frame_rgbs = np.stack(frame_rgbs, axis=0)
car_masks_inf = np.stack(car_masks_inf, axis=0)
print(f"[+] Loaded {num_cams} frames and computed {car_masks_inf.shape} masks.")

# Load the Desert Road mesh
mesh_path = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v1/desert_road/masked_reconstruction.ply"
mesh = o3d.io.read_triangle_mesh(str(mesh_path))
verts = np.asarray(mesh.vertices)
mesh.compute_vertex_normals()
normals = np.asarray(mesh.vertex_normals)
N_verts = len(verts)

# Load fallback point cloud from recovery_v1 or recompute
best_cos = np.full(N_verts, -1.0, dtype=np.float32)
best_colors = np.zeros((N_verts, 3), dtype=np.float32)
observation_counts = np.zeros(N_verts, dtype=np.int32)

for i in range(num_cams):
    R = extrinsics[i, :3, :3]
    t = extrinsics[i, :3, 3]
    K = intrinsics[i]

    P_cam = (R @ verts.T + t[:, None]).T
    Z_c = P_cam[:, 2]

    valid_z = Z_c > 0.05
    u = K[0, 0] * (P_cam[:, 0] / np.maximum(Z_c, 1e-4)) + K[0, 2]
    v = K[1, 1] * (P_cam[:, 1] / np.maximum(Z_c, 1e-4)) + K[1, 2]

    in_frame = valid_z & (u >= 2.0) & (u <= W_inf - 3.0) & (v >= 2.0) & (v <= H_inf - 3.0)

    u_f = np.clip(u, 0, W_inf - 1).astype(np.int32)
    v_f = np.clip(v, 0, H_inf - 1).astype(np.int32)

    # DYNAMIC MASK CHECK
    is_dynamic = car_masks_inf[i, v_f, u_f]

    dist = np.linalg.norm(P_cam, axis=1, keepdims=True) + 1e-6
    rays = P_cam / dist
    N_c = (R @ normals.T).T
    cos_theta = -np.sum(rays * N_c, axis=1)

    # Valid observation must be in frame, NOT dynamic, and facing camera
    valid_obs = in_frame & (~is_dynamic) & (cos_theta > 0.05)
    observation_counts[valid_obs] += 1

    facing = valid_obs & (cos_theta > best_cos)
    if facing.any():
        best_colors[facing] = frame_rgbs[i, v_f[facing], u_f[facing]] / 255.0
        best_cos[facing] = cos_theta[facing]

uncovered = best_cos <= 0.05
print(f"Mask-aware coloring stats:")
print(f"  Total vertices: {N_verts}")
print(f"  Direct unmasked photographic vertices: {(~uncovered).sum()} ({(~uncovered).mean()*100:.2f}%)")
print(f"  Uncovered vertices: {uncovered.sum()}")
print(f"  Mean valid observations per vertex: {observation_counts.mean():.2f}")

# Fallback for uncovered vertices from nearest covered vertices
if uncovered.any():
    covered_idx = np.where(~uncovered)[0]
    pcd_covered = o3d.geometry.PointCloud()
    pcd_covered.points = o3d.utility.Vector3dVector(verts[covered_idx])
    pcd_covered.colors = o3d.utility.Vector3dVector(best_colors[covered_idx])
    kdtree = o3d.geometry.KDTreeFlann(pcd_covered)
    for idx in np.where(uncovered)[0]:
        [k, nn_idx, _] = kdtree.search_knn_vector_3d(verts[idx], 1)
        if k > 0:
            best_colors[idx] = np.asarray(pcd_covered.colors)[nn_idx[0]]

mesh.vertex_colors = o3d.utility.Vector3dVector(best_colors.astype(np.float64))

from scripts.run_sprint11_quality_recovery import render_card_preview

DIAG_DIR = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v2/diagnostics"
DIAG_DIR.mkdir(parents=True, exist_ok=True)
preview_path = DIAG_DIR / "desert_road_mask_aware_preview.png"
render_card_preview(mesh, preview_path, title="Desert Road (Mask-Aware Texture)", orientation_class="corridor_forward")
print(f"[+] Saved preview to {preview_path}")
