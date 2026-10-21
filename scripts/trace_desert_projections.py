import sys
import numpy as np
import cv2
from pathlib import Path
from PIL import Image

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
sys.path.insert(0, str(PROJECT_ROOT))
from src.reconstruction.photographic_texture import extract_native_frames, scale_camera_intrinsics

pred_dir = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions"
native_dir = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/native_frames"

poses_data = np.load(pred_dir / "camera_poses.npz")
extrinsics = poses_data["extrinsics"]
intrinsics = poses_data["intrinsics"]
scaled_intrinsics = scale_camera_intrinsics(intrinsics, (518, 518), (1280, 720))

# Check camera positions
for i in range(8):
    R = extrinsics[i, :3, :3]
    t = extrinsics[i, :3, 3]
    # Camera center in world coords: C = -R^T @ t
    C = -R.T @ t
    print(f"Camera {i}: C = {np.round(C, 3)}, t = {np.round(t, 3)}")

# Check projection of right sand points at Z = -2.0, -1.5, -1.0, -0.5, 0.0, 0.5
test_pts = [
    ("Z=-2.0, X=+0.3", np.array([0.3, 0.18, -2.0])),
    ("Z=-1.5, X=+0.3", np.array([0.3, 0.16, -1.5])),
    ("Z=-1.0, X=+0.3", np.array([0.3, 0.14, -1.0])),
    ("Z=-0.5, X=+0.3", np.array([0.3, 0.11, -0.5])),
    ("Z=0.0,  X=+0.3", np.array([0.3, 0.08, 0.0])),
    ("Z=0.5,  X=+0.3", np.array([0.3, 0.05, 0.5])),
]

print("\n--- Projecting right sand points into cameras ---")
for label, pt in test_pts:
    print(f"\nPoint {label}:")
    for i in range(8):
        R = extrinsics[i, :3, :3]
        t = extrinsics[i, :3, 3]
        K = scaled_intrinsics[i]
        P_c = R @ pt + t
        z_c = P_c[2]
        if z_c > 0.05:
            u = K[0, 0] * (P_c[0] / z_c) + K[0, 2]
            v = K[1, 1] * (P_c[1] / z_c) + K[1, 2]
            in_view = (0 <= u < 1280) and (0 <= v < 720)
            if in_view:
                print(f"  Cam {i}: P_c = {np.round(P_c, 3)}, (u,v) = ({u:.1f}, {v:.1f}) IN_VIEW")
            else:
                print(f"  Cam {i}: P_c = {np.round(P_c, 3)}, (u,v) = ({u:.1f}, {v:.1f}) out-of-bounds")
        else:
            print(f"  Cam {i}: z_c = {z_c:.3f} BEHIND_CAM")
