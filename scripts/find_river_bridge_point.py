import numpy as np
import open3d as o3d
from pathlib import Path
import cv2

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")

# Predictions for river_bridge
pred_dir = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/predictions"
poses_data = np.load(pred_dir / "camera_poses.npz")
depth_data = np.load(pred_dir / "depth_predictions.npz")

extrinsics = poses_data["extrinsics"]
intrinsics = poses_data["intrinsics"]
d_maps = np.squeeze(depth_data["depth"])

print(f"Num cams: {len(extrinsics)}, depth shape: {d_maps.shape}")

# In frame 7:
# The bridge is in the center-right of frame 7.
# Let's inspect frame 7 image size
im7 = cv2.imread(str(PROJECT_ROOT / "outputs/accuracy_experiment/sprint7_single_window_stabilization/river_bridge/native_frames/frame_0007_47.998s.png"))
H7, W7 = im7.shape[:2]
print(f"Frame 7 shape: {W7}x{H7}")

# Scale intrinsics to native frame 7
sx = W7 / 518.0
sy = H7 / 518.0
K7 = intrinsics[7].copy()
K7[0, 0] *= sx
K7[0, 2] *= sx
K7[1, 1] *= sy
K7[1, 2] *= sy

R7 = extrinsics[7, :3, :3]
t7 = extrinsics[7, :3, 3]
# Camera 7 center in world coordinates:
C7 = -R7.T @ t7
print(f"Cam 7 center: {np.round(C7, 3)}")

# In frame 7, the bridge is roughly at u in [650, 950], v in [500, 650] (if 1280x720) or normalized:
# Let's inspect where pixels in frame 7 map to 3D using depth
d7 = cv2.resize(d_maps[7], (W7, H7), interpolation=cv2.INTER_LINEAR)
# Let's sample a few pixels around the bridge in frame 7:
u_b = int(W7 * 0.68)
v_b = int(H7 * 0.76)
z_b = d7[v_b, u_b]
x_c = (u_b - K7[0, 2]) * z_b / K7[0, 0]
y_c = (v_b - K7[1, 2]) * z_b / K7[1, 1]
P_w = R7.T @ (np.array([x_c, y_c, z_b]) - t7)
print(f"Bridge 3D point in world coords: {np.round(P_w, 3)}")

# Let's also check camera orientation for frame 7:
# Forward ray of camera 7 in world coordinates is R7.T @ [0, 0, 1]
cam7_forward = R7.T @ np.array([0, 0, 1])
cam7_up = R7.T @ np.array([0, -1, 0])
print(f"Cam 7 forward ray in world: {np.round(cam7_forward, 3)}")
print(f"Cam 7 up vector in world: {np.round(cam7_up, 3)}")
