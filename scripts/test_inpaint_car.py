import cv2
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
native_dir = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/native_frames"
native_paths = sorted(list(native_dir.glob("frame_*.png")))
frames = [cv2.imread(str(p)) for p in native_paths]
H_nat, W_nat = frames[0].shape[:2]

# Test inpainting car in frame 0
im0 = frames[0].copy()
hsv0 = cv2.cvtColor(im0, cv2.COLOR_BGR2HSV)
teal = (hsv0[:, :, 0] >= 85) & (hsv0[:, :, 0] <= 115) & (hsv0[:, :, 1] >= 60) & (hsv0[:, :, 2] >= 50)
n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(teal.astype(np.uint8))
best = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
x, y, w, h, area = stats[best]
car_mask = np.zeros((H_nat, W_nat), dtype=np.uint8)
car_mask[max(0, y - 180):min(H_nat - 1, y + h + 50), max(0, x - 45):min(W_nat - 1, x + w + 85)] = 255

inp = cv2.inpaint(im0, car_mask, 15, cv2.INPAINT_TELEA)
cv2.imwrite("scratch_inpainted_frame0.png", inp)
print("Saved scratch_inpainted_frame0.png")
