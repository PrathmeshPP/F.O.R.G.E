import cv2
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
frame0_path = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/native_frames/frame_0000_0.000s.png"
im = cv2.imread(str(frame0_path))
H, W = im.shape[:2]

# In frame 0:
# Bounding box around car + balloons:
# x: ~480 to 720
# y: ~370 to 570
crop = im[360:580, 470:730]
cv2.imwrite("outputs/accuracy_experiment/sprint7h_final_defect_recovery/desert_road/stage_audit/veh_crop.png", crop)

# Let's inspect HSV in this region
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
# Car teal body
teal_mask = (hsv[:, :, 0] >= 80) & (hsv[:, :, 0] <= 120) & (hsv[:, :, 1] >= 50) & (hsv[:, :, 2] >= 40)
# White roof / stripes
white_mask = (hsv[:, :, 1] <= 45) & (hsv[:, :, 2] >= 160) & (hsv[:, :, 0] >= 0)
# Balloons: colorful (red, green, purple, yellow, orange)
balloon_mask = ((hsv[:, :, 1] >= 80) & (hsv[:, :, 2] >= 70)) & ~teal_mask
# Cast shadow on right: dark asphalt
shadow_mask = (im[:, :, 0] < 60) & (im[:, :, 1] < 60) & (im[:, :, 2] < 60)

print(f"Teal pixels: {teal_mask.sum()}, White pixels: {white_mask.sum()}")
