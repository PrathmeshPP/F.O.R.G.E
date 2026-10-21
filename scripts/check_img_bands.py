import cv2
import numpy as np

img = cv2.imread("outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/candidate_a/candidate_a_top_verified.png")
# The road is vertically oriented in the center.
# Let's inspect a column in the left sand shoulder: x = 360, y from 150 to 450
col = img[150:450, 360] # BGR
lum = 0.299 * col[:, 2] + 0.587 * col[:, 1] + 0.114 * col[:, 0]

print("Y, BGR, Lum:")
for y_idx in range(0, len(lum), 15):
    y = 150 + y_idx
    b, g, r = col[y_idx]
    print(f"y={y}: BGR=({b},{g},{r}), Lum={lum[y_idx]:.1f}")
