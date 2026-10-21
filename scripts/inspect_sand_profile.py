import sys
import numpy as np
import open3d as o3d
from pathlib import Path


PROJECT_ROOT = Path("c:/Users/25beevdt025/Desktop/FORGE")
p_7g_a = PROJECT_ROOT / "outputs/accuracy_experiment/sprint7g_four_scene_recovery/desert_road/candidate_a"

m = o3d.io.read_triangle_mesh(str(p_7g_a / "surface_mesh_photographic.ply"))
v = np.asarray(m.vertices)
c = np.asarray(m.vertex_colors)

# Left sand: X between -0.6 and -0.2
left_sand = (v[:, 0] >= -0.6) & (v[:, 0] <= -0.2)
v_sand = v[left_sand]
c_sand = c[left_sand]

# Sort by Z
order = np.argsort(v_sand[:, 2])
v_sand = v_sand[order]
c_sand = c_sand[order]

# Compute luminance
lum = 0.299 * c_sand[:, 0] + 0.587 * c_sand[:, 1] + 0.114 * c_sand[:, 2]

# Bin by Z in 50 bins
z_bins = np.linspace(v_sand[:, 2].min(), v_sand[:, 2].max(), 51)
lum_means = []
z_centers = []
for i in range(50):
    mask = (v_sand[:, 2] >= z_bins[i]) & (v_sand[:, 2] < z_bins[i+1])
    if mask.sum() > 5:
        lum_means.append(lum[mask].mean())
        z_centers.append((z_bins[i] + z_bins[i+1]) / 2.0)

for z, l in zip(z_centers, lum_means):
    print(f"Z={z:+.3f}: Sand Luminance = {l:.4f}")
