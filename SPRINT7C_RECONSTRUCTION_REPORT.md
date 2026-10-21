# F.O.R.G.E — Sprint 7C: Complete Reconstruction Recovery Report for All Remaining Benchmark Scenes

**Execution Date:** 2026-10-20  
**Pipeline:** F.O.R.G.E Single-Window 3D Reconstruction Engine (Sprint 7C Stabilized)  
**Deliverables Directory:** `outputs/accuracy_experiment/sprint7c_remaining_scenes/`  
**Test Suite Status:** 66/66 Unit Tests Passing (100% Pass Rate)

---

## Executive Summary

Sprint 7C achieves full, end-to-end 3D reconstruction recovery for the three remaining benchmark scenes in the F.O.R.G.E suite:
1. `campus_0049` (Aerial campus survey with academic buildings, courtyards, solar arrays)
2. `forest_road` (Aerial forest canopy survey with coniferous/deciduous crowns and asphalt road)
3. `river_bridge` (Forward corridor survey of concrete bridge crossing, riverbanks, and water channel)

In accordance with explicit project directives, the previously reconstructed `desert_road` dataset has been kept completely intact under `outputs/accuracy_experiment/sprint7_single_window_stabilization/desert_road/`, and its visual acceptance status remains explicitly marked as **`NOT ACCEPTED / FAILED`** due to unresolved fan-shaped stepped shoulder geometry.

Every newly processed scene produced genuine, verified 3D assets on disk—including raw Poisson meshes, repaired topological meshes, multi-material Wavefront OBJs, unified 4096×4096 UV texture atlases, photographic vertex-colored PLYs, multi-angle perspective renders, diagnostic sheets, pairwise registration metrics, and machine-readable metadata.

---

## Combined 4-Scene Master Benchmark Summary

| Scene Name | Ingested Frames & Native Res | Fused Points | Vertices / Triangles | Photographic Coverage (%) | SHA256 Geometry Hash (Repaired Mesh) | Acceptance Gates (A / B / C / D) | Final Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **campus_0049** | 8 frames (3840×2160) | 507,887 | 134,451 / 261,364 | 74.31% | `4296eff65699bb0536...` | PASS / PASS / **PARTIAL** / PASS | **CONDITIONALLY ACCEPTED** |
| **desert_road** | 12 frames (1920×1080) | 288,541 | 131,090 / 262,176 | 76.51% | `1ef91cbdbb41b490d1...` | PASS / PARTIAL / **FAIL** / PASS | **NOT ACCEPTED** |
| **forest_road** | 10 frames (1280×720) | 266,361 | 436,940 / 868,924 | 80.38% | `addb84eeb2316fbfbc...` | PASS / PASS / **PASS** / PASS | **ACCEPTED** |
| **river_bridge**| 8 frames (3840×2160) | 256,663 | 132,648 / 261,178 | 88.78% | `d323187b6c044776b7...` | PASS / PASS / **PASS** / PASS | **ACCEPTED** |

Combined visual preview contact sheet:  
`outputs/accuracy_experiment/sprint7c_remaining_scenes/all_scenes_comparison_contact_sheet.png`

---

## Detailed Scene Reconstruction Analyses

### 1. Scene 1: Campus 0049

#### A. Source Video Provenance & Viewpoint Geometry
- **Source Video:** `C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (162.2 MB, 3840×2160 @ 29.97 fps)
- **Temporal Window:** 8 frames across 16.10s (`t = [0.0, 2.302, 4.605, 6.907, 9.193, 11.495, 13.797, 16.099]`)
- **Camera Baseline Forensic Audit:**
  - `total_baseline_m`: **0.0223 m (22.3 mm)**
  - `baseline_extents_xyz_m`: `[0.0216, 0.0223, 0.0185] m`
  - `mean_step_distance_m`: `0.0105 m (10.5 mm)`
  - *Key Finding:* The drone was essentially hovering in stationary position in the air with negligible lateral translation ($< 2.3\text{ cm}$). Frames 0..3 were acquired at near-vertical nadir ($< 1^\circ$ gimbal tilt), while frames 4..7 engaged camera gimbal panning down to $30^\circ - 45^\circ$.

#### B. Geometric Root Cause: Facade Smearing vs. Source Limitations
- **Root Cause Discovered:**
  1. *Source Limitation:* Because the drone hovered in place without translating laterally around the building perimeters, vertical building facades are viewed at extreme grazing incidence angles ($\cos\theta < 0.15$) from above. True photogrammetric depth triangulation of vertical walls requires lateral baseline displacement.
  2. *Pipeline Remedy:* Depth jump filtering threshold $rtol = 0.12$ cleanly isolates vertical roof-to-courtyard step discontinuities without eroding legitimate perimeter edges. Poisson depth 9 with support distance factor 4.2 preserves sharp roof boundaries, courtyard paths, and rooftop solar installations without bridging empty courtyard space.
  3. *Photographic Projection:* Multi-view texture blending incorporates edge-feathered cosine weighting (`feather_px = 180.0`) and rejects extreme grazing rays ($\cos\theta < 0.08$), preventing high-angle roof pixels from stretching down vertical walls.

#### C. Quantitative Registration Diagnostics
- Overlap point counts: 126,800 to 261,508 points per adjacent frame pair.
- Inlier ratios ($< 5\text{ cm}$): 71.5% to 100.0% (mean 84.7%).
- Median residuals: 0.9 mm to 13.2 mm; Inlier RMSE: 1.4 mm to 15.3 mm.
- Final Mesh Complexity: 134,451 vertices, 261,364 triangles.
- Photographic Coverage: 74.31% (194,219 faces textured).

---

### 2. Scene 2: Forest Road

#### A. Source Video Provenance & Trajectory
- **Source Video:** `C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\forest_road.mp4` (10.6 MB, 1280×720 @ 29.97 fps)
- **Temporal Window:** 10 frames across 30.41s (`t = [0.0, 3.378, 6.757, 10.135, 13.513, 16.892, 20.270, 23.649, 27.027, 30.405]`)
- **Camera Baseline Forensic Audit:**
  - `total_baseline_m`: **0.3423 m (34.2 cm)**
  - `baseline_extents_xyz_m`: `[0.2206, 0.0976, 0.3423] m`
  - `mean_step_distance_m`: `0.0489 m (4.89 cm)`
  - *Key Finding:* Stable linear flight corridor above the tree line with consistent forward progression along the $Z$ axis.

#### B. Geometric Root Cause: Canopy Voids vs. Reconstruction Artifacts
- **Root Cause Discovered:**
  1. *Authentic Physical Voids:* The gaps between tree crowns are real, open-air spatial voids between individual biological trees, not photogrammetric dropouts or missing data.
  2. *Defect in Baseline Meshing:* Prior naive meshing algorithms attempted to enforce watertight manifold surfaces, generating artificial, rubbery "curtains" and sheets of polygons bridging the open air between distinct crowns.
  3. *Pipeline Remedy:* We calibrated Poisson reconstruction with `support_distance_factor = 3.0` and `gentle_trim_quantile = 0.008`. In addition, multi-view consistency filtering (`min_cam_support = 2`) pruned isolated sky noise. The resulting mesh accurately represents discrete tree crown envelopes and cleanly exposes the asphalt roadway beneath without artificial webbing.

#### C. Quantitative Registration Diagnostics
- Overlap point counts: 254,147 to 257,871 points per adjacent pair.
- Inlier ratios ($< 5\text{ cm}$): **99.0% to 99.9%** across all pairs.
- Median residuals: **2.3 mm to 3.4 mm**; Inlier RMSE: 6.6 mm to 9.2 mm.
- Final Mesh Complexity: 436,940 vertices, 868,924 triangles.
- Photographic Coverage: **80.38%** (698,445 faces textured).

---

### 3. Scene 3: River / Bridge

#### A. Source Video Provenance & Flight Path
- **Source Video:** `C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4` (737.3 MB, 3840×2160 @ 29.97 fps)
- **Temporal Window:** 8 frames across 48.0s (`t = [17.985, 22.272, 26.560, 30.847, 35.135, 39.423, 43.710, 47.998]`)
- **Camera Baseline Forensic Audit:**
  - `total_baseline_m`: **0.5185 m (51.85 cm)**
  - `baseline_extents_xyz_m`: `[0.2016, 0.1252, 0.5185] m`
  - `mean_step_distance_m`: `0.0885 m (8.85 cm)`
  - *Key Finding:* Forward diagonal flight approaching and crossing the concrete bridge structure over the riverbed corridor.

#### B. Geometric Root Cause: Reflective Water Spikes vs. Solid Structure
- **Root Cause Discovered:**
  1. *Reflective Water Instability:* Dynamic specular reflections on moving water cause depth model ambiguity, yielding low confidence estimates ($c \approx 1.0 - 1.2$). When unconstrained, these points manifest as vertical spikes and jagged noise in the riverbed.
  2. *Solid Structure Stability:* Solid concrete bridge deck surfaces, guard rails, support piers, and rocky riverbanks exhibit high confidence ($c \ge 1.8 - 33.0$).
  3. *Pipeline Remedy:* We enforced a strict confidence threshold `conf_min = 1.8` combined with depth jump filtering `rtol = 0.08`. Furthermore, ground/deck normals were constrained to point upward ($-Y$ in OpenCV camera convention) to prevent normal inversion. This pruned water spikes while retaining sharp deck geometry, railings, and embankment terrain.

#### C. Quantitative Registration Diagnostics
- Overlap point counts: 80,489 to 171,558 points per adjacent pair.
- Inlier ratios ($< 5\text{ cm}$): 76.2% to 100.0% (Pairs F2-F3: 99.8%, F3-F4: 100.0%).
- Median residuals: 1.3 mm to 4.4 mm; Inlier RMSE: 2.9 mm to 11.7 mm.
- Final Mesh Complexity: 132,648 vertices, 261,178 triangles.
- Photographic Coverage: **88.78%** (231,880 faces textured).

---

### 4. Scene 4: Desert Road (Retained & Evaluated)

#### Status: NOT ACCEPTED (VISUAL ACCEPTANCE GATE FAILED)
- **Retained Deliverables Path:** `outputs/accuracy_experiment/sprint7_single_window_stabilization/desert_road/`
- **Reference Candidate:** `surface_mesh_photographic.ply` (SHA256: `1ef91cbdbb41b490d1e5...`)
- **Forensic Assessment:**
  - As established in Sprint 7B, the render of Desert Road exhibits repeating, fan-shaped stepped sandy shoulder surfaces along the highway corridor.
  - While corridor trimming narrowed the spatial bounds, it did not resolve the fundamental multi-baseline disparity in low-texture desert terrain where depth prediction drift creates stacked planar surfaces.
  - This scene is explicitly marked as **`NOT ACCEPTED / FAILED`** in `sprint7c_master_report.json` and in the combined contact sheet. No false claims of geometric acceptance are made.

---

## Acceptance Gate Evaluations

### Gate A: Coordinate Correctness
- **Criteria:** Valid camera transformations, dimension extents, intrinsic scaling (native 4K/720p to inference 688×384), and depth-to-point correspondence.
- **Evaluation:**
  - `campus_0049`: **PASS** (extents: `[5.33, 3.77, 0.36] m`, correct nadir-to-oblique rotation)
  - `forest_road`: **PASS** (extents: `[1.77, 0.97, 0.47] m`, forward Z flight progression)
  - `river_bridge`: **PASS** (extents: `[3.88, 0.75, 3.08] m`, correct bridge orientation)
  - `desert_road`: **PASS** (extents: `[2.22, 0.63, 2.76] m`)

### Gate B: Geometric Consistency
- **Criteria:** No unexplained duplicate surfaces, gross registration failures, or unsupported geometry dominating the scene.
- **Evaluation:**
  - `campus_0049`: **PASS** (Pairwise median residual $0.9 - 13.2\text{ mm}$, zero mesh self-intersections)
  - `forest_road`: **PASS** (Pairwise median residual $2.3 - 3.4\text{ mm}$, inlier ratio $>99\%$, zero artificial curtains)
  - `river_bridge`: **PASS** (Pairwise median residual $1.3 - 4.4\text{ mm}$, water spikes pruned, solid deck preserved)
  - `desert_road`: **PARTIAL** (Stepped shoulder layers remain present)

### Gate C: Visual Integrity
- **Criteria:** Major structures recognizable, boundaries preserved, voids distinguished from defects, and no deceptive rendering tricks.
- **Evaluation:**
  - `campus_0049`: **PARTIAL** (Rooftops, solar arrays, walkways, and courtyards clearly reconstructed; vertical facades limited by hovering drone baseline)
  - `forest_road`: **PASS** (Distinct tree crowns preserved, natural open-air voids unwebbed, road clearly visible)
  - `river_bridge`: **PASS** (Solid bridge deck, railings, piers, and riverbanks sharply resolved)
  - `desert_road`: **FAIL** (Fan-shaped stepped shoulder artifacts remain visible)

### Gate D: Reproducibility
- **Criteria:** Full provenance recorded, deterministic outputs, geometry SHA256 hashes generated, test suite passing.
- **Evaluation:**
  - All scenes: **PASS** (Full execution logs, metadata JSONs, diagnostics JSONs, and 66 passing unit tests)

---

## Verification and Unit Test Summary

A dedicated test suite was implemented and executed:
`tests/test_sprint7c_remaining_scenes.py`

Test Results:
```
Ran 7 tests in 0.524s
OK
```

Full repository test suite execution:
```
Ran 66 tests in 2.252s
OK (All 66 passing, 0 failures, 0 errors)
```
- Tested deliverable completeness across all 3 scene directories.
- Verified Open3D mesh loading, vertex/triangle bounds, non-degeneracy.
- Verified 4096×4096 texture atlas integrity.
- Verified multi-angle preview renders (top, oblique, closeup, bridge-focused).
- Verified combined comparison sheet and master JSON report validity.
