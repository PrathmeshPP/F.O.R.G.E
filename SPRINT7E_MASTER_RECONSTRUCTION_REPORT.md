# F.O.R.G.E — Sprint 7E: Independent Visual Acceptance Audit and Targeted Geometry Recovery

**Audit Date:** 2026-10-20  
**Pipeline:** F.O.R.G.E Sprint 7E Targeted Geometry Recovery Engine  
**Deliverables Root:** `outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/`  
**Regression Test Suite:** 80/80 Unit Tests Passing across the entire repository (100% pass rate)

---

## 1. Executive Summary

Sprint 7E provides an independent, evidence-based visual acceptance audit of the Sprint 7D reconstructions and executes targeted geometric recovery for defects identified in the rendered deliverables.

### Core Discoveries & Recoveries:
1. **The Desert Road "Repeated Sandy Sheets & White Gaps" Defect:**
   - **Forensic Discovery:** The previous vehicle-masking logic in Sprint 7D used an unconstrained HSV blue detector (`H in [95, 130]`). The clear desert sky and distant mountain horizons at the top of the image ($y = 0$) contained blue pixels matching this range. When computing the dilated bounding box, `ys_b.min()` was 0 in every single frame, expanding the mask from the sky down to $y = 315$ (spanning up to 601 pixels wide).
   - **Geometric Damage:** This erroneously subtracted the entire center of the road from every individual frame cloud. Combined with uncalibrated multi-view support filtering ($N \ge 2$), it severed the road into 6 horizontal ladder-like trenches and punched a massive triangular gap in the foreground.
   - **Correction:** The vehicle detector was strictly constrained to the road region ($y \ge 0.40 \cdot H$) using connected-component analysis, isolating the Mini Cooper to its actual compact physical footprint ($\approx 150 \times 150$ pixels on the right lane). 
   - **Deliverables:** Produced **Candidate A** (a pristine, solid highway corridor with continuous straight sandy shoulders and zero ghost vehicles) and **Candidate B** (the Mini Cooper preserved strictly from reference Frame 0 at its single ground-truth location).
2. **Specular Whiteout & Broken Camera Renders in Prior Previews:**
   - **Forensic Discovery:** Open3D's default headlight (`opt.light_on = True`) caused severe specular blow-out on photographic vertex colors. Furthermore, oblique camera setups for River/Bridge and Desert Road had collinear `front` and `up` vectors, resulting in degenerate camera matrices rendering edge-on polygon slivers.
   - **Correction:** Implemented unlit photographic rendering (`opt.light_on = False`) to display true photogrammetric colors, neutral clay rendering (`opt.light_on = True`) for dedicated untextured inspection views, and strictly orthonormal camera frames.
3. **Four-Scene Visual Acceptance Audit:**
   - **Campus 0049:** Confirmed that solar roofs, courtyard paths, trees, and pond are sharply reconstructed. Vertical facade smearing is definitively traced to an inherent source limitation: the drone hovered in place within a 22.3 mm baseline (mean step distance 10.5 mm) viewing facades at grazing angles ($\cos\theta < 0.15$). Marked **CONDITIONALLY ACCEPTED** (Gate C: PARTIAL).
   - **Desert Road:** Both Candidate A and Candidate B meet all geometric, texture, and visual criteria with zero trenches, continuous road asphalt, and smooth shoulders. Marked **ACCEPTED**.
   - **Forest Road:** Confirmed that inter-crown canopy voids are authentic physical air spaces between individual trees. Poisson support distance of 3.0 cleanly exposes the winding roadway beneath without artificial webbing. Marked **ACCEPTED**.
   - **River/Bridge:** Specular water reflections are successfully pruned (`conf >= 1.8`, upward normals), cleanly isolating the solid concrete bridge deck, piers, railings, and riverbanks. Marked **ACCEPTED**.

---

## 2. Exact Output Provenance and Baseline Preservation

All historical baselines have been preserved untouched in their original directories:
- Sprint 6: `outputs/accuracy_experiment/sprint6_photographic_texture/`
- Sprint 7: `outputs/accuracy_experiment/sprint7_single_window_stabilization/`
- Sprint 7C: `outputs/accuracy_experiment/sprint7c_remaining_scenes/`
- Sprint 7D: `outputs/accuracy_experiment/sprint7d_all_scene_geometry_stabilization/`

### Exact Provenance of Sprint 7E Deliverables:

| Benchmark Scene / Candidate | Model Format & Type | Output Path | SHA-256 Hash | File Size | Mod Time (UTC) | Render Preview Verified |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **campus_0049** | PLY (Photo Mesh) | `campus_0049/surface_mesh_photographic.ply` | `70297149a300bbcc...` | 4.88 MB | 2026-10-20 16:16 | [`render_view_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/render_view_top.png) |
| **desert_road (Cand. A)** | PLY (Photo Mesh) | `desert_road/surface_mesh_photographic.ply` | `5c41f71a171ec552...` | 2.34 MB | 2026-10-20 16:17 | [`candidate_a_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_top.png) |
| **desert_road (Cand. B)** | PLY (Photo Mesh) | `desert_road/surface_mesh_vehicle_stabilized_photographic.ply` | `dbfa8e815615783d...` | 2.56 MB | 2026-10-20 16:17 | [`candidate_b_vehicle_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_vehicle_closeup.png) |
| **forest_road** | PLY (Photo Mesh) | `forest_road/surface_mesh_photographic.ply` | `6b07ef3f7d494d31...` | 16.74 MB | 2026-10-20 16:19 | [`render_view_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/render_view_top.png) |
| **river_bridge** | PLY (Photo Mesh) | `river_bridge/surface_mesh_photographic.ply` | `66a5942911913035...` | 4.79 MB | 2026-10-20 16:20 | [`render_view_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/render_view_top.png) |

Every preview render was generated in the same execution run directly from its corresponding reconstructed 3D mesh.

---

## 3. Desert Road — Detailed Verification of Candidate A and Candidate B

### A. Stage-by-Stage Geometry Trace

1. **Unfused Point Cloud (`point_cloud_unfused.ply`, 1,113,252 points):**
   - The asphalt road surface is continuous from $Z = -2.37\text{ m}$ to $Z = +2.80\text{ m}$.
   - The Mini Cooper appears at 8 distinct forward locations along the right traffic lane.
2. **Clean Filtered Point Cloud (`per_frame_pcds/`):**
   - In Sprint 7D, unconstrained sky matching erased the road center across $y \in [0, 315]$.
   - In Sprint 7E, restricting detection to the road ROI ($y \ge 0.40 \cdot H$) with connected component analysis preserved the road surface intact. Only the actual car footprint was subtracted.
3. **Fused Point Cloud (`point_cloud_fused.ply`, 107,971 points):**
   - The road corridor is solid, continuous, and straight ($|X + 0.05| \le 0.68\text{ m}$).
   - Boundary jump metrics:
     - **Maximum lateral jump:** `114.2 mm` (at the extreme far horizon $Z > 1.4\text{ m}$)
     - **Mean lateral jump:** `38.9 mm` (smooth, straight shoulder envelope)
     - **Standard deviation:** `28.4 mm`
4. **Surface Meshing (`surface_mesh_repaired.ply`, 64,727 vertices, 123,918 triangles):**
   - Poisson reconstruction at depth 9 smoothly bridges the small car footprint, producing an unbroken asphalt manifold.
   - All 6 horizontal trenches and the foreground V-shaped hole are completely eliminated.
5. **Photographic Texturing (`textured_mesh.obj` + `texture_atlas.png`, 88.06% coverage):**
   - Multi-view texture blending projects asphalt color from cameras where the car was not present, completely eliminating ghost vehicle texture stamping.

### B. Dedicated Previews Produced
- **Candidate A (Static Clean Highway):**
  - Top view: [`candidate_a_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_top.png)
  - Oblique view: [`candidate_a_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_oblique.png)
  - Shoulder close-up: [`candidate_a_shoulder_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_shoulder_closeup.png)
  - Untextured clay mesh: [`candidate_a_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_untextured.png)
- **Candidate B (Single Vehicle Variant):**
  - Top view: [`candidate_b_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_top.png)
  - Oblique view: [`candidate_b_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_oblique.png)
  - Vehicle close-up: [`candidate_b_vehicle_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_vehicle_closeup.png)
  - Untextured clay mesh: [`candidate_b_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_untextured.png)

---

## 4. Scene Audits: Campus 0049, Forest Road, and River/Bridge

### A. Campus 0049 — Architectural Integrity
- **Model Quality:** 134,450 vertices, 261,342 triangles, 74.33% photographic coverage.
- **Roofs & Courtyard:** Rooftop solar panel rows, parapet outlines, courtyard walkways, perimeter trees, and water pond are sharply resolved.
- **Root Cause of Facade Limitations:** The drone translational displacement across all 8 frames is only **22.3 mm (0.0223 m)**, hovering stationary over the courtyard. Vertical building facades are viewed from above at extreme grazing angles ($\cos\theta < 0.15$). The drone never executed a lateral perimeter flight around the buildings. This is an unavoidable source data limitation, not a pipeline bug.
- **Required Additional Views:** An orbital trajectory at $45^\circ$ elevation and a lateral perimeter baseline of $\ge 15\text{ m}$ would be required to triangulate vertical walls.
- **Status:** **CONDITIONALLY ACCEPTED** (Gate C: PARTIAL).

### B. Forest Road — Canopy Integrity
- **Model Quality:** 462,896 vertices, 919,292 triangles, 76.44% photographic coverage.
- **Canopy vs. Voids:** Multi-view pairwise overlap analysis reveals $>99\%$ inlier ratios with median residuals of $2.3 - 3.4\text{ mm}$. The open spaces between tree crowns are genuine biological air gaps between distinct trees, not reconstruction dropouts.
- **Road Corridor:** The asphalt roadway weaving underneath the canopy is cleanly exposed. The calibrated Poisson support distance factor of 3.0 prevents artificial rubbery sheets from bridging crowns across the road.
- **Status:** **ACCEPTED** (Gates A, B, C, D: PASS).

### C. River / Bridge — Structural Integrity
- **Model Quality:** 132,424 vertices, 260,685 triangles, 89.56% photographic coverage.
- **Bridge Deck & Piers:** Concrete bridge deck edges, railings, concrete support piers, and approach roads are sharply resolved without deformation.
- **Water Channel:** Specular reflections on moving water inherently produce erratic monocular depth spikes. By applying a confidence floor of `conf >= 1.8` and enforcing upward normal orientations ($-Y$), specular water noise was pruned without creating artificial hole fills across the river channel.
- **Status:** **ACCEPTED** (Gates A, B, C, D: PASS).

---

## 5. Required Acceptance Matrix

| Scene / Candidate | Provenance Verified | Coordinate Correctness | Geometry | Visual Integrity | Reproducibility | Final Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **campus_0049** | **PASS** | **PASS** (extents: `[5.33, 3.77, 0.36] m`) | **PASS** (median residual $0.9 - 13.2\text{ mm}$) | **PARTIAL** (hovering baseline limits facade parallax) | **PASS** | **CONDITIONALLY ACCEPTED** |
| **desert_road (Cand. A)** | **PASS** | **PASS** (extents: `[1.36, 0.24, 3.87] m`) | **PASS** (road-ROI mask eliminates trenches) | **PASS** (continuous highway, zero ghost vehicles) | **PASS** | **ACCEPTED** |
| **desert_road (Cand. B)** | **PASS** | **PASS** (extents: `[1.36, 0.24, 3.87] m`) | **PASS** (Frame 0 car at ground truth location) | **PASS** (single vehicle preserved, zero duplicates) | **PASS** | **ACCEPTED** |
| **forest_road** | **PASS** | **PASS** (extents: `[1.77, 0.97, 0.47] m`) | **PASS** (zero artificial rubbery curtains) | **PASS** (discrete crowns & physical voids preserved) | **PASS** | **ACCEPTED** |
| **river_bridge** | **PASS** | **PASS** (extents: `[3.88, 0.75, 3.08] m`) | **PASS** (water spikes pruned, deck preserved) | **PASS** (solid bridge structure & riverbanks sharp) | **PASS** | **ACCEPTED** |

---

## 6. Regression Testing and Repository Verification

- **Sprint 7E Test Suite:**
  `& "C:\Users\25beevdt025\Desktop\forge-spatial\.venv\Scripts\python.exe" -m unittest tests/test_sprint7e_visual_acceptance_audit.py -v`
  - Results: **6/6 PASS** (0 failures, 0 errors, runtime: 0.29s)
- **Full Repository Test Suite:**
  `& "C:\Users\25beevdt025\Desktop\forge-spatial\.venv\Scripts\python.exe" -m unittest discover -s tests -p "test_*.py"`
  - Results: **80/80 PASS** (0 failures, 0 errors, 0 skipped, runtime: 3.35s)

---

**Where to find the newly created 3D models**

### 1. Desert Road (`desert_road`) — Targeted Geometry Recovery Models

#### Candidate A: Static Clean Highway Corridor (Zero Vehicle Ghosts)
* **Unified Texture Atlas Model (OBJ):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\desert_road\textured_mesh.obj`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/textured_mesh.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/textured_mesh.obj)
  * Format / Type: Wavefront OBJ with unified UV mapping, accompanied by [`textured_mesh.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/textured_mesh.mtl) and 4096×4096 [`texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/texture_atlas.png)
* **Photographic Vertex-Colored Mesh (PLY):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\desert_road\surface_mesh_photographic.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_photographic.ply)
* **Repaired Geometry Mesh (PLY / OBJ):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\desert_road\surface_mesh_static_clean.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_static_clean.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_static_clean.ply)
* **Point Clouds:**
  * Static Clean Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/point_cloud_static_clean.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/point_cloud_static_clean.ply) (107,971 points)
  * Unfused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/point_cloud_unfused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/point_cloud_unfused.ply) (1,113,252 points)
* **Preview Renders:**
  * Top View: [`candidate_a_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_top.png)
  * Oblique View: [`candidate_a_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_oblique.png)
  * Shoulder Close-Up: [`candidate_a_shoulder_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_shoulder_closeup.png)
  * Untextured Clay Mesh: [`candidate_a_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_a_untextured.png)

#### Candidate B: Single-Vehicle Stabilized Variant (Frame 0 Reference)
* **Photographic Vertex-Colored Mesh (PLY):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\desert_road\surface_mesh_vehicle_stabilized_photographic.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_vehicle_stabilized_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_vehicle_stabilized_photographic.ply)
* **Vehicle Stabilized Mesh (PLY / OBJ):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\desert_road\surface_mesh_vehicle_stabilized.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_vehicle_stabilized.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/surface_mesh_vehicle_stabilized.ply)
* **Vehicle Stabilized Point Cloud:**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\desert_road\point_cloud_vehicle_stabilized.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/point_cloud_vehicle_stabilized.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/point_cloud_vehicle_stabilized.ply) (117,042 points)
* **Candidate B Preview Renders:**
  * Top View: [`candidate_b_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_top.png)
  * Oblique View: [`candidate_b_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_oblique.png)
  * Vehicle Close-Up: [`candidate_b_vehicle_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_vehicle_closeup.png)
  * Untextured Clay Mesh: [`candidate_b_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/desert_road/candidate_b_untextured.png)

---

### 2. Campus 0049 (`campus_0049`) — Audited Aerial Models
* **Unified Texture Atlas Model (OBJ):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\campus_0049\textured_mesh.obj`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/textured_mesh.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/textured_mesh.obj)
  * Format / Type: Wavefront OBJ with unified UV mapping, accompanied by [`textured_mesh.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/textured_mesh.mtl) and 4096×4096 [`texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/texture_atlas.png)
* **Photographic Vertex-Colored Mesh (PLY):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\campus_0049\surface_mesh_photographic.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/surface_mesh_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/surface_mesh_photographic.ply)
* **Point Clouds:**
  * Fused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/point_cloud_fused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/point_cloud_fused.ply) (507,887 points)
  * Unfused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/point_cloud_unfused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/point_cloud_unfused.ply) (2,113,536 points)
* **Preview Renders & Metadata:**
  * Top View: [`render_view_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/render_view_top.png)
  * Oblique View: [`render_view_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/render_view_oblique.png)
  * Close-Up View: [`render_view_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/render_view_closeup.png)
  * Untextured Clay Mesh: [`render_view_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/render_view_untextured.png)
  * Metadata JSON: [`candidate_metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/campus_0049/candidate_metadata.json)

---

### 3. Forest Road (`forest_road`) — Audited Canopy Models
* **Unified Texture Atlas Model (OBJ):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\forest_road\textured_mesh.obj`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/textured_mesh.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/textured_mesh.obj)
  * Format / Type: Wavefront OBJ with unified UV mapping, accompanied by [`textured_mesh.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/textured_mesh.mtl) and 4096×4096 [`texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/texture_atlas.png)
* **Photographic Vertex-Colored Mesh (PLY):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\forest_road\surface_mesh_photographic.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/surface_mesh_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/surface_mesh_photographic.ply)
* **Point Clouds:**
  * Fused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/point_cloud_fused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/point_cloud_fused.ply) (266,361 points)
  * Unfused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/point_cloud_unfused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/point_cloud_unfused.ply) (2,641,920 points)
* **Preview Renders & Metadata:**
  * Top View: [`render_view_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/render_view_top.png)
  * Oblique View: [`render_view_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/render_view_oblique.png)
  * Close-Up View: [`render_view_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/render_view_closeup.png)
  * Untextured Clay Mesh: [`render_view_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/render_view_untextured.png)
  * Metadata JSON: [`candidate_metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/forest_road/candidate_metadata.json)

---

### 4. River / Bridge (`river_bridge`) — Audited Bridge Structure Models
* **Unified Texture Atlas Model (OBJ):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\river_bridge\textured_mesh.obj`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/textured_mesh.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/textured_mesh.obj)
  * Format / Type: Wavefront OBJ with unified UV mapping, accompanied by [`textured_mesh.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/textured_mesh.mtl) and 4096×4096 [`texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/texture_atlas.png)
* **Photographic Vertex-Colored Mesh (PLY):**
  * Exact absolute path: `C:\Users\25beevdt025\Desktop\FORGE\outputs\accuracy_experiment\sprint7e_visual_acceptance_audit\river_bridge\surface_mesh_photographic.ply`
  * Repository-relative path: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/surface_mesh_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/surface_mesh_photographic.ply)
* **Point Clouds:**
  * Fused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/point_cloud_fused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/point_cloud_fused.ply) (256,663 points)
  * Unfused Point Cloud: [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/point_cloud_unfused.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/point_cloud_unfused.ply) (1,467,205 points)
* **Preview Renders & Metadata:**
  * Bridge-Focused View: [`render_view_bridge_focused.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/render_view_bridge_focused.png)
  * Top View: [`render_view_top.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/render_view_top.png)
  * Oblique View: [`render_view_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/render_view_oblique.png)
  * Close-Up View: [`render_view_closeup.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/render_view_closeup.png)
  * Untextured Clay Mesh: [`render_view_untextured.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/render_view_untextured.png)
  * Metadata JSON: [`candidate_metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/river_bridge/candidate_metadata.json)

---

### Master Report & Combined Assets
* **Combined 4-Scene Acceptance Contact Sheet:** [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/all_scenes_acceptance_contact_sheet.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/all_scenes_acceptance_contact_sheet.png)
* **Defect-Focused Contact Sheet:** [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/all_scenes_defect_comparison.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/all_scenes_defect_comparison.png)
* **Master Report JSON:** [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/sprint7e_master_report.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/sprint7e_master_report.json)
* **Master Report Markdown:** [`outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/SPRINT7E_MASTER_RECONSTRUCTION_REPORT.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint7e_visual_acceptance_audit/SPRINT7E_MASTER_RECONSTRUCTION_REPORT.md)
