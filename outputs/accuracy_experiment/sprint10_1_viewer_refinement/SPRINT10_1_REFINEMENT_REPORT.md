# F.O.R.G.E — Sprint 10.1: Viewer Presentation, Camera Framing, and Metadata Consistency Report

**Sprint Phase:** Sprint 10.1 Viewer Refinement & National Hackathon Presentation  
**Baseline Model Freeze:** `outputs/accuracy_experiment/sprint9_video_to_viewer_release` (**100% Frozen & Unmodified**)  
**Application Platform:** F.O.R.G.E Explorer (`viewer/`) built with React 19, TypeScript 5.8, Three.js r182, and Vite 8  
**Verification Date:** 2026-10-20 21:18:58 UTC  
**Verification Result:** **ALL ACCEPTANCE CRITERIA MET (100% PASS)**

---

## 1. Executive Summary

Sprint 10.1 focused exclusively on **viewer presentation, metadata consistency, camera framing, and demonstration usability** for a national-level hackathon presentation. All Sprint 9 3D reconstruction model assets remained completely frozen in their verified release directories: **no depth re-inference, no mesh re-triangulation, no vertex modification, and no texture regeneration were performed.**

### Core Deliverables Achieved:
1. **Geometry-Count Discrepancy Resolved:** Unified metrics across the viewport overlay HUD and inspection panel into unambiguous, standardized categories:
   - **Unique Geometry Vertices:** Topological vertex count in the source mesh file ($V$).
   - **Rendered Position Entries:** Active Three.js WebGL GPU position-buffer elements ($N_{\text{pos}} = 3 \times F$ for non-indexed OBJ; $V$ for indexed PLY).
   - **Triangle Count:** Actual rendered planar triangles ($F$).
   - **Source Asset Filename:** Exact asset file actively bound in WebGL memory (`textured_mesh.obj` vs `surface_mesh_*.ply`).
   - Standardized component [`GeometryStatsDisplay.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/GeometryStatsDisplay.tsx) deployed across all views with explanatory technical notes.
2. **Dynamic Camera Framing & Fit-to-View:**
   - Implemented viewer-only translation to center each mesh at $(0, 0, 0)$ without altering raw vertex coordinates.
   - Calculated dynamic bounding spheres and view frustum distances based on viewport aspect ratio and field of view ($45^\circ$).
   - Dynamic near/far clipping plane calculation ($D/150$ to $D \times 35$) preventing near-plane clipping and z-fighting.
3. **Desert Road Candidate A vs Candidate B Demonstration:**
   - Matched initial corridor framing at $[0.0, 3.2, 1.8] \to [0.0, 0.0, -0.4]$.
   - Synchronized camera control toggle with live state propagation.
   - Persistent `CANDIDATE A` and `CANDIDATE B` status badges.
   - Dedicated **Focus Vehicle** action translating the camera directly to $[0.33, 0.28, 0.98] \to [0.084, -0.053, 0.536]$ (framing only; model geometry frozen).
   - Collapsible close-up inset overlay showing verified reference Mini Cooper volume at $[0.03, 0.04, 0.45]$ with planar asphalt continuity.
   - **Full Road** return action restoring corridor overview.
4. **Scene-Specific Inspection Presets:**
   - **Campus 0049:** Overview ($[0, 4.2, 3.2] \to [0, 0, 0]$) and dedicated **Facade View** ($[0.55, 0.75, 0.95] \to [0.013, -0.079, 0.008]$), preserving honest drone hovering telemetry limits.
   - **Forest Road:** Canopy overview ($[0, 2.5, 1.6] \to [0, 0, 0]$) and dedicated **Roadway View** ($[0.021, 0.92, 0.82] \to [0.021, 0.089, -0.035]$) inspecting double yellow lane lines and asphalt continuity.
   - **River/Bridge:** Dedicated **Bridge Structure** view ($[1.27, 1.13, 0.52] \to [0.673, 0.232, -0.681]$) framing the bridge deck, railings, and piers, plus valley overview.
5. **Rigorous Verification:**
   - 6/6 regression tests passing in [`tests/test_sprint10_1_viewer_refinement.py`](file:///c:/Users/25beevdt025/Desktop/FORGE/tests/test_sprint10_1_viewer_refinement.py).
   - 4/4 demo hardening tests passing in [`tests/test_sprint10_demo_hardening.py`](file:///c:/Users/25beevdt025/Desktop/FORGE/tests/test_sprint10_demo_hardening.py).
   - Production Vite frontend build passing in 413ms.
   - 7 fresh high-resolution Edge browser application screenshots captured live.

---

## 2. Geometry-Count Semantics & Resolution

### Root Cause Analysis
During Sprint 10 review, an apparent discrepancy was observed between the Viewport overlay ("Vertices: 316,401" for Desert Cand A) and the Inspection Panel ("Vertices: 53,069").

Our forensic investigation traced both values:
- The source mesh file `textured_mesh.obj` contains **53,069 unique vertex coordinates** (`v ...`) and **105,467 polygonal faces** (`f ...`).
- When Three.js `OBJLoader` loads a Wavefront OBJ mesh, it constructs an **unindexed `BufferGeometry`**, where each triangle face is unpacked into three independent GPU vertex positions:
  $$\text{Rendered Position Entries} = 105,467 \times 3 = 316,401$$
- Conversely, `PLYLoader` constructs an **indexed `BufferGeometry`**, where `geometry.attributes.position.count` matches the source mesh's unique vertex count ($53,069$).
- Previously, the Viewport overlay displayed `geom.attributes.position.count` labeled generically as "Vertices", while the Inspector displayed `scene.vertex_count` (unique vertices). Both values were technically correct, but described different layers of the graphics stack.

### Explicit Semantics Implemented
We introduced a unified, standardized component [`GeometryStatsDisplay.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/GeometryStatsDisplay.tsx) used identically in both the Viewport HUD and the Inspection Panel:

| Metric Name | Semantics | OBJ Representation | PLY Representation |
| :--- | :--- | :--- | :--- |
| **Source Asset Filename** | Active file loaded in WebGL memory | `textured_mesh.obj` | `surface_mesh_*.ply` |
| **Unique Geometry Vertices** | Topological vertex count in source file | $V$ (e.g. 53,069) | $V$ (e.g. 53,069) |
| **Rendered Position Entries** | WebGL GPU position buffer allocation | $3 \times F$ (e.g. 316,401) | $V$ (e.g. 53,069) |
| **Triangle Count** | Verified planar triangle primitives | $F$ (e.g. 105,467) | $F$ (e.g. 105,467) |
| **Buffer Structure** | GPU buffer index layout | Non-Indexed (3 pos/tri) | Indexed Unique Array |

Where metrics cannot be obtained or are in transit, `Unavailable` is displayed rather than inventing values.

---

## 3. Cryptographic Asset Audit Across All Representations

Every representation across all five scenes was verified against disk byte counts and SHA-256 digests:

| Scene / Candidate | Representation | Source File | File Size (Bytes) | Verified SHA-256 Digest | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Campus 0049** | Textured OBJ | `textured_mesh.obj` | 31,281,376 | `7278af1cbeb04ce8c31d83db50b09d59ea9516903f62f4561b3d05bbd62daf82` | **PASS** |
| | Photo PLY | `surface_mesh_photographic.ply` | 9,840,793 | `143a0e9d21b543ec9863dfcede67b0818b277f70d2c2d3d0387014d01533308f` | **PASS** |
| | Untextured PLY | `surface_mesh_repaired.ply` | 9,840,793 | `adb753a0a5d7201504ed9ff493a5174cb879cc230182b8a13b3905563985fceb` | **PASS** |
| **Desert Road A** | Textured OBJ | `textured_mesh.obj` | 14,301,214 | `6d91b36b94afbda4fef3aa7e25ae1ac55b0ad5fc1807e061e8693468bb24d433` | **PASS** |
| | Photo PLY | `surface_mesh_photographic.ply` | 4,077,915 | `5625a5a2436abd6bca780b86efde8687246675f0e47b270f37440b006099ae44` | **PASS** |
| | Untextured PLY | `surface_mesh_static_clean.ply` | 4,077,915 | `b442150ca2d5a4b1d196c8f83d9f36b2aa330a7b815b708c9a1a17f177259a11` | **PASS** |
| **Desert Road B** | Textured OBJ | `textured_mesh.obj` | 14,308,562 | `ee0fdd35524fff4357c048c29a4454fb982a7283ac1183f8d6ad863822f3a0f4` | **PASS** |
| | Photo PLY | `surface_mesh_vehicle_stabilized_photographic.ply` | 4,080,210 | `674ad0ec3d0b40a8fdb9f0680301912a0d7e3a0552c4ef3526740e285f053edf` | **PASS** |
| | Untextured PLY | `surface_mesh_vehicle_stabilized.ply` | 4,080,210 | `3088a560227c9b821b289ac7c4ec3a23ccacda3d6d056c84d7b56a955828f185` | **PASS** |
| **Forest Road** | Textured OBJ | `textured_mesh.obj` | 117,409,995 | `f05f497181361cf4ecb98926d843f8a178c4b48d8c662ab8604e2f779fa2d86b` | **PASS** |
| | Photo PLY | `surface_mesh_photographic.ply` | 35,558,818 | `1cb5896612a8b1d040b35553ea2000df97d7a3311c59e4e2f42afcdd6c63bfc0` | **PASS** |
| | Untextured PLY | `surface_mesh_repaired.ply` | 35,558,818 | `1d635dce8c9d023093d4b43fdeb420be4f366fc6d08aa6c4a391121cf03f875c` | **PASS** |
| **River/Bridge** | Textured OBJ | `textured_mesh.obj` | 32,270,618 | `fb28f5522dbae5d28ee9358d8e8056a1252ef8f56db39ccf2984b3c7c28808a9` | **PASS** |
| | Photo PLY | `surface_mesh_photographic.ply` | 10,142,855 | `4be4958e3b81a0a8ea2ca0ad3c7cb18c39e196884065920c5c11abaad9d7758c` | **PASS** |
| | Untextured PLY | `surface_mesh_repaired.ply` | 10,142,855 | `7d049e9e2e680a6c002b362d4f6e7a501fa01896aedfd3cd82922d0a7f7f9be7` | **PASS** |

Candidate A and Candidate B load distinct assets with verified cryptographic differences reflecting vehicle removal vs vehicle preservation.

---

## 4. Camera Framing & Viewport Geometry Transforms

### Viewer-Only Centering Transform
To allow smooth OrbitControls rotation around the visual centroid of each scene without mutating the raw vertex coordinates, Three.js applies a viewer-only group offset:
$$\vec{x}_{\text{viewer}} = \vec{x}_{\text{raw}} - \vec{c}_{\text{bbox}}$$
Raw vertex coordinates exported via the Asset Download tool retain their unnormalized, unshifted model coordinates.

### Adaptive Bounding Sphere & Near/Far Clipping Planes
For each loaded model:
1. Bounding sphere radius $R$ is extracted from `Box3.getBoundingSphere()`.
2. Given vertical field of view $\theta = 45^\circ$ ($0.785$ rad) and aspect ratio $A = W/H$:
   $$D = \max\left(\frac{R}{\sin(\theta / 2)}, \frac{R}{\sin(\arctan(\tan(\theta / 2) \times A))}\right) \times 1.15$$
3. Camera clipping bounds are dynamically set:
   $$\text{near} = \max(0.01, D / 150), \quad \text{far} = \max(1000, D \times 35)$$
   This eliminates geometry clipping on wide corridor assets while completely avoiding depth buffer z-fighting.

### Scene-Specific Camera Framing Presets

| Scene | Preset Name | Camera Position $[X, Y, Z]$ | LookAt Target $[X, Y, Z]$ | Purpose & Inspection Feature |
| :--- | :--- | :--- | :--- | :--- |
| **Campus 0049** | Overview | $[0.0, 4.2, 3.2]$ | $[0.0, 0.0, 0.0]$ | Aerial overview of grounds, pond, paths, and roofs |
| | Facade View | $[0.55, 0.75, 0.95]$ | $[0.013, -0.079, 0.008]$ | Building facade inspection (observes drone hover parallax limits) |
| **Desert Road A** | Corridor View | $[0.0, 3.2, 1.8]$ | $[0.0, 0.0, -0.4]$ | Continuous static highway corridor overview |
| **Desert Road B** | Full Road | $[0.0, 3.2, 1.8]$ | $[0.0, 0.0, -0.4]$ | Full highway overview with preserved vehicle in corridor context |
| | Focus Vehicle | $[0.33, 0.28, 0.98]$ | $[0.084, -0.053, 0.536]$ | Direct close-up on preserved Mini Cooper (zero road box) |
| **Forest Road** | Canopy View | $[0.0, 2.5, 1.6]$ | $[0.0, 0.0, 0.0]$ | Mountain terrain and dense tree crowns with natural voids |
| | Roadway View | $[0.021, 0.92, 0.82]$ | $[0.021, 0.089, -0.035]$ | Downward corridor framing double yellow markings & asphalt |
| **River/Bridge** | Bridge Structure | $[1.27, 1.13, 0.52]$ | $[0.673, 0.232, -0.681]$ | Structure view centering deck, guard railings, piers & riverbed |
| | Valley View | $[0.0, 4.0, 3.0]$ | $[0.0, 0.0, 0.0]$ | Riverbed channel and highway approach roadway overview |

---

## 5. Desert Road Candidate A vs Candidate B Demonstration Hardening

### Key Comparison Capabilities Implemented:
1. **Matched Framing & Linked Orbit Controls:** In comparison mode, both viewports initialize with identical camera orientation ($[0, 3.2, 1.8] \to [0, 0, -0.4]$). When "Cameras Linked" is active, orbit, pan, and zoom interactions in either viewport synchronize smoothly to the other.
2. **Persistent Identity Badges:** Permanent `CANDIDATE A` and `CANDIDATE B` header pills guarantee judges always know which variant is being inspected.
3. **Focus Vehicle Action:** Clicking `Focus Vehicle` repositions the WebGL camera directly in front of the preserved Mini Cooper. The underlying mesh vertices are completely frozen; only camera framing changes.
4. **Docked Vehicle Inset Overlay:** An interactive inset card on Candidate B provides the verified close-up view alongside technical documentation:
   > "Reference vehicle preserved at `[0.03, 0.04, 0.45]` with planar road integration and zero rectangular box. Camera framing only; model geometry frozen."

---

## 6. Live Application Screenshot Evidence

All screenshots were captured directly from the running production application using Microsoft Edge (Chromium):

| Screenshot | File Path | Resolution | Description & Verified Features |
| :--- | :--- | :--- | :--- |
| **Campus 0049 Overview** | `screenshots/demo_campus_0049_overview.png` | 1440x900 | Complete campus architectural survey with floating HUD and Inspector. |
| **Campus 0049 Facade** | `screenshots/demo_campus_0049_facade.png` | 1440x900 | Dedicated facade inspection view highlighting physical telemetry constraints. |
| **Desert Road A** | `screenshots/demo_desert_candidate_a.png` | 1440x900 | Clean highway corridor with zero vehicles and 30-iteration Taubin smoothed shoulders. |
| **Desert Road B Vehicle Focus** | `screenshots/demo_desert_candidate_b_vehicle_focus.png` | 1440x900 | Close-up framing of preserved Mini Cooper resting naturally on asphalt. |
| **Desert Road Comparison** | `screenshots/demo_desert_comparison.png` | 1440x900 | Side-by-side comparison mode with linked cameras, badges, and vehicle inset card. |
| **Forest Road Roadway** | `screenshots/demo_forest_road_roadway.png` | 1440x900 | Downward corridor view with clear double yellow road markings. |
| **River/Bridge Structure** | `screenshots/demo_river_bridge_structure.png` | 1440x900 | Structural view centering bridge deck, railings, piers, and river channel. |

---

## 7. Automated Test Results

### 1. Sprint 10.1 Regression Suite (`pytest tests/test_sprint10_1_viewer_refinement.py -v`)
- `test_01_manifest_version_and_freeze`: **PASSED** (version 1.1.0, Sprint 10.1, baseline freeze verified)
- `test_02_all_representations_exist_and_match_hashes`: **PASSED** (all 15 representations match disk bytes & SHA-256)
- `test_03_geometry_count_semantics_obj_vs_ply`: **PASSED** (OBJ unique $V$, rendered $3 \times F$, PLY $V$)
- `test_04_desert_road_candidate_a_and_b_provenance`: **PASSED** (distinct hashes, different vertex/face counts)
- `test_05_camera_framing_and_scene_presets`: **PASSED** (all scene camera targets and presets verified)
- `test_06_desert_road_vehicle_focus_coordinates`: **PASSED** (centered vehicle coordinates verified at $[0.084, -0.053, 0.536]$)

**Result:** **6 passed in 4.09s (100% pass rate)**

### 2. Sprint 10 Demo Hardening Suite (`pytest tests/test_sprint10_demo_hardening.py -v`)
- `test_01_launcher_script_and_demo_guide`: **PASSED**
- `test_02_demo_release_deliverables_and_screenshots`: **PASSED**
- `test_03_model_texture_integrity`: **PASSED**
- `test_04_live_viewer_and_asset_endpoints`: **PASSED**

**Result:** **4 passed in 0.04s (100% pass rate)**

---

## 8. Academic Honesty Disclosures

1. **Reconstruction Models Frozen:** No model vertices or textures were modified during Sprint 10.1. All models remain exact outputs of Sprint 9 (`outputs/accuracy_experiment/sprint9_video_to_viewer_release`).
2. **Campus 0049 Facade Limitations:** Drone hover telemetry (< 0.10m translation) provides narrow grazing-angle rays. Facades exhibit oblique stretching because unseen geometry is honestly not fabricated.
3. **Desert Road Candidate B Vehicle:** The vehicle focus action adjusts the camera matrix only; it does not synthesize or highlight geometry.
4. **River/Bridge Water Specularity:** Water surface reflection noise was attenuated during multi-view fusion; bridge deck, railings, piers, and banks reconstruct with verified geometric continuity.
5. **Coordinate Frame:** All coordinates remain model-relative arbitrary units with uncalibrated metric scale.

---

**Where to find the refined demonstration application**

The refined F.O.R.G.E demonstration platform and all verification evidence are available at the following locations:

- **Viewer Source Code:**
  - Standardized geometry statistics component: [`viewer/src/components/GeometryStatsDisplay.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/GeometryStatsDisplay.tsx)
  - 3D WebGL viewport with adaptive framing: [`viewer/src/components/Viewport3D.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/Viewport3D.tsx)
  - Candidate A/B comparison with vehicle focus & inset: [`viewer/src/components/ComparisonView.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/ComparisonView.tsx)
  - Docked toolbar with scene presets: [`viewer/src/components/Toolbar.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/Toolbar.tsx)
  - Inspection and provenance panel: [`viewer/src/components/InspectionPanel.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/components/InspectionPanel.tsx)
  - Root application state: [`viewer/src/App.tsx`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/App.tsx)
- **Active Model Manifest:**
  - Source manifest: [`viewer/src/data/model-manifest.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/src/data/model-manifest.json)
  - Production distribution manifest: [`viewer/dist/model-manifest.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/dist/model-manifest.json)
- **One-Command Launch Script:**
  - Windows batch launcher: [`start_forge_explorer.bat`](file:///c:/Users/25beevdt025/Desktop/FORGE/start_forge_explorer.bat)
  - Python HTTP server: [`scripts/serve_viewer.py`](file:///c:/Users/25beevdt025/Desktop/FORGE/scripts/serve_viewer.py) (running on `http://127.0.0.1:5173/`)
- **Production Build:**
  - Built bundle: [`viewer/dist/`](file:///c:/Users/25beevdt025/Desktop/FORGE/viewer/dist/) (Vite v8.3.4, exit code 0)
- **New Screenshots:**
  - High-resolution Edge browser captures: [`outputs/accuracy_experiment/sprint10_1_viewer_refinement/screenshots/`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint10_1_viewer_refinement/screenshots/)
- **Validation and Integrity Reports:**
  - Metadata and representation integrity: [`outputs/accuracy_experiment/sprint10_1_viewer_refinement/metadata_integrity_report.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint10_1_viewer_refinement/metadata_integrity_report.json)
  - Sprint 10.1 validation report: [`outputs/accuracy_experiment/sprint10_1_viewer_refinement/sprint10_1_validation.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint10_1_viewer_refinement/sprint10_1_validation.json)
  - Refinement report: [`outputs/accuracy_experiment/sprint10_1_viewer_refinement/SPRINT10_1_REFINEMENT_REPORT.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint10_1_viewer_refinement/SPRINT10_1_REFINEMENT_REPORT.md)
  - Repository root copy: [`SPRINT10_1_REFINEMENT_REPORT.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/SPRINT10_1_REFINEMENT_REPORT.md)
- **Live Demonstration Guide:**
  - Five-minute judge presentation sequence: [`docs/SPRINT10_LIVE_DEMO_GUIDE.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/docs/SPRINT10_LIVE_DEMO_GUIDE.md)

### Executed Tests & Browser Interactions
- **Automated Tests Executed:**
  - `pytest tests/test_sprint10_1_viewer_refinement.py -v`: 6 passed in 4.09s
  - `pytest tests/test_sprint10_demo_hardening.py -v`: 4 passed in 0.04s
- **Production Build Executed:**
  - `vite build`: Completed with exit code 0 (1,946 modules transformed in 413ms)
- **Live Browser Interactions Executed (Microsoft Edge):**
  1. Campus 0049 selection, overview framing, and "Facade View" inspection.
  2. Desert Road Candidate A selection and full corridor inspection.
  3. Desert Road Candidate B selection and "Focus Vehicle" close-up framing.
  4. Desert Road Candidate A/B comparison activation, synchronized camera linking, and vehicle inset card verification.
  5. Forest Road selection and "Roadway View" framing verifying double yellow lane markings.
  6. River/Bridge selection and "Bridge Structure" framing centering the bridge deck, railings, piers, and riverbed channel.
