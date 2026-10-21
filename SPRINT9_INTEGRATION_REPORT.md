# F.O.R.G.E — Sprint 9 Integration & Viewer Release Report
**Release Run Identifier:** `sprint9_video_to_viewer_release`  
**Timestamp:** `2026-10-20T20:40:05Z`  
**Application:** F.O.R.G.E Explorer (React 19 + Three.js r186 + TypeScript + Vite)  
**Server Architecture:** Python Companion Server (`scripts/serve_viewer.py`) on `http://127.0.0.1:5173`  
**Validation Browser:** Microsoft Edge (Chromium Headless, WebGL GPU accelerated, 1440x900)  

---

## 1. Executive Integration Summary

In Sprint 9, the complete end-to-end photogrammetric workflow was executed and validated:
`Source Videos -> Keyframe Selection -> Depth & Camera Pose Inference -> Point Cloud Fusion -> Surface Reconstruction -> Photographic Projection -> Quality Validation -> Asset Manifest -> F.O.R.G.E Explorer`.

All 5 benchmark model targets were reconstructed, registered with genuine disk SHA-256 hashes in `viewer/src/data/model-manifest.json`, and interactively loaded in F.O.R.G.E Explorer with 0 console errors and 0 WebGL failures.

| Scene Target | Default Representation | Active Asset Path | Disk Size | Verified SHA-256 (64-char Hex) | WebGL Render Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Campus 0049** | Textured OBJ (Atlas) | `/models/campus_0049/textured_mesh.obj` | 31.28 MB | `7278af1cbeb04ce8c31d83db50b09d59ea9516903f62f4561b3d05bbd62daf82` | **PASS (Rendered)** |
| **Desert Road (Cand A)** | Textured OBJ (Atlas) | `/models/desert_road/candidate_a/textured_mesh.obj` | 14.30 MB | `8ae94a504a7ee5f4107b5387d3e05a5a73e59048a17fe68c92bda639cb8853b0` | **PASS (Rendered)** |
| **Desert Road (Cand B)** | Textured OBJ (Atlas) | `/models/desert_road/candidate_b/textured_mesh.obj` | 14.31 MB | `1681a28a3036495be134aee491ce0cf354aa0cae0d297ff0df1f38eec5b4a9f2` | **PASS (Rendered)** |
| **Forest Road** | Textured OBJ (Atlas) | `/models/forest_road/textured_mesh.obj` | 117.41 MB | `f05f497181361cf4ecb98926d843f8a178c4b48d8c662ab8604e2f779fa2d86b` | **PASS (Rendered)** |
| **River/Bridge** | Textured OBJ (Atlas) | `/models/river_bridge/textured_mesh.obj` | 32.27 MB | `fb28f5522dbae5d28ee9358d8e8056a1252ef8f56db39ccf2984b3c7c28808a9` | **PASS (Rendered)** |

---

## 2. Interactive Feature & Mode Verification

### 2.1 Scene Switching & Orbit Controls
- Verified switching between all 5 scene library cards.
- Offscreen mouse drag interactions verify orbit, pan, and zoom functionality without WebGL context loss or memory leaks.

### 2.2 Dual-Viewport Comparison Mode (Desert Road Candidate A vs Candidate B)
- Activated via `.quick-compare-box button` in the left sidebar.
- Left Viewport: Desert Road Candidate A (Clean static highway corridor, moving vehicle completely removed, seamless shoulders).
- Right Viewport: Desert Road Candidate B (Frame 0 reference Mini Cooper vehicle preserved via GrabCut shape-aware mask with ground shadow, zero rectangular road box).
- Synchronized camera orbit and simultaneous WebGL rendering verified.

### 2.3 Photographic & Geometric Representation Switching
- All models support switching between:
  1. `textured_obj`: 3D OBJ + Material MTL + 4K Texture Atlas PNG.
  2. `photographic_ply`: Full-resolution vertex-colored PLY.
  3. `untextured_ply`: Pure geometric surface mesh for structural and defect inspection.

---

## 3. Browser Automation Screenshot Evidence

All screenshots were captured live from the running application at `http://127.0.0.1:5173/` and saved to:
`outputs/accuracy_experiment/sprint9_video_to_viewer_release/viewer_screenshots/`

1. `screenshot_campus_0049.png` (393,345 bytes) — Architectural complex, building blocks, and courtyard.
2. `screenshot_desert_road_candidate_a.png` (295,963 bytes) — Clean static highway corridor.
3. `screenshot_desert_road_candidate_b.png` (298,137 bytes) — Preserved reference vehicle with natural asphalt integration.
4. `screenshot_forest_road.png` (417,520 bytes) — Canopy voids and road-focused yellow markings.
5. `screenshot_river_bridge.png` (376,506 bytes) — Centered highway bridge deck, railings, and piers.
6. `screenshot_desert_road_comparison.png` (355,768 bytes) — Dual-viewport side-by-side comparison mode.

---

## 4. Software Regression & Integrity Test Results

The full test suite was executed against the release:
```bash
pytest tests/test_sprint9_video_to_viewer.py -v
```
**Outcome:** `6 passed in 2.03s` (100% pass rate).
- `test_01_source_inventory_validity`: PASSED
- `test_02_scene_directories_and_required_artifacts`: PASSED
- `test_03_master_release_deliverables`: PASSED
- `test_04_manifest_schema_and_disk_hash_match`: PASSED
- `test_05_obj_mtl_texture_reference_integrity`: PASSED
- `test_06_live_viewer_asset_serving`: PASSED

---

## 5. Provenance & Coordinate System Status

- **Coordinate Frame:** All models are expressed in verified model-relative coordinate frames with correct orientation conventions (-Y up in OpenCV camera system).
- **Physical Scale & Georeferencing:** Explicitly documented as uncalibrated. No claims of physical scale or geospatial accuracy are made, in strict compliance with Sprint 9 requirements.
