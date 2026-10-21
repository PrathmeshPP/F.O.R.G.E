# F.O.R.G.E — Sprint 10.2: Standalone 3D Model Deliverables and Manual Inspection Report

**Sprint Phase:** Sprint 10.2 Standalone 3D Model Deliverables & Independent Manual Inspection  
**Reconstruction Source Baseline:** `outputs/accuracy_experiment/sprint9_video_to_viewer_release/` (**100% Frozen & Genuine**)  
**Delivered Review Directory:** [`outputs/manual_model_review/`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/)  
**Master Review ZIP Archive:** [`outputs/manual_model_review/forge_models_for_manual_inspection.zip`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forge_models_for_manual_inspection.zip) (354.20 MB)  
**Date of Execution:** 2026-10-21 13:28:10 UTC  
**Execution Status:** **ALL FIVE SCENES EXPORTED, INDEPENDENTLY LOADED, AND VALIDATED (100% PASS)**

---

## 1. Executive Summary

Sprint 10.2 successfully extracted, standardized, and validated **standalone 3D model files** that can be opened and inspected independently of the F.O.R.G.E Explorer website. 

All viewer styling and presentation tasks were paused. The priority was exclusively to verify that genuine 3D model artifacts derived from the four source videos exist on disk, are completely valid and nonempty, and can be loaded directly by external 3D software (Blender, MeshLab, CloudCompare, Windows 3D Viewer).

### Key Deliverables Completed:
1. **Physical Filesystem Audit:** Inspected every historical model in `outputs/accuracy_experiment/sprint9_video_to_viewer_release/` directly via disk read operations. All five scene targets contain valid, nonzero, nonempty 3D triangle meshes, point clouds, and 4K texture atlases.
2. **Dedicated Export Layout:** Created `outputs/manual_model_review/` with 5 scene subdirectories:
   - `campus_0049/`
   - `desert_road_candidate_a/`
   - `desert_road_candidate_b/`
   - `forest_road/`
   - `river_bridge/`
3. **Four Standard Inspection Formats per Scene:**
   - **A. Untextured 3D Mesh:** `model_geometry.ply` and `model_geometry.obj`
   - **B. Photographic 3D Mesh:** `model_photographic.ply` (with vertex RGB colors)
   - **C. Textured 3D Model:** `model_textured.obj`, `model_textured.mtl`, and `texture_atlas.png`
   - **D. Self-Contained Viewing Format:** `model_preview.glb` (glTF 2.0 binary with embedded PBR textures)
4. **Independent Library Validation:** Every exported model was independently loaded and verified with Python `trimesh` v4.6.4 and Open3D v0.19.0. All coordinates are finite (0 NaNs, 0 Infs), all meshes have positive 3D extents, and all texture references resolve cleanly.
5. **Fresh Offscreen Renders:** Generated supplementary `render_overview.png` and `render_oblique.png` directly from the delivered model files using Open3D.
6. **Master ZIP Package:** Built `outputs/manual_model_review/forge_models_for_manual_inspection.zip` (354.20 MB, SHA-256: `65f84bfe2950e1ae535b5c60478863d7174229310362357fdd120a96511404f0`) containing all 5 scenes ready for judges to download and inspect.
7. **Complete Documentation:** Published [`MODEL_CATALOG.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/MODEL_CATALOG.md), [`model_integrity_report.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/model_integrity_report.json), and this report.

---

## 2. Physical Model Filesystem Audit

We inspected the actual filesystem across all expected directories under `outputs/accuracy_experiment/sprint9_video_to_viewer_release/`. No values were inferred from reports or screenshots.

### Audit Summary:

| Target Scene | Source Files Found | Triangle Mesh | Point Clouds | Texture Atlas | Video Provenance Traced |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Campus 0049** | OBJ (31.3 MB), MTL (176 B), PLY photo (9.8 MB), PLY geom (9.8 MB) | Yes (128,336 v / 253,487 f) | Yes (Fused 13.7 MB, Unfused 57.1 MB) | Yes (4096x4096, 29.3 MB) | `dji_fly_20260727_172754_0049_1785175051818_video.mp4` (4K DJI Drone) |
| **Desert Road A** | OBJ (14.3 MB), MTL (175 B), PLY photo (4.1 MB), PLY geom (4.1 MB) | Yes (53,069 v / 105,467 f) | Yes (Fused 1.7 MB, Unfused 30.1 MB) | Yes (4096x4096, 8.8 MB) | `desert_road.mp4` (720p 477 frames, moving vehicles removed) |
| **Desert Road B** | OBJ (14.3 MB), MTL (175 B), PLY photo (4.1 MB), PLY geom (4.1 MB) | Yes (53,101 v / 105,518 f) | Yes (Fused 1.8 MB, Unfused 30.1 MB) | Yes (4096x4096, 8.8 MB) | `desert_road.mp4` (720p 477 frames, vehicle preserved at [0.03, 0.04, 0.45]) |
| **Forest Road** | OBJ (117.4 MB), MTL (176 B), PLY photo (35.6 MB), PLY geom (35.6 MB) | Yes (462,896 v / 919,292 f) | Yes (Fused 7.2 MB, Unfused 71.3 MB) | Yes (4096x4096, 25.1 MB) | `forest_road.mp4` (720p 730 frames, canopy & winding roadway) |
| **River/Bridge** | OBJ (32.3 MB), MTL (176 B), PLY photo (10.1 MB), PLY geom (10.1 MB) | Yes (132,424 v / 260,685 f) | Yes (Fused 6.9 MB, Unfused 39.6 MB) | Yes (4096x4096, 20.8 MB) | `segment_52s_64s.mp4` (4K 750 frames, concrete bridge structure & piers) |

The Sprint 9 reconstruction models are completely genuine, fully formed, and traceable to the raw input videos. Rerunning depth inference from scratch was unnecessary and would violate the instruction to freeze baseline quality unless missing or corrupt.

---

## 3. Delivered Standalone Models & Mesh Statistics

For each scene, the delivered files under `outputs/manual_model_review/` were audited for exact statistics:

| Scene Directory | Delivered Models | Unique Vertices | Triangles | Bounding Box Dimensions ($S_x \times S_y \times S_z$) | Texture Atlas | GLB Size |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `campus_0049/` | `model_geometry.ply`, `.obj`, `model_photographic.ply`, `model_textured.obj`, `.mtl`, `model_preview.glb` | 128,336 | 253,487 | $5.314 \times 3.802 \times 0.349$ | 4096x4096 PNG (29.3 MB) | 48.0 MB |
| `desert_road_candidate_a/` | `model_geometry.ply`, `.obj`, `model_photographic.ply`, `model_textured.obj`, `.mtl`, `model_preview.glb` | 53,069 | 105,467 | $1.205 \times 0.281 \times 4.647$ | 4096x4096 PNG (8.8 MB) | 20.2 MB |
| `desert_road_candidate_b/` | `model_geometry.ply`, `.obj`, `model_photographic.ply`, `model_textured.obj`, `.mtl`, `model_preview.glb` | 53,101 | 105,518 | $1.203 \times 0.280 \times 4.644$ | 4096x4096 PNG (8.8 MB) | 20.2 MB |
| `forest_road/` | `model_geometry.ply`, `.obj`, `model_photographic.ply`, `model_textured.obj`, `.mtl`, `model_preview.glb` | 462,896 | 919,292 | $1.685 \times 0.937 \times 0.471$ | 4096x4096 PNG (25.1 MB) | 92.2 MB |
| `river_bridge/` | `model_geometry.ply`, `.obj`, `model_photographic.ply`, `model_textured.obj`, `.mtl`, `model_preview.glb` | 132,424 | 260,685 | $3.900 \times 0.740 \times 3.083$ | 4096x4096 PNG (20.8 MB) | 39.4 MB |

---

## 4. Independent Library Loading & Validation Results

Every delivered model was programmatically loaded and verified using two independent 3D software libraries: **`trimesh` (v4.6.4)** and **`open3d` (v0.19.0)**.

### Verification Checklist:
- **Geometry Nonempty:** All triangle meshes contain $\ge 53,069$ vertices and $\ge 105,467$ faces (**PASS**).
- **Coordinate Finiteness:** Checked `np.all(np.isfinite(mesh.vertices))` across all 5 models: **0 NaNs, 0 Infs** (**PASS**).
- **Nonzero Spatial Extents:** All bounding box spans along $X, Y, Z$ are strictly positive ($> 0.28\text{ units}$) (**PASS**).
- **Texture Reference Integrity:** In all `model_textured.obj` files, `mtllib model_textured.mtl` links to `model_textured.mtl`, which points to `texture_atlas.png` located in the exact same directory (**PASS**).
- **Reloading Saved Exports:** All exported `.glb`, `.ply`, and `.obj` files were closed and reopened successfully by both `trimesh` and `open3d` (**PASS**).
- **External Tool Availability:** Blender Foundation and CloudCompare executable binaries are not present in standard Windows paths on this headless CLI environment. As directed by Section 5, this condition is explicitly documented, and full programmatic verification via `trimesh` and `open3d` serves as the primary verification evidence.

---

## 5. Convenient ZIP Package Details

To facilitate distribution and judge evaluation, a unified standalone review ZIP package was compiled:

- **Package Path:** [`outputs/manual_model_review/forge_models_for_manual_inspection.zip`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forge_models_for_manual_inspection.zip)
- **File Size:** **371,407,846 bytes** (354.20 MB)
- **SHA-256 Digest:** `65f84bfe2950e1ae535b5c60478863d7174229310362357fdd120a96511404f0`
- **Contents:**
  - `README.txt` (opening instructions and dataset overview)
  - 5 scene directories (`campus_0049/`, `desert_road_candidate_a/`, `desert_road_candidate_b/`, `forest_road/`, `river_bridge/`)
  - 45 total inspection artifacts (`.ply`, `.obj`, `.mtl`, `.png`, `.glb`, `.json`)
  - Excludes raw multi-gigabyte point clouds to keep package download fast and responsive.

---

**Where to find the standalone 3D models**

All standalone 3D models, companion material files, texture atlases, previews, metadata files, and the master inspection archive are located under:

`outputs/manual_model_review/`

### 1. Master Inspection Package & Catalogs
- **Convenient ZIP Package:** [`outputs/manual_model_review/forge_models_for_manual_inspection.zip`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forge_models_for_manual_inspection.zip)
  - Size: `371,407,846 bytes` (354.20 MB)
  - SHA-256: `65f84bfe2950e1ae535b5c60478863d7174229310362357fdd120a96511404f0`
- **Comprehensive Model Catalog:** [`outputs/manual_model_review/MODEL_CATALOG.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/MODEL_CATALOG.md)
- **Machine-Readable Integrity Report:** [`outputs/manual_model_review/model_integrity_report.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/model_integrity_report.json)
- **Sprint 10.2 Review Report:** [`outputs/manual_model_review/MANUAL_MODEL_REVIEW_REPORT.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/MANUAL_MODEL_REVIEW_REPORT.md)

### 2. Standalone Scene Directories & Artifact Paths

#### A. Campus 0049 (`outputs/manual_model_review/campus_0049/`)
- **Untextured PLY:** [`campus_0049/model_geometry.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_geometry.ply) (9,840,793 B | `adb753a0a5d7201504ed9ff493a5174cb879cc230182b8a13b3905563985fceb`)
- **Untextured OBJ:** [`campus_0049/model_geometry.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_geometry.obj) (18,416,777 B | `2f9864ef0cf5d3a51752ec247321e1069ec4a7c062829ec0bcf5f7690fe40632`)
- **Photographic PLY:** [`campus_0049/model_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_photographic.ply) (9,840,793 B | `143a0e9d21b543ec9863dfcede67b0818b277f70d2c2d3d0387014d01533308f`)
- **Textured OBJ:** [`campus_0049/model_textured.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_textured.obj) (31,281,376 B | `a793fc0c59828b6d4eeff00e57608e068cfb9ef50c18d9628b0dd4eb086df111`)
- **Material MTL:** [`campus_0049/model_textured.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_textured.mtl) (176 B | `fa1e793910c2c3116dfbb6d795b8796aa4ee9965d3ecad3f0e0ca0ea8d3b8790`)
- **Texture Atlas:** [`campus_0049/texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/texture_atlas.png) (29,301,787 B | `f08969f697ee359409893d5f302066fa49231f6186401fa95bc0b05b821415ae`)
- **Self-Contained GLB:** [`campus_0049/model_preview.glb`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_preview.glb) (48,016,988 B | `e3b2e9894458f278eb17f461e712395642e0329057864f1fe1bb5a2ee218ad5f`)
- **Previews & Metadata:** [`render_overview.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/render_overview.png), [`render_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/render_oblique.png), [`metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/metadata.json)

#### B. Desert Road Candidate A (`outputs/manual_model_review/desert_road_candidate_a/`)
- **Untextured PLY:** [`desert_road_candidate_a/model_geometry.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_geometry.ply) (4,077,915 B | `b442150ca2d5a4b1d196c8f83d9f36b2aa330a7b815b708c9a1a17f177259a11`)
- **Untextured OBJ:** [`desert_road_candidate_a/model_geometry.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_geometry.obj) (7,631,490 B | `a79679261a8b668f4477db1953bbd3bc14c5d576a804ec8d01132569e2264639`)
- **Photographic PLY:** [`desert_road_candidate_a/model_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_photographic.ply) (4,077,915 B | `5625a5a2436abd6bca780b86efde8687246675f0e47b270f37440b006099ae44`)
- **Textured OBJ:** [`desert_road_candidate_a/model_textured.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_textured.obj) (14,301,214 B | `3255152fe1fae498308cfdcff57ffea876be8e4695eb1df6aa27cf0ce5fe0ca3`)
- **Material MTL:** [`desert_road_candidate_a/model_textured.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_textured.mtl) (175 B | `5c9f53822b31a89c9e83a48e6583971c26b9116e02613e54bdf164101e4c703b`)
- **Texture Atlas:** [`desert_road_candidate_a/texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/texture_atlas.png) (8,765,408 B | `8c4b67a346cb80d7ac3be8af2a2724f31334538509fd6b08b9f7f89484ca803d`)
- **Self-Contained GLB:** [`desert_road_candidate_a/model_preview.glb`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_preview.glb) (20,157,424 B | `fc70bf81e5b85e05ca8121d5a7d6537bfce2435f3dfcf70fe00742f534433d7b`)
- **Previews & Metadata:** [`render_overview.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/render_overview.png), [`render_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/render_oblique.png), [`metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/metadata.json)

#### C. Desert Road Candidate B (`outputs/manual_model_review/desert_road_candidate_b/`)
- **Untextured PLY:** [`desert_road_candidate_b/model_geometry.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_geometry.ply) (4,080,210 B | `3088a560227c9b821b289ac7c4ec3a23ccacda3d6d056c84d7b56a955828f185`)
- **Untextured OBJ:** [`desert_road_candidate_b/model_geometry.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_geometry.obj) (7,635,510 B | `c2a688a44ec1057e2bbdcf3ea3778a8761fe60f8607aa5267be76e0544f128e4`)
- **Photographic PLY:** [`desert_road_candidate_b/model_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_photographic.ply) (4,080,210 B | `674ad0ec3d0b40a8fdb9f0680301912a0d7e3a0552c4ef3526740e285f053edf`)
- **Textured OBJ:** [`desert_road_candidate_b/model_textured.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_textured.obj) (14,308,562 B | `0dc25e2434685ff41b5fa8a2524330ea740263c9b7fae12e1f57b6f6345d2e07`)
- **Material MTL:** [`desert_road_candidate_b/model_textured.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_textured.mtl) (175 B | `5c9f53822b31a89c9e83a48e6583971c26b9116e02613e54bdf164101e4c703b`)
- **Texture Atlas:** [`desert_road_candidate_b/texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/texture_atlas.png) (8,765,408 B | `8c4b67a346cb80d7ac3be8af2a2724f31334538509fd6b08b9f7f89484ca803d`)
- **Self-Contained GLB:** [`desert_road_candidate_b/model_preview.glb`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_preview.glb) (20,165,808 B | `d061fbb974c53d9e8fe9b9df7be720a454d6fbbfd397d92176b6b7a956272e73`)
- **Previews & Metadata:** [`render_overview.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/render_overview.png), [`render_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/render_oblique.png), [`metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/metadata.json)

#### D. Forest Road (`outputs/manual_model_review/forest_road/`)
- **Untextured PLY:** [`forest_road/model_geometry.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_geometry.ply) (35,558,818 B | `1d635dce8c9d023093d4b43fdeb420be4f366fc6d08aa6c4a391121cf03f875c`)
- **Untextured OBJ:** [`forest_road/model_geometry.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_geometry.obj) (66,547,152 B | `ee95a94042898b958c8a14b30e46b2b406e4179e83ae410fe3f16d56dfd8ee1c`)
- **Photographic PLY:** [`forest_road/model_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_photographic.ply) (35,558,818 B | `1cb5896612a8b1d040b35553ea2000df97d7a3311c59e4e2f42afcdd6c63bfc0`)
- **Textured OBJ:** [`forest_road/model_textured.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_textured.obj) (117,409,995 B | `17346fa0d1df643666f2c54bb0490b6ef6d628eb50ee515c0e1763137b77cecb`)
- **Material MTL:** [`forest_road/model_textured.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_textured.mtl) (176 B | `fa1e793910c2c3116dfbb6d795b8796aa4ee9965d3ecad3f0e0ca0ea8d3b8790`)
- **Texture Atlas:** [`forest_road/texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/texture_atlas.png) (25,149,397 B | `445210986ff9eb452b48d2eb9bf9fc551a31d996cb031cba1ffc774d2847a988`)
- **Self-Contained GLB:** [`forest_road/model_preview.glb`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_preview.glb) (92,249,272 B | `014eb0238d38e244ef0a3c7dd5006b00516ca6fbf072fdfbc05ecb371cf12ce2`)
- **Previews & Metadata:** [`render_overview.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/render_overview.png), [`render_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/render_oblique.png), [`metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/metadata.json)

#### E. River/Bridge (`outputs/manual_model_review/river_bridge/`)
- **Untextured PLY:** [`river_bridge/model_geometry.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_geometry.ply) (10,142,855 B | `7d049e9e2e680a6c002b362d4f6e7a501fa01896aedfd3cd82922d0a7f7f9be7`)
- **Untextured OBJ:** [`river_bridge/model_geometry.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_geometry.obj) (18,946,846 B | `1219b6e686ee46db7534c03bfe38a6a6fa34d5cf1567ba054593e031eb05d544`)
- **Photographic PLY:** [`river_bridge/model_photographic.ply`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_photographic.ply) (10,142,855 B | `4be4958e3b81a0a8ea2ca0ad3c7cb18c39e196884065920c5c11abaad9d7758c`)
- **Textured OBJ:** [`river_bridge/model_textured.obj`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_textured.obj) (32,270,618 B | `a7ff8b0453531bcfc239d5e3057e62a4d0f0eb21cf8b7c7b2c58908ea03eebaa`)
- **Material MTL:** [`river_bridge/model_textured.mtl`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_textured.mtl) (176 B | `fa1e793910c2c3116dfbb6d795b8796aa4ee9965d3ecad3f0e0ca0ea8d3b8790`)
- **Texture Atlas:** [`river_bridge/texture_atlas.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/texture_atlas.png) (20,836,942 B | `0d91d9600aee5ce795ff81cbbfa254ff98bf7d9a18012e873528b34fba31c778`)
- **Self-Contained GLB:** [`river_bridge/model_preview.glb`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_preview.glb) (39,431,884 B | `4f3dd48d28fa6b825227096fb21f15aa86dd68735df3f382a970be96ff3ec0b0`)
- **Previews & Metadata:** [`render_overview.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/render_overview.png), [`render_oblique.png`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/render_oblique.png), [`metadata.json`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/metadata.json)

---

### Reconstruction vs Exported Baseline Attribution
- **Unchanged Reconstruction Baseline:** All geometry, vertex coordinates, and texture pixel data were generated during Sprint 9 from the four raw source videos. No geometry was re-inferred or altered during Sprint 10.2.
- **Format-Converted Deliverables:** The `.obj` pure geometry files, `.glb` self-contained binaries, updated relative `.mtl` descriptors, offscreen preview renders, and `.zip` distribution packages are format conversions and packaging of the verified Sprint 9 reconstruction baseline.
