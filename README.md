# F.O.R.G.E — Feed-forward Optical Reconstruction & Geometry Engine

F.O.R.G.E is a spatial intelligence and 3D scene reconstruction platform built on feed-forward vision foundation models (VGGT-Ω). It processes monocular aerial drone and vehicular corridor video sequences into metric 3D point clouds, dense depth maps, camera trajectory poses, textured triangle surface meshes, and interactive web visualization assets.

---

## Architecture & System Overview

The platform consists of two primary subsystems:
1. **F.O.R.G.E Core Reconstruction Pipeline (`src/`, `scripts/`)**: Feed-forward foundation model inference, pinhole depth backprojection, multi-view consistency filtering, Poisson/BPA surface meshing, photographic KD-tree color transfer, and Umeyama–RANSAC 7-DoF geospatial alignment.
2. **F.O.R.G.E Explorer Interactive Viewer (`viewer/`, `scripts/serve_viewer.py`)**: React 18, TypeScript, Vite, and Three.js web application for real-time 3D inspection, camera framing presets, multi-representation rendering (textured, photographic, untextured, point cloud), and split-view comparative analysis.

```
                      [ Monocular Source Video ]
                                  │
                                  ▼
                     [ Frame Sampling & Telemetry ]
                                  │
                                  ▼
                [ VGGT-Ω 1B Feed-Forward Foundation Model ]
                                  │
                ┌─────────────────┴─────────────────┐
                ▼                                   ▼
      [ Metric Depth & Confidence ]      [ Camera Poses (R, T, K) ]
                │                                   │
                └─────────────────┬─────────────────┘
                                  ▼
                   [ Dense Point Cloud Fusion ]
                                  │
                                  ▼
                     [ Surface Mesh Generation ]
                    (Poisson / Ball-Pivoting)
                                  │
                                  ▼
                  [ Photographic Texture Mapping ]
                                  │
          ┌───────────────────────┴───────────────────────┐
          ▼                                               ▼
[ F.O.R.G.E Explorer Viewer ]              [ Standalone Inspection Exports ]
  (Three.js Web Application)                 (OBJ, PLY, GLB, MTL, 4K Atlases)
```

---

## Benchmark Scenes & Reconstruction Baseline

The project evaluates four diverse real-world video sources across five benchmark entries:

| Scene Entry | Category | Source Video | Extracted Geometry | Key Scene Characteristics |
| :--- | :--- | :--- | :--- | :--- |
| **1. Campus 0049** | Architectural Survey | `dji_fly_20260727_172754_0049_...mp4` | 128k vertices, 253k triangles | Institutional courtyard, buildings, reflecting pool, walkways. |
| **2. Desert Road (Candidate A)** | Highway Corridor | `desert_road.mp4` | 53k vertices, 105k triangles | Static highway corridor across sand dunes; moving traffic removed. |
| **3. Desert Road (Candidate B)** | Dynamic Preservation | `desert_road.mp4` | 53k vertices, 105k triangles | Preserves reference vehicle (Mini Cooper with rooftop balloons). |
| **4. Forest Road** | Terrain & Vegetation | `forest_road.mp4` | 462k vertices, 919k triangles | Dense mountain tree canopy, winding asphalt road with yellow lines. |
| **5. River/Bridge** | Civil Infrastructure | `segment_52s_64s.mp4` | 132k vertices, 260k triangles | Dual-span highway bridge, concrete piers, riverbed and approach road. |

> [!NOTE]
> **Coordinate Scale & Calibration Notice:** Coordinates are model-relative. Metric scale is uncalibrated and arbitrary unless explicitly aligned via GPS telemetry. Unseen building facades, foliage voids, and reflective water specular noise reflect physical capture limitations and are honestly not synthesized.

---

## Standalone 3D Model Deliverables (`outputs/manual_model_review/`)

All five scenes are delivered in self-contained formats for direct inspection in **Blender**, **MeshLab**, **CloudCompare**, or **Windows 3D Viewer** completely independently of the web viewer:

- **Untextured 3D Mesh**: `model_geometry.ply` and `model_geometry.obj` (clean coordinate geometry with zero texture dependencies).
- **Photographic 3D Mesh**: `model_photographic.ply` (vertex-colored RGB point/mesh representation).
- **Textured 3D Model**: `model_textured.obj`, `model_textured.mtl`, and `texture_atlas.png` (4096×4096 diffuse map).
- **Self-Contained Preview**: `model_preview.glb` (glTF 2.0 binary with embedded PBR materials for 1-click inspection).
- **Master Inspection Package**: `forge_models_for_manual_inspection.zip` (354.2 MB archive containing all models and textures).

Detailed manual opening instructions and SHA-256 hashes are cataloged in [`outputs/manual_model_review/MODEL_CATALOG.md`](outputs/manual_model_review/MODEL_CATALOG.md).

---

## Repository Structure

```
FORGE/
 ├── data/
 │    └── source_videos/             # Verified original source drone & benchmark footage
 ├── docs/
 │    ├── ASSET_MANIFEST.md          # Comprehensive catalog of all 480+ project assets
 │    ├── asset_manifest.json        # Machine-readable asset verification inventory
 │    ├── SPRINT10_LIVE_DEMO_GUIDE.md# Presenter demo script & camera framing guide
 │    └── OPENCV_COMPLETION_ROADMAP.md # Mathematical formulation & technical roadmap
 ├── outputs/
 │    ├── manual_model_review/       # Sprint 10.2 standalone OBJ, PLY, GLB, and ZIP packages
 │    ├── accuracy_experiment/
 │    │    ├── sprint9_video_to_viewer_release/ # Authoritative frozen baseline reconstructions
 │    │    ├── sprint10_demo_release/           # Demo hardening deliverables & screenshots
 │    │    ├── sprint10_1_viewer_refinement/   # Framing presets & metadata verification
 │    │    └── gps_aligned_v1/                  # Umeyama-RANSAC metric-georeferenced models
 │    ├── dji_0049_validation/       # Campus 0049 camera poses & validation artifacts
 │    └── dji_river_bridge_validation/ # River Bridge source segment & validation predictions
 ├── scripts/
 │    ├── serve_viewer.py            # Zero-dependency Python HTTP server with Range support
 │    ├── build_manual_model_review.py # Standalone model packaging & GLB export runner
 │    ├── align_reconstruction_gps.py # Umeyama-RANSAC 7-DoF batch geospatial alignment
 │    ├── run_pipeline.py            # End-to-end video-to-predictions pipeline
 │    ├── reconstruct_mesh.py        # Surface meshing & KD-tree vertex color interpolation
 │    ├── generate_point_cloud.py    # Depth backprojection and voxel fusion CLI
 │    └── preview_point_cloud.py     # Orthographic projection visual QA generator
 ├── src/
 │    ├── geo/                       # Telemetry parsing, Umeyama-RANSAC, and quality gates
 │    ├── reconstruction/            # Backprojection, multi-view consistency, voxel fusion
 │    ├── vggt/                      # VGGT-Ω model loading, inference, and camera decoding
 │    └── video/                     # Frame extraction, temporal sampling, and selection
 ├── tests/
 │    ├── test_sprint10_2_standalone_review.py # Standalone review deliverable validation
 │    ├── test_sprint10_1_viewer_refinement.py # Camera framing & metadata integrity
 │    ├── test_sprint10_demo_hardening.py      # Demo release & HTTP server endpoint tests
 │    └── test_geo_alignment.py      # Telemetry, Umeyama-RANSAC, and quality gate tests
 ├── viewer/                         # F.O.R.G.E Explorer React/TypeScript/Three.js Web Application
 │    ├── dist/                      # Pre-built production distribution for instant demo
 │    ├── src/                       # Components, Three.js canvas, shaders, and data model
 │    └── package.json               # Frontend dependencies (React, Three.js, Lucide, Vite)
 ├── start_forge_explorer.bat        # 1-click Windows demo launcher
 ├── requirements.txt                # Python core pipeline dependencies
 ├── requirements-viewer.txt         # Optional Open3D viewer dependency
 ├── .gitattributes                  # Git LFS tracking rules for 3D assets, videos, and archives
 └── README.md
```

---

## Quickstart & Launching the Explorer

### 1. Launch with One Click (Windows)
Double-click `start_forge_explorer.bat` or run:

```cmd
start_forge_explorer.bat
```

This starts the local asset-serving daemon and launches `http://localhost:5173` in your default browser.

### 2. Manual Server Launch
```bash
python scripts/serve_viewer.py --port 5173
```

Navigate to `http://localhost:5173`. The server handles Range requests for 4K video playback, serves static models with CORS headers, and includes an automated health check at `/health`.

---

## Git LFS & Asset Retrieval

Large binary assets (3D meshes, self-contained GLB files, ZIP packages, and MP4 videos) are tracked via **Git LFS**.

### Initial Setup
```bash
# Install Git LFS hooks
git lfs install

# Clone the repository
git clone https://github.com/PrathmeshPP/FORGE.git
cd FORGE

# Pull all tracked LFS assets
git lfs pull
```

To pull assets for a specific scene only:
```bash
git lfs pull --include="outputs/manual_model_review/campus_0049/*"
```

---

## Running the Automated Test Suite

The test suite validates pipeline integrity, mathematical alignment, coordinate finiteness, and standalone deliverables:

```bash
# Run comprehensive sprint verification suite
pytest tests/test_sprint10_demo_hardening.py tests/test_sprint10_1_viewer_refinement.py tests/test_sprint10_2_standalone_review.py -v
```

---

## License & Attribution

- **F.O.R.G.E Platform**: Developed as part of the Smart India Hackathon (SIH) spatial intelligence challenge.
- **Foundation Model**: VGGT-Ω (Visual Geometry Grounded Transformer for Monocular 3D Reconstruction).
- **Telemetry & Video**: Captured via DJI Air 3S drone and public highway dashcam benchmarks.
