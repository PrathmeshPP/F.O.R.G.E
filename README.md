# F.O.R.G.E — Fast Optical Reconstruction & Geospatial Estimation

[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Three.js](https://img.shields.io/badge/Three.js-r186-black.svg)](https://threejs.org/)
[![Vite](https://img.shields.io/badge/Vite-8.3-646CFF.svg)](https://vitejs.dev/)
[![Git LFS](https://img.shields.io/badge/Git%20LFS-3.5%20GB-orange.svg)](https://git-lfs.github.com/)
[![License Status](https://img.shields.io/badge/License-All%20Rights%20Reserved%20(Pending%20Selection)-lightgrey.svg)](docs/THIRD_PARTY_ASSET_PROVENANCE.md)

F.O.R.G.E is a spatial intelligence and 3D scene reconstruction platform built around feed-forward vision foundation models (VGGT-Ω). It processes monocular aerial drone and corridor video sequences into metric 3D point clouds, dense depth maps, camera trajectory poses, textured triangle surface meshes, and interactive web visualization assets.

---

## Architecture & System Overview

The platform consists of two primary subsystems:
1. **F.O.R.G.E Core Reconstruction Pipeline (`src/`, `scripts/`)**: Feed-forward foundation model inference, pinhole depth backprojection, multi-view consistency filtering, Poisson/BPA surface meshing, photographic KD-tree color transfer, and Umeyama–RANSAC 7-DoF geospatial alignment.
2. **F.O.R.G.E Explorer Interactive Viewer (`viewer/`, `scripts/serve_viewer.py`)**: React 19, TypeScript, Vite, and Three.js web application for real-time 3D inspection, camera framing presets, multi-representation rendering (textured, photographic, untextured, point cloud), and split-view comparative analysis.

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

The project evaluates four real-world video sources across five benchmark entries:

| Scene Entry | Category | Source Video | Geometry | Metric & Calibration Status | Key Scene Characteristics |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1. Campus 0049** | Architectural Survey | `dji_fly_...0049...mp4` | 128k v, 253k f | Model-relative; uncalibrated scale. Non-georeferenced. | Institutional courtyard, buildings, reflecting pool, walkways. Hovering drone limits facade angles. |
| **2. Desert Road (Candidate A)** | Highway Corridor | `desert_road.mp4` | 53k v, 105k f | Camera-relative plane fit. Uncalibrated scale. | Static clean highway across sand dunes; moving traffic removed. |
| **3. Desert Road (Candidate B)** | Dynamic Preservation | `desert_road.mp4` | 53k v, 105k f | Camera-relative plane fit. Uncalibrated scale. | Highway corridor preserving single reference vehicle at `[0.03, 0.04, 0.45]`. |
| **4. Forest Road** | Terrain & Vegetation | `forest_road.mp4` | 462k v, 919k f | Model-relative; uncalibrated scale. Non-georeferenced. | Dense mountain tree canopy, winding asphalt road with yellow lane markings. |
| **5. River/Bridge** | Civil Infrastructure | `segment_52s_64s.mp4` | 132k v, 260k f | Metric GPS georeferenced via 7-DoF Umeyama–RANSAC alignment. | Dual-span highway bridge, concrete piers, riverbed channel and approach road. |

> [!IMPORTANT]
> **Reconstruction Baseline & Deliverable Clarification:**
> - **Coordinate Scale & Calibration:** Unless explicitly georeferenced via drone telemetry (e.g., River/Bridge), scene coordinates are model-relative. Metric scale is arbitrary. Unseen building facades, foliage voids, and reflective water specular noise reflect physical capture geometry and are honestly not synthesized.
> - **Deliverable Provenance:** Standalone deliverables in `outputs/manual_model_review/` are **format-converted inspection packages** (OBJ, PLY, GLB) generated from the frozen Sprint 9 baseline (`outputs/accuracy_experiment/sprint9_video_to_viewer_release/`), not newly run reconstructions.

---

## Canonical Asset Deliverables

The repository maintains two canonical deliverables directories:

### 1. Live Web Viewer Baseline (`outputs/accuracy_experiment/sprint9_video_to_viewer_release/`)
The authoritative frozen baseline served directly to the F.O.R.G.E Explorer web application:
- `textured_mesh.obj` & `textured_mesh.mtl`: Polygonal Wavefront mesh with explicit UV mapping.
- `texture_atlas.png`: 4096×4096 unified diffuse texture atlas.
- `surface_mesh_photographic.ply`: KD-tree vertex-colored photographic mesh.
- `surface_mesh_repaired.ply`: Watertight geometric surface mesh.
- Verified render previews and candidate metadata JSONs.

### 2. Standalone Manual Review Packages (`outputs/manual_model_review/`)
Independent deliverables for direct inspection in **Blender**, **MeshLab**, **CloudCompare**, or **Windows 3D Viewer**:
- `model_geometry.ply` & `model_geometry.obj`: Clean coordinate geometry with zero texture dependencies.
- `model_photographic.ply`: Photographic vertex-colored RGB mesh.
- `model_textured.obj`, `model_textured.mtl`, and `texture_atlas.png`: Textured model for 3D DCC tools.
- `model_preview.glb`: Self-contained glTF 2.0 binary with embedded textures for 1-click inspection.
- `forge_models_for_manual_inspection.zip` (354.2 MB): Consolidated master archive containing all 5 scenes.

Detailed opening guides and SHA-256 checksums are cataloged in [`outputs/manual_model_review/MODEL_CATALOG.md`](outputs/manual_model_review/MODEL_CATALOG.md).

---

## Source Video Licensing & Provenance

The four source videos used across the five scenes have distinct redistribution terms:

| Video Asset | Path | Size | Origin / Rights Holder | License | Redistribution Status |
| :--- | :--- | :---: | :--- | :--- | :--- |
| **Campus 0049** | `data/source_videos/dji_fly_...0049...mp4` | 154.5 MB | Attributed to Team (`PrathmeshPP` / SIH) | First-party proprietary | **UNCONFIRMED** (Plausible First-Party Inference; Pending Formal Release) |
| **River/Bridge** | `data/source_videos/segment_52s_64s.mp4` | 66.8 MB | Attributed to Team (`PrathmeshPP` / SIH) | First-party proprietary | **UNCONFIRMED** (Plausible First-Party Inference; Pending Formal Release) |
| **Desert Road** | `data/source_videos/desert_road.mp4` | 6.2 MB | Meta AI & Univ. of Oxford (`VGGT-Omega`) | [Meta FAIR Noncommercial License](docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt) | **RESTRICTED** (Noncommercial Research only; AUP restrictions apply) |
| **Forest Road** | `data/source_videos/forest_road.mp4` | 10.2 MB | Meta AI & Univ. of Oxford (`VGGT-Omega`) | [Meta FAIR Noncommercial License](docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt) | **RESTRICTED** (Noncommercial Research only; AUP restrictions apply) |

> [!NOTE]
> Detailed legal analysis, upstream acceptable use policy constraints (including military/warfare prohibitions), and project licensing options for the repository owner are fully documented in [`docs/THIRD_PARTY_ASSET_PROVENANCE.md`](docs/THIRD_PARTY_ASSET_PROVENANCE.md). A complete copy of the upstream Meta FAIR Noncommercial Research License is provided at [`docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt`](docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt).

---

## Repository Documentation Index

| Document | Purpose & Contents |
| :--- | :--- |
| [`docs/THIRD_PARTY_ASSET_PROVENANCE.md`](docs/THIRD_PARTY_ASSET_PROVENANCE.md) | Exhaustive legal provenance, upstream URLs, Meta FAIR terms, and project licensing options. |
| [`docs/REPOSITORY_ASSET_POLICY.md`](docs/REPOSITORY_ASSET_POLICY.md) | Durable three-tier artifact policy (Git vs Git LFS vs External untracked foundation weights). |
| [`docs/GIT_LFS_AND_STORAGE.md`](docs/GIT_LFS_AND_STORAGE.md) | Git LFS inventory, bandwidth limits, selective checkout commands, and historical purge caveats. |
| [`docs/ASSET_MANIFEST.md`](docs/ASSET_MANIFEST.md) | Human-readable verification manifest of all 484 tracked project deliverables. |
| [`docs/asset_manifest.json`](docs/asset_manifest.json) | Machine-readable SHA-256 asset manifest with source video verification statuses. |
| [`docs/SPRINT10_LIVE_DEMO_GUIDE.md`](docs/SPRINT10_LIVE_DEMO_GUIDE.md) | Complete presenter walk-through script and camera framing guide for evaluators. |
| [`docs/OPENCV_COMPLETION_ROADMAP.md`](docs/OPENCV_COMPLETION_ROADMAP.md) | Mathematical formulation for GPS alignment, pose recovery, and texturing. |

---

## Installation & Quickstart

### 1. Prerequisites
- **Python:** 3.11 or 3.12 recommended.
- **Node.js:** v18+ (for building the viewer).
- **Git LFS:** Required for retrieving large 3D models and source videos.
- **CUDA GPU:** Strongly recommended for inference (`cuda:0`), optional for viewer inspection.

### 2. Git LFS Checkout
```bash
# Install Git LFS hooks
git lfs install

# Clone the repository
git clone https://github.com/PrathmeshPP/FORGE.git
cd FORGE

# Pull all tracked LFS assets (~3.5 GB)
git lfs pull
```

*Selective checkout options (to pull only specific scenes or viewer meshes) are documented in [`docs/GIT_LFS_AND_STORAGE.md`](docs/GIT_LFS_AND_STORAGE.md).*

### 3. Python Environment Setup
```bash
# Create and activate virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
pip install -r requirements-viewer.txt
```

### 4. Launching F.O.R.G.E Explorer

#### One-Click Launch (Windows)
Double-click `start_forge_explorer.bat` or execute in command prompt:
```cmd
start_forge_explorer.bat
```

#### Manual Launch
```bash
python scripts/serve_viewer.py --port 5173
```
Open [http://localhost:5173](http://localhost:5173) in Chrome, Edge, or Firefox.

---

## Automated Test Suite

F.O.R.G.E includes comprehensive unit and integration tests verifying manifest accuracy, geometry extents, coordinate finiteness, texture bindings, and HTTP asset serving:

```bash
# Run comprehensive release test suites
pytest tests/test_sprint9_video_to_viewer.py tests/test_sprint10_demo_hardening.py tests/test_sprint10_1_viewer_refinement.py tests/test_sprint10_2_standalone_review.py -v
```

---

## Project License & Ownership

- **Repository Owner:** [PrathmeshPP](https://github.com/PrathmeshPP)
- **Current License Status:** **All Rights Reserved** (Pending formal license selection by owner; see [`docs/THIRD_PARTY_ASSET_PROVENANCE.md`](docs/THIRD_PARTY_ASSET_PROVENANCE.md)).
- **Third-Party Research Materials:** `desert_road.mp4`, `forest_road.mp4`, and VGGT-Ω foundation model code/weights are licensed under the **Meta FAIR Noncommercial Research License (v1, Oct 2024)**. Commercial exploitation is prohibited without separate licensing from Meta AI.
