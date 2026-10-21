# F.O.R.G.E — Repository Asset and Artifact Lifecycle Policy

> **Document Version:** 1.0.0  
> **Date:** October 2026  
> **Repository:** [https://github.com/PrathmeshPP/FORGE](https://github.com/PrathmeshPP/FORGE)  
> **Status:** Active Project Storage & Maintenance Standard

---

## 1. Purpose and Guiding Principles

This document formalizes the storage, tracking, and retention policy for all assets across the **F.O.R.G.E** repository.

The goal is to maintain a repository that is **recoverable, verifiable, and lightweight to clone**, while guaranteeing that essential deliverables, benchmarks, and interactive viewer assets remain reproducible without unbounded storage inflation.

---

## 2. Three-Tier Storage Architecture

All repository files, intermediate outputs, and binaries are categorized into one of three distinct tiers:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       TIER 1: STANDARD GIT VERSIONING                       │
│  • Pure text files, source code, configs, test suites, reports, manifests  │
│  • Max recommended per-file size: < 100 KB (strictly < 1 MB)               │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
┌──────────────────────────────────────┴──────────────────────────────────────┐
│                    TIER 2: GIT LARGE FILE STORAGE (LFS)                     │
│  • Approved canonical source videos (MP4)                                   │
│  • Canonical web viewer mesh baselines (OBJ, PLY, MTL, PNG atlas)           │
│  • Standalone manual inspection deliverables (OBJ, PLY, GLB, master ZIP)    │
│  • Essential numerical prediction tensors (NPZ, PT)                         │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
┌──────────────────────────────────────┴──────────────────────────────────────┐
│                      TIER 3: EXTERNAL / UNTRACKED CACHE                     │
│  • Foundation model weights (> 1 GB, e.g. VGGT-Omega checkpoint)             │
│  • Ephemeral point cloud fusion caches and intermediate voxel grids         │
│  • Superseded experimental runs and debug render scratch files              │
│  • Redundant per-scene sub-archives                                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Tier 1: Tracked Directly in Standard Git

The following items are tracked directly as standard Git objects:
- **Core Pipeline Source Code:** All Python packages under `src/` (`src/geo/`, `src/reconstruction/`, `src/vggt/`, `src/video/`) and root executable scripts (`run_pipeline.py`, `scripts/*.py`).
- **Web Viewer Source Code:** All TypeScript, TSX, CSS, SVG, and HTML files under `viewer/src/`, `viewer/public/`, and root build configurations (`viewer/vite.config.ts`, `viewer/package.json`).
- **Automated Test Suites:** All pytest modules under `tests/` and associated configuration fixtures.
- **Dependency and Build Manifests:** `requirements.txt`, `requirements-viewer.txt`, `viewer/package.json`, `viewer/package-lock.json`.
- **Engineering Documentation & Reports:** All technical reports, sprint audit documents, architecture specifications, and forensic writeups across the root directory and `docs/`.
- **Machine-Readable Metadata and Manifests:** `docs/asset_manifest.json`, `docs/ASSET_MANIFEST.md`, `viewer/src/data/model-manifest.json`, scene candidate metadata JSONs.
- **Lightweight Static UI Assets:** Favicons, SVG icons, and low-resolution thumbnails (< 100 KB).

---

## 4. Tier 2: Tracked Through Git Large File Storage (Git LFS)

The following items represent large binaries that are version-controlled via Git LFS using pointer references in the main tree:

### Tracked File Patterns (`.gitattributes`)
```gitattributes
# 3D Model Geometry & Meshes
*.obj filter=lfs diff=lfs merge=lfs -text
*.ply filter=lfs diff=lfs merge=lfs -text
*.glb filter=lfs diff=lfs merge=lfs -text
*.gltf filter=lfs diff=lfs merge=lfs -text

# Video & Media Streams
*.mp4 filter=lfs diff=lfs merge=lfs -text
*.mov filter=lfs diff=lfs merge=lfs -text
*.avi filter=lfs diff=lfs merge=lfs -text

# Deliverable Archives
*.zip filter=lfs diff=lfs merge=lfs -text
*.tar filter=lfs diff=lfs merge=lfs -text
*.gz filter=lfs diff=lfs merge=lfs -text

# Numerical Tensors & Checkpoint Weights
*.npz filter=lfs diff=lfs merge=lfs -text
*.pt filter=lfs diff=lfs merge=lfs -text
*.pth filter=lfs diff=lfs merge=lfs -text
*.bin filter=lfs diff=lfs merge=lfs -text
```

### Justified Canonical LFS Assets in F.O.R.G.E
1. **Source Videos (`data/source_videos/`):**
   - `dji_fly_...0049...mp4` (154.5 MB)
   - `segment_52s_64s.mp4` (66.8 MB)
   - `desert_road.mp4` (6.2 MB)
   - `forest_road.mp4` (10.2 MB)
2. **Canonical Web Viewer Baseline (`outputs/accuracy_experiment/sprint9_video_to_viewer_release/`):**
   - Untextured geometric meshes (`surface_mesh_repaired.ply`)
   - Photographic vertex-colored meshes (`surface_mesh_photographic.ply`)
   - Textured polygonal meshes (`textured_mesh.obj`, `textured_mesh.mtl`, `texture_atlas.png`)
3. **Standalone Manual Review Deliverables (`outputs/manual_model_review/`):**
   - Untextured OBJ and PLY geometry (`model_geometry.obj`, `model_geometry.ply`)
   - Photographic vertex PLY (`model_photographic.ply`)
   - Textured OBJ and atlas (`model_textured.obj`, `texture_atlas.png`)
   - Standalone glTF binary (`model_preview.glb`)
   - Master review package (`forge_models_for_manual_inspection.zip`, 354.2 MB)
4. **Essential Numerical Outputs:**
   - Inference predictions (`predictions_raw.pt`)
   - GPS-aligned camera poses (`camera_poses_gps_aligned.npz`)

---

## 5. Tier 3: Stored Outside Git History (Untracked / External)

To prevent repository bloat, the following artifacts **must NEVER be committed directly to Git or uploaded as redundant LFS objects**:

### Excluded Asset Categories
1. **Foundation Model Weights:**
   - Foundation checkpoints (> 1 GB) such as `vggt_omega_1b_512.pt` (4.48 GB) must be acquired from their official distribution repository (Hugging Face).
2. **Intermediate Transient Reconstruction Caches:**
   - Dense multi-frame point cloud dumps, raw un-stitched window point clouds, temporary KD-tree search caches, and intermediate Marching Cubes voxel grids.
3. **Superseded or Failed Experimental Sprints:**
   - Diagnostic iteration runs (`sprint7*`, `quality_recovery_v2`, `regression_diagnosis_*`, `high_quality_multiframe_v1`).
   - These are excluded via `.gitignore` to preserve local research continuity without committing 20+ GB to the remote.
4. **Redundant Duplicate Archives:**
   - Per-scene sub-ZIP archives (e.g., `campus_0049.zip`, `forest_road.zip`) when individual unpacked files and a single consolidated master archive already exist.
5. **Ephemeral Debug Screenshots and Visual Dumps:**
   - Root-level scratch captures (`scratch_*.png`, `scratch_*.ply`, `test_*.png`).

---

## 6. External Asset Acquisition Specification

For external assets not checked into the repository, engineers must follow these acquisition specifications:

### Foundation Model: VGGT-Omega-1B-512
- **Asset Name:** `vggt_omega_1b_512.pt`
- **Expected Local Path:** Configured via `VGGT_OMEGA_CHECKPOINT` environment variable, or placed under `models/vggt_omega_1b_512.pt`.
- **Upstream Source:** Hugging Face model repository `facebook/VGGT-Omega`.
- **Direct URL:** `https://huggingface.co/facebook/VGGT-Omega/blob/main/vggt_omega_1b_512.pt`
- **Access Procedure:** Automated request approval on Hugging Face (noncommercial research affiliation required).
- **Exact File Size:** 4,814,948,352 bytes (~4.48 GB).
- **Applicable License:** Meta FAIR Noncommercial Research License (v1, Oct 2024).

---

## 7. Rules for Proposing and Adding New Large Binaries

Before staging any binary file exceeding **500 KB**, engineers must verify:
1. **Is the asset canonical?** Does it represent a finalized milestone deliverable, or is it an intermediate artifact that can be recomputed by running a pipeline script?
2. **Is it already covered by `.gitattributes`?** Verify with `git check-attr -a path/to/file` that LFS filters are active before running `git add`.
3. **Does a compressed or optimized alternative exist?** E.g., convert dense uncompressed point clouds to clean indexed meshes or glTF binaries.
4. **Bandwidth Impact:** Consider the cumulative clone footprint for continuous integration runners and new developers.
