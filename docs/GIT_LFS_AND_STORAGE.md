# F.O.R.G.E — Git LFS and Repository Storage Optimization Guide

> **Document Version:** 1.0.0  
> **Date:** October 2026  
> **Repository:** [https://github.com/PrathmeshPP/FORGE](https://github.com/PrathmeshPP/FORGE)  
> **Target Audience:** Maintainers, Release Managers, and Contributing Engineers

---

## 1. Storage Inventory & Architecture Overview

F.O.R.G.E relies on Git Large File Storage (Git LFS) to manage large multimedia inputs, 3D surface meshes, and numerical inference arrays without bloating the core Git commit graph.

### Verified Local Measurements vs Remote Estimates

| Metric | Measured Value | Notes / Verification Basis |
| :--- | :--- | :--- |
| **Total Git Tracked Files** | 718 files | Verified via `git ls-files` (post-cleanup) |
| **Current Files Tracked by LFS (`HEAD`)** | 156 files / 3,594.19 MB (~3.51 GiB) | Active LFS pointers checked out in the current working tree |
| **Historical LFS Objects Retained in History** | 161 objects / 3,946.52 MB (~3.85 GiB) | Referenced across Git history (includes 5 historical ZIPs in commit `428a455` totaling 354.1 MB) |
| **Local Git LFS Cache (`.git/lfs/objects`)** | 3,646.37 MB (~3.56 GiB) | Local deduplicated Git LFS object blob store |
| **Complete Working Tree (incl. Ignored Caches)** | ~24.5 GB | Includes ~20.9 GB of untracked research scratch runs in `outputs/` |
| **Core Git Object Database (`.git/objects`)** | ~543 MB | Standard Git packfiles and commit tree objects |
| **GitHub Remote LFS Quota (Official Current Limit)** | **10 GiB Storage / 10 GiB/mo Bandwidth** | GitHub Free and Pro accounts include 10 GiB of storage and 10 GiB of monthly bandwidth. Historical usage (~3.85 GiB) consumes ~38.5% of the included allocation. |
| **Actual Account-Wide Remote Billing Usage** | **Not Accessible Locally** | Account-wide usage across all repositories requires GitHub Billing API access / token; single-repo usage estimated at ~3.85 GiB based on LFS object history. |

---

## 2. File Types Tracked Under Git LFS

The `.gitattributes` configuration enforces Git LFS tracking for specific binary file extensions:

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

### Breakdown of Tracked LFS Storage by Extension

```
  Extension      Active Size       Description / Primary Assets
  ─────────────────────────────────────────────────────────────────────────────
  .obj           1,345.30 MB       Textured and untextured polygonal Wavefront meshes
  .ply           1,289.42 MB       Dense geometric and photographic vertex-colored meshes
  .zip             355.95 MB       Master inspection archive (forge_models_for_manual_inspection.zip)
  .mp4             304.48 MB       Original 4 source videos (Campus 0049, River/Bridge, Desert, Forest)
  .glb             210.96 MB       Self-contained glTF 2.0 binary review deliverables
  .pt               48.39 MB       Inference output prediction tensors (predictions_raw.pt)
  .npz              39.69 MB       GPS-aligned camera pose arrays and depth maps
  ─────────────────────────────────────────────────────────────────────────────
  TOTAL          3,594.19 MB       (~3.51 GB)
```

---

## 3. Storage Optimization: Removal of Redundant Per-Scene ZIPs

During the repository audit, an unnecessary triplication of manual review deliverables was identified under `outputs/manual_model_review/`:

1. **Unpacked Scene Directories:** All individual models (`.obj`, `.ply`, `.glb`, `.mtl`, `.png`) already exist uncompressed in `outputs/manual_model_review/<scene>/`.
2. **Master Inspection ZIP:** A single consolidated archive `forge_models_for_manual_inspection.zip` (354.2 MB) packages all 5 scenes for offline manual review.
3. **Redundant Per-Scene ZIPs:** Five individual archives:
   - `campus_0049.zip` (82.5 MB)
   - `desert_road_candidate_a.zip` (29.9 MB)
   - `desert_road_candidate_b.zip` (29.9 MB)
   - `forest_road.zip` (145.5 MB)
   - `river_bridge.zip` (66.4 MB)

**Remediation Action:**  
The five per-scene ZIP archives (totaling **354.1 MB**) were removed from the tracked working tree. Because the individual unpacked scene files and the consolidated master archive remain completely intact, zero deliverables or inspection workflows are compromised.

---

## 4. Crucial Git LFS Retention Mechanics & Historical Purging Notice

> [!IMPORTANT]
> **Removing files from the latest Git commit does NOT automatically delete previous LFS objects from remote storage.**

When a file is removed via `git rm`:
1. It is removed from the current commit's working tree and will **not be downloaded** during fresh checkouts of `main`.
2. However, the LFS object blob remains permanently stored in GitHub's backend storage because earlier historical commits (such as `428a455`) still reference it.
3. Git history is intentionally preserved without destructive force-pushes or history rewriting.

### If Remote Storage Purging is Required in the Future:
If the repository owner decides to reclaim remote GitHub LFS storage quota by deleting historical LFS blobs:
1. **Explicit Owner Authorization Required:** Rewriting Git history modifies all subsequent commit hashes and breaks existing cloned repositories.
2. **Migration Tool:** Use `git-filter-repo` or `git lfs migrate` to rewrite past commits.
3. **Purge Remote Objects:** Use the GitHub API or repository settings under **Settings -> Stored Objects** to purge unreferenced LFS objects.

---

## 5. Developer Quick-Start: Cloning and Retrieving Assets

### Standard Clone (Pulls Code Only, Fast)
By default, Git LFS downloads small text pointer files during `git clone`:
```bash
git clone https://github.com/PrathmeshPP/FORGE.git
cd FORGE
```

### Pull All Large Deliverables (Viewer Meshes & Videos)
To fetch all LFS assets (~3.5 GB):
```bash
git lfs install
git lfs pull
```

### Selective Checkout (Save Bandwidth & Disk Space)
If working only on specific scenes or code components, use selective LFS inclusion rules:

#### Option A: Pull Only Web Viewer Baseline Meshes (~690 MB)
```bash
git lfs pull --include="outputs/accuracy_experiment/sprint9_video_to_viewer_release/**"
```

#### Option B: Pull Only Source Videos (~304 MB)
```bash
git lfs pull --include="data/source_videos/**"
```

#### Option C: Pull Only Standalone GLB / Review Packages (~565 MB)
```bash
git lfs pull --include="outputs/manual_model_review/*.zip,outputs/manual_model_review/**/*.glb"
```

---

## 6. How Developers Should Handle New Large Assets

To maintain repository hygiene and avoid accidental large commits:

1. **Verify Before Staging:**
   ```bash
   git check-attr -a path/to/large_file.obj
   ```
   Confirm that `filter: lfs` is reported.
2. **Never Commit Intermediate Caches:**
   Ensure temporary files conform to `.gitignore` rules. Do not force-add (`git add -f`) ignored directories.
3. **Foundation Model Weights Policy:**
   Keep foundation weights (`.pt`, `.safetensors`, `.ckpt`) out of Git. Direct code to read from external paths or environment variables (`VGGT_OMEGA_CHECKPOINT`).
