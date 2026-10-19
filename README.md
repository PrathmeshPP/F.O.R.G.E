# F.O.R.G.E — Feed-forward Optical Reconstruction & Geometry Engine

F.O.R.G.E is a spatial intelligence and 3D scene reconstruction pipeline built on feed-forward foundation models (VGGT-Ω). It processes monocular aerial/drone video sequences into metric 3D point clouds, dense depth maps, and camera trajectory poses.

---

## Architecture & Current Pipeline

The system is organized into modular phases:

1. **Video Ingestion & Frame Extraction (`src/video/extract_frames.py`)**:
   - Samples ordered RGB frames from video inputs at configurable FPS and frame caps.
   - Embeds millisecond timestamps into file naming and extracted metadata.

2. **Spatial Inference Engine (`src/vggt/inference.py`)**:
   - Loads the local VGGT-Ω 1B foundation model (`vggt_omega_1b_512.pt`) into GPU memory.
   - Executes FP16-autocast forward passes with `torch.inference_mode()`.
   - Predicts per-frame metric depth, depth confidence, and decodes $[R \mid T]$ extrinsics and $K$ intrinsics.

3. **Dense 3D Reconstruction (`src/reconstruction/point_cloud.py`)**:
   - Vectorized pinhole backprojection from metric depth into camera space:
     $$X = (u - c_x) \cdot \frac{Z}{f_x}, \quad Y = (v - c_y) \cdot \frac{Z}{f_y}, \quad Z = \text{depth}$$
   - Transforms camera coordinates into global world space using camera extrinsics:
     $$p_{\text{world}} = R^T \cdot (p_{\text{cam}} - T)$$
   - Samples matched RGB pixel colors and filters invalid/low-confidence points.
   - Exports high-performance binary little-endian PLY files and bounding-box metadata.

4. **Visual Quality Assurance (`scripts/preview_point_cloud.py`)**:
   - Renders multi-view orthographic projections (X-Z Top-Down, X-Y Frontal, Z-Y Side Profile) using native NumPy and Pillow.
   - Preserves true metric aspect ratios for spatial geometry verification.

5. **Camera Trajectory & Parallax Diagnostics (`scripts/analyze_trajectory_quality.py`, `scripts/screen_candidate_videos.py`)**:
   - Analyzes predicted camera poses, translational baseline vs. angular rotation, depth consistency, and parallax dispersion.
   - Screens prospective video candidates to ensure footage possesses adequate baseline for multi-view stereo reconstruction.

6. **Surface Mesh Reconstruction & Presentation (`scripts/reconstruct_mesh.py`)**:
   - Computes watertight Poisson surface meshes from dense point clouds with strict distance support and edge-length pruning.
   - Restores authentic source RGB colors to mesh vertices via KD-tree inverse-distance-weighted interpolation.
   - Creates a separate, neutral rectangular exhibition base for professional presentation without fabricating terrain or altering spatial measurements.

7. **Geospatial Alignment & Telemetry Engine (`src/geo/`, `scripts/align_reconstruction_gps.py`)**:
   - Extracts onboard drone GPS telemetry from embedded metadata (`djmd_protobuf`) and companion subtitle tracks (`.SRT`).
   - Projects WGS-84 geodetic coordinates to local metric East-North-Up (ENU) coordinates.
   - Performs trajectory baseline conditioning analysis, detecting ill-conditioned stationary hovers against the GNSS noise floor.
   - Estimates optimal 7-DoF similarity transformations (scale, 3D rotation, 3D translation) via robust Umeyama–RANSAC alignment.
   - Transforms 3D geometry and camera trajectories into georeferenced metric coordinate space.

8. **Automated Quality Gates & Verification (`src/geo/quality_gates.py`)**:
   - Evaluates multi-criteria geometric, topological, kinematic, and geospatial acceptance gates.
   - Enforces vertex density, triangle completeness, aspect-ratio needle outlier limits, and topological clustering.
   - Detects 1D collinear hover orientation defects ($R_{3,3} > 0.85$ optical tilt into sky).

---

## Repository Structure

```
FORGE/
 ├── run_pipeline.py                 # End-to-end video-to-predictions pipeline
 ├── requirements.txt                # Core pipeline dependency specification
 ├── requirements-viewer.txt         # Open3D 3D viewer dependency specification
 ├── src/
 │    ├── video/                     # Frame extraction, temporal sampling, and adaptive selection
 │    ├── vggt/                      # VGGT-Omega loading, inference, and camera decoding
 │    ├── reconstruction/            # Depth backprojection, voxel fusion, and generalized pipeline
 │    └── geo/                       # Telemetry extraction, Umeyama-RANSAC, and quality gates
 ├── scripts/
 │    ├── align_reconstruction_gps.py # Dataset-wide batch GPS alignment runner
 │    ├── generate_contact_sheets.py # Multi-view diagnostic contact sheet generator
 │    ├── generate_point_cloud.py    # CLI point cloud generation runner
 │    ├── preview_point_cloud.py     # Dependency-free orthographic QA previewer
 │    ├── view_point_cloud.py        # Interactive Open3D 3D point cloud visualizer
 │    ├── analyze_trajectory_quality.py # Trajectory, parallax, and baseline diagnostic tool
 │    ├── screen_candidate_videos.py # Drone video candidate screening & motion analysis
 │    ├── reconstruct_mesh.py        # Surface mesh reconstruction & presentation base
 │    ├── audit_accuracy.py          # Geometric accuracy audit & multi-view depth agreement
 │    └── render_model_comparisons.py # Offscreen multi-angle comparative rendering diagnostic
 ├── tests/
 │    ├── test_vggt_loading.py       # Model loading and GPU smoke test
 │    ├── test_geo_alignment.py      # Automated tests for telemetry, Umeyama-RANSAC & quality gates
 │    ├── test_general_pipeline.py   # General reconstruction pipeline automated tests
 │    └── test_point_cloud_accuracy.py # Point cloud fusion and backprojection tests
 └── README.md
```

---

## Setup & Prerequisites

- **OS**: Windows 11 / Linux (tested on Windows 11)
- **Python**: 3.11.9
- **GPU**: NVIDIA GPU with CUDA support (tested on RTX 2070 SUPER, 8 GB VRAM)
- **Core Dependencies**:
  - `torch==2.8.0+cu126`, `torchvision==0.23.0+cu126`
  - `numpy==1.26.4` (required by VGGT-Ω; `numpy < 2`)
  - `opencv-python==4.11.0.86`, `Pillow`, `einops`, `safetensors`
  - `vggt-omega` local package
- **Interactive Viewer Dependency**:
  - `open3d==0.20.0` (listed in `requirements-viewer.txt`)

### Checkpoint Configuration

Set the path to your local VGGT-Ω model checkpoint via environment variable or CLI argument:

```bash
# Windows PowerShell
$env:VGGT_OMEGA_CHECKPOINT = "C:\path\to\vggt_omega_1b_512.pt"

# Linux / macOS
export VGGT_OMEGA_CHECKPOINT="/path/to/vggt_omega_1b_512.pt"
```

If not set, the pipeline defaults to the local development checkpoint path or accepts `--checkpoint <path>`.

---

## Usage

### 1. Run Video Inference Pipeline
Extract frames, execute VGGT-Ω spatial inference, and save metric depth and camera poses:

```bash
python run_pipeline.py --video path/to/input.mp4 --fps 1.0 --max-frames 3 --output-dir outputs/run_01
```

### 2. Generate 3D Point Cloud
Backproject predicted depth maps into a colored PLY point cloud:

```bash
python scripts/generate_point_cloud.py \
    --depth outputs/run_01/predictions/depth_predictions.npz \
    --poses outputs/run_01/predictions/camera_poses.npz \
    --frames outputs/run_01/frames \
    --output-ply outputs/run_01/reconstruction/point_cloud.ply \
    --output-meta outputs/run_01/reconstruction/reconstruction_metadata.json
```

### 3. Render Visual QA Preview
Render top-down, frontal, and profile projections to verify reconstruction geometry:

```bash
python scripts/preview_point_cloud.py \
    --input-ply outputs/run_01/reconstruction/point_cloud.ply \
    --output-png outputs/run_01/reconstruction/point_cloud_preview.png
```

### 4. Interactive 3D Viewer (Open3D)
Launch the interactive 3D viewer window with RGB vertex colors and coordinate axes:

```bash
python scripts/view_point_cloud.py --input outputs/run_01/reconstruction/point_cloud.ply
```

Viewer options:
- Mouse controls: Left-click drag (Orbit/Rotate), Right-click drag / Shift+Left drag (Pan), Scroll wheel (Zoom), `R` (Reset view), `Q` / `Esc` (Close).
- Non-destructive downsampling for faster rendering (never alters original file):
  ```bash
  python scripts/view_point_cloud.py --input outputs/run_01/reconstruction/point_cloud.ply --voxel-size 0.05
  ```
- Integrity validation without launching GUI:
  ```bash
  python scripts/view_point_cloud.py --input outputs/run_01/reconstruction/point_cloud.ply --verify-only
  ```

### 5. Trajectory & Parallax Quality Diagnostics
Analyze predicted camera poses, translational baseline, and depth consistency:

```bash
python scripts/analyze_trajectory_quality.py \
    --poses outputs/run_01/predictions/camera_poses.npz \
    --depth outputs/run_01/predictions/depth_predictions.npz \
    --output-json outputs/run_01/predictions/trajectory_quality.json
```

### 6. Drone Video Candidate Screening
Evaluate candidate drone videos or segments for translation vs. pure rotation and parallax dispersion before running full inference:

```bash
python scripts/screen_candidate_videos.py
```

### 7. Surface Mesh Reconstruction & Rectangular Presentation
Generate a watertight surface mesh with authentic KD-tree vertex color interpolation and an optional neutral rectangular display plinth:

```bash
python scripts/reconstruct_mesh.py \
    --input outputs/run_01/reconstruction/point_cloud.ply \
    --output-dir outputs/run_01/reconstruction \
    --voxel-size 0.01 \
    --poisson-depth 9 \
    --max-support-dist 0.02 \
    --max-edge-length 0.05 \
    --add-presentation-base
```

View the reconstructed presentation mesh directly in Open3D:

```bash
python scripts/reconstruct_mesh.py --view-only --output-dir outputs/run_01/reconstruction
```

### 8. Geometric Accuracy Audit & Multi-View Validation
Perform quantitative multi-view depth agreement, ray intersection angle, and coordinate convention verification:

```bash
python scripts/audit_accuracy.py
```

Generate consistent offscreen multi-angle comparative rendering sheets (Top, Frontal, Oblique) for baseline models and point clouds:

```bash
python scripts/render_model_comparisons.py
```

### 9. Validation Baselines & Metric-Aligned Models

The repository tracks validated baseline models, repaired candidates, and execution metadata:
- **River Bridge Flight Corridor (`outputs/accuracy_experiment/gps_aligned_v1/river_bridge_flight_corridor/`)**: Georeferenced metric model (`surface_mesh_gps_aligned.ply`, 323k vertices, 637k triangles) solved across 84.8 m active forward flight corridor, achieving 100% inlier alignment at 0.236 m RMSE.
- **Campus 0049 Baseline & Repaired Models (`outputs/dji_0049_validation/`, `outputs/accuracy_experiment/generalization_disaster_validation_v1/campus_0049/`)**: Authentic colored surface mesh baselines, visual QA previews, camera poses, and execution metadata.
- **River / Bridge Baseline (`outputs/dji_river_bridge_validation/`)**: Validated forward-translation corridor exhibiting clean multi-view parallax (`mesh_experiment_v2/surface_mesh.ply`).
- **Audit & Alignment Evidence (`outputs/accuracy_experiment/gps_aligned_v1/`)**: Contains consolidated `batch_alignment_report.json`, `batch_alignment_summary.csv`, and visual comparison sheets.

### 10. Robust GPS Telemetry Alignment Runner
Perform automated trajectory baseline conditioning, Umeyama-RANSAC 7-DoF alignment, and quality gate assessment across all project videos:

```bash
python scripts/align_reconstruction_gps.py --all
```

### 11. Diagnostic Comparison Contact Sheet Generator
Generate high-resolution multi-view comparison sheets comparing original baselines, repaired candidates, and metric models:

```bash
python scripts/generate_contact_sheets.py
```

### 12. Run Automated Test Suite
Run the full test suite covering synthetic 7-DoF recovery, RANSAC outlier rejection, trajectory conditioning, quality gates, and reconstruction pipelines:

```bash
python -m unittest discover tests
```
