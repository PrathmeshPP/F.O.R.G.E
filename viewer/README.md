# F.O.R.G.E Explorer — Interactive 3D Reconstruction Platform (Sprint 10)

**F.O.R.G.E Explorer** is an engineering-grade interactive 3D inspection and demonstration web application built with **React 19**, **Vite 8**, **TypeScript**, and **Three.js (r186)**. It allows developers, faculty evaluators, and hackathon judges to interactively inspect, compare, and validate 3D reconstructions produced by the F.O.R.G.E feed-forward photogrammetry pipeline.

---

## 1. Quick Start

### One-Command Windows Launch
From the repository root, double-click or run:
```cmd
start_forge_explorer.bat
```
This launcher:
1. Detects the verified Python environment.
2. Confirms the existence of the Sprint 9 model release assets (`outputs/accuracy_experiment/sprint9_video_to_viewer_release/`).
3. Confirms the production frontend build (`viewer/dist/`).
4. Avoids duplicate server instances on port `5173`.
5. Starts the asset server and opens `http://localhost:5173/` in your default browser.

### Manual Development Server
```bash
cd viewer
npm run dev
```
Open [http://localhost:5173/](http://localhost:5173/) in your web browser.

### Production Build Command
To rebuild the optimized production frontend bundle:
```bash
cd viewer
node ./node_modules/vite/bin/vite.js build
```
The output is written to `viewer/dist/` and served through `scripts/serve_viewer.py`.

---

## 2. Key Capabilities & Visual Design

- **Unified 3-Panel Inspection Layout:**
  - **Left Panel (Scene Library):** Selectable gallery of all 5 benchmark scenes with live status badges (`PASS` / `PARTIAL`), preview thumbnails, and geometry complexity metrics.
  - **Center Panel (Interactive 3D Viewport):** Real WebGL canvas with OrbitControls (orbit, pan, zoom, fit to view, top/front/side/iso view presets, ground grid, coordinate axes, wireframe overlay, and background themes).
  - **Right Panel (Inspection & Provenance):** Live scene metadata, exact bounding-box coordinates, file sizes, verified SHA-256 digests with one-click copy, coordinate status notices, and known limitations.
- **7-Stage Engineering Pipeline Modal (`P` Shortcut):**
  - Interactive walkthrough of the full feed-forward pipeline: **Source Video → Frame Selection → Depth & Camera Inference → Point Cloud Fusion → Surface Mesh Reconstruction → Photographic Texturing → Interactive Inspection**.
  - Includes problem statement comparison against NeRF/3DGS and technical judge methodology disclosures.
- **Side-by-Side Dual Viewport Comparison (`C` Shortcut):**
  - Dedicated comparison mode for **Desert Road Candidate A vs Candidate B**.
  - Synchronized camera motion toggle allowing lockstep rotation, panning, and zoom across both variants to inspect vehicle isolation and seamless asphalt integration.
- **Multiple Representation Formats:**
  - **Textured OBJ (Unified Atlas):** Full material pipeline with `mtllib` and `map_Kd` texture atlas.
  - **Photographic PLY:** High-density mesh with per-vertex RGB photographic colors.
  - **Untextured Geometry:** Pure geometric mesh for inspecting surface curvature and boundary regularity.
- **One-Click Asset Bundle Export:**
  - Direct browser packaging via JSZip: bundles `.obj`, `.mtl`, and `.png` texture atlas with a `BUNDLE_PROVENANCE.json` manifest into a single `.zip` ready for Blender, Unreal Engine, or CloudCompare.
  - Direct download of original unnormalized `.ply` models preserving source coordinates.

---

## 3. Supported Benchmark Scenes

| Scene Identifier | Display Name | Category | Acceptance Status | Geometry Complexity | Active Source Video |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `campus_0049` | Campus 0049 | Architectural Survey | `PARTIAL` *(Hover baseline limitation)* | 128,336 v / 253,487 f | 4K 60fps Drone Video |
| `desert_road_candidate_a` | Desert Road — Candidate A | Highway Infrastructure | `PASS` *(Zero vehicles)* | 53,069 v / 105,467 f | HD 24fps Dashcam Video |
| `desert_road_candidate_b` | Desert Road — Candidate B | Highway Infrastructure | `PASS` *(Zero road box)* | 53,101 v / 105,518 f | HD 24fps Dashcam Video |
| `forest_road` | Forest Road | Terrain & Vegetation | `PASS` *(Canopy voids verified)* | 462,896 v / 919,292 f | HD 24fps Dashcam Video |
| `river_bridge` | River/Bridge | Civil Infrastructure | `PASS` *(Deck & piers verified)* | 132,424 v / 260,685 f | 4K 60fps Drone Video |

---

## 4. Navigation & Keyboard Shortcuts

| Action | Mouse / Trackpad | Shortcut |
| :--- | :--- | :--- |
| **Orbit / Rotate** | Left Click + Drag | — |
| **Pan Viewport** | Right Click + Drag | — |
| **Zoom In / Out** | Scroll Wheel / Pinch | — |
| **Top Preset View** | — | `1` |
| **Front Preset View** | — | `2` |
| **Side Preset View** | — | `3` |
| **Isometric Preset** | — | `4` |
| **Reset Camera** | — | `R` |
| **Toggle Wireframe** | — | `W` |
| **Toggle Ground Grid** | — | `G` |
| **Toggle Dual Comparison** | — | `C` |
| **Pipeline Architecture Modal** | — | `P` |
| **Controls Help Modal** | — | `?` |
