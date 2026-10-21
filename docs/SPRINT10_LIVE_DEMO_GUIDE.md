# F.O.R.G.E — Sprint 10 Live Demonstration Guide & Technical Defense
**Event:** National Hackathon Engineering Demonstration  
**Platform:** F.O.R.G.E Explorer (`http://localhost:5173/`)  
**Baseline Model Freeze:** `outputs/accuracy_experiment/sprint9_video_to_viewer_release/`  
**Launcher Script:** [`start_forge_explorer.bat`](file:///c:/Users/25beevdt025/Desktop/FORGE/start_forge_explorer.bat)  

---

## 1. Five-Minute Judge Demonstration Sequence

| Timestamp | Phase | Screen Action & Demonstration Steps | Talking Points & Key Takeaways |
| :--- | :--- | :--- | :--- |
| **0:00 – 0:45** | **Problem & Hook** | 1. Double-click `start_forge_explorer.bat`.<br>2. Browser opens to F.O.R.G.E Explorer.<br>3. Click **"Pipeline Architecture"** in the top navigation bar. | *"Welcome judges. F.O.R.G.E solves a fundamental bottleneck in spatial computing: traditional photogrammetry (COLMAP) takes hours, and neural representations (NeRF/3DGS) require dense per-scene optimization that produces non-standard volumetric representations. F.O.R.G.E executes a feed-forward transformer foundation model that turns everyday consumer video into standard, textured 3D meshes in under 45 seconds without per-scene training."* |
| **0:45 – 1:45** | **Architectural Survey (Campus 0049)** | 1. Select **Campus 0049** card.<br>2. Orbit using Left-Click drag.<br>3. Press `W` to show wireframe, then `W` again.<br>4. Switch representation to **Untextured Geometric PLY**. | *"This is an aerial drone survey of an institutional campus. Notice the high-fidelity reconstruction of building roofs, walkways, reflecting pond, and surrounding trees. We explicitly tag this scene as 'Conditional PARTIAL' because drone hover telemetry (<0.10m translation) provides near-zero horizontal baseline for vertical facades behind rooflines. Rather than fabricating or hallucinating unseen geometry, our pipeline preserves genuine observations and discloses physical constraints honestly."* |
| **1:45 – 2:45** | **Dynamic Objects (Desert Road A vs B)** | 1. Click **"Compare A vs B"** in header (or press `C`).<br>2. Orbit the synchronized dual view.<br>3. Inspect left corridor (Cand A), then zoom in on the right vehicle (Cand B). | *"Here we demonstrate how F.O.R.G.E handles dynamic obstacles on a highway. In Candidate A (left), moving vehicles, shadows, and balloons were segmented out across all frames, regularized with 30 Taubin smoothing iterations to create a clean, seamless highway corridor with 0 interior holes. In Candidate B (right), rather than leaving an artificial rectangular road step, our pixel-accurate GrabCut instance mask preserves the exact volume and ground shadow of a single reference Mini Cooper at [0.03, 0.04, 0.45], seamlessly integrated into the road plane."* |
| **2:45 – 3:30** | **Complex Topography (Forest & River)** | 1. Click **"Single View"**.<br>2. Select **Forest Road**, zoom downward through the canopy.<br>3. Select **River/Bridge**, orbit around bridge deck and piers. | *"Forest Road validates organic vegetation: tree crowns form closed manifold surfaces while authentic open-air voids are preserved without artificial hole-filling. The downward view clearly reveals asphalt and crisp double yellow lines. In River/Bridge, our targeted pinhole camera frames the concrete deck, railings, and vertical piers at [1.131, 0.166, 0.526], while specular river reflections are cleanly filtered during fusion to avoid false water geometry."* |
| **3:30 – 4:15** | **Technical Inspection & Provenance** | 1. Inspect the right-hand **Inspection & Provenance** panel.<br>2. Highlight Bounding Box spans ($X, Y, Z$) and SHA-256 hash.<br>3. Click the copy icon next to the SHA-256 hash. | *"Every model in F.O.R.G.E Explorer is traceable directly to source video frames. The inspection panel displays real-time bounding box extents in model space, vertex and face counts, and genuine 64-character SHA-256 digests recomputed directly from disk. We make no unverified claims of physical millimeter scale or geospatial coordinates—all dimensions are clearly documented in model-relative coordinate frames."* |
| **4:15 – 5:00** | **Interoperability & Export Bundle** | 1. In Inspection panel, click **"Download Textured Bundle (.zip)"**.<br>2. Show downloaded ZIP containing `.obj`, `.mtl`, `texture_atlas.png`, and `BUNDLE_PROVENANCE.json`. | *"Unlike NeRFs, F.O.R.G.E delivers standard, production-ready assets. With one click, judges can export the complete textured package into Blender, Unreal Engine, Unity, or CloudCompare with intact UV coordinates and original geometry. This is an end-to-end, reproducible spatial intelligence pipeline."* |

---

## 2. Technical Defense & Judge Q&A Guide

### Q1: How are monocular depth and camera poses inferred without stereo calibration?
**Defensible Answer:**
> *"We utilize the VGGT-Ω 1B vision foundation model architecture. By processing video keyframes through alternating spatial and temporal self-attention transformer layers, the model estimates both relative metric-scale depth maps and pairwise camera extrinsics $[R \mid t]$ simultaneously. This eliminates the need for manual feature matching (SIFT) or known camera intrinsics, achieving inference across 8–10 frames in under 25 seconds on a consumer GPU (NVIDIA RTX 2070 SUPER)."*

### Q2: How do you go from depth maps to clean, watertight triangle meshes?
**Defensible Answer:**
> *"The depth maps are backprojected into camera space using the estimated intrinsics. Points from all keyframes are concatenated and voxel-downsampled using a hybrid KD-Tree with normal estimation. We then apply Screened Poisson Surface Reconstruction. To resolve high-frequency marching-cubes facet creases without boundary shrinkage, we apply 30 iterations of volume-preserving Taubin smoothing ($\lambda = 0.5$, $\mu = -0.53$). Boundary diagnostics confirm 0 interior holes and minimal edge jumps."*

### Q3: How does photographic texture projection avoid ghosting and multi-camera seamlines?
**Defensible Answer:**
> *"Standard multi-view texturing often suffers from visible diagonal seams where adjacent cameras overlap. We implemented a separable raised-cosine windowing function that feathers image borders ($270\text{px}$ bottom, $200\text{px}$ margins) combined with per-camera RGB gain compensation. Faces are assigned camera visibility based on the ray-normal dot product $\cos\theta$ penalized by distance, and textures are baked into a single unified 4096² texture atlas (`texture_atlas.png`)."*

### Q4: How are moving objects handled between frames?
**Defensible Answer:**
> *"Moving objects violate the static multi-view photogrammetric assumption. In Desert Road Candidate A, dynamic objects (moving vehicle and rooftop balloons) were segmented out across frames, allowing background road pixels from preceding/succeeding frames to fill the occluded asphalt. In Candidate B, we preserved one reference vehicle from Frame 0 using GrabCut shape-aware instance segmentation + shadow masking. Surrounding frames 1..7 project onto the static road, eliminating the artificial rectangular step present in earlier naive crops."*

### Q5: What are the fundamental limits of the reconstruction quality?
**Defensible Answer:**
> *"We identify three physical limitations and document them transparently:
> 1. **Baseline Parallax:** A hovering drone (Campus 0049) with $<0.10\text{m}$ lateral translation cannot mathematically triangulate vertical facades occluded by roof overhangs.
> 2. **Scale Ambiguity:** Monocular vision cannot determine absolute physical scale without an external reference (e.g., ground control point or known camera baseline). We report model-space units honestly.
> 3. **Specular Reflectance:** Water surfaces (River Bridge) violate Lambertian reflectance assumptions; we filter high-variance specular depth returns to protect solid bridge piers."*

### Q6: What quantitative evidence supports the geometric accuracy?
**Defensible Answer:**
> *"We provide four tiers of verifiable evidence:
> 1. **Boundary Loop Diagnostics:** Automated edge-topology analysis confirms zero interior topological holes in all road corridors.
> 2. **Multi-View Geometric Cross-Validation:** Side-by-side renders (`baseline_vs_new.png`) from identical calibrated virtual pinholes demonstrate consistent road alignment and absence of facet artifacts.
> 3. **Cryptographic Provenance:** Every asset referenced in `model-manifest.json` matches genuine 64-character SHA-256 disk hashes verified via automated pytest suites.
> 4. **Live Browser Rendering:** All 5 models load in real WebGL viewports with zero console errors and full 60 FPS orbit interaction."*

### Q7: How would this architecture scale to longer videos or city-scale corridors?
**Defensible Answer:**
> *"For longer sequences, the pipeline uses a sliding-window temporal keyframe selector with pose graph optimization (PGO) or global bundle adjustment across window boundaries. By processing local sub-maps in feed-forward chunks and aligning them via trajectory continuity, F.O.R.G.E scales linearly $O(N)$ with video duration rather than the quadratic $O(N^2)$ complexity of traditional global bundle adjustment."*

---

## 3. Keyboard Shortcut Quick Reference Card

| Key | Function | Presentation Purpose |
| :---: | :--- | :--- |
| `1` | **Top View** | Demonstrates corridor width, road alignment, and ground markings. |
| `2` | **Front View** | Shows vertical height profile and horizon separation. |
| `3` | **Side View** | Displays road gradient and structural pier elevation. |
| `4` | **Isometric View** | Ideal default presentation angle showcasing 3D depth. |
| `R` | **Reset Camera** | Re-centers and fits the model within the viewport. |
| `W` | **Toggle Wireframe** | Proves genuine triangle mesh density and regularized topology. |
| `G` | **Toggle Ground Grid** | Provides spatial grounding in model space. |
| `C` | **Toggle Comparison** | Instantly switches between Desert Road A/B dual-view and single-view. |
| `P` | **Pipeline Architecture** | Opens 7-stage workflow diagram and technical judge disclosures. |
| `?` | **Help Modal** | Displays full navigation gestures and shortcut summary. |
