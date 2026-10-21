# F.O.R.G.E — Standalone 3D Model Catalog (Sprint 10.2)

**Release Directory:** [`outputs/manual_model_review/`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/)  
**Master Inspection Archive:** [`outputs/manual_model_review/forge_models_for_manual_inspection.zip`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forge_models_for_manual_inspection.zip)  
**Package Size:** 354.20 MB (371,407,846 bytes)  
**Package SHA-256:** `65f84bfe2950e1ae535b5c60478863d7174229310362357fdd120a96511404f0`  
**Date of Verification:** 2026-10-21 13:28:10 UTC  

---

## Overview

This catalog provides independent, inspectable 3D model deliverables derived from the four benchmark video datasets. All models have been verified to contain nonempty geometric meshes with finite coordinates, valid materials, and resolving texture paths. They are fully usable outside the F.O.R.G.E Explorer website in industry-standard tools like Blender, MeshLab, CloudCompare, and Windows 3D Viewer.

---

## 1. Scene Index & Delivered Formats

Every scene directory contains the complete set of required deliverables:
1. **Untextured 3D Mesh (`model_geometry.ply` & `model_geometry.obj`):** Clean repaired geometry without vertex colors or material dependencies.
2. **Photographic 3D Mesh (`model_photographic.ply`):** Full geometric mesh with vertex colors (RGB) derived directly from multi-view video projection.
3. **Textured 3D Model (`model_textured.obj`, `model_textured.mtl`, `texture_atlas.png`):** Wavefront OBJ mesh with UV coordinates and 4096x4096 texture atlas map.
4. **Self-Contained Viewing Format (`model_preview.glb`):** Single-file binary glTF 2.0 with embedded PBR materials and textures for instant one-click preview.
5. **Verified Renders & Metadata (`render_overview.png`, `render_oblique.png`, `metadata.json`):** Offscreen inspection renders and full provenance telemetry.

---

## Scene: Campus 0049

- **Scene Key:** `campus_0049`
- **Category:** Architectural Survey
- **Source Video:** `dji_fly_20260727_172754_0049_1785175051818_video.mp4` (3840x2160, 59.94 fps, 966 frames)
- **Video SHA-256:** `7cf250cfb4dd90e6388b5dd2d3d146209cb0c3bbff8ba1dc2d123b7e5d9c3c6a`
- **Unique Vertices:** **128,336**
- **Faces / Triangles:** **253,487**
- **Rendered Position Entries (OBJ):** **760,461**
- **Bounding Box Range:**
  - $X \in [-2.519, 2.794]$ (Span: 5.314)
  - $Y \in [-0.572, 3.230]$ (Span: 3.802)
  - $Z \in [0.568, 0.917]$ (Span: 0.349)
- **Coordinate Status:** Model-relative coordinate frame (arbitrary units). Metric scale uncalibrated. Not georeferenced.
- **Notice / Notes:** Aerial institutional campus survey with buildings, courtyard walkways, reflecting pond, and landscaping. Preserves physical drone hovering telemetry baseline (< 0.10m translation); unseen building facades are honestly not synthesized.

### Delivered Artifacts

| Filename | Description | File Size | SHA-256 Hash |
| :--- | :--- | :--- | :--- |
| [`model_geometry.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_geometry.ply) | Untextured 3D Mesh (Polygon File Format, binary LE) | 9,840,793 B | `adb753a0a5d7201504ed9ff493a5174cb879cc230182b8a13b3905563985fceb` |
| [`model_geometry.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_geometry.obj) | Untextured 3D Mesh (Wavefront OBJ, pure geometry) | 18,328,044 B | `1daef6e7f552a580be47a55cdea00b968bb056138384f283f96c163b6fcde92f` |
| [`model_photographic.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_photographic.ply) | Photographic 3D Mesh (Vertex Colors RGB, binary LE) | 9,840,793 B | `143a0e9d21b543ec9863dfcede67b0818b277f70d2c2d3d0387014d01533308f` |
| [`model_textured.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_textured.obj) | Textured 3D Model (Wavefront OBJ, UV mapped) | 31,281,377 B | `d34f2b01280942da8741c93a3f9952d7ac498021b1cb436bb4d3e78730d649fc` |
| [`model_textured.mtl`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_textured.mtl) | Material Library Definition (MTL) | 176 B | `c32e0bf6cdd60037a49ec7c3075e930e3b748657266802e7512363c14f6993cf` |
| [`texture_atlas.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/texture_atlas.png) | Texture Atlas Map (4096x4096 PNG) | 29,301,787 B | `35b3ebe60fdcc893162d5e68c6c147afe5498a3dda5df9bd9dc9c55fdd7c3fd3` |
| [`model_preview.glb`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/model_preview.glb) | Self-Contained 3D Model (glTF 2.0 Binary with embedded PBR textures) | 48,372,492 B | `33cc62d81833c5f74a88cb6436a18e5e2434193c82ffa76b2480da68b254827e` |
| [`render_overview.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/render_overview.png) | Offscreen Render — Overhead Overview | 214,768 B | `aeafa272011f99f2b00154bdea247c5b1fd6b3f76eaf4c0b014915e895db7e3d` |
| [`render_oblique.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/campus_0049/render_oblique.png) | Offscreen Render — Oblique Perspective | 179,041 B | `3d91dac9bf5ba4ac2b9d31bbf0065e41085b59748e2f2a3d1e0333b59b62db72` |

---

## Scene: Desert Road — Candidate A

- **Scene Key:** `desert_road_candidate_a`
- **Category:** Highway Infrastructure
- **Source Video:** `desert_road.mp4` (1280x720, 23.976 fps, 477 frames)
- **Video SHA-256:** `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`
- **Unique Vertices:** **53,069**
- **Faces / Triangles:** **105,467**
- **Rendered Position Entries (OBJ):** **316,401**
- **Bounding Box Range:**
  - $X \in [-0.655, 0.550]$ (Span: 1.205)
  - $Y \in [-0.047, 0.233]$ (Span: 0.281)
  - $Z \in [-2.411, 2.235]$ (Span: 4.647)
- **Coordinate Status:** Camera/model-relative coordinates. Road plane fitted at Y = -0.019475*X - 0.057574*Z + 0.088181. Metric scale uncalibrated.
- **Notice / Notes:** Straight two-lane desert highway corridor across dunes with moving vehicles, shadows, and balloons removed. Clean regular shoulders smoothed with 30 Taubin iterations and pristine asphalt roadway.

### Delivered Artifacts

| Filename | Description | File Size | SHA-256 Hash |
| :--- | :--- | :--- | :--- |
| [`model_geometry.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_geometry.ply) | Untextured 3D Mesh (Polygon File Format, binary LE) | 4,077,915 B | `b442150ca2d5a4b1d196c8f83d9f36b2aa330a7b815b708c9a1a17f177259a11` |
| [`model_geometry.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_geometry.obj) | Untextured 3D Mesh (Wavefront OBJ, pure geometry) | 7,631,490 B | `0b197d6a16d35ef72d626f869cde15d2bab9079a2cac34aabc418a551fea49e9` |
| [`model_photographic.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_photographic.ply) | Photographic 3D Mesh (Vertex Colors RGB, binary LE) | 4,077,915 B | `5625a5a2436abd6bca780b86efde8687246675f0e47b270f37440b006099ae44` |
| [`model_textured.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_textured.obj) | Textured 3D Model (Wavefront OBJ, UV mapped) | 14,301,215 B | `fdc5d57a65f7113191f21f2ba7084e540a6bd9b5b7dde0755ec366cbe64ecff2` |
| [`model_textured.mtl`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_textured.mtl) | Material Library Definition (MTL) | 175 B | `9768a160330c54d7a314081360d0c164902303a256327653c3c8fd6049edcc33` |
| [`texture_atlas.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/texture_atlas.png) | Texture Atlas Map (4096x4096 PNG) | 8,765,408 B | `8c4b67a346cb80d7ac3be8af2a2724f31334538509fd6b08b9f7f89484ca803d` |
| [`model_preview.glb`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/model_preview.glb) | Self-Contained 3D Model (glTF 2.0 Binary with embedded PBR textures) | 20,157,424 B | `dd7c799df2f227b4eb4f302c867e8b148e8340208e3325e56913f8b42fe2a80f` |
| [`render_overview.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/render_overview.png) | Offscreen Render — Overhead Overview | 67,561 B | `578a5ce742efa8edd62039a3ea564e4d06c3454e7ca80c15a47a6cba7f5a1e13` |
| [`render_oblique.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_a/render_oblique.png) | Offscreen Render — Oblique Perspective | 61,430 B | `afaff77a0c110ebdcc87da9fd349a5959cebb4aa2fca7c54e292a3a640b49542` |

---

## Scene: Desert Road — Candidate B

- **Scene Key:** `desert_road_candidate_b`
- **Category:** Highway Infrastructure
- **Source Video:** `desert_road.mp4` (1280x720, 23.976 fps, 477 frames)
- **Video SHA-256:** `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`
- **Unique Vertices:** **53,101**
- **Faces / Triangles:** **105,518**
- **Rendered Position Entries (OBJ):** **316,554**
- **Bounding Box Range:**
  - $X \in [-0.656, 0.548]$ (Span: 1.203)
  - $Y \in [-0.047, 0.233]$ (Span: 0.280)
  - $Z \in [-2.408, 2.236]$ (Span: 4.644)
- **Coordinate Status:** Camera/model-relative coordinates. Vehicle elevated at reference coordinate [0.03, 0.04, 0.45]. Metric scale uncalibrated.
- **Notice / Notes:** Highway corridor preserving a single reference vehicle (Mini Cooper with rooftop balloons) from Frame 0. Shape-aware instance mask integration with zero rectangular road box and seamless planar road continuity.

### Delivered Artifacts

| Filename | Description | File Size | SHA-256 Hash |
| :--- | :--- | :--- | :--- |
| [`model_geometry.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_geometry.ply) | Untextured 3D Mesh (Polygon File Format, binary LE) | 4,080,210 B | `3088a560227c9b821b289ac7c4ec3a23ccacda3d6d056c84d7b56a955828f185` |
| [`model_geometry.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_geometry.obj) | Untextured 3D Mesh (Wavefront OBJ, pure geometry) | 7,635,396 B | `d4e936326204f2773fe40fa19fdba496a36ac03828ebf26edae008657ac5dba0` |
| [`model_photographic.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_photographic.ply) | Photographic 3D Mesh (Vertex Colors RGB, binary LE) | 4,080,210 B | `674ad0ec3d0b40a8fdb9f0680301912a0d7e3a0552c4ef3526740e285f053edf` |
| [`model_textured.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_textured.obj) | Textured 3D Model (Wavefront OBJ, UV mapped) | 14,308,563 B | `dc3607d35cb3469debff597cfc1ea3f9d5ac62d3a90e982940d6f8c17c32fe1a` |
| [`model_textured.mtl`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_textured.mtl) | Material Library Definition (MTL) | 175 B | `9768a160330c54d7a314081360d0c164902303a256327653c3c8fd6049edcc33` |
| [`texture_atlas.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/texture_atlas.png) | Texture Atlas Map (4096x4096 PNG) | 8,765,408 B | `8c4b67a346cb80d7ac3be8af2a2724f31334538509fd6b08b9f7f89484ca803d` |
| [`model_preview.glb`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/model_preview.glb) | Self-Contained 3D Model (glTF 2.0 Binary with embedded PBR textures) | 20,162,884 B | `750a30ae96ae09a08272f29d46a9daa882714b62d2e41e48e9413969e0eb5fa5` |
| [`render_overview.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/render_overview.png) | Offscreen Render — Overhead Overview | 68,739 B | `e429660d4961b8385c1c38b7fd1df70dd0abcd5cc72e060bda7d5aa3330d09ed` |
| [`render_oblique.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/desert_road_candidate_b/render_oblique.png) | Offscreen Render — Oblique Perspective | 61,635 B | `b34df575517024d17bf04495b704db37bbb82422b2added9c8ed85faaed99239` |

---

## Scene: Forest Road

- **Scene Key:** `forest_road`
- **Category:** Terrain & Vegetation
- **Source Video:** `forest_road.mp4` (1280x720, 23.976 fps, 730 frames)
- **Video SHA-256:** `e5e847d9b3863d29c12501347074c58c6cc4e68fb91f4c963bbe3f91294a169a`
- **Unique Vertices:** **462,896**
- **Faces / Triangles:** **919,292**
- **Rendered Position Entries (OBJ):** **2,757,876**
- **Bounding Box Range:**
  - $X \in [-0.883, 0.801]$ (Span: 1.685)
  - $Y \in [-0.507, 0.430]$ (Span: 0.937)
  - $Z \in [0.759, 1.230]$ (Span: 0.471)
- **Coordinate Status:** Model-relative coordinates. Metric scale uncalibrated.
- **Notice / Notes:** Dense mountain forest canopy with winding two-lane paved roadway and double yellow lane markings. Validated closed tree-crown manifold structures with natural open-air canopy voids.

### Delivered Artifacts

| Filename | Description | File Size | SHA-256 Hash |
| :--- | :--- | :--- | :--- |
| [`model_geometry.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_geometry.ply) | Untextured 3D Mesh (Polygon File Format, binary LE) | 35,558,818 B | `1d635dce8c9d023093d4b43fdeb420be4f366fc6d08aa6c4a391121cf03f875c` |
| [`model_geometry.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_geometry.obj) | Untextured 3D Mesh (Wavefront OBJ, pure geometry) | 70,107,867 B | `51d28d50ff566f44d6583a26c44957a77636211f45a101854f9d47d7c1768aa2` |
| [`model_photographic.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_photographic.ply) | Photographic 3D Mesh (Vertex Colors RGB, binary LE) | 35,558,818 B | `1cb5896612a8b1d040b35553ea2000df97d7a3311c59e4e2f42afcdd6c63bfc0` |
| [`model_textured.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_textured.obj) | Textured 3D Model (Wavefront OBJ, UV mapped) | 117,409,996 B | `c36661af14d0a3d164370a436e1cbc8d0f6c1756ec367b20fc2b85ff44bb7628` |
| [`model_textured.mtl`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_textured.mtl) | Material Library Definition (MTL) | 176 B | `c32e0bf6cdd60037a49ec7c3075e930e3b748657266802e7512363c14f6993cf` |
| [`texture_atlas.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/texture_atlas.png) | Texture Atlas Map (4096x4096 PNG) | 25,149,397 B | `c7cc73c3cc954045729402eaf785fbacd4d34e249efb9b019aba01f3fa28dea9` |
| [`model_preview.glb`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/model_preview.glb) | Self-Contained 3D Model (glTF 2.0 Binary with embedded PBR textures) | 92,249,276 B | `125d6a5a20e8abab600fd49b478cc4063f47d95e4aefdcd6d55adc8e036d8faf` |
| [`render_overview.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/render_overview.png) | Offscreen Render — Overhead Overview | 242,504 B | `68e316d1a643bf7113a57173dcaa33c5458cc95075a1043970b7f9a004b12418` |
| [`render_oblique.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/forest_road/render_oblique.png) | Offscreen Render — Oblique Perspective | 231,078 B | `0baa9424a8aad0b8a2a3676a28248206ac5650a5ba7db2af3db6c17d2cdd5eb4` |

---

## Scene: River/Bridge

- **Scene Key:** `river_bridge`
- **Category:** Civil Infrastructure
- **Source Video:** `segment_52s_64s.mp4` (3840x2160, 59.94 fps, 750 frames)
- **Video SHA-256:** `0d1a647a0a0363bed400317a2b3495ad5926d469a5dd7a8b127fee037e7409cb`
- **Unique Vertices:** **132,424**
- **Faces / Triangles:** **260,685**
- **Rendered Position Entries (OBJ):** **782,055**
- **Bounding Box Range:**
  - $X \in [-1.492, 2.408]$ (Span: 3.900)
  - $Y \in [-0.436, 0.304]$ (Span: 0.740)
  - $Z \in [-0.335, 2.749]$ (Span: 3.083)
- **Coordinate Status:** Model-relative coordinate frame. Bridge deck centered at coordinates [1.131, 0.166, 0.526]. Metric scale uncalibrated.
- **Notice / Notes:** Concrete highway bridge deck with guard railings and vertical support piers crossing a riverbed channel with approach roadway and riparian banks. Reflective water surface specular noise attenuated during multi-view fusion.

### Delivered Artifacts

| Filename | Description | File Size | SHA-256 Hash |
| :--- | :--- | :--- | :--- |
| [`model_geometry.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_geometry.ply) | Untextured 3D Mesh (Polygon File Format, binary LE) | 10,142,855 B | `7d049e9e2e680a6c002b362d4f6e7a501fa01896aedfd3cd82922d0a7f7f9be7` |
| [`model_geometry.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_geometry.obj) | Untextured 3D Mesh (Wavefront OBJ, pure geometry) | 19,138,823 B | `0e75db13f508bafcff0f2adebfe903bb081a2a1acdc31ee88645874c73e3fe6a` |
| [`model_photographic.ply`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_photographic.ply) | Photographic 3D Mesh (Vertex Colors RGB, binary LE) | 10,142,855 B | `4be4958e3b81a0a8ea2ca0ad3c7cb18c39e196884065920c5c11abaad9d7758c` |
| [`model_textured.obj`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_textured.obj) | Textured 3D Model (Wavefront OBJ, UV mapped) | 32,270,619 B | `fdde2c03c09bd55db5777bde0bc32392b115326b68ee543c8a5f13ea2d7f4ff6` |
| [`model_textured.mtl`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_textured.mtl) | Material Library Definition (MTL) | 176 B | `c32e0bf6cdd60037a49ec7c3075e930e3b748657266802e7512363c14f6993cf` |
| [`texture_atlas.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/texture_atlas.png) | Texture Atlas Map (4096x4096 PNG) | 20,836,942 B | `ad35dd6447f098b57192b622c9a32af1c7a5f22dd2bab53da0bad54c23ab57c3` |
| [`model_preview.glb`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/model_preview.glb) | Self-Contained 3D Model (glTF 2.0 Binary with embedded PBR textures) | 40,262,068 B | `13db2cfb85f1c69ab7af954a06092dbf7ad606b6e731614793af43f131da1ec2` |
| [`render_overview.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/render_overview.png) | Offscreen Render — Overhead Overview | 69,938 B | `4f49bfa9a51c1428d7de7e41f6c5004d79b1574478e4a0d8ee2df26df2988845` |
| [`render_oblique.png`](file:///C:/Users/25beevdt025/Desktop/FORGE/outputs/manual_model_review/river_bridge/render_oblique.png) | Offscreen Render — Oblique Perspective | 176,360 B | `69d8469759107d3bb663b7e3d92b11629f0bd8cdd7e44e64f143806bcdc1ca0e` |

---

## 2. Step-by-Step Manual Opening Instructions

### A. Windows 3D Viewer (Built into Windows 10/11)
1. Navigate to any scene folder under `outputs/manual_model_review/`.
2. Double-click `model_preview.glb`.
3. The model loads with full PBR shading, realistic lighting, and embedded 4K textures in a native Windows window.
4. Alternatively, open `model_textured.obj` directly.

### B. Blender (v3.x / v4.x)
1. Launch Blender and start a **General** scene. Delete the default cube (`X` -> Delete).
2. **Option 1 (Self-Contained GLB):**
   - Click `File -> Import -> glTF 2.0 (.glb/gltf)`.
   - Select `outputs/manual_model_review/<scene_name>/model_preview.glb`.
   - Switch viewport shading to **Material Preview** (top right sphere icon) or press `Z -> Material Preview`.
   - The model appears instantly with textures and UVs bound.
3. **Option 2 (Textured OBJ):**
   - Click `File -> Import -> Wavefront (.obj)`.
   - Select `outputs/manual_model_review/<scene_name>/model_textured.obj`.
   - Blender automatically reads `model_textured.mtl` and links `texture_atlas.png`.
   - Press `Z -> Material Preview` to inspect textured polygons.
4. **Option 3 (Photographic PLY):**
   - Click `File -> Import -> Stanford (.ply)`.
   - Select `outputs/manual_model_review/<scene_name>/model_photographic.ply`.
   - In viewport shading, select **Color -> Attribute / Vertex Color**.

### C. MeshLab
1. Launch MeshLab.
2. Click `File -> Import Mesh...` (or `Ctrl+I`).
3. Select `model_textured.obj` or `model_photographic.ply` or `model_geometry.ply`.
4. If opening `model_textured.obj`, ensure `texture_atlas.png` resides in the same folder.
5. Inspect triangle wireframes by selecting `Render -> Show Box` or `Render -> Shading -> Flat Wireframe`.

### D. CloudCompare
1. Launch CloudCompare.
2. Click `File -> Open` (or `Ctrl+O`).
3. Select `model_photographic.ply` or `model_geometry.ply`.
4. In the import prompt, accept standard coordinates.
5. The mesh renders with RGB vertex colors and scalar fields intact.

---

## 3. Format Distinctions & Conversion Provenance

- **Source Models (Sprint 9):** All models originate from the frozen Sprint 9 video-to-viewer reconstruction release (`outputs/accuracy_experiment/sprint9_video_to_viewer_release/`).
- **No Coordinate Modification:** All raw spatial coordinates, vertex orders, and triangle normal orientations are strictly preserved from the source models.
- **Untextured OBJ (`model_geometry.obj`):** Created via direct Open3D topological export with cleared vertex colors, ensuring maximum compatibility with CAD and geometry-only modeling software.
- **Self-Contained GLB (`model_preview.glb`):** Created via `trimesh` glTF 2.0 binary packaging, embedding the high-resolution texture atlas directly into the file container.
