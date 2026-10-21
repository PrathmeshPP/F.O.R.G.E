# F.O.R.G.E — Sprint 11 Release Manifest: Recovery V2.1 Models & Dynamic Masking

> **Document Version:** 1.0.0  
> **Release Target:** F.O.R.G.E Sprint 11 Canonical 3D Reconstruction Models  
> **Repository:** [https://github.com/PrathmeshPP/FORGE](https://github.com/PrathmeshPP/FORGE)  
> **Primary Branch:** `main`  
> **Release Date:** October 2026  

---

## 1. Executive Summary & Selected Model Matrix

This release manifest formally catalogs the canonical 3D model deliverables, material definitions, high-resolution texture atlases, diagnostic comparisons, and reproducible generation scripts for the F.O.R.G.E Sprint 11 release.

In accordance with visual inspections, photogrammetric review, and automated regression suites, the canonical model version for each of the four benchmark scenes is:

| Benchmark Scene | Chosen Release Version | Justification & Architectural Findings | Primary Model Formats | Status |
| :--- | :--- | :--- | :--- | :--- |
| **1. Campus 0049** | **Recovery V2.1** | Audited manual-review baseline mesh (128,336 vertices, 253,487 triangles) retained with 4096×4096 texture atlas. Sharp ground lettering ("FIRE LANE", "STOP") and courtyard paving preserved without synthetic hallucination. Smooth facade rounding documented as monocular hovering drone telemetry artifact (< 0.10 m translation baseline). | GLB, OBJ+MTL, PLY, 4K Atlas | **Verified Canonical** |
| **2. Desert Road** | **Recovery V2.1** | Dynamic vehicle & unmasked floating multicolored balloon projections completely excised via dilated YOLOv8n-seg masks and fronto-parallel color projection (residual saturated red balloon vertices: 0 vs 35 in V2). Road surface planar to 2.0 mm mean deviation (0 vehicle bumps). All 8 texture atlas tiles cleanly inpainted. | GLB, OBJ+MTL, PLY, 4K Atlas | **Verified Canonical** |
| **3. Forest Road** | **Recovery V2** | High-fidelity tree canopy and mountain asphalt corridor (445,824 vertices, 889,653 triangles) preserved without unnecessary re-reconstruction. Moving vehicle completely excised (0 residual vehicle points). | GLB, OBJ, PLY | **Verified Canonical** |
| **4. River/Bridge** | **Recovery V2** | Dual-span structural concrete bridge, piers, and riverbanks (127,887 vertices, 251,054 triangles) preserved without unnecessary re-reconstruction. Specular water noise trimmed safely. | GLB, OBJ, PLY | **Verified Canonical** |

---

## 2. Exhaustive Per-Scene Deliverable Inventory

### 2.1 Campus 0049 — Recovery V2.1 (Audited Reference & 4K Atlas)
- **Directory:** `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/`
- **Topology:** 128,336 vertices, 253,487 triangles
- **Material Dependency:** `masked_reconstruction.obj` declares `mtllib masked_reconstruction.mtl` and binds `usemtl Material_Atlas` referencing `map_Kd texture_atlas.png`.

| Repository-Relative Path | Artifact Role | File Size | SHA-256 Checksum | Storage Tier |
| :--- | :--- | :--- | :--- | :--- |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.glb` | Standalone glTF 2.0 Binary Model (embedded 4K texture) | 48,372,492 bytes (46.13 MB) | `33cc62d81833c5f74a88cb6436a18e5e2434193c82ffa76b2480da68b254827e` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.obj` | Wavefront OBJ Polygonal Mesh (UV mapped) | 31,281,384 bytes (29.83 MB) | `fa18c9e2a3380e2bc1d4b38db750355ac70c3688ca163607846c267dc34b2fdf` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.mtl` | Wavefront MTL Material Library | 176 bytes (0.00 MB) | `c32e0bf6cdd60037a49ec7c3075e930e3b748657266802e7512363c14f6993cf` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.ply` | Stanford PLY Coordinate Mesh (photographic vertex colors) | 9,840,793 bytes (9.38 MB) | `143a0e9d21b543ec9863dfcede67b0818b277f70d2c2d3d0387014d01533308f` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/texture_atlas.png` | 4096×4096 High-Resolution Diffuse Texture Atlas | 29,301,787 bytes (27.94 MB) | `35b3ebe60fdcc893162d5e68c6c147afe5498a3dda5df9bd9dc9c55fdd7c3fd3` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction_preview.png` | 3-Panel Inspection Preview (Textured / Nadir / Detail) | 298,822 bytes (0.28 MB) | `aa972aff7c54bfc8fa04116d340d86c7fbb960ec6ba9a829dfd2719c4dfbb3b7` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/neutral_geometry_preview.png` | Neutral Grey Material Render (Geometry Integrity) | 105,465 bytes (0.10 MB) | `a02bfffe75a679518c1dd4edeaf47b4096b2c32ca0e9b51ee75e2ef833152e1b` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/texture_detail_preview.png` | Close-Up Texture Detail Render (Courtyard & Lettering) | 1,227,103 bytes (1.17 MB) | `31e2690078f85c790a6cec5b5179ff3ce7f54a921befd1dc5d5f5ddab6f4e317` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/campus_geometry_vs_texture_audit.png` | 4-Zone Side-by-Side Audit Sheet (Textured vs Neutral) | 4,434,103 bytes (4.23 MB) | `a603653a40e060c51121d4df12a5ae4d67319349890f9f30b91d29d44e54e4df` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/reconstruction_metadata.json` | Provenance Hashes & Distortion Attribution Report | 2,652 bytes (0.00 MB) | `21c620e1709666b93af5962913b1c892da78b846710e98573cc13cbafe09f48e` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/diagnostics/audit_1_ground_lettering.png` | Close-up Comparison: Ground Lettering & Markings | 1,227,103 bytes (1.17 MB) | `31e2690078f85c790a6cec5b5179ff3ce7f54a921befd1dc5d5f5ddab6f4e317` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/diagnostics/audit_2_courtyard_paving.png` | Close-up Comparison: Crosswalks & Courtyard Pavers | 1,237,226 bytes (1.18 MB) | `a48e07a9e8c6c6bbf2b9858f1e4c5a0bd4524bf7383a1af75b18a17923741a48` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/diagnostics/audit_3_building_facades.png` | Close-up Comparison: Building Facades & Rooflines | 1,015,847 bytes (0.97 MB) | `d38da9819e809dc7d2a2d03e3084516b766fc1b1524b02ad04a5ba27fd9cca4b` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/diagnostics/audit_4_trees_landscaping.png` | Close-up Comparison: Courtyard Trees & Planters | 954,204 bytes (0.91 MB) | `7fc6a45b9abac7a9be44ef78678086aed5037a07f8e8ad431ad71ceb51e4a675` | Git Blob |

---

### 2.2 Desert Road — Recovery V2.1 (Complete Vehicle & Balloon Cleanup)
- **Directory:** `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/`
- **Topology:** 53,069 vertices, 105,467 triangles
- **Corridor Flatness:** Road plane mean residual = 2.0 mm (0.0020 model units); max deviation < 12 mm.
- **Color Attribution:** Fronto-parallel projection with dynamic vehicle/balloon rejection. Saturated red vertices = 0 (vs 35 in V2).
- **Material Dependency:** `masked_reconstruction.obj` declares `mtllib masked_reconstruction.mtl` and binds `usemtl Material_Atlas` referencing `map_Kd texture_atlas.png`.

| Repository-Relative Path | Artifact Role | File Size | SHA-256 Checksum | Storage Tier |
| :--- | :--- | :--- | :--- | :--- |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.glb` | Standalone glTF 2.0 Binary Model (embedded clean atlas) | 11,484,784 bytes (10.95 MB) | `6ac4cbf538c68c70acb1f8df6f39df5f11ace87cf1abb7105a0385f4158b4b04` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.obj` | Wavefront OBJ Polygonal Mesh (UV mapped) | 14,301,222 bytes (13.64 MB) | `0f169ca493f3945afb91292299716dc368809051093f48f04bd1500e250de71d` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.mtl` | Wavefront MTL Material Library | 185 bytes (0.00 MB) | `f2f54a8a17581f0f2bc09a24bc44cea145e2a72d1a83132535550129a22aa9fb` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.ply` | Stanford PLY Coordinate Mesh (photographic vertex colors) | 4,077,915 bytes (3.89 MB) | `8bcebf6bf31ff62a10ccb045cdf64345e19004e0a33aef55c1d439076ba565e4` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/texture_atlas.png` | 4096×4096 Clean Texture Atlas (inpainted vehicle regions) | 8,332,389 bytes (7.95 MB) | `fd6848e36adfd489b29d7c743b3beaa6fe3014b286db8c94fa2d80942aa5e370` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction_preview.png` | 3-Panel Inspection Preview (Oblique / Nadir / Detail) | 236,035 bytes (0.23 MB) | `17fc8202656f2d60fec1b70d4d985a0c29d866978e60d5c7bc90decf6229a7b0` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/neutral_geometry_preview.png` | Neutral Grey Material Render (Planar Geometry Integrity) | 37,488 bytes (0.04 MB) | `96e7d23de82c5b26d76b671cf8d79ef7a513f75109490fe2957894e637a8dbb9` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/texture_detail_preview.png` | Close-Up Texture Detail Render (Clean Road Centerline) | 117,193 bytes (0.11 MB) | `f4b0117240ea9cc21e82e4e4bfe2ad5fce16bc28097a79eb1c75207a8ab20063` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/road_texture_before_after.png` | Direct Before/After Close-Up (Recovery V2 vs V2.1) | 13,296 bytes (0.01 MB) | `ed510eee8581ac7f28066434cd951e7f6b3fc5d8c9b26b526e636682483e7691` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/road_surface_detail_closeup.png` | 3-Mode Road Close-Up Contact Sheet (Grey / Vertex / Atlas) | 213,760 bytes (0.20 MB) | `72fcc6a31ced844d14ecbfc8688fc5c327f090515c36b196bbf3a4408722b48c` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/before_after_comparison.png` | 3-Tier Multi-Sprint Comparison Sheet (V1 vs V2 vs V2.1) | 631,622 bytes (0.60 MB) | `204359922948c6a36029d144a51728a2842ad5ab0c8e31573d539ac0f3c6c8f5` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/reconstruction_metadata.json` | Provenance Hashes, Plane Fit Metrics & Defect Diagnosis | 2,463 bytes (0.00 MB) | `4278d12f79e87dc55e87b9c730eb6a94d278ce5402ae3c26810b1a6b91dc21f5` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/diagnostics/diag_1_neutral_grey_material.png` | Diagnostic Render: Neutral Grey Material (Geometry Integrity) | 37,098 bytes (0.04 MB) | `a4117ab17c26f2baa716f0001940f6e4ca4de12efae2bb0d2b2869ee9c95435b` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/diagnostics/diag_2_vertex_colors_only.png` | Diagnostic Render: Fronto-Parallel Vertex Colors Only | 60,428 bytes (0.06 MB) | `0fc427694c8cfa7206fbf42a310e551496f4a6d77a5e3bf503c9ddc72663855f` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/diagnostics/diag_3_full_textured.png` | Diagnostic Render: Full Inpainted 4K Texture Atlas | 37,098 bytes (0.04 MB) | `a4117ab17c26f2baa716f0001940f6e4ca4de12efae2bb0d2b2869ee9c95435b` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/diagnostics/diag_3modes_road_closeup.png` | Diagnostic Render: 3-Mode Road Centerline Close-up | 213,760 bytes (0.20 MB) | `72fcc6a31ced844d14ecbfc8688fc5c327f090515c36b196bbf3a4408722b48c` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_00.png` | Refined Vehicle + Balloon Binary Mask (Frame 00) | 3,155 bytes (0.00 MB) | `45da85d713c77d6ee0761a129188d30e154e17e4726d6692bbdaee6d44a1936c` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_01.png` | Refined Vehicle + Balloon Binary Mask (Frame 01) | 3,010 bytes (0.00 MB) | `f04ef7d8b5eb488f28c53a63ecda5eeb949b28205efc42023bf7c7bc99d94411` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_02.png` | Refined Vehicle + Balloon Binary Mask (Frame 02) | 3,054 bytes (0.00 MB) | `df1542fdb728c75dd9b6bc455364817a022d4f20ec719c8f6259250005ca79e4` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_03.png` | Refined Vehicle + Balloon Binary Mask (Frame 03) | 3,020 bytes (0.00 MB) | `0f0556b6aa99e190e29b1979b068ee1824efbcffb175ff41fe33ebceec872439` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_04.png` | Refined Vehicle + Balloon Binary Mask (Frame 04) | 2,902 bytes (0.00 MB) | `95a7b6cf071fe0eb4eaec4825595c25dbcb4081c7e2d93e110cb16db7db8e820` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_05.png` | Refined Vehicle + Balloon Binary Mask (Frame 05) | 2,735 bytes (0.00 MB) | `e1d441f71dfb99f24ba801a6132711679ec3643b2f216223ba99f925f6eefc4d` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_06.png` | Refined Vehicle + Balloon Binary Mask (Frame 06) | 2,812 bytes (0.00 MB) | `451c0ee10787b8f2d5e5330e7ef054ba52f75968f9b90c918ee08518cf037701` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masks/mask_frame_07.png` | Refined Vehicle + Balloon Binary Mask (Frame 07) | 2,756 bytes (0.00 MB) | `1d856d33f269a9e3344d2847c92a95df93c52a0a2f5ecb0ea4101e134639e6a3` | Git Blob |

---

### 2.3 Forest Road — Recovery V2 (Intentionally Retained)
- **Directory:** `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/`
- **Topology:** 445,824 vertices, 889,653 triangles
- **Vehicle Filtering:** Dynamic vehicle excised during point cloud fusion; clean asphalt corridor.

| Repository-Relative Path | Artifact Role | File Size | SHA-256 Checksum | Storage Tier |
| :--- | :--- | :--- | :--- | :--- |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction.glb` | Standalone glTF 2.0 Binary Model | 17,810,004 bytes (16.98 MB) | `20cb3b325fefce87a3350ad4209174b3efac28b58bdd407f85ebeda2ca630735` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction.obj` | Wavefront OBJ Polygonal Mesh | 79,656,310 bytes (75.97 MB) | `de131d7114ebc3a908bab96d32267972660525332b0f762e319e6ab4276dbb3d` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction.ply` | Stanford PLY Coordinate Mesh (dense vertex colors) | 34,302,839 bytes (32.71 MB) | `afb6a514124011ef30f9c89b16ed75aed23366a1296dda5b0af26cf4bfea1788` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction_preview.png` | 3-Panel Inspection Preview | 460,116 bytes (0.44 MB) | `bc81d03218b33b333316e1e587b204ab7d24a529bc629e298183a191ca6ba381` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/neutral_geometry_preview.png` | Neutral Grey Material Render | 189,016 bytes (0.18 MB) | `a61d43c005001324651d7c2da564c997ab739f47afe4481ff95436eea41a655f` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/reconstruction_metadata.json` | Provenance Hashes & Extraction Metadata | 2,148 bytes (0.00 MB) | `9bf7bd14fc38c708479dc24bc411c39b9ce9ba97b03afc339f7809baf17133c4` | Git Blob |

---

### 2.4 River/Bridge — Recovery V2 (Intentionally Retained)
- **Directory:** `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/`
- **Topology:** 127,887 vertices, 251,054 triangles
- **Structure:** Civil infrastructure bridge deck, structural concrete piers, river channel banks.

| Repository-Relative Path | Artifact Role | File Size | SHA-256 Checksum | Storage Tier |
| :--- | :--- | :--- | :--- | :--- |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction.glb` | Standalone glTF 2.0 Binary Model | 5,059,824 bytes (4.83 MB) | `2db3e110fe17348a7bfe57f5d549dcc118843a347aeba33af5c20fbce8da8491` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction.obj` | Wavefront OBJ Polygonal Mesh | 21,786,747 bytes (20.78 MB) | `7a4cd81d1665e2be52dbdf437ab30d5ed4be0db3a38038b631d9ab5287fa2cec` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction.ply` | Stanford PLY Coordinate Mesh (dense vertex colors) | 9,786,265 bytes (9.33 MB) | `b2ffd5934c9a433c853f499fe5ecedd1d0c5e650ba1a9f98e5b0faa311ec3ef5` | **Git LFS** |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction_preview.png` | 3-Panel Inspection Preview | 336,585 bytes (0.32 MB) | `acbc9b785e5330bb0790c0fcafa8ac5ec928a4f1c50e21f36eb96fffc8bb2b5b` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/neutral_geometry_preview.png` | Neutral Grey Material Render | 114,578 bytes (0.11 MB) | `b20835e00707c20a57711a453f027c6c21e6139fcdc7e647688828bac2a4d2eb` | Git Blob |
| `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/reconstruction_metadata.json` | Provenance Hashes & Georeferencing Alignment Data | 2,203 bytes (0.00 MB) | `e9160dc38bc3ecbb7212c22e0a32ffc87119adac47bb7ba38adb8e820c50ac76` | Git Blob |

---

### 2.5 Consolidated Sprint Metadata
| Repository-Relative Path | Artifact Role | File Size | SHA-256 Checksum | Storage Tier |
| :--- | :--- | :--- | :--- | :--- |
| `outputs/dynamic_object_masking/sprint11/recovery_v2_1/reconstruction_summary.json` | Master Sprint 11 Recovery V2.1 Metadata Summary | 2,563 bytes (0.00 MB) | `fd54ff0d77c7ec90a3f67e149f7212962d366b1969f83182e85e8ba27259d6d1` | Git Blob |

---

## 3. Reproducibility Scripts, Validation Suites & Tests

The release includes the exact scripts and automated pytest suites required to reproduce, audit, and validate these models from scratch:

| Path | Purpose & Reproducible Role |
| :--- | :--- |
| `scripts/run_recovery_v2_1_desert_road.py` | Complete end-to-end generator for Desert Road V2.1 (YOLO mask dilation, balloon segmentation, Telea atlas inpainting, fronto-parallel color projection, OBJ/MTL/GLB export). |
| `scripts/compare_campus_geometry_and_texture.py` | Diagnostic generator for Campus 0049 (renders 4 side-by-side Textured vs Neutral Grey close-ups, builds audit contact sheet, exports deliverables). |
| `scripts/run_recovery_v2_all_scenes.py` | Master pipeline execution script for multi-scene quality recovery. |
| `scripts/verify_recovery_deliverables.py` | Independent reload verification script checking geometry, bounds, topology, and material references. |
| `scripts/build_recovery_comparison_sheet.py` | Visual contact sheet generator compiling before/after multi-sprint matrices. |
| `tests/test_sprint11_recovery_v2_1.py` | Pytest suite validating Recovery V2.1 deliverables, zero residual vehicle marks, 4K atlas integrity, and repository cleanliness. |
| `tests/test_sprint11_recovery_v2.py` | Pytest suite validating Recovery V2 structural topology, reloadability, and plane fit residuals across all 4 scenes. |
| `tests/test_sprint11_dynamic_masking.py` | 11 unit and integration tests verifying YOLO segmentation, mask dilation, depth coordinate scaling, and point cloud filtering. |
| `tests/test_sprint9_video_to_viewer.py` | 6 regression tests verifying the frozen Sprint 9 baseline and Web Viewer assets remain 100% intact. |

---

## 4. Git LFS Storage Accounting

All 14 large 3D meshes and 4K texture atlases are tracked under Git LFS via `.gitattributes`:

```gitattributes
# 3D Model Geometry & Meshes
*.obj filter=lfs diff=lfs merge=lfs -text
*.ply filter=lfs diff=lfs merge=lfs -text
*.glb filter=lfs diff=lfs merge=lfs -text
*.gltf filter=lfs diff=lfs merge=lfs -text

# High-Resolution Texture Atlases (> 5 MB)
*texture_atlas*.png filter=lfs diff=lfs merge=lfs -text
```

### Sprint 11 Newly Added LFS Objects (14 files / ~310.3 MB):
1. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.glb` (46.13 MB)
2. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.obj` (29.83 MB)
3. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/masked_reconstruction.ply` (9.38 MB)
4. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/campus_0049/texture_atlas.png` (27.94 MB)
5. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.glb` (10.95 MB)
6. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.obj` (13.64 MB)
7. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/masked_reconstruction.ply` (3.89 MB)
8. `outputs/dynamic_object_masking/sprint11/recovery_v2_1/desert_road/texture_atlas.png` (7.95 MB)
9. `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction.glb` (16.98 MB)
10. `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction.obj` (75.97 MB)
11. `outputs/dynamic_object_masking/sprint11/recovery_v2/forest_road/masked_reconstruction.ply` (32.71 MB)
12. `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction.glb` (4.83 MB)
13. `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction.obj` (20.78 MB)
14. `outputs/dynamic_object_masking/sprint11/recovery_v2/river_bridge/masked_reconstruction.ply` (9.33 MB)

Total repository LFS consumption post-release is estimated at **~3.90 GiB**, well within GitHub's **10 GiB** standard LFS quota (~39.0% utilized).

---

## 5. Excluded Data & Cache Policy

To keep the repository clean and avoid disposable clutter, the following items are intentionally excluded from Git tracking:
- **Pretrained Foundation Model Weights:** `yolov8n-seg.pt` (6.7 MB) and VGGT-Omega foundation checkpoints (> 1 GB) are cached locally and excluded via `.gitignore`. Acquisition instructions and checksums are provided in `docs/SPRINT11_DYNAMIC_MASKING_REPORT.md`.
- **Ephemeral Scratch Runs:** Intermediate failed experiments under `outputs/dynamic_object_masking/sprint11/recovery_v1/`, per-frame frame dumps (`outputs/frames/`), raw predictions (`outputs/predictions/`), and OpenGL screen capture dumps (`ScreenCapture_*.png`) remain untracked.
- **Virtual Environments & Node Modules:** `.venv/`, `node_modules/`, and `__pycache__/` are ignored.

---

## 6. Licensing & Asset Provenance Constraints

All published models and media honor their documented provenance terms:
- **Campus 0049 & River/Bridge:** First-party drone video telemetry captured by team (`PrathmeshPP`).
- **Desert Road & Forest Road:** Source video sequences originating from Meta AI & University of Oxford under the [Meta FAIR Noncommercial Research License](licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt). Models derived from these videos are restricted to noncommercial research use. Military, warfare, surveillance, or unrestricted commercial deployment is strictly prohibited by upstream Acceptable Use Policies.
- **YOLOv8 Segmentation Engine:** Ultralytics AGPL-3.0. F.O.R.G.E decouples the segmentation module behind `BaseDynamicSegmenter` in `src/detection/base.py`. Pretrained weights are not distributed within the Git repository.

Refer to [`docs/THIRD_PARTY_ASSET_PROVENANCE.md`](THIRD_PARTY_ASSET_PROVENANCE.md) for full legal disclosures.
