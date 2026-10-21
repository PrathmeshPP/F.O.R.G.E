# F.O.R.G.E — Third-Party Asset Provenance and Licensing Audit

> **Document Version:** 1.0.0  
> **Date:** October 2026  
> **Repository:** [https://github.com/PrathmeshPP/FORGE](https://github.com/PrathmeshPP/FORGE)  
> **Maintainer / Release Manager Audit Report**

---

## 1. Executive Summary & Redistribution Boundaries

This document establishes the authoritative licensing, ownership, and redistribution provenance for all external media assets, benchmark videos, foundation models, and bundled software libraries utilized across the **F.O.R.G.E** (Fast Optical Reconstruction & Geospatial Estimation) repository.

### Key Findings Matrix

| Asset Group | Count / Size | Ownership / Upstream | Applicable License | Redistribution Status | Permitted Usage |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Drone Aerial Videos** (`Campus 0049`, `River/Bridge`) | 2 files / 221.3 MB | Project Authors (`PrathmeshPP` / SIH Team) | First-party proprietary / Author cleared | **CLEARED** | Full redistribution under project terms |
| **Benchmark Example Videos** (`Desert Road`, `Forest Road`) | 2 files / 16.4 MB | Meta AI & Visual Geometry Group (Univ. of Oxford) | Meta FAIR Noncommercial Research License (v1, Oct 2024) | **RESTRICTED** | Permitted for **Noncommercial Research / Educational** use with attribution; commercial redistribution prohibited |
| **Foundation Model Weights** (`vggt_omega_1b_512.pt`) | ~4.5 GB (External) | Meta AI (`facebook/VGGT-Omega`) | Meta FAIR Noncommercial Research License | **EXTERNAL ONLY** | Downloaded separately from official Hugging Face repository upon approved access; untracked in Git |
| **Intermediate Inference Tensors** (`predictions_raw.pt`, `*.npz`) | 11 files / 88.1 MB | F.O.R.G.E Pipeline Generated | Derived from first-party inference execution | **CLEARED** | Distributed under project research terms |
| **Reconstruction Meshes & Textures** (OBJ, PLY, GLB, PNG) | 5 Scenes / 3.4 GB | F.O.R.G.E Pipeline Generated | First-party derived research deliverables | **CLEARED** | Noncommercial research and evaluation distribution |
| **Frontend Libraries** (Three.js, React, Lucide, JSZip) | Package manifests | Respective Open Source Projects | MIT / ISC / Apache 2.0 | **CLEARED** | Permissive open source compliance via `viewer/package.json` |
| **Python Dependencies** (PyTorch, Open3D, OpenCV, NumPy) | Dependency manifests | Respective Open Source Projects | BSD-3 / Apache 2.0 / MIT | **CLEARED** | Permissive open source compliance via `requirements.txt` |
| **Root Project License** | Root repository | PrathmeshPP (Repository Owner) | **UNSET (Pending Owner Selection)** | **RESERVED** | All Rights Reserved by default under Berne Convention |

---

## 2. Source Video Provenance Audit

F.O.R.G.E utilizes four source video sequences across its five evaluation scenes. The four source videos fall into two distinct legal and provenance categories:

```
                            ┌────────────────────────────────────────┐
                            │      F.O.R.G.E Source Video Corpus      │
                            └───────────────────┬────────────────────┘
                                                │
                 ┌──────────────────────────────┴──────────────────────────────┐
                 ▼                                                             ▼
   ┌───────────────────────────┐                                 ┌───────────────────────────┐
   │    First-Party Drone      │                                 │     Upstream Benchmark    │
   │    Field Data (2 clips)   │                                 │     Research (2 clips)    │
   ├───────────────────────────┤                                 ├───────────────────────────┤
   │ • Campus 0049             │                                 │ • Desert Road             │
   │ • River/Bridge            │                                 │ • Forest Road             │
   │                           │                                 │                           │
   │ Origin: DJI Air 3S flight │                                 │ Origin: VGGT-Ω (Meta/VGG) │
   │ License: Project Authors  │                                 │ License: Meta FAIR Noncom.│
   │ Status: Fully Cleared     │                                 │ Status: Noncommercial    │
   └───────────────────────────┘                                 └───────────────────────────┘
```

---

### Asset 1: Campus 0049 Drone Flight
- **Repository Path:** `data/source_videos/dji_fly_20260727_172754_0049_1785175051818_video.mp4`
- **File Size:** 162,041,214 bytes (154.53 MB)
- **SHA-256 Hash:** `7cf250cfb4dd90e6388b5dd2d3d146209cb0c3bbff8ba1dc2d123b7e5d9c3c6a`
- **Git LFS OID:** `7cf250cfb4dd90e6388b5dd2d3d146209cb0c3bbff8ba1dc2d123b7e5d9c3c6a`
- **Technical Specs:** 3840×2160 (4K UHD), 59.94 fps, 966 frames, 16.116s, HEVC/H.265.
- **Embedded Telemetry:** Contains proprietary DJI metadata (`djmd_protobuf`) embedded in auxiliary MP4 data streams, recording flight GPS coordinates (Gandhinagar/Ahmedabad institutional campus), barometric altitude, gimbal pitch/roll/yaw, and camera intrinsics.
- **Upstream Origin / Author:** Captured on July 27, 2026 by the F.O.R.G.E project authors (`PrathmeshPP` and Smart India Hackathon team) utilizing a DJI Air 3S unmanned aerial vehicle.
- **Applicable License:** Author / First-Party Proprietary.
- **Redistribution Status:** **Explicitly Permitted**. The authors own all copyright and intellectual property rights in this recording.
- **Attribution & Usage Requirements:** Credit the F.O.R.G.E project authors when publishing or distributing results derived from this flight.

---

### Asset 2: River/Bridge Corridor Flight
- **Repository Path:** `data/source_videos/segment_52s_64s.mp4`
- **Canonical Release Path:** `outputs/dji_river_bridge_validation/segment_52s_64s.mp4`
- **File Size:** 70,005,949 bytes (66.76 MB)
- **SHA-256 Hash:** `0d1a647a0a0363bed400317a2b3495ad5926d469a5dd7a8b127fee037e7409cb`
- **Git LFS OID:** `0d1a647a0a0363bed400317a2b3495ad5926d469a5dd7a8b127fee037e7409cb`
- **Technical Specs:** 3840×2160 (4K UHD), 59.94 fps, 750 frames, 12.513s, FMP4/MPEG-4.
- **Parent Flight Recording:** Extracted from raw flight recording `dji_fly_20260727_172812_0050_1785175896547_video.mp4` (703.0 MB, SHA-256: `b539bb2bb59776...`), frames spanning timestamps 52.0s through 64.5s.
- **Upstream Origin / Author:** Captured on July 27, 2026 by the F.O.R.G.E project authors using a DJI Air 3S drone surveying a river bridge structure.
- **Applicable License:** Author / First-Party Proprietary.
- **Redistribution Status:** **Explicitly Permitted**.
- **Attribution & Usage Requirements:** Credit the F.O.R.G.E project authors.

---

### Asset 3: Desert Road Benchmark Sequence
- **Repository Path:** `data/source_videos/desert_road.mp4`
- **File Size:** 6,539,814 bytes (6.24 MB)
- **SHA-256 Hash:** `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`
- **Git LFS OID:** `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`
- **Technical Specs:** 1280×720 (HD), 23.976 fps, 477 frames, 19.895s, H.264/AVC.
- **Reconstruction Relevance:** Basis for **Desert Road Candidate A** (static highway corridor with moving objects removed) and **Desert Road Candidate B** (corridor preserving reference vehicle at `[0.03, 0.04, 0.45]`).
- **Upstream Project:** [VGGT-Ω: Visual Geometry Ground Truth](http://vggt-omega.github.io/)  
  Official Codebase: [facebook/VGGT-Omega](https://github.com/facebook/VGGT-Omega) (also hosted at Oxford VGG)  
  Paper: *VGGT-Ω*, Jianyuan Wang, Minghao Chen, Shangzhan Zhang, Nikita Karaev, Johannes Schönberger, Patrick Labatut, Piotr Bojanowski, David Novotny, Andrea Vedaldi, Christian Rupprecht (arXiv:2605.15195).  
  Original File Location: `examples/desert_road.mp4`
- **Author / Rights Holder:** Meta AI (Meta Platforms, Inc.) and Visual Geometry Group (University of Oxford).
- **Applicable License:** **Meta FAIR Noncommercial Research License (v1, Last Updated October 16, 2024)** incorporating the FAIR Acceptable Use Policy.
- **Redistribution Terms & Scope:**
  - **Research Materials Definition:** Explicitly encompasses demonstration materials, example datasets, models, software, and algorithms distributed under the agreement.
  - **Grant of Rights (Section 1.a):** Non-exclusive, worldwide, royalty-free limited license to use, reproduce, distribute, copy, create derivative works of, and modify the Research Materials.
  - **Commercial Restriction (Section 1.b.i):** *"You will not use the Research Materials or any outputs or results of the Research Materials in connection with any commercial uses or for any uses other than Noncommercial Research Uses."*
  - **Downstream Distribution (Section 1.b.ii):** Derivative works or redistributions must be made available strictly under the terms of the Meta FAIR Noncommercial Research License, and a copy of the license agreement must be provided to recipients.
  - **Mandatory Attribution (Section 1.b.iii):** Any publication utilizing this sequence must acknowledge and cite the VGGT-Ω publication.
- **Redistribution Status:** **RESTRICTED TO NONCOMMERCIAL RESEARCH**.
- **Exposure / Compliance Assessment:**
  - `desert_road.mp4` is tracked through Git LFS in this public repository.
  - Because F.O.R.G.E is an academic / hackathon demonstration project conducting research into video-to-3D reconstruction, redistribution is legally compliant under Section 1.a **provided that the project remains strictly noncommercial and provides the required attribution**.
  - **Commercial Warning:** Any downstream commercial enterprise or deployment fork must decouple or license alternative commercial training/benchmark clips.

---

### Asset 4: Forest Road Benchmark Sequence
- **Repository Path:** `data/source_videos/forest_road.mp4`
- **File Size:** 10,681,415 bytes (10.19 MB)
- **SHA-256 Hash:** `e5e847d9b3863d29c12501347074c58c6cc4e68fb91f4c963bbe3f91294a169a`
- **Git LFS OID:** `e5e847d9b3863d29c12501347074c58c6cc4e68fb91f4c963bbe3f91294a169a`
- **Technical Specs:** 1280×720 (HD), 23.976 fps, 730 frames, 30.447s, H.264/AVC.
- **Reconstruction Relevance:** Basis for **Forest Road** scene (mountain forest corridor, two-lane asphalt roadway, dense coniferous trees).
- **Upstream Project:** [facebook/VGGT-Omega](https://github.com/facebook/VGGT-Omega) (University of Oxford & Meta AI). Original File Location: `examples/forest_road.mp4`.
- **Author / Rights Holder:** Meta AI and Visual Geometry Group, University of Oxford.
- **Applicable License:** **Meta FAIR Noncommercial Research License (v1, Oct 16, 2024)**.
- **Redistribution Status:** **RESTRICTED TO NONCOMMERCIAL RESEARCH** (identical terms and restrictions to Asset 3).

---

## 3. Pretrained Foundation Model Weights & Numerical Outputs

### Pretrained Weights (`vggt_omega_1b_512.pt`)
- **Upstream Checkpoint:** `VGGT-Omega-1B-512`
- **Official Access Location:** [https://huggingface.co/facebook/VGGT-Omega](https://huggingface.co/facebook/VGGT-Omega)
- **Size on Disk:** 4,814,948,352 bytes (~4.48 GB)
- **License:** Meta FAIR Noncommercial Research License.
- **Policy in F.O.R.G.E:**
  - Foundation weights **are NOT tracked in Git or Git LFS**.
  - `.gitignore` explicitly excludes `models/`, `*.ckpt`, and `*.safetensors`.
  - The inference engine (`src/vggt/inference.py`) dynamically reads the local checkpoint from the environment variable:
    ```bash
    export VGGT_OMEGA_CHECKPOINT="/path/to/vggt_omega_1b_512.pt"
    ```
  - Developers and evaluators must register on Hugging Face to obtain approved access to the foundation weights.

### Pipeline Predictions & Tensors (`predictions_raw.pt`, `*.npz`)
- **Repository Paths:**
  - `outputs/dji_0049_validation/predictions/predictions_raw.pt` (16.2 MB)
  - `outputs/dji_river_bridge_validation/predictions/predictions_raw.pt` (16.2 MB)
  - `outputs/accuracy_experiment/river_bridge_flight_corridor/predictions/predictions_raw.pt` (16.0 MB)
  - `outputs/accuracy_experiment/gps_aligned_v1/*/camera_poses_gps_aligned.npz` (5.1 MB each)
- **Provenance:** Generated deterministically by the F.O.R.G.E inference and geospatial alignment scripts.
- **License:** First-party research outputs derived under the Meta FAIR Noncommercial license. Safe for noncommercial research distribution.

---

## 4. Third-Party Code, Libraries, and Bundled Assets

| Component | Filesystem Location | Upstream / Maintainer | License | Compliance Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Three.js** | `viewer/package.json` (`three@^0.186.1`) | Ricardo Cabello (Mr.doob) | MIT License | Standard 3D web rendering engine; license preserved in npm bundle. |
| **React & React-DOM** | `viewer/package.json` (`react@^19.3.1`) | Meta Platforms, Inc. | MIT License | Frontend framework. |
| **Lucide Icons** | `viewer/package.json` (`lucide-react@^1.60.0`) | Lucide Contributors | ISC License | Vector iconography. |
| **JSZip** | `viewer/package.json` (`jszip@^3.10.2`) | Stuart Knightley | MIT / GPLv3 dual | Client-side ZIP decompression utility. |
| **Vite** | `viewer/package.json` (`vite@^8.3.4`) | Evan You & Vite contributors | MIT License | Local dev server and build tool. |
| **PyTorch / Torchvision** | `requirements.txt` (`torch>=2.4.0`) | PyTorch Foundation / Meta AI | Modified BSD 3-Clause | Deep learning compute runtime. |
| **Open3D** | `requirements.txt` (`open3d>=0.18.0`) | Intel Labs & Open3D contributors | MIT License | 3D mesh and point cloud processing library. |
| **OpenCV** | `requirements.txt` (`opencv-python>=4.8.0`) | OpenCV Team | Apache 2.0 License | Image decoding and camera frame extraction. |
| **NumPy** | `requirements.txt` (`numpy==1.26.4`) | NumPy Developers | BSD 3-Clause | Numerical array computing (pinned < 2.0). |

---

## 5. F.O.R.G.E Project License Status and Recommendations

### Current State
As of checkpoint `428a455`, **F.O.R.G.E does not contain a root-level `LICENSE` file**.

### Legal Implications
Under international copyright law (including the Berne Convention for the Protection of Literary and Artistic Works and international jurisprudence):
1. **Absence of a License = All Rights Reserved:** When code is published to a public GitHub repository without an explicit license, the creator retains all exclusive copyrights.
2. **GitHub Terms of Service Scope:** GitHub's standard Terms of Service grant other users the right to view and fork the repository *within the GitHub platform*, but do **NOT** grant rights to reproduce, distribute, modify, create derivative works of, or use the code commercially outside GitHub.
3. **Downstream Developer Ambiguity:** Without a license file, external researchers, evaluators, and engineers cannot legally incorporate F.O.R.G.E into downstream tools or commercial workflows without risk of copyright infringement.

### Recommended Licensing Options for Owner Selection

Because assigning legal rights is solely the prerogative of the repository owner (`PrathmeshPP`), the options below outline the trade-offs:

| Option | Recommended For | Permissions & Conditions | Interaction with VGGT-Ω |
| :--- | :--- | :--- | :--- |
| **Option A: MIT License** *(Permissive)* | Maximum open-source adoption, community contributions, and lightweight distribution. | Anyone can use, copy, modify, merge, publish, distribute, and sublicense, provided the copyright notice is retained. | Source code is MIT, but documentation must explicitly note that `desert_road.mp4`, `forest_road.mp4`, and VGGT weights remain under the Meta FAIR Noncommercial License. |
| **Option B: Apache 2.0** *(Permissive + Patent Protection)* | Professional or institutional research with formal contributor and patent protection. | Grants copyright and patent licenses, requires preservation of notices, and protects authors from patent litigation. | Same as MIT: applies cleanly to code; third-party benchmark assets must retain noncommercial notices. |
| **Option C: CC BY-NC 4.0 or FAIR-aligned Noncommercial** | Complete harmony with the upstream Meta FAIR Noncommercial License. | Permits sharing and adaptation exclusively for noncommercial purposes; requires attribution. | **Best legal symmetry.** Prevents third parties from commercializing F.O.R.G.E without author permission, directly mirroring the upstream VGGT-Ω license. |
| **Option D: All Rights Reserved (Proprietary / Evaluator Notice)** | Competitions, hackathons, or pending intellectual property / patent filings. | Code and models may be inspected and evaluated by hackathon judges, but no redistribution or reuse is authorized. | Fully protects author IP; leaves open future commercialization or patenting. |

---

## 6. Required Academic Citation & Attribution

Per Section 1.b.iii of the Meta FAIR Noncommercial Research License, all research or publications incorporating the Desert Road or Forest Road reconstructions must include the following formal citation:

```bibtex
@misc{wang2026vggtomega,
  title={VGGT-$\Omega$: Visual Geometry Ground Truth}, 
  author={Jianyuan Wang and Minghao Chen and Shangzhan Zhang and Nikita Karaev and Johannes Sch{\"o}nberger and Patrick Labatut and Piotr Bojanowski and David Novotny and Andrea Vedaldi and Christian Rupprecht},
  year={2026},
  eprint={2605.15195},
  archivePrefix={arXiv},
  primaryClass={cs.CV}
}
```

For the F.O.R.G.E project and first-party drone datasets (Campus 0049 and River/Bridge):

```bibtex
@misc{forge2026,
  title={F.O.R.G.E: Fast Optical Reconstruction & Geospatial Estimation},
  author={Prathmesh and F.O.R.G.E Contributors},
  year={2026},
  howpublished={\url{https://github.com/PrathmeshPP/FORGE}}
}
```
