# F.O.R.G.E — Third-Party Asset Provenance, Licensing Audit & Acceptable Use Review

> **Document Version:** 1.1.0  
> **Date:** October 2026  
> **Repository:** [https://github.com/PrathmeshPP/FORGE](https://github.com/PrathmeshPP/FORGE)  
> **Maintainer / Release Manager Audit Report**

---

## 1. Executive Summary & Redistribution Boundaries

This document establishes the authoritative licensing, legal ownership, acceptable use compliance, and redistribution provenance for all external media assets, benchmark videos, foundation models, and bundled software libraries utilized across the **F.O.R.G.E** (Fast Optical Reconstruction & Geospatial Estimation) repository.

### Key Findings & Legal Classification Matrix

| Asset Group | Count / Size | Ownership / Upstream | Applicable License | Redistribution Status | Permitted Usage & Conditions | Outstanding Concerns |
| :--- | :--- | :--- | :--- | :---: | :--- | :--- |
| **Drone Aerial Videos** (`Campus 0049`, `River/Bridge`) | 2 files / 221.3 MB | Attributed to Project Authors (`PrathmeshPP` / SIH Team) | First-party proprietary / Unsigned | **UNCONFIRMED** *(Plausible First-Party Inference)* | Pending formal owner written confirmation / copyright release. | Primary evidence confirms capture hardware (DJI Air 3S) and local machine paths; no formal written release exists in repository. |
| **Benchmark Example Videos** (`Desert Road`, `Forest Road`) | 2 files / 16.4 MB | Meta AI & Visual Geometry Group, Univ. of Oxford | [Meta FAIR Noncommercial Research License (v1, Oct 2024)](docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt) | **RESTRICTED** | **Noncommercial Research / Educational use only** with mandatory attribution and redistribution of agreement. | **Commercial use prohibited.** Downstream distribution must include license copy. Tactical/military use prohibited by AUP. |
| **Foundation Model Weights** (`vggt_omega_1b_512.pt`) | ~4.48 GB (External) | Meta AI (`facebookresearch/vggt-omega`) | Meta FAIR Noncommercial Research License | **EXTERNAL ONLY** | Downloaded separately from official Hugging Face repository upon approved access; untracked in Git. | Subject to Meta FAIR Noncommercial terms and Acceptable Use Policy. |
| **Intermediate Inference Tensors** (`predictions_raw.pt`, `*.npz`) | 11 files / 88.1 MB | F.O.R.G.E Pipeline Generated | Derived research outputs | **RESTRICTED (FAIR Noncommercial)** | Permitted under noncommercial research terms. | Bound by upstream noncommercial research outputs restriction (Section 1.b.i). |
| **Reconstruction Meshes & Textures** (OBJ, PLY, GLB, PNG) | 5 Scenes / 3.4 GB | F.O.R.G.E Pipeline Generated | Mixed: Derived from first-party & benchmark footage | **MIXED** | Drone scenes subject to owner confirmation; Benchmark scenes strictly noncommercial research. | Desert Road and Forest Road meshes are derived outputs bound by FAIR Noncommercial license. |
| **Frontend Libraries** (Three.js, React, Lucide, JSZip) | Package manifests | Respective Open Source Projects | MIT / ISC / Apache 2.0 | **CLEARED** | Permissive open source compliance via `viewer/package.json`. | Standard npm dependencies. |
| **Python Dependencies** (PyTorch, Open3D, OpenCV, NumPy) | Dependency manifests | Respective Open Source Projects | BSD-3 / Apache 2.0 / MIT | **CLEARED** | Permissive open source compliance via `requirements.txt`. | Standard Python dependencies. |
| **Root Project License** | Root repository | PrathmeshPP (Repository Owner) | **UNSET (Pending Owner Selection)** | **RESERVED** | All Rights Reserved by default under international copyright law (Berne Convention). | No root LICENSE file exists. Code cannot be legally reused by third parties outside GitHub until a license is selected. |

---

## 2. Upstream VGGT-Omega Licensing and Acceptable Use Compliance

### 2.1 Upstream Source & Research Materials Scope
- **Official Upstream Repository:** [https://github.com/facebookresearch/vggt-omega](https://github.com/facebookresearch/vggt-omega)  
- **Official Upstream License:** [https://github.com/facebookresearch/vggt-omega/blob/main/LICENSE](https://github.com/facebookresearch/vggt-omega/blob/main/LICENSE)  
- **Paper Citation:** *VGGT-Ω: Visual Geometry Ground Truth*, Jianyuan Wang et al. (arXiv:2605.15195).  
- **Local Verbatim Copy in FORGE:** [`docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt`](docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt)

Under the Agreement, **"Research Materials"** is defined comprehensively:
> *"Documentation and the models, software and algorithms, including machine-learning model code, trained model weights, inference-enabling code, training-enabling code, fine-tuning enabling code, demonstration materials and other elements of the foregoing distributed by Meta and made available under this Agreement."*

This definition explicitly encompasses:
1. The benchmark source clips `desert_road.mp4` and `forest_road.mp4` (demonstration materials distributed by Meta in the upstream repository under `examples/`).
2. The foundation checkpoint weights `vggt_omega_1b_512.pt`.
3. Code adapted or interfaced with `vggt_omega` in `src/vggt/`.
4. Downstream outputs and results derived from these research materials (e.g., 3D point clouds and meshes for Desert Road and Forest Road).

---

### 2.2 Core License Terms and Redistribution Mandates

#### A. Noncommercial Research-Use Limitation (Section 1.b.i)
> *"You will not use the Research Materials or any outputs or results of the Research Materials in connection with any commercial uses or for any uses other than Noncommercial Research Uses."*
- **Impact on F.O.R.G.E:** F.O.R.G.E and any models or outputs derived from VGGT-Ω (including the Desert Road and Forest Road reconstructions) **cannot be commercialized, sold, integrated into paid services, or distributed for commercial advantage**.

#### B. Mandatory Downstream License Distribution (Section 1.b.ii)
> *"Distribution of Research Materials, and any derivative works thereof, are subject to the terms of this Agreement. If you distribute or make the Research Materials, or any derivative works thereof, available to a third party, you may only do so under the terms of this Agreement. You shall also provide a copy of this Agreement to such third party."*
- **Compliance in F.O.R.G.E:** Because F.O.R.G.E distributes `desert_road.mp4` and `forest_road.mp4` via Git LFS, the repository is legally obligated to bundle a complete copy of the Meta FAIR Noncommercial Research License.  
  **Remediation:** The complete, unabridged upstream license text has been placed at [`docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt`](docs/licenses/META_FAIR_NONCOMMERCIAL_LICENSE.txt) and linked in documentation.

#### C. Mandatory Academic Attribution (Section 1.b.iii)
> *"If you submit for publication the results of research you perform on, using, or otherwise in connection with Research Materials, you must acknowledge the use of Research Materials in your publication."*
- **Compliance in F.O.R.G.E:** Formal BibTeX citations are provided in [Section 6](#6-required-academic-citation--attribution) and in `README.md`.

---

### 2.3 FAIR Acceptable Use Policy Restrictions

The Agreement explicitly incorporates the **FAIR Acceptable Use Policy**, which establishes strict prohibitions. Users agree not to use or allow others to use Research Materials to:

> **Section 2:** *"Engage in, promote, incite, facilitate, or assist in the planning or development of activities that present a risk of death or bodily harm to individuals, including use of research artifacts related to the following:*  
> *a. **Military, warfare, nuclear industries or applications, espionage, use for materials or activities that are subject to the International Traffic Arms Regulations (ITAR) maintained by the United States Department of State**;*  
> *b. Guns and illegal weapons (including weapon development);*  
> *c. Illegal drugs and regulated/controlled substances;*  
> *d. **Operation of critical infrastructure, transportation technologies, or heavy machinery**;*  
> *e. Self-harm or harm to others...*"

---

### 2.4 Critical Legal Compatibility Question: "Tactical Drone Pipeline" vs FAIR Acceptable Use Policy

In earlier project sprints, technical reports, and hackathon documentation, F.O.R.G.E has been described as a **"tactical drone reconstruction pipeline"**, an **"aerial corridor reconnaissance engine"**, or a system for **"defense and disaster response reconnaissance"**.

> [!WARNING]
> **CRITICAL LEGAL COMPATIBILITY NOTICE:**  
> If F.O.R.G.E is intended for **military, defense intelligence, warfare, combat battlefield reconnaissance, espionage, or weaponized drone guidance**, its use of VGGT-Ω model weights, source code, and demonstration clips is **STRICTLY PROHIBITED** under Section 2.a of the Meta FAIR Acceptable Use Policy.
> 
> Furthermore, if the system is intended for the real-time operational control or automated navigation of transportation technologies or critical public infrastructure, it may conflict with Section 2.d.

**Required Maintainer / Owner Actions:**
1. **Do Not Rebrand Merely to Evade:** Merely removing the word "tactical" or renaming the repository does not alter the actual intended use. If the actual application is military, defense, or warfare, use of VGGT-Ω constitutes a breach of the Meta FAIR License.
2. **Clarification of Intended Use:** The repository owner must clarify whether F.O.R.G.E is strictly an **academic/civilian research platform** for computer vision benchmarking, architectural heritage survey, and civilian disaster damage assessment.
3. **Decoupling Strategy if Intended for Defense:** If the project's primary mission remains tactical defense or military operations, development using VGGT-Ω must stop, and the pipeline must be refactored to rely exclusively on unrestricted, permissive photogrammetry foundations (such as COLMAP, OpenSfM, or permissive Gaussian Splatting under BSD/Apache-2.0 licenses).

---

## 3. Source Video Provenance Audit

### 3.1 Campus 0049 Drone Flight
- **Repository Path:** `data/source_videos/dji_fly_20260727_172754_0049_1785175051818_video.mp4`
- **File Size:** 162,041,214 bytes (154.53 MB)
- **SHA-256 Hash:** `7cf250cfb4dd90e6388b5dd2d3d146209cb0c3bbff8ba1dc2d123b7e5d9c3c6a`
- **Technical Specs:** 3840×2160 (4K UHD), 59.94 fps, 966 frames, 16.116s, HEVC/H.265.
- **Physical Evidence on Record:**
  - Embedded auxiliary streams contain binary DJI telemetry protobuf (`djmd_protobuf`) recording drone serial number, barometric altitude, gimbal pitch/roll/yaw, and GPS coordinates (~23.19°N, ~72.63°E, corresponding to an institutional campus in Gandhinagar, Gujarat, India).
  - Recording timestamp: July 27, 2026 at 17:27:54 local time.
  - Filesystem location: Present on author workstation (`C:\Users\25beevdt025\Desktop\`) and within local Smart India Hackathon (`SIH`) directory trees.
- **Absence of Legal Clearance Documentation:**
  - **No written property release, flight permission log, pilot copyright assignment, or signed waiver exists in the repository.**
  - While it is a reasonable and plausible inference that this video was recorded by the SIH project team, **this inference does not constitute formal legal proof of copyright ownership or redistribution clearance**.
- **Audit Classification:** **UNCONFIRMED / PLAUSIBLE FIRST-PARTY INFERENCE (Pending Formal Owner Confirmation)**.
- **Action Required:** Repository owner (`PrathmeshPP`) should provide formal written statement or license confirmation certifying that the team owns the copyright to this flight and explicitly authorizes public redistribution under the chosen project license.

---

### 3.2 River/Bridge Corridor Flight
- **Repository Path:** `data/source_videos/segment_52s_64s.mp4`
- **Canonical Release Path:** `outputs/dji_river_bridge_validation/segment_52s_64s.mp4`
- **File Size:** 70,005,949 bytes (66.76 MB)
- **SHA-256 Hash:** `0d1a647a0a0363bed400317a2b3495ad5926d469a5dd7a8b127fee037e7409cb`
- **Technical Specs:** 3840×2160 (4K UHD), 59.94 fps, 750 frames, 12.513s, FMP4/MPEG-4.
- **Physical Evidence on Record:**
  - Trimmed segment spanning seconds 52.0s through 64.5s extracted from raw parent flight recording `dji_fly_20260727_172812_0050_1785175896547_video.mp4` (703.0 MB, SHA-256: `b539bb2bb5977630...`).
  - Recorded sequentially with flight 0049 on July 27, 2026 via DJI Air 3S drone.
- **Absence of Legal Clearance Documentation:**
  - Same as Campus 0049: no written release, property waiver, or pilot assignment document is checked into the repository.
- **Audit Classification:** **UNCONFIRMED / PLAUSIBLE FIRST-PARTY INFERENCE (Pending Formal Owner Confirmation)**.
- **Action Required:** Repository owner confirmation required before declaring fully cleared.

---

### 3.3 Desert Road Benchmark Sequence
- **Repository Path:** `data/source_videos/desert_road.mp4`
- **File Size:** 6,539,814 bytes (6.24 MB)
- **SHA-256 Hash:** `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`
- **Upstream Origin:** Bundled sample material in `facebookresearch/vggt-omega` under `examples/desert_road.mp4`.
- **Rights Holder:** Meta AI and Visual Geometry Group, University of Oxford.
- **Applicable License:** Meta FAIR Noncommercial Research License (v1, Oct 2024).
- **Audit Classification:** **RESTRICTED TO NONCOMMERCIAL RESEARCH USE**.
- **Redistribution Status:** Publicly downloadable via Git LFS in this repository. Permitted strictly for noncommercial research and education with attribution; commercial redistribution is prohibited.

---

### 3.4 Forest Road Benchmark Sequence
- **Repository Path:** `data/source_videos/forest_road.mp4`
- **File Size:** 10,681,415 bytes (10.19 MB)
- **SHA-256 Hash:** `e5e847d9b3863d29c12501347074c58c6cc4e68fb91f4c963bbe3f91294a169a`
- **Upstream Origin:** Bundled sample material in `facebookresearch/vggt-omega` under `examples/forest_road.mp4`.
- **Rights Holder:** Meta AI and Visual Geometry Group, University of Oxford.
- **Applicable License:** Meta FAIR Noncommercial Research License (v1, Oct 2024).
- **Audit Classification:** **RESTRICTED TO NONCOMMERCIAL RESEARCH USE** (identical terms to Desert Road).

---

## 4. Coverage Matrix: Which F.O.R.G.E Components Are Covered?

| Component / Path | Covered by Meta FAIR License? | Justification / Legal Basis |
| :--- | :---: | :--- |
| `data/source_videos/desert_road.mp4` | **YES** | Research Materials: demonstration material from upstream VGGT-Ω. |
| `data/source_videos/forest_road.mp4` | **YES** | Research Materials: demonstration material from upstream VGGT-Ω. |
| `vggt_omega_1b_512.pt` (external) | **YES** | Research Materials: official Meta trained checkpoint weights. |
| `src/vggt/inference.py` | **YES** (Derivative/Interface) | Encapsulates VGGT-Ω inference; imports `vggt_omega` package. |
| Reconstructions of Desert Road (Candidates A & B) | **YES** (Outputs/Results) | Bound by Section 1.b.i: *"any outputs or results of the Research Materials"*. |
| Reconstructions of Forest Road | **YES** (Outputs/Results) | Bound by Section 1.b.i. |
| `data/source_videos/dji_fly_...0049...mp4` | **NO** | Independent aerial drone flight; no Meta/VGGT research materials embodied. |
| `data/source_videos/segment_52s_64s.mp4` | **NO** | Independent aerial drone flight; no Meta/VGGT research materials embodied. |
| `src/geo/` (Umeyama RANSAC, GPS Alignment) | **NO** | First-party mathematical algorithms authored independently. |
| `viewer/` (React, TypeScript, Three.js application) | **NO** | First-party web rendering application authored independently. |

---

## 5. F.O.R.G.E Project License Status and Recommendations

### 5.1 Current State: Unset (All Rights Reserved)
As of checkpoint `428a455` and follow-up `996427f`, **F.O.R.G.E does not contain a root-level `LICENSE` file**.

### 5.2 Separation of First-Party Code License from Third-Party Restrictions
Third-party research restrictions (such as Meta FAIR on benchmark clips and model weights) apply specifically to those research materials and their derived outputs. They **do not dictate or constrain what license the owner chooses for independent, project-authored source code** (such as `src/geo/`, `src/video/`, and `viewer/`).

However, if project code is released under a permissive license (e.g., MIT or Apache 2.0), the documentation must clearly state that **the repository as a whole contains third-party benchmark assets and model integrations that remain subject to the Meta FAIR Noncommercial Research License**.

### 5.3 Licensing Options for Owner Selection

| Option | Recommended For | Permissions & Conditions | Interaction with VGGT-Ω |
| :--- | :--- | :--- | :--- |
| **Option A: MIT License** *(Permissive)* | Maximum open-source community adoption and lightweight contribution. | Anyone can use, copy, modify, merge, publish, and distribute project-authored code, provided copyright notice is retained. | Project code is MIT; third-party benchmark clips and model weights remain restricted to noncommercial research under Meta FAIR. |
| **Option B: Apache 2.0** *(Permissive + Patent Protection)* | Formal institutional research with explicit patent defense and contributor terms. | Grants copyright and patent licenses; requires preservation of notices and modification markers. | Same as MIT: applies to first-party code; benchmark assets remain noncommercial. |
| **Option C: CC BY-NC 4.0 / FAIR-aligned Noncommercial** | Complete legal harmony with upstream VGGT-Ω. | Permits sharing and adaptation exclusively for noncommercial purposes; requires attribution. | **Optimal legal symmetry.** Prevents third parties from commercializing F.O.R.G.E without author permission, directly mirroring upstream terms. |
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

For the F.O.R.G.E platform and first-party drone datasets (Campus 0049 and River/Bridge):

```bibtex
@misc{forge2026,
  title={F.O.R.G.E: Fast Optical Reconstruction & Geospatial Estimation},
  author={Prathmesh and F.O.R.G.E Contributors},
  year={2026},
  howpublished={\url{https://github.com/PrathmeshPP/FORGE}}
}
```
