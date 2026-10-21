# F.O.R.G.E — Original Source Video Location and Verification Audit

> **Audit Type:** Read-Only Filesystem Audit  
> **Timestamp (UTC):** 2026-10-21T15:31:21Z  
> **Status:** All four original source videos successfully located, verified, and SHA-256 hash-matched against reconstruction baseline.

---

## Executive Summary: Verified Source Video Inventory

| Scene Name | Target Filename | File Status | Authoritative Absolute Path | Repo-Relative Path | Size (MB) | Resolution | FPS | Frames | Duration | SHA-256 Match |
| :--- | :--- | :---: | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Campus 0049** | `dji_fly_20260727_172754_0049_1785175051818_video.mp4` | **EXISTS** | `c:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4` | *External (Desktop/SIH)* | 154.53 MB | 3840x2160 | 59.94 | 966 | 16.116s | **MATCH (PASS)** |
| **Desert Road (A & B)** | `desert_road.mp4` | **EXISTS** | `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\desert_road.mp4` | *External (Desktop/SIH)* | 6.24 MB | 1280x720 | 23.976 | 477 | 19.895s | **MATCH (PASS)** |
| **Forest Road** | `forest_road.mp4` | **EXISTS** | `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\forest_road.mp4` | *External (Desktop/SIH)* | 10.19 MB | 1280x720 | 23.976 | 730 | 30.447s | **MATCH (PASS)** |
| **River/Bridge** | `segment_52s_64s.mp4` | **EXISTS** | `c:\Users\25beevdt025\Desktop\FORGE\outputs\dji_river_bridge_validation\segment_52s_64s.mp4` | `outputs\dji_river_bridge_validation\segment_52s_64s.mp4` | 66.76 MB | 3840x2160 | 59.94 | 750 | 12.513s | **MATCH (PASS)** |

---

## 1. Audit of Previously Reported Search Locations

The filesystem was audited across the four search locations specified in the mission instructions:

### Path 1: `data/source_videos/`
- **Filesystem Status:** **DOES NOT EXIST** (`C:\Users\25beevdt025\Desktop\FORGE\data\source_videos\` is absent on disk).
- **File Checks:**
  - `data/source_videos/dji_fly_20260727_172754_0049_1785175051818_video.mp4`: **MISSING**
  - `data/source_videos/desert_road.mp4`: **MISSING**
  - `data/source_videos/forest_road.mp4`: **MISSING**
  - `data/source_videos/segment_52s_64s.mp4`: **MISSING**
- **Notes:** In prior reports, `data/source_videos/` was used as a conceptual reference. No physical video directory was created within the repository at that path.

### Path 2: `C:\Users\25beevdt025\Desktop\`
- **Filesystem Status:** **EXISTS**
- **File Checks:**
  - `dji_fly_20260727_172754_0049_1785175051818_video.mp4`: **EXISTS** (154.53 MB, SHA-256: `7cf250cfb4dd90e6388b5dd2d3d146209cb0c3bbff8ba1dc2d123b7e5d9c3c6a`)
  - `desert_road.mp4`: **MISSING** (Located in SIH examples directory)
  - `forest_road.mp4`: **MISSING** (Located in SIH examples directory)
  - `segment_52s_64s.mp4`: **MISSING** (Located in FORGE `outputs/dji_river_bridge_validation/`)
  - *Supplementary:* `dji_fly_20260727_172812_0050_1785175896547_video.mp4` **EXISTS** (703.00 MB, raw parent flight from which `segment_52s_64s.mp4` was extracted).

### Path 3: `C:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\`
- **Filesystem Status:** **EXISTS**
- **File Checks:**
  - `dji_fly_20260727_172754_0049_1785175051818_video.mp4`: **MISSING**
  - `desert_road.mp4`: **EXISTS** (6.24 MB, SHA-256: `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`)
  - `forest_road.mp4`: **EXISTS** (10.19 MB, SHA-256: `e5e847d9b3863d29c12501347074c58c6cc4e68fb91f4c963bbe3f91294a169a`)
  - `segment_52s_64s.mp4`: **MISSING**

### Path 4: `outputs\dji_river_bridge_validation\`
- **Filesystem Status:** **EXISTS** (`C:\Users\25beevdt025\Desktop\FORGE\outputs\dji_river_bridge_validation\`)
- **File Checks:**
  - `dji_fly_20260727_172754_0049_1785175051818_video.mp4`: **MISSING**
  - `desert_road.mp4`: **MISSING**
  - `forest_road.mp4`: **MISSING**
  - `segment_52s_64s.mp4`: **EXISTS** (66.76 MB, SHA-256: `0d1a647a0a0363bed400317a2b3495ad5926d469a5dd7a8b127fee037e7409cb`)

---

## 2. Detailed Per-Scene Verification & Provenance

### Scene 1: Campus 0049
- **Target Video:** `dji_fly_20260727_172754_0049_1785175051818_video.mp4`
- **Physical Status:** **EXISTS**
- **Authoritative Absolute Path:** `c:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4`
- **Repository-Relative Path:** *External to repository (stored directly on Desktop root)*
- **File Size:** 162041214 bytes (154.53 MB)
- **Video Resolution:** 3840x2160 (3840x2160, 4K UHD)
- **Frame Rate:** 59.94 fps
- **Total Frame Count:** 966 frames
- **Video Duration:** 16.116 seconds
- **Video Codec:** `hevc` (HEVC / H.265)
- **SHA-256 Hash:** `7cf250cfb4dd90e6388b5dd2d3d146209cb0c3bbff8ba1dc2d123b7e5d9c3c6a`
- **Classification:** Original source video (DJI Air 3S drone raw flight)
- **Reconstruction Provenance:** Matches recorded SHA-256 in Sprint 9 run manifest, model catalog, and `source_inventory.json`.
- **Duplicate Copies Discovered on Machine:**
  - `c:\Users\25beevdt025\Desktop\SIH\drone_3d_pipeline\data\input\run_20260906_002704\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)
  - `c:\Users\25beevdt025\Desktop\SIH\drone_3d_pipeline\data\input\run_20260906_003034\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)
  - `c:\Users\25beevdt025\Desktop\SIH\drone_3d_pipeline\data\input\run_20260906_174917\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\drone_3d_pipeline\data\input\run_20260906_002704\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\drone_3d_pipeline\data\input\run_20260906_003034\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\drone_3d_pipeline\data\input\run_20260906_174917\dji_fly_20260727_172754_0049_1785175051818_video.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)
  - `c:\Users\25beevdt025\Desktop\SIH\models and videos\V1.mp4` (154.53 MB, SHA-256 identical: `7cf250cfb4dd90e6...`)

### Scene 2: Desert Road (Candidate A & Candidate B)
- **Target Video:** `desert_road.mp4`
- **Physical Status:** **EXISTS**
- **Authoritative Absolute Path:** `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\desert_road.mp4`
- **Repository-Relative Path:** *External to repository (stored in SIH submodule workspace)*
- **File Size:** 6539814 bytes (6.24 MB)
- **Video Resolution:** 1280x720 (1280x720, HD)
- **Frame Rate:** 23.976 fps
- **Total Frame Count:** 477 frames
- **Video Duration:** 19.895 seconds
- **Video Codec:** `h264` (H.264 / AVC)
- **SHA-256 Hash:** `49b52f8a127db51724df71a0808e0308c09c293946e24fd26cb2d1066069de18`
- **Classification:** Original benchmark source video (VGGT-Omega examples)
- **Reconstruction Provenance:** Matches recorded SHA-256 in Sprint 9 run manifest and model catalog. Used for both static highway Candidate A and vehicle-preserving Candidate B.
- **Duplicate Copies Discovered on Machine:**
  - `c:\Users\25beevdt025\Desktop\coding\third_party\vggt-omega-main\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)
  - `c:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\third_party\vggt-omega-main\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\third_party\vggt_omega\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)
  - `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)
  - `c:\Users\25beevdt025\Desktop\sih1\third_party\vggt-omega-main\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)
  - `c:\Users\25beevdt025\Desktop\sih1\third_party\vggt_omega\examples\desert_road.mp4` (6.24 MB, SHA-256 identical: `49b52f8a127db517...`)

### Scene 3: Forest Road
- **Target Video:** `forest_road.mp4`
- **Physical Status:** **EXISTS**
- **Authoritative Absolute Path:** `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\forest_road.mp4`
- **Repository-Relative Path:** *External to repository (stored in SIH submodule workspace)*
- **File Size:** 10681415 bytes (10.19 MB)
- **Video Resolution:** 1280x720 (1280x720, HD)
- **Frame Rate:** 23.976 fps
- **Total Frame Count:** 730 frames
- **Video Duration:** 30.447 seconds
- **Video Codec:** `h264` (H.264 / AVC)
- **SHA-256 Hash:** `e5e847d9b3863d29c12501347074c58c6cc4e68fb91f4c963bbe3f91294a169a`
- **Classification:** Original benchmark source video (VGGT-Omega examples)
- **Reconstruction Provenance:** Matches recorded SHA-256 in Sprint 9 run manifest and model catalog.
- **Duplicate Copies Discovered on Machine:**
  - `c:\Users\25beevdt025\Desktop\coding\third_party\vggt-omega-main\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)
  - `c:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\third_party\vggt-omega-main\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)
  - `c:\Users\25beevdt025\Desktop\SIH\SIH\third_party\vggt_omega\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)
  - `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)
  - `c:\Users\25beevdt025\Desktop\sih1\third_party\vggt-omega-main\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)
  - `c:\Users\25beevdt025\Desktop\sih1\third_party\vggt_omega\examples\forest_road.mp4` (10.19 MB, SHA-256 identical: `e5e847d9b3863d29...`)

### Scene 4: River/Bridge
- **Target Video:** `segment_52s_64s.mp4`
- **Physical Status:** **EXISTS**
- **Authoritative Absolute Path:** `c:\Users\25beevdt025\Desktop\FORGE\outputs\dji_river_bridge_validation\segment_52s_64s.mp4`
- **Repository-Relative Path:** `outputs\dji_river_bridge_validation\segment_52s_64s.mp4`
- **File Size:** 70005949 bytes (66.76 MB)
- **Video Resolution:** 3840x2160 (3840x2160, 4K UHD)
- **Frame Rate:** 59.94 fps
- **Total Frame Count:** 750 frames
- **Video Duration:** 12.513 seconds
- **Video Codec:** `FMP4` (FMP4 / MPEG-4 Part 2)
- **SHA-256 Hash:** `0d1a647a0a0363bed400317a2b3495ad5926d469a5dd7a8b127fee037e7409cb`
- **Classification:** Trimmed segment source video (sliced 52s-64s from full DJI Air 3S flight 0050)
- **Reconstruction Provenance:** Matches recorded SHA-256 in Sprint 9 run manifest, `dji_river_bridge_validation/metadata.json`, and model catalog.
- **Raw Parent Drone Flight Video:**
  - **Filename:** `dji_fly_20260727_172812_0050_1785175896547_video.mp4`
  - **Location:** `c:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4`
  - **Size:** 737144565 bytes (703.0 MB)
  - **Resolution:** 3840x2160, 59.94 fps, 4965 frames, 82.833s
  - **SHA-256 Hash:** `b539bb2bb5977630528c40fd5df9605d9032c9b18daa8df38209ef040e57225f`
  - **Relationship:** `segment_52s_64s.mp4` represents the stationary hover segment from timestamp 52.0s to 64.0s of this full 82.8s flight recording.

---

## Exact locations of the four original videos

Copy and paste the following absolute Windows paths directly into File Explorer:

1. **Campus 0049:**
   `c:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4`

2. **Desert Road:**
   `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\desert_road.mp4`

3. **Forest Road:**
   `c:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main\examples\forest_road.mp4`

4. **River/Bridge:**
   `c:\Users\25beevdt025\Desktop\FORGE\outputs\dji_river_bridge_validation\segment_52s_64s.mp4`

*Optional Raw Parent Flight for River/Bridge:*
   `c:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4`
