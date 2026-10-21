# F.O.R.G.E — Sprint 10 Demonstration Readiness Report
**Release Version:** Sprint 10 National Hackathon Release  
**Timestamp:** `2026-10-20T20:59:07Z`  
**One-Command Launcher:** [`start_forge_explorer.bat`](file:///c:/Users/25beevdt025/Desktop/FORGE/start_forge_explorer.bat)  
**Live Application URL:** `http://localhost:5173/`  
**Model Baseline Freeze:** [`outputs/accuracy_experiment/sprint9_video_to_viewer_release/`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint9_video_to_viewer_release)  
**Live Demo Guide:** [`docs/SPRINT10_LIVE_DEMO_GUIDE.md`](file:///c:/Users/25beevdt025/Desktop/FORGE/docs/SPRINT10_LIVE_DEMO_GUIDE.md)  

---

## 1. Executive Summary & Acceptance Gates

Sprint 10 hardens the F.O.R.G.E Explorer platform and verified Sprint 9 reconstruction outputs into an engineering-grade, reliable, presentation-ready demonstration for hackathon judges and evaluators.

All acceptance gates are met:
1. **One-Command Launch:** [`start_forge_explorer.bat`](file:///c:/Users/25beevdt025/Desktop/FORGE/start_forge_explorer.bat) detects Python, validates the model release baseline and production build, avoids duplicate port 5173 instances, and launches the browser automatically.
2. **True Model Provenance:** All 5 scene entries load verified Sprint 9 3D assets directly from `outputs/accuracy_experiment/sprint9_video_to_viewer_release/`. No synthetic mockups or outdated Sprint 7H files are referenced.
3. **Interactive 7-Stage Workflow Modal (`P` Shortcut):** Communicates the core problem F.O.R.G.E solves, the 7-stage feed-forward photogrammetry pipeline, and includes honest academic/industry methodology disclosures.
4. **Desert Road Comparison (`C` Shortcut):** Side-by-side synchronized dual-viewport inspection contrasting Candidate A (clean static highway corridor) and Candidate B (GrabCut shape-aware reference vehicle preservation without rectangular road steps).
5. **Dependable Asset Export:** One-click JSZip packaging downloads OBJ + MTL + 4K Texture Atlas with local relative references and intact coordinates.
6. **Regression Verification:** Full 18/18 multi-sprint test suite passing (100% pass rate).
7. **Real Browser Validation:** Microsoft Edge headless captures of all 5 scene models and dual-viewport mode with zero console errors.

---

## 2. Benchmark Scenes & Demonstration Matrix

| Benchmark Target | Active Format & Size | Vertices / Faces | Bounding Box ($X, Y, Z$) | Acceptance Status | Demonstration Highlight |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Campus 0049** | Textured OBJ (31.28 MB)<br>Photo PLY (9.84 MB) | 128,336 v<br>253,487 f | Width: 2.15u<br>Height: 0.88u<br>Depth: 1.95u | **Conditional PARTIAL** | Roofs, walkways, pond, and trees. Honest disclosure of drone hover parallax limitation (<0.10m translation). |
| **Desert Road (Cand A)** | Textured OBJ (14.30 MB)<br>Photo PLY (4.08 MB) | 53,069 v<br>105,467 f | Width: 1.85u<br>Height: 0.25u<br>Depth: 3.84u | **PASS** | Clean highway corridor. Moving vehicles, shadows, and balloons completely masked across all frames. 0 interior holes. |
| **Desert Road (Cand B)** | Textured OBJ (14.31 MB)<br>Photo PLY (4.08 MB) | 53,101 v<br>105,518 f | Width: 1.85u<br>Height: 0.25u<br>Depth: 3.84u | **PASS** | Reference Mini Cooper preserved via pixel-accurate GrabCut instance mask. Zero rectangular road step. |
| **Forest Road** | Textured OBJ (117.41 MB)<br>Photo PLY (35.56 MB) | 462,896 v<br>919,292 f | Width: 2.30u<br>Height: 1.15u<br>Depth: 2.80u | **PASS** | Complex organic vegetation: closed tree-crown manifolds, open-air canopy voids, and downward view of double yellow lane lines. |
| **River/Bridge** | Textured OBJ (32.27 MB)<br>Photo PLY (10.14 MB) | 132,424 v<br>260,685 f | Width: 2.45u<br>Height: 0.95u<br>Depth: 2.20u | **PASS** | Targeted pinhole camera framing bridge deck, railings, and vertical piers at [1.131, 0.166, 0.526]; specular water noise filtered. |

---

## 3. Demonstration Screenshots & Evidence

Captured live from the running F.O.R.G.E Explorer platform under [`outputs/accuracy_experiment/sprint10_demo_release/screenshots/`](file:///c:/Users/25beevdt025/Desktop/FORGE/outputs/accuracy_experiment/sprint10_demo_release/screenshots/):
1. `demo_campus_0049.png` (395 KB): Institutional campus buildings, walkways, and pond.
2. `demo_desert_candidate_a.png` (299 KB): Clean highway corridor with zero vehicles and seamless shoulders.
3. `demo_desert_candidate_b.png` (301 KB): Preserved Mini Cooper vehicle with natural asphalt integration.
4. `demo_desert_comparison.png` (358 KB): Side-by-side synchronized comparison mode.
5. `demo_forest_road.png` (418 KB): Forest canopy voids and road markings.
6. `demo_river_bridge.png` (382 KB): Structural bridge deck, railings, piers, and channel.

---

## 4. Test Suite and Build Summary

- **Production Build:** `viewer/dist/` compiled via Vite 8.3.4 (404ms build time).
- **Automated Regression Suite:**
  ```bash
  pytest tests/test_sprint9_video_to_viewer.py tests/test_sprint8_viewer_platform.py tests/test_sprint7h_final_defect_recovery.py -v
  ```
  **18 passed in 2.30s (100% pass rate)**.
