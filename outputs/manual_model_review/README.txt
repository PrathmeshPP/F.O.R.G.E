F.O.R.G.E — Standalone 3D Models for Manual Inspection Package
Sprint 10.2 Technical Deliverable
National Level Hackathon Demonstration

This package contains actual 3D model files derived from 4 source video datasets:
1. campus_0049/               (Architectural Survey: multi-story buildings, walkways, pond)
2. desert_road_candidate_a/   (Clean static highway corridor, moving objects removed)
3. desert_road_candidate_b/   (Highway corridor preserving single reference vehicle at [0.03, 0.04, 0.45])
4. forest_road/               (Mountain forest canopy & winding two-lane paved roadway)
5. river_bridge/              (Concrete bridge deck, guard railings, vertical support piers & channel)

Each scene contains:
- model_geometry.ply          (Untextured geometric mesh)
- model_geometry.obj          (Untextured pure geometric Wavefront OBJ)
- model_photographic.ply      (Photographic vertex-colored PLY)
- model_textured.obj          (Textured Wavefront OBJ with UV coordinates)
- model_textured.mtl          (Material definition referencing texture_atlas.png)
- texture_atlas.png           (4096x4096 high-resolution texture map)
- model_preview.glb           (Self-contained glTF 2.0 binary with embedded textures)
- render_overview.png         (High-resolution overview preview)
- render_oblique.png          (High-resolution oblique perspective preview)
- metadata.json               (Provenance, vertex/face counts, bounding box, SHA-256 hashes)

How to open:
- Windows 3D Viewer: Double-click 'model_preview.glb' or 'model_textured.obj'
- Blender 3.x / 4.x: File -> Import -> glTF 2.0 (.glb) or Wavefront (.obj)
- MeshLab: File -> Import Mesh... -> choose .ply or .obj
- CloudCompare: File -> Open -> choose .ply or .obj
