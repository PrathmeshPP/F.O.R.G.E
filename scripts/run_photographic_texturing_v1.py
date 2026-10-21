"""F.O.R.G.E — Sprint 4: Photographic Texture Realism and Multi-View Appearance Reconstruction.

Master Execution Script:
Executes the photogrammetric appearance reconstruction pipeline across all 4 benchmark scenes:
- campus_0049
- desert_road
- forest_road
- river_bridge

Strictly outputs to:
outputs/accuracy_experiment/photographic_texturing_v1/<scene>/
"""

from __future__ import annotations

import os
import sys
import time
import json
import shutil
import argparse
from pathlib import Path
from typing import Dict, Any, List, Tuple

import cv2
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.photographic_texture import (
    compute_geometry_hash,
    extract_native_frames,
    scale_camera_intrinsics,
    MultiViewTextureEngine
)
from src.reconstruction.pipeline import render_standard_views

TEXTURING_SCENES = {
    "campus_0049": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4",
        "baseline_mesh_path": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/surface_mesh_repaired.ply",
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/predictions",
        "orientation_class": "nadir_overhead",
        "atlas_size": 4096,
        "moving_object_strategy": None,
        "description": "Campus aerial drone survey: building facades, solar roofs, walkways, and courtyard"
    },
    "desert_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\desert_road.mp4",
        "baseline_mesh_path": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/surface_mesh_repaired.ply",
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions",
        "orientation_class": "forward_corridor",
        "atlas_size": 2048,
        "moving_object_strategy": "desert_road_vehicle", # Dynamic vehicle isolation
        "description": "Desert highway corridor: dark asphalt road, yellow dashed lines, blue Mini Cooper, and sand shoulders"
    },
    "forest_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\forest_road.mp4",
        "baseline_mesh_path": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/surface_mesh_repaired.ply",
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/predictions",
        "orientation_class": "nadir_overhead",
        "atlas_size": 4096,
        "moving_object_strategy": None,
        "description": "Forest canopy drone survey: coniferous & deciduous crowns, needle foliage, and asphalt roadway"
    },
    "river_bridge": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4",
        "baseline_mesh_path": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/surface_mesh_repaired.ply",
        "predictions_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/predictions",
        "orientation_class": "forward_corridor",
        "atlas_size": 4096,
        "moving_object_strategy": None,
        "description": "Bridge crossing & riverbed corridor: concrete construction, red dirt roadway, riverbank terrain"
    }
}


def render_comparison_panels(
    mesh_baseline: o3d.geometry.TriangleMesh,
    mesh_photo_ply: o3d.geometry.TriangleMesh,
    atlas_image_path: Path,
    orientation_class: str,
    output_png: Path,
    scene_name: str,
    meta_info: Dict[str, Any]
) -> None:
    """Renders whole-scene and close-up comparisons into a matched 4-panel comparison contact sheet."""
    W_panel, H_panel = 640, 480
    header_h = 75
    total_w = W_panel * 4
    total_h = header_h + H_panel

    canvas = Image.new("RGB", (total_w, total_h), (14, 18, 24))
    draw = ImageDraw.Draw(canvas)

    # Header
    title = f"F.O.R.G.E SPRINT 4 APPEARANCE COMPARISON — {scene_name.upper()}"
    subtitle = f"Multi-View Photographic Texturing vs Averaged Vertex Colors | {meta_info.get('description', '')}"
    draw.text((25, 14), title, fill=(255, 255, 255))
    draw.text((25, 42), subtitle, fill=(180, 205, 230))

    ref_bbox = mesh_baseline.get_axis_aligned_bounding_box()
    ctr_pt = ref_bbox.get_center()

    def capture_mesh_view(m, zoom=0.75, closeup=False):
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=W_panel, height=H_panel)
        vis.add_geometry(m)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.10, 0.14])
        opt.mesh_color_option = o3d.visualization.MeshColorOption.Color

        ctrl = vis.get_view_control()
        ctrl.set_lookat(ctr_pt)
        if orientation_class == "nadir_overhead":
            if not closeup:
                ctrl.set_front([0.0, 0.0, -1.0])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.75)
            else:
                ctrl.set_front([0.3, 0.4, -0.85])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.30) # High-mag zoom
        else:
            if not closeup:
                ctrl.set_front([0.0, -0.2, -0.98])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.75)
            else:
                ctrl.set_front([0.0, -0.3, -0.95])
                ctrl.set_up([0.0, -1.0, 0.0])
                ctrl.set_zoom(0.30) # High-mag zoom

        vis.poll_events()
        vis.update_renderer()
        buf = vis.capture_screen_float_buffer(do_render=True)
        vis.destroy_window()
        return Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))

    # Panel 1: Baseline Vertex-Color Overview
    img_base = capture_mesh_view(mesh_baseline, closeup=False)
    canvas.paste(img_base, (0, header_h))

    # Panel 2: Photographic Appearance Overview
    img_photo = capture_mesh_view(mesh_photo_ply, closeup=False)
    canvas.paste(img_photo, (W_panel, header_h))

    # Panel 3: Close-Up Zoom (Detail Comparison)
    img_zoom = capture_mesh_view(mesh_photo_ply, closeup=True)
    canvas.paste(img_zoom, (W_panel * 2, header_h))

    # Panel 4: Texture Atlas Thumbnail
    atlas_img = Image.open(atlas_image_path).resize((W_panel, H_panel), Image.Resampling.LANCZOS)
    canvas.paste(atlas_img, (W_panel * 3, header_h))

    # Badges
    panels = [
        (0, "1. Candidate A: Baseline Colors", "(Averaged Vertex Gradients)", (240, 140, 80)),
        (W_panel, "2. Candidate B: Photographic Appearance", "(Direct Fronto-Parallel Rays)", (100, 240, 160)),
        (W_panel * 2, "3. Zoom Inspection (Photographic Detail)", "(Sharp Boundaries & Markings)", (100, 220, 255)),
        (W_panel * 3, "4. Candidate C: UV Texture Atlas", f"({atlas_image_path.name})", (255, 220, 100))
    ]

    for x_off, p_title, p_sub, p_col in panels:
        draw.rectangle([x_off + 12, header_h + 12, x_off + 380, header_h + 54], fill=(10, 14, 20))
        draw.text((x_off + 20, header_h + 16), p_title, fill=p_col)
        draw.text((x_off + 20, header_h + 34), p_sub, fill=(200, 220, 240))

    output_png.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_png, format="PNG")


def process_scene_texturing(
    scene_key: str,
    scene_cfg: Dict[str, Any],
    out_root: Path
) -> Dict[str, Any]:
    print(f"\n{'='*80}")
    print(f"SPRINT 4 PHOTOGRAPHIC TEXTURING: {scene_key.upper()}")
    print(f"Objective: {scene_cfg['description']}")
    print(f"{'='*80}")

    scene_out = out_root / scene_key
    scene_out.mkdir(parents=True, exist_ok=True)
    t0_start = time.time()

    # Step 1: Load and Verify Baseline Geometry
    print("\n[Step 1/6] Loading and verifying baseline geometry...")
    baseline_ply = scene_cfg["baseline_mesh_path"]
    if not baseline_ply.is_file():
        raise FileNotFoundError(f"Baseline mesh not found: {baseline_ply}")

    mesh_baseline = o3d.io.read_triangle_mesh(str(baseline_ply))
    baseline_hash = compute_geometry_hash(mesh_baseline)
    verts_orig = np.asarray(mesh_baseline.vertices).copy()
    tris_orig = np.asarray(mesh_baseline.triangles).copy()

    print(f"  Loaded baseline: {len(verts_orig):,} vertices, {len(tris_orig):,} triangles")
    print(f"  Baseline SHA256 Geometry Hash: {baseline_hash}")

    # Save immutable baseline copy
    immutable_baseline_ply = scene_out / "surface_mesh_baseline.ply"
    o3d.io.write_triangle_mesh(str(immutable_baseline_ply), mesh_baseline)

    # Step 2: Ingest Camera Poses & Depth Predictions
    print("\n[Step 2/6] Loading camera poses and depth maps...")
    pred_dir = scene_cfg["predictions_dir"]
    poses_data = np.load(pred_dir / "camera_poses.npz")
    depth_data = np.load(pred_dir / "depth_predictions.npz")

    extrinsics = poses_data["extrinsics"]
    intrinsics = poses_data["intrinsics"]
    depth_maps = depth_data["depth"]

    # Retrieve Timestamps from cached frames
    cached_frame_paths = sorted((scene_cfg["baseline_mesh_path"].parent / "frames").glob("*.png"))
    timestamps = []
    for p in cached_frame_paths:
        parts = p.stem.split("_")
        for part in parts:
            if part.endswith("s"):
                try:
                    timestamps.append(float(part[:-1]))
                except ValueError:
                    pass

    # Step 3: Extract Native Uncompressed High-Resolution Video Frames
    print("\n[Step 3/6] Extracting native uncompressed frames from source video...")
    native_frames_dir = scene_out / "native_frames"
    extracted_frames_info = extract_native_frames(
        video_path=scene_cfg["video_path"],
        timestamps=timestamps,
        out_dir=native_frames_dir
    )
    native_frame_paths = [info[0] for info in extracted_frames_info]
    native_w, native_h = extracted_frames_info[0][1], extracted_frames_info[0][2]
    infer_h, infer_w = depth_maps.shape[1], depth_maps.shape[2]

    print(f"  Extracted {len(native_frame_paths)} frames at native resolution: {native_w}x{native_h}")
    print(f"  Inference resolution was: {infer_w}x{infer_h}")

    # Scale camera intrinsics to native resolution
    scaled_intrinsics = scale_camera_intrinsics(
        intrinsics=intrinsics,
        infer_res=(infer_w, infer_h),
        native_res=(native_w, native_h)
    )

    # Step 4: Multi-View Occlusion & Visibility Analysis
    print("\n[Step 4/6] Executing multi-view visibility and occlusion analysis...")
    engine = MultiViewTextureEngine(
        mesh=mesh_baseline,
        extrinsics=extrinsics,
        intrinsics_native=scaled_intrinsics,
        frame_paths=native_frame_paths,
        depth_maps=depth_maps,
        moving_object_strategy=scene_cfg["moving_object_strategy"]
    )

    best_cam_per_face, quality_scores, vis_stats = engine.compute_face_visibility_and_scores()
    print(f"  Photographic face coverage: {vis_stats['photographic_coverage_pct']}% ({vis_stats['direct_photographic_faces']:,} / {vis_stats['total_faces']:,} faces)")

    # Step 5: Export Models
    print("\n[Step 5/6] Exporting Multi-Material OBJ, Unified UV Atlas OBJ, and Photographic PLY...")
    # 5A: Multi-Material Photographic OBJ (1:1 direct mapping to native frames)
    multiview_obj_path = scene_out / "multiview_textured_mesh.obj"
    engine.export_multiview_obj(multiview_obj_path, best_cam_per_face)

    # 5B: Unified Texture Atlas Model
    atlas_obj_path = scene_out / "textured_mesh.obj"
    atlas_obj_path, atlas_png_path, atlas_stats = engine.export_packed_atlas_obj(
        output_obj_path=atlas_obj_path,
        best_cam_per_face=best_cam_per_face,
        atlas_size=scene_cfg["atlas_size"]
    )

    # 5C: Photographic Vertex-Colored PLY
    photo_ply_path = scene_out / "surface_mesh_photographic.ply"
    engine.export_photographic_vertex_ply(photo_ply_path, best_cam_per_face)

    # Verify that geometry was 100% preserved
    mesh_after = o3d.io.read_triangle_mesh(str(photo_ply_path))
    after_hash = compute_geometry_hash(mesh_after)
    assert baseline_hash == after_hash, f"CRITICAL ERROR: Geometry hash changed! {baseline_hash} != {after_hash}"
    print(f"  [GEOMETRY GATE PASS] Verified exact geometry preservation: {after_hash}")

    # Step 6: Render Multi-Panel Comparative Contact Sheets
    print("\n[Step 6/6] Generating matched multi-panel comparative contact sheets...")
    contact_sheet_path = scene_out / "texture_comparison_contact_sheet.png"
    render_comparison_panels(
        mesh_baseline=mesh_baseline,
        mesh_photo_ply=mesh_after,
        atlas_image_path=atlas_png_path,
        orientation_class=scene_cfg["orientation_class"],
        output_png=contact_sheet_path,
        scene_name=scene_key,
        meta_info=scene_cfg
    )

    # Also render single views
    ref_bbox = mesh_baseline.get_axis_aligned_bounding_box()
    views_photo = render_standard_views(mesh_after, ref_bbox, is_mesh=True, orientation_class=scene_cfg["orientation_class"])
    for vname, img in views_photo.items():
        img.save(scene_out / f"render_photographic_view_{vname}.png")

    total_time = time.time() - t0_start

    # Comprehensive Metadata JSON
    metadata = {
        "pipeline": "F.O.R.G.E Sprint 4: Photographic Texture Realism",
        "scene_name": scene_key,
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "description": scene_cfg["description"],
        "source_video": scene_cfg["video_path"],
        "geometry_integrity_verification": {
            "baseline_mesh_path": str(scene_cfg["baseline_mesh_path"].resolve()),
            "immutable_baseline_copy": str(immutable_baseline_ply.resolve()),
            "vertex_count": len(verts_orig),
            "triangle_count": len(tris_orig),
            "sha256_hash_baseline": baseline_hash,
            "sha256_hash_textured": after_hash,
            "geometry_preserved": True
        },
        "optical_calibration": {
            "native_video_resolution": [native_w, native_h],
            "inference_resolution": [infer_w, infer_h],
            "scaling_factors_sx_sy": [round(native_w / infer_w, 4), round(native_h / infer_h, 4)],
            "frames_ingested": len(native_frame_paths),
            "frame_timestamps_sec": timestamps
        },
        "visibility_and_occlusion": {
            "moving_object_strategy": scene_cfg["moving_object_strategy"],
            "photographic_coverage_pct": vis_stats["photographic_coverage_pct"],
            "direct_photographic_faces": vis_stats["direct_photographic_faces"],
            "total_faces": vis_stats["total_faces"],
            "per_camera_face_distribution": vis_stats["per_camera_face_counts"]
        },
        "texture_deliverables": {
            "multiview_obj": {
                "obj_path": str(multiview_obj_path.resolve()),
                "mtl_path": str((scene_out / "multiview_textured_mesh.mtl").resolve()),
                "materials_count": len(native_frame_paths),
                "texture_resolution": f"{native_w}x{native_h} per material"
            },
            "unified_atlas_obj": {
                "obj_path": str(atlas_obj_path.resolve()),
                "mtl_path": str((scene_out / "textured_mesh.mtl").resolve()),
                "texture_atlas_png": str(atlas_png_path.resolve()),
                "atlas_resolution": [scene_cfg["atlas_size"], scene_cfg["atlas_size"]],
                "atlas_size_mb": atlas_stats["atlas_size_mb"]
            },
            "photographic_vertex_ply": {
                "ply_path": str(photo_ply_path.resolve())
            }
        },
        "renders": {
            "comparison_contact_sheet": str(contact_sheet_path.resolve()),
            "view_top": str((scene_out / "render_photographic_view_top.png").resolve()),
            "view_oblique": str((scene_out / "render_photographic_view_oblique.png").resolve()),
            "view_frontal": str((scene_out / "render_photographic_view_frontal.png").resolve())
        },
        "total_processing_time_sec": round(total_time, 2)
    }

    meta_path = scene_out / "texture_metadata.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    print(f"\n[SUMMARY] Scene {scene_key} texturing completed successfully in {total_time:.2f}s!")
    print(f"  Multi-Material OBJ: {multiview_obj_path.resolve()}")
    print(f"  Unified Atlas OBJ:  {atlas_obj_path.resolve()}")
    print(f"  Photographic PLY:   {photo_ply_path.resolve()}")
    print(f"  Metadata JSON:      {meta_path.resolve()}")
    return metadata


def main():
    parser = argparse.ArgumentParser(description="F.O.R.G.E Sprint 4 Photographic Texturing")
    parser.add_argument("--scenes", nargs="+", default=["campus_0049", "desert_road", "forest_road", "river_bridge"])
    parser.add_argument("--out-dir", type=str, default="outputs/accuracy_experiment/photographic_texturing_v1")
    args = parser.parse_args()

    out_root = (PROJECT_ROOT / args.out_dir).resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    print("=" * 80)
    print("F.O.R.G.E SPRINT 4: PHOTOGRAPHIC TEXTURE REALISM & APPEARANCE RECONSTRUCTION")
    print(f"Output Directory: {out_root}")
    print(f"Target Scenes: {args.scenes}")
    print("=" * 80)

    all_summaries = {}
    for sk in args.scenes:
        if sk not in TEXTURING_SCENES:
            print(f"[ERROR] Scene {sk} not in TEXTURING_SCENES, skipping.")
            continue
        cfg = TEXTURING_SCENES[sk]
        summary = process_scene_texturing(sk, cfg, out_root)
        all_summaries[sk] = summary

    # Master Report
    master_report = {
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "output_directory": str(out_root),
        "scenes_processed": list(all_summaries.keys()),
        "scenes": all_summaries
    }
    master_file = out_root / "sprint4_texturing_master_report.json"
    with open(master_file, "w", encoding="utf-8") as f:
        json.dump(master_report, f, indent=2)

    print("\n" + "=" * 80)
    print("ALL SPRINT 4 SCENES TEXTURED AND VALIDATED SUCCESSFULLY")
    print(f"Master Report JSON: {master_file}")
    print("=" * 80)


if __name__ == "__main__":
    main()
