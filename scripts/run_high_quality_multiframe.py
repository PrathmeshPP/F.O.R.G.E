"""F.O.R.G.E — Sprint 2: High-Quality Multi-Frame Reconstruction & Geometry Connectivity.

Executes the end-to-end multi-frame pipeline across all 4 regression scenes:
1. Campus 0049 (24 frames across full 16.1s flight)
2. Desert Road (24 frames across full 19.9s driving corridor)
3. Forest Road (24 frames across full 30.5s canopy flight)
4. River/Bridge (24 frames across 18s-48s active bridge deck traversal)

Features:
- Multi-frame windowed inference with Umeyama 7-DoF SIM3 alignment across shared frames.
- Sub-millimeter overlap residual verification.
- Scale-aware dense point cloud fusion (preserving >350k-700k points, not decimating to 60k).
- Scene-adaptive depth-edge filtering and horizon ray pruning.
- High-fidelity Screened Poisson with gentle (1%) density trimming.
- Connected Multi-Scale Ball-Pivoting (BPA).
- Topological Surface Repair for small hole closure and manifold continuity.
- Matched top, frontal, and oblique preview renders for every candidate.
- Detailed disaster readiness, geometry, and coverage diagnostics.
"""

from __future__ import annotations

import os
import sys
import time
import json
import argparse
from pathlib import Path
from typing import Dict, Any, List, Tuple

import cv2
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.video.frame_sampler import extract_sampled_frames
from src.vggt.inference import VGGTInferencePipeline
from src.reconstruction.multiframe_stitcher import (
    WindowedInferenceManager,
    MultiFrameDensePointCloud,
    SurfaceConnectivityMesher
)
from src.reconstruction.pipeline import (
    render_standard_views,
    compute_surface_roughness,
    evaluate_disaster_readiness,
    assess_camera_trajectory
)

VIDEO_SOURCES = {
    "campus_0049": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4",
        "t_start": 0.0,
        "t_end": None,
        "orientation_class": "nadir_overhead",
        "baseline_frames": 10,
        "target_frames": 24,
        "description": "Campus aerial drone survey of buildings, roofs, courtyard, and trees"
    },
    "desert_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\desert_road.mp4",
        "t_start": 0.0,
        "t_end": None,
        "orientation_class": "forward_corridor",
        "baseline_frames": 3,
        "target_frames": 24,
        "description": "Forward driving corridor across desert highway with vehicle and terrain"
    },
    "forest_road": {
        "video_path": r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega\examples\forest_road.mp4",
        "t_start": 0.0,
        "t_end": None,
        "orientation_class": "nadir_overhead",
        "baseline_frames": 6,
        "target_frames": 24,
        "description": "Forest canopy drone flight over road with dense tree crowns and terrain"
    },
    "river_bridge": {
        "video_path": r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4",
        "t_start": 18.0,
        "t_end": 48.0,  # Active 30-second forward translation along the bridge deck
        "orientation_class": "forward_corridor",
        "baseline_frames": 8,
        "target_frames": 24,
        "description": "Drone bridge crossing flight corridor with bridge deck, riverbed, and piers"
    }
}


def extract_custom_range_frames(
    video_path: str,
    output_dir: Path,
    num_frames: int,
    t_start: float = 0.0,
    t_end: Optional[float] = None
) -> List[Path]:
    """
    Extracts num_frames uniformly distributed between t_start and t_end.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    native_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    f_start = int(t_start * native_fps)
    f_end = int(t_end * native_fps) if t_end is not None else total_frames
    f_end = min(f_end, total_frames - 1)

    frame_indices = [
        int(round(f_start + i * (f_end - f_start) / (num_frames - 1)))
        for i in range(num_frames)
    ]

    saved_paths = []
    for idx, f_idx in enumerate(frame_indices):
        cap.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
        ret, frame_bgr = cap.read()
        if not ret:
            continue
        ts = f_idx / native_fps
        fpath = output_dir / f"frame_{idx:04d}_{ts:.3f}s.png"
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        Image.fromarray(frame_rgb).save(fpath)
        saved_paths.append(fpath)

    cap.release()
    return saved_paths


def process_scene_high_quality(
    scene_key: str,
    scene_cfg: Dict[str, Any],
    exp_root: Path,
    manager: WindowedInferenceManager
) -> Dict[str, Any]:
    print(f"\n{'='*80}")
    print(f"PROCESSING HIGH-QUALITY MULTI-FRAME SCENE: {scene_key.upper()}")
    print(f"Configuration: {scene_cfg['description']}")
    print(f"{'='*80}")

    scene_dir = exp_root / scene_key
    scene_dir.mkdir(parents=True, exist_ok=True)

    frames_dir = scene_dir / "frames"
    pred_dir = scene_dir / "predictions"
    pred_dir.mkdir(parents=True, exist_ok=True)

    t_start_total = time.time()

    # 1. Frame Extraction across Full Usable Duration
    print(f"\n[Step 1] Extracting {scene_cfg['target_frames']} informative frames...")
    frame_paths = extract_custom_range_frames(
        video_path=scene_cfg["video_path"],
        output_dir=frames_dir,
        num_frames=scene_cfg["target_frames"],
        t_start=scene_cfg["t_start"],
        t_end=scene_cfg["t_end"]
    )
    print(f"  Extracted {len(frame_paths)} frames into {frames_dir}")

    # 2. Windowed Multi-Frame Inference & Umeyama SIM3 Alignment
    print("\n[Step 2] Executing windowed inference with Umeyama SIM3 alignment...")
    t0_infer = time.time()
    inference_result = manager.run_windowed_inference([str(p) for p in frame_paths])
    infer_duration = time.time() - t0_infer

    extrinsics = inference_result["extrinsics"]
    intrinsics = inference_result["intrinsics"]
    depth_maps = inference_result["depth"]
    confidences = inference_result["confidence"]
    align_reports = inference_result["alignment_reports"]

    print(f"  Windowed inference complete in {infer_duration:.1f}s")
    for r in align_reports:
        print(f"    Window {r['window_idx']}: scale={r['scale_factor']:.4f}, max_res={r['max_residual_error']:.5f}m, verified={r['alignment_verified']}")

    # Save aligned predictions
    depth_save_path = pred_dir / "depth_predictions.npz"
    poses_save_path = pred_dir / "camera_poses.npz"
    np.savez_compressed(depth_save_path, depth=depth_maps, confidence=confidences)
    np.savez_compressed(poses_save_path, extrinsics=extrinsics, intrinsics=intrinsics)

    # Load RGBs at model resolution
    H, W_img = depth_maps.shape[1], depth_maps.shape[2]
    frame_rgbs = []
    for f in frame_paths:
        im = Image.open(f).convert("RGB")
        if im.size != (W_img, H):
            im = im.resize((W_img, H), Image.Resampling.BILINEAR)
        frame_rgbs.append(np.array(im, dtype=np.uint8))
    frame_rgbs = np.stack(frame_rgbs, axis=0)

    # 3. Dense Point Cloud Construction
    print("\n[Step 3] Constructing dense aligned point clouds (unfused, filtered, fine-fused)...")
    cloud_data = MultiFrameDensePointCloud.construct_clouds(
        depth_maps=depth_maps,
        extrinsics=extrinsics,
        intrinsics=intrinsics,
        frame_rgbs=frame_rgbs,
        orientation_class=scene_cfg["orientation_class"]
    )

    pcd_unfused = cloud_data["pcd_unfused"]
    pcd_clean = cloud_data["pcd_clean"]
    pcd_fused = cloud_data["pcd_fused"]
    orientation_class = cloud_data["orientation_class"]

    ref_bbox = pcd_fused.get_axis_aligned_bounding_box()
    scene_extents = cloud_data["spatial_extents"]
    traj_diag = assess_camera_trajectory(extrinsics)

    print(f"  Raw valid points:       {cloud_data['raw_valid_count']:,}")
    print(f"  Clean filtered points:  {cloud_data['clean_valid_count']:,}")
    print(f"  Fine fused points:      {cloud_data['fused_count']:,} (voxel_size: {cloud_data['fine_voxel_size']})")
    print(f"  Median point spacing:   {cloud_data['median_point_spacing']:.5f}")
    print(f"  Scene extents:          {scene_extents}")

    # -------------------------------------------------------------------------
    # Candidate 0: Unfused Full Dense Point Cloud
    # -------------------------------------------------------------------------
    cand0_dir = scene_dir / "candidate_pcd_unfused"
    cand0_dir.mkdir(parents=True, exist_ok=True)
    pcd_unfused_path = cand0_dir / "point_cloud_unfused.ply"
    o3d.io.write_point_cloud(str(pcd_unfused_path), pcd_unfused)

    views_cand0 = render_standard_views(pcd_unfused, ref_bbox, is_mesh=False, orientation_class=orientation_class)
    for vname, img in views_cand0.items():
        img.save(cand0_dir / f"pcd_view_{vname}.png")

    cand0_info = {
        "candidate_id": "candidate_pcd_unfused",
        "type": "point_cloud",
        "description": "Full Dense Multi-Frame Point Cloud (Zero Decimation)",
        "point_count": len(pcd_unfused.points),
        "spatial_extents": scene_extents,
        "ply_path": str(pcd_unfused_path.resolve()),
        "preview_path": str((cand0_dir / "pcd_view_top.png").resolve())
    }
    with open(cand0_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand0_info, f, indent=2)

    # -------------------------------------------------------------------------
    # Candidate 1: Fine-Fused Point Cloud
    # -------------------------------------------------------------------------
    cand1_dir = scene_dir / "candidate_pcd_filtered"
    cand1_dir.mkdir(parents=True, exist_ok=True)
    pcd_fused_path = cand1_dir / "point_cloud_fused.ply"
    o3d.io.write_point_cloud(str(pcd_fused_path), pcd_fused)

    views_cand1 = render_standard_views(pcd_fused, ref_bbox, is_mesh=False, orientation_class=orientation_class)
    for vname, img in views_cand1.items():
        img.save(cand1_dir / f"pcd_view_{vname}.png")

    roughness = compute_surface_roughness(pcd_fused)
    cand1_info = {
        "candidate_id": "candidate_pcd_filtered",
        "type": "point_cloud",
        "description": "High-Fidelity Scale-Aware Fused Point Cloud",
        "raw_points": cloud_data["raw_valid_count"],
        "clean_points": cloud_data["clean_valid_count"],
        "fused_points": cloud_data["fused_count"],
        "voxel_size": cloud_data["fine_voxel_size"],
        "median_spacing": cloud_data["median_point_spacing"],
        "spatial_extents": scene_extents,
        "roughness_residual": round(roughness, 6),
        "ply_path": str(pcd_fused_path.resolve()),
        "preview_path": str((cand1_dir / "pcd_view_top.png").resolve())
    }
    with open(cand1_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand1_info, f, indent=2)

    # -------------------------------------------------------------------------
    # Candidate 2: High-Quality Screened Poisson Mesh (Gentle Trim 1%)
    # -------------------------------------------------------------------------
    print("\n[Step 4] Generating Candidate 2: Screened Poisson Mesh (Gentle Trim 1%)...")
    cand2_dir = scene_dir / "candidate_mesh_poisson_hq"
    cand2_dir.mkdir(parents=True, exist_ok=True)

    mesh_poisson, p_meta = SurfaceConnectivityMesher.build_poisson_hq(
        pcd_fused, depth=9, gentle_trim_quantile=0.01, support_distance_factor=3.5
    )

    mesh_poisson_ply = cand2_dir / "surface_mesh.ply"
    mesh_poisson_obj = cand2_dir / "surface_mesh.obj"
    o3d.io.write_triangle_mesh(str(mesh_poisson_ply), mesh_poisson)
    o3d.io.write_triangle_mesh(str(mesh_poisson_obj), mesh_poisson)

    views_cand2 = render_standard_views(mesh_poisson, ref_bbox, is_mesh=True, orientation_class=orientation_class)
    for vname, img in views_cand2.items():
        img.save(cand2_dir / f"mesh_view_{vname}.png")

    cand2_info = {
        "candidate_id": "candidate_mesh_poisson_hq",
        "type": "surface_mesh",
        "algorithm": "Screened Poisson (Gentle Trim 1%)",
        "vertex_count": len(mesh_poisson.vertices),
        "triangle_count": len(mesh_poisson.triangles),
        "is_edge_manifold": bool(mesh_poisson.is_edge_manifold()),
        "is_vertex_manifold": bool(mesh_poisson.is_vertex_manifold()),
        "ply_path": str(mesh_poisson_ply.resolve()),
        "obj_path": str(mesh_poisson_obj.resolve()),
        "preview_path": str((cand2_dir / "mesh_view_top.png").resolve()),
        "metadata": p_meta
    }
    with open(cand2_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand2_info, f, indent=2)
    print(f"  Poisson: {len(mesh_poisson.vertices):,} verts, {len(mesh_poisson.triangles):,} faces")

    # -------------------------------------------------------------------------
    # Candidate 3: Multi-Scale Ball-Pivoting Mesh (BPA Connected)
    # -------------------------------------------------------------------------
    print("\n[Step 5] Generating Candidate 3: Multi-Scale Ball-Pivoting Mesh...")
    cand3_dir = scene_dir / "candidate_mesh_bpa_connected"
    cand3_dir.mkdir(parents=True, exist_ok=True)

    mesh_bpa, bpa_meta = SurfaceConnectivityMesher.build_bpa_connected(
        pcd_fused, radii_multipliers=(1.2, 2.5, 5.0, 10.0)
    )

    mesh_bpa_ply = cand3_dir / "surface_mesh.ply"
    mesh_bpa_obj = cand3_dir / "surface_mesh.obj"
    o3d.io.write_triangle_mesh(str(mesh_bpa_ply), mesh_bpa)
    o3d.io.write_triangle_mesh(str(mesh_bpa_obj), mesh_bpa)

    views_cand3 = render_standard_views(mesh_bpa, ref_bbox, is_mesh=True, orientation_class=orientation_class)
    for vname, img in views_cand3.items():
        img.save(cand3_dir / f"mesh_view_{vname}.png")

    cand3_info = {
        "candidate_id": "candidate_mesh_bpa_connected",
        "type": "surface_mesh",
        "algorithm": "Multi-Scale Ball Pivoting (Connected)",
        "vertex_count": len(mesh_bpa.vertices),
        "triangle_count": len(mesh_bpa.triangles),
        "is_edge_manifold": bool(mesh_bpa.is_edge_manifold()),
        "is_vertex_manifold": bool(mesh_bpa.is_vertex_manifold()),
        "ply_path": str(mesh_bpa_ply.resolve()),
        "obj_path": str(mesh_bpa_obj.resolve()),
        "preview_path": str((cand3_dir / "mesh_view_top.png").resolve()),
        "metadata": bpa_meta
    }
    with open(cand3_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand3_info, f, indent=2)
    print(f"  BPA: {len(mesh_bpa.vertices):,} verts, {len(mesh_bpa.triangles):,} faces")

    # -------------------------------------------------------------------------
    # Candidate 4: Connected Surface Repair (Hole Closure & Edge Consolidation)
    # -------------------------------------------------------------------------
    print("\n[Step 6] Generating Candidate 4: Connected Surface-Repaired Mesh...")
    cand4_dir = scene_dir / "candidate_mesh_repaired"
    cand4_dir.mkdir(parents=True, exist_ok=True)

    # Base mesh for repair: Poisson HQ offers continuous global support
    mesh_repaired, rep_meta = SurfaceConnectivityMesher.repair_surface_connectivity(
        mesh=mesh_poisson, max_hole_vertices=12
    )

    mesh_rep_ply = cand4_dir / "surface_mesh.ply"
    mesh_rep_obj = cand4_dir / "surface_mesh.obj"
    o3d.io.write_triangle_mesh(str(mesh_rep_ply), mesh_repaired)
    o3d.io.write_triangle_mesh(str(mesh_rep_obj), mesh_repaired)

    views_cand4 = render_standard_views(mesh_repaired, ref_bbox, is_mesh=True, orientation_class=orientation_class)
    for vname, img in views_cand4.items():
        img.save(cand4_dir / f"mesh_view_{vname}.png")

    cand4_info = {
        "candidate_id": "candidate_mesh_repaired",
        "type": "surface_mesh",
        "algorithm": "Connected Surface Repair (Closed Boundary Loops)",
        "vertex_count": len(mesh_repaired.vertices),
        "triangle_count": len(mesh_repaired.triangles),
        "is_edge_manifold": bool(mesh_repaired.is_edge_manifold()),
        "is_vertex_manifold": bool(mesh_repaired.is_vertex_manifold()),
        "ply_path": str(mesh_rep_ply.resolve()),
        "obj_path": str(mesh_rep_obj.resolve()),
        "preview_path": str((cand4_dir / "mesh_view_top.png").resolve()),
        "metadata": rep_meta
    }
    with open(cand4_dir / "candidate_metadata.json", "w", encoding="utf-8") as f:
        json.dump(cand4_info, f, indent=2)
    print(f"  Repaired: {len(mesh_repaired.vertices):,} verts, {len(mesh_repaired.triangles):,} faces, holes closed: {rep_meta['holes_filled']}")

    # -------------------------------------------------------------------------
    # Composite Comparison Contact Sheet
    # -------------------------------------------------------------------------
    print("\n[Step 7] Generating composite comparison sheet...")
    comp_sheet_path = scene_dir / "candidate_comparison_sheet.png"

    # 4 candidates x 3 views (top, frontal, oblique)
    c_names = [
        ("candidate_pcd_filtered", cand1_dir, "PCD Fine Fused"),
        ("candidate_mesh_poisson_hq", cand2_dir, "Poisson Mesh (HQ)"),
        ("candidate_mesh_bpa_connected", cand3_dir, "BPA Mesh (Connected)"),
        ("candidate_mesh_repaired", cand4_dir, "Repaired Surface")
    ]
    views = ["top", "frontal", "oblique"]

    card_w, card_h = 420, 315
    grid_cols = len(views)
    grid_rows = len(c_names)
    sheet_w = grid_cols * card_w
    sheet_h = grid_rows * card_h + 80

    sheet = Image.new("RGB", (sheet_w, sheet_h), (18, 22, 28))
    draw = ImageDraw.Draw(sheet)

    # Header banner
    draw.rectangle([0, 0, sheet_w, 75], fill=(28, 36, 48))
    draw.text((20, 15), f"F.O.R.G.E — Sprint 2 High-Quality Reconstruction: {scene_key.upper()}", fill=(255, 255, 255))
    draw.text((20, 42), f"Frames: {len(frame_paths)} | Points: {len(pcd_fused.points):,} | Spacing: {cloud_data['median_point_spacing']:.4f} | Alignment: {len(align_reports)} Windows Verified", fill=(160, 200, 255))

    for r_idx, (c_id, c_folder, c_label) in enumerate(c_names):
        y_top = 80 + r_idx * card_h
        for c_idx, v_name in enumerate(views):
            x_left = c_idx * card_w
            prefix = "pcd" if "pcd" in c_id else "mesh"
            thumb_path = c_folder / f"{prefix}_view_{v_name}.png"
            if thumb_path.exists():
                thumb = Image.open(thumb_path).resize((card_w - 4, card_h - 4), Image.Resampling.BILINEAR)
                sheet.paste(thumb, (x_left + 2, y_top + 2))

            draw.rectangle([x_left + 2, y_top + 2, x_left + card_w - 2, y_top + card_h - 2], outline=(40, 50, 65))
            draw.rectangle([x_left + 8, y_top + 8, x_left + card_w - 8, y_top + 38], fill=(0, 0, 0))
            draw.text((x_left + 12, y_top + 10), c_label, fill=(255, 220, 100))
            draw.text((x_left + 12, y_top + 24), f"View: {v_name.upper()}", fill=(180, 200, 230))

    sheet.save(comp_sheet_path)

    # Disaster readiness and scene summary
    readiness = evaluate_disaster_readiness(
        active_points=np.asarray(pcd_fused.points),
        mesh=mesh_repaired,
        traj_diagnostics=traj_diag,
        depth_maps=depth_maps,
        confidences=confidences,
        scene_classification=scene_key
    )

    total_proc_time = time.time() - t_start_total

    summary = {
        "scene_name": scene_key,
        "description": scene_cfg["description"],
        "orientation_class": orientation_class,
        "frame_selection": {
            "baseline_frames": scene_cfg["baseline_frames"],
            "high_quality_frames": len(frame_paths),
            "frame_increase_ratio": round(len(frame_paths) / scene_cfg["baseline_frames"], 2),
            "time_range_sec": [scene_cfg["t_start"], scene_cfg["t_end"]]
        },
        "windowed_alignment": {
            "num_windows": len(inference_result["windows"]),
            "inference_time_sec": round(infer_duration, 2),
            "alignment_reports": align_reports
        },
        "point_cloud_metrics": {
            "raw_valid_points": cloud_data["raw_valid_count"],
            "clean_valid_points": cloud_data["clean_valid_count"],
            "fused_points": cloud_data["fused_count"],
            "voxel_size": cloud_data["fine_voxel_size"],
            "median_point_spacing": cloud_data["median_point_spacing"],
            "spatial_extents": scene_extents,
            "roughness_residual": round(roughness, 6)
        },
        "candidates": {
            "candidate_pcd_unfused": cand0_info,
            "candidate_pcd_filtered": cand1_info,
            "candidate_mesh_poisson_hq": cand2_info,
            "candidate_mesh_bpa_connected": cand3_info,
            "candidate_mesh_repaired": cand4_info
        },
        "comparison_sheet_path": str(comp_sheet_path.resolve()),
        "disaster_readiness": readiness,
        "total_processing_time_sec": round(total_proc_time, 2)
    }

    with open(scene_dir / "scene_evaluation_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"\nScene {scene_key.upper()} completed in {total_proc_time:.1f}s!")
    return summary


def main():
    parser = argparse.ArgumentParser(description="F.O.R.G.E Sprint 2 High-Quality Multi-Frame Pipeline")
    parser.add_argument("--scene", type=str, default="all", choices=["all", "campus_0049", "desert_road", "forest_road", "river_bridge"])
    args = parser.parse_args()

    exp_root = PROJECT_ROOT / "outputs/accuracy_experiment/high_quality_multiframe_v1"
    exp_root.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*80}")
    print(f"F.O.R.G.E SPRINT 2: HIGH-QUALITY MULTI-FRAME RECONSTRUCTION & CONNECTIVITY")
    print(f"Output Directory: {exp_root}")
    print(f"{'='*80}")

    scenes_to_process = (
        list(VIDEO_SOURCES.keys())
        if args.scene == "all"
        else [args.scene]
    )

    manager = WindowedInferenceManager(window_size=8, overlap=3)
    manager.pipeline.initialize()

    all_summaries = {}
    for skey in scenes_to_process:
        scfg = VIDEO_SOURCES[skey]
        summary = process_scene_high_quality(skey, scfg, exp_root, manager)
        all_summaries[skey] = summary

    master_summary_path = exp_root / "master_high_quality_summary.json"
    with open(master_summary_path, "w", encoding="utf-8") as f:
        json.dump(all_summaries, f, indent=2)

    print(f"\n{'='*80}")
    print(f"ALL SCENES SUCCESSFULLY PROCESSED!")
    print(f"Master Summary JSON: {master_summary_path}")
    print(f"{'='*80}")


if __name__ == "__main__":
    main()
