"""F.O.R.G.E — Sprint 11 Recovery: End-to-End 3D Model Generation & Validation.

Generates and exports actual, inspectable 3D models (OBJ, PLY, GLB, and preview PNGs)
directly into outputs/dynamic_object_masking/sprint11/<scene_name>/ with YOLO dynamic-object masking.

Strictly local operations: No git push, no commits, no remote modifications.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
import trimesh
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Search candidate directories for external vggt_omega repository if needed
for _candidate_vggt_dir in [
    r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt_omega",
    r"C:\Users\25beevdt025\Desktop\SIH\third_party\vggt-omega-main",
    r"C:\Users\25beevdt025\Desktop\coding\third_party\vggt_omega",
]:
    if os.path.isdir(_candidate_vggt_dir) and _candidate_vggt_dir not in sys.path:
        sys.path.insert(0, _candidate_vggt_dir)

from src.detection.yolo_segmenter import YOLOInstanceSegmenter
from src.detection.temporal_tracker import LightweightTemporalTracker
from src.detection.visualizer import create_diagnostic_tripanel, render_detection_overlay
from src.reconstruction.pipeline import (
    GeneralReconstructionPipeline,
    PipelineConfig,
    render_standard_views
)


SCENE_CONFIGS: Dict[str, Dict[str, Any]] = {
    "desert_road": {
        "video_path": PROJECT_ROOT / "data" / "source_videos" / "desert_road.mp4",
        "frames_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/frames",
        "depth_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions/depth_predictions.npz",
        "poses_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/desert_road/predictions/camera_poses.npz",
        "classification": "forward_corridor",
        "poisson_depth": 8,
        "density_quantile": 0.01,
        "voxel_size": "auto",
        "dilation": 6,
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"]
    },
    "campus_0049": {
        "video_path": PROJECT_ROOT / "data" / "source_videos" / "dji_fly_20260727_172754_0049_1785175051818_video.mp4",
        "frames_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/frames",
        "depth_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/predictions/depth_predictions.npz",
        "poses_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/campus_0049/predictions/camera_poses.npz",
        "classification": "nadir_overhead",
        "poisson_depth": 8,
        "density_quantile": 0.01,
        "voxel_size": "auto",
        "dilation": 5,
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"]
    },
    "forest_road": {
        "video_path": PROJECT_ROOT / "data" / "source_videos" / "forest_road.mp4",
        "frames_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/frames",
        "depth_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/predictions/depth_predictions.npz",
        "poses_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/forest_road/predictions/camera_poses.npz",
        "classification": "nadir_overhead",
        "poisson_depth": 8,
        "density_quantile": 0.01,
        "voxel_size": "auto",
        "dilation": 5,
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"]
    },
    "river_bridge": {
        "video_path": PROJECT_ROOT / "data" / "source_videos" / "segment_52s_64s.mp4",
        "frames_dir": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/frames",
        "depth_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/predictions/depth_predictions.npz",
        "poses_npz": PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2/river_bridge/predictions/camera_poses.npz",
        "classification": "forward_corridor",
        "poisson_depth": 8,
        "density_quantile": 0.01,
        "voxel_size": "auto",
        "dilation": 5,
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"]
    }
}


def compute_file_sha256(filepath: Path) -> str:
    """Calculates SHA-256 hash of a file."""
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()


def export_mesh_all_formats(
    mesh_o3d: o3d.geometry.TriangleMesh,
    base_path_without_ext: Path
) -> Dict[str, Dict[str, Any]]:
    """
    Exports an Open3D triangle mesh to OBJ, PLY, and GLB formats.
    Returns metadata dict for each exported file.
    """
    v_arr = np.asarray(mesh_o3d.vertices)
    f_arr = np.asarray(mesh_o3d.triangles)
    c_arr = np.asarray(mesh_o3d.vertex_colors)

    results: Dict[str, Dict[str, Any]] = {}

    # 1. Export PLY
    ply_path = base_path_without_ext.with_suffix(".ply")
    o3d.io.write_triangle_mesh(str(ply_path), mesh_o3d)
    if ply_path.is_file() and ply_path.stat().st_size > 0:
        results["ply"] = {
            "path": str(ply_path.resolve()),
            "size_bytes": ply_path.stat().st_size,
            "sha256": compute_file_sha256(ply_path)
        }

    # 2. Export OBJ
    obj_path = base_path_without_ext.with_suffix(".obj")
    o3d.io.write_triangle_mesh(str(obj_path), mesh_o3d)
    if obj_path.is_file() and obj_path.stat().st_size > 0:
        results["obj"] = {
            "path": str(obj_path.resolve()),
            "size_bytes": obj_path.stat().st_size,
            "sha256": compute_file_sha256(obj_path)
        }

    # 3. Export GLB (via trimesh for maximum compatibility and embedded color attributes)
    glb_path = base_path_without_ext.with_suffix(".glb")
    try:
        colors_u8 = (c_arr * 255.0).astype(np.uint8) if len(c_arr) == len(v_arr) else None
        tri_mesh = trimesh.Trimesh(
            vertices=v_arr,
            faces=f_arr,
            vertex_colors=colors_u8,
            process=False
        )
        glb_bytes = tri_mesh.export(file_type="glb")
        with open(glb_path, "wb") as f_glb:
            f_glb.write(glb_bytes)

        if glb_path.is_file() and glb_path.stat().st_size > 0:
            results["glb"] = {
                "path": str(glb_path.resolve()),
                "size_bytes": glb_path.stat().st_size,
                "sha256": compute_file_sha256(glb_path)
            }
    except Exception as e:
        print(f"[!] Warning: GLB export failed: {e}")

    return results


def render_mesh_preview(
    mesh_o3d: o3d.geometry.TriangleMesh,
    out_preview_path: Path,
    title: str = "3D Reconstruction Preview"
) -> Path:
    """
    Renders Top, Frontal, and Oblique views and stitches them into a composite preview image.
    """
    bbox = mesh_o3d.get_axis_aligned_bounding_box()
    views = render_standard_views(mesh_o3d, bbox, width=800, height=600, is_mesh=True)

    # Stitch into a 3-panel horizontal preview banner
    top_img = views.get("top", Image.new("RGB", (800, 600), (40, 40, 40)))
    oblique_img = views.get("oblique", Image.new("RGB", (800, 600), (40, 40, 40)))
    frontal_img = views.get("frontal", Image.new("RGB", (800, 600), (40, 40, 40)))

    banner_w = 800 * 3
    banner_h = 600 + 60
    banner = Image.new("RGB", (banner_w, banner_h), (25, 28, 35))
    draw = ImageDraw.Draw(banner)

    # Paste renders
    banner.paste(oblique_img, (0, 60))
    banner.paste(top_img, (800, 600 // 2 - 300 + 60))
    banner.paste(frontal_img, (1600, 60))

    # Add header annotations
    draw.text((30, 18), f"{title} | Oblique Perspective", fill=(0, 255, 200))
    draw.text((830, 18), f"{title} | Top Nadir View", fill=(0, 255, 200))
    draw.text((1630, 18), f"{title} | Frontal Corridor View", fill=(0, 255, 200))

    banner.save(out_preview_path, format="PNG")
    return out_preview_path


def validate_mesh_file(file_path: Path) -> Dict[str, Any]:
    """
    Independently loads an exported mesh file using Open3D or Trimesh and verifies:
    - File size > 0
    - Non-empty vertices & triangles
    - All coordinates finite
    - Positive bounding box extents
    """
    if not file_path.is_file():
        raise FileNotFoundError(f"Exported mesh file not found: {file_path}")

    size_bytes = file_path.stat().st_size
    if size_bytes == 0:
        raise ValueError(f"Exported mesh file is empty (0 bytes): {file_path}")

    # Load with Open3D
    suffix = file_path.suffix.lower()
    if suffix in [".obj", ".ply"]:
        mesh = o3d.io.read_triangle_mesh(str(file_path))
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
    elif suffix == ".glb":
        tri_m = trimesh.load(str(file_path), file_type="glb")
        if isinstance(tri_m, trimesh.Scene):
            geoms = list(tri_m.geometry.values())
            if not geoms:
                raise ValueError("GLB scene contains no geometry")
            tri_m = trimesh.util.concatenate(geoms)
        vertices = np.asarray(tri_m.vertices)
        triangles = np.asarray(tri_m.faces)
    else:
        raise ValueError(f"Unsupported validation format: {suffix}")

    v_count = len(vertices)
    f_count = len(triangles)

    if v_count == 0 or f_count == 0:
        raise ValueError(f"Mesh has invalid geometry: {v_count} vertices, {f_count} faces")

    if not np.isfinite(vertices).all():
        raise ValueError("Mesh contains non-finite (NaN or Inf) vertex coordinates")

    extents = np.ptp(vertices, axis=0)
    if (extents <= 0).any():
        raise ValueError(f"Mesh has non-positive extents: {extents}")

    return {
        "verified": True,
        "format": suffix.lstrip("."),
        "file_size_bytes": size_bytes,
        "file_size_mb": round(size_bytes / (1024 * 1024), 3),
        "vertex_count": v_count,
        "triangle_count": f_count,
        "extents": extents.round(4).tolist(),
        "bounds_min": vertices.min(axis=0).round(4).tolist(),
        "bounds_max": vertices.max(axis=0).round(4).tolist(),
        "sha256": compute_file_sha256(file_path)
    }


def generate_scene_3d_models(scene_name: str) -> Dict[str, Any]:
    """
    Executes full end-to-end 3D reconstruction and dynamic masking for a given scene,
    exporting OBJ, PLY, GLB, and preview PNG directly into outputs/dynamic_object_masking/sprint11/<scene_name>/.
    """
    cfg = SCENE_CONFIGS.get(scene_name)
    if not cfg:
        raise ValueError(f"Unknown scene: {scene_name}")

    scene_out = PROJECT_ROOT / "outputs" / "dynamic_object_masking" / "sprint11" / scene_name
    scene_out.mkdir(parents=True, exist_ok=True)
    masks_dir = scene_out / "masks"
    masks_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*75}")
    print(f"[*] F.O.R.G.E Sprint 11 Recovery — 3D Model Generation: {scene_name}")
    print(f"[*] Target Output Directory: {scene_out}")
    print(f"{'='*75}")

    # 1. Load source frames
    frames_dir = cfg["frames_dir"]
    frame_files = sorted(
        [p for p in frames_dir.iterdir() if p.suffix.lower() in [".png", ".jpg", ".jpeg"]],
        key=lambda p: p.name
    )
    print(f"[+] Loaded {len(frame_files)} source frames from: {frames_dir}")

    # 2. Run YOLO instance segmentation on CUDA
    print(f"[+] Initializing YOLOInstanceSegmenter on CUDA (classes={cfg['classes']})...")
    segmenter = YOLOInstanceSegmenter(
        model_path=r"C:\Users\25beevdt025\Desktop\SIH\yolov8n-seg.pt",
        target_classes=cfg["classes"],
        conf_threshold=0.25,
        iou_threshold=0.45,
        dilation_pixels=cfg["dilation"],
        device="cuda"
    )

    tracker = LightweightTemporalTracker()
    masks_list: List[np.ndarray] = []
    detection_records: List[Dict[str, Any]] = []

    for idx, fpath in enumerate(frame_files):
        rgb = np.array(Image.open(fpath).convert("RGB"))
        res = segmenter.segment_frame(rgb, frame_idx=idx, timestamp_sec=idx * 2.836)
        res = tracker.update(res)

        masks_list.append(res.combined_mask)
        detection_records.append(res.to_dict())

        # Save individual mask image
        mask_path = masks_dir / f"mask_frame_{idx:04d}.png"
        cv2.imwrite(str(mask_path), res.combined_mask)

        print(f"    Frame {idx} ({fpath.name}): {len(res.instances)} dynamic instances, {res.coverage_ratio*100:.2f}% masked, {res.inference_time_ms:.1f}ms")

    dynamic_masks = np.stack(masks_list, axis=0)

    # 3. Pipeline Configuration
    pipe_cfg = PipelineConfig(
        voxel_size=cfg["voxel_size"],
        mesh_method="poisson",
        poisson_depth=cfg["poisson_depth"],
        density_quantile=cfg["density_quantile"]
    )

    # -------------------------------------------------------------
    # 4. Generate Masked 3D Model (Primary Deliverable)
    # -------------------------------------------------------------
    print(f"\n[+] Executing Masked 3D Reconstruction Pipeline...")
    pipeline_masked = GeneralReconstructionPipeline(config=pipe_cfg)
    temp_masked_dir = scene_out / "_temp_masked"
    temp_masked_dir.mkdir(parents=True, exist_ok=True)

    t0_m = time.perf_counter()
    report_m = pipeline_masked.run(
        depth_npz_path=str(cfg["depth_npz"]),
        poses_npz_path=str(cfg["poses_npz"]),
        frames_dir=str(cfg["frames_dir"]),
        output_dir=str(temp_masked_dir),
        scene_classification=cfg["classification"],
        dynamic_masks=dynamic_masks
    )
    t_masked = time.perf_counter() - t0_m
    print(f"[+] Masked reconstruction complete in {t_masked:.2f}s")

    # Load generated mesh from temp output and export directly to target names
    raw_mesh_ply = temp_masked_dir / "surface_mesh.ply"
    mesh_masked = o3d.io.read_triangle_mesh(str(raw_mesh_ply))

    masked_base = scene_out / "masked_reconstruction"
    exported_masked = export_mesh_all_formats(mesh_masked, masked_base)

    preview_masked_path = scene_out / "masked_reconstruction_preview.png"
    render_mesh_preview(mesh_masked, preview_masked_path, title=f"F.O.R.G.E Sprint 11: {scene_name} (Masked)")
    print(f"[+] Rendered preview banner: {preview_masked_path}")

    # -------------------------------------------------------------
    # 5. Generate Unmasked 3D Model (Comparison Baseline)
    # -------------------------------------------------------------
    print(f"\n[+] Executing Unmasked 3D Reconstruction Pipeline...")
    pipeline_unmasked = GeneralReconstructionPipeline(config=pipe_cfg)
    temp_unmasked_dir = scene_out / "_temp_unmasked"
    temp_unmasked_dir.mkdir(parents=True, exist_ok=True)

    t0_u = time.perf_counter()
    report_u = pipeline_unmasked.run(
        depth_npz_path=str(cfg["depth_npz"]),
        poses_npz_path=str(cfg["poses_npz"]),
        frames_dir=str(cfg["frames_dir"]),
        output_dir=str(temp_unmasked_dir),
        scene_classification=cfg["classification"],
        dynamic_masks=None
    )
    t_unmasked = time.perf_counter() - t0_u
    print(f"[+] Unmasked reconstruction complete in {t_unmasked:.2f}s")

    raw_unmasked_ply = temp_unmasked_dir / "surface_mesh.ply"
    mesh_unmasked = o3d.io.read_triangle_mesh(str(raw_unmasked_ply))

    unmasked_base = scene_out / "unmasked_reconstruction"
    exported_unmasked = export_mesh_all_formats(mesh_unmasked, unmasked_base)

    preview_unmasked_path = scene_out / "unmasked_reconstruction_preview.png"
    render_mesh_preview(mesh_unmasked, preview_unmasked_path, title=f"F.O.R.G.E Sprint 11: {scene_name} (Unmasked)")
    print(f"[+] Rendered preview banner: {preview_unmasked_path}")

    # Clean up temp directories
    import shutil
    shutil.rmtree(temp_masked_dir, ignore_errors=True)
    shutil.rmtree(temp_unmasked_dir, ignore_errors=True)

    # -------------------------------------------------------------
    # 6. Independent Model Validation
    # -------------------------------------------------------------
    print(f"\n[+] Validating exported 3D model files independently...")
    validations: Dict[str, Any] = {}
    for fmt, info in exported_masked.items():
        v_res = validate_mesh_file(Path(info["path"]))
        validations[f"masked_{fmt}"] = v_res
        print(f"    [✓] Validated {fmt.upper()}: {v_res['vertex_count']:,} vertices, {v_res['triangle_count']:,} triangles, {v_res['file_size_mb']} MB (SHA-256: {v_res['sha256'][:16]}...)")

    for fmt, info in exported_unmasked.items():
        v_res = validate_mesh_file(Path(info["path"]))
        validations[f"unmasked_{fmt}"] = v_res
        print(f"    [✓] Validated {fmt.upper()}: {v_res['vertex_count']:,} vertices, {v_res['triangle_count']:,} triangles, {v_res['file_size_mb']} MB (SHA-256: {v_res['sha256'][:16]}...)")

    # -------------------------------------------------------------
    # 7. Write reconstruction_metadata.json
    # -------------------------------------------------------------
    pts_raw_u = report_u["point_cloud"]["raw_filtered_points"]
    pts_raw_m = report_m["point_cloud"]["raw_filtered_points"]
    dyn_removed = report_m["point_cloud"]["dynamic_object_filtering"]["dynamic_points_removed"]

    metadata = {
        "scene_name": scene_name,
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source_video": {
            "path": str(cfg["video_path"]),
            "sha256": compute_file_sha256(cfg["video_path"]),
            "frames_processed": len(frame_files)
        },
        "masking_configuration": {
            "model": "yolov8n-seg.pt",
            "classes": cfg["classes"],
            "dilation_pixels": cfg["dilation"],
            "temporal_tracking": True
        },
        "point_cloud_comparison": {
            "unmasked_raw_points": pts_raw_u,
            "masked_raw_points": pts_raw_m,
            "dynamic_points_removed": dyn_removed,
            "dynamic_points_removal_pct": round((dyn_removed / max(1, pts_raw_u)) * 100.0, 4),
            "unmasked_fused_points": report_u["point_cloud"]["active_points"],
            "masked_fused_points": report_m["point_cloud"]["active_points"]
        },
        "exported_models": {
            "masked": {
                "obj": str((masked_base.with_suffix(".obj")).resolve()),
                "ply": str((masked_base.with_suffix(".ply")).resolve()),
                "glb": str((masked_base.with_suffix(".glb")).resolve()) if (masked_base.with_suffix(".glb")).is_file() else None,
                "preview_png": str(preview_masked_path.resolve())
            },
            "unmasked": {
                "obj": str((unmasked_base.with_suffix(".obj")).resolve()),
                "ply": str((unmasked_base.with_suffix(".ply")).resolve()),
                "glb": str((unmasked_base.with_suffix(".glb")).resolve()) if (unmasked_base.with_suffix(".glb")).is_file() else None,
                "preview_png": str(preview_unmasked_path.resolve())
            }
        },
        "independent_validation": validations,
        "per_frame_detections": detection_records
    }

    meta_path = scene_out / "reconstruction_metadata.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    print(f"[+] Saved reconstruction metadata: {meta_path}")
    print(f"\n[✓] {scene_name} 3D Model Generation & Validation Complete!")
    return metadata


def main():
    parser = argparse.ArgumentParser(description="F.O.R.G.E Sprint 11 Recovery — Generate & Validate 3D Models")
    parser.add_argument(
        "--scene",
        choices=["desert_road", "campus_0049", "forest_road", "river_bridge", "all"],
        default="desert_road",
        help="Benchmark scene to reconstruct (default: desert_road)"
    )
    args = parser.parse_args()

    scenes = (
        ["desert_road", "campus_0049", "forest_road", "river_bridge"]
        if args.scene == "all"
        else [args.scene]
    )

    results = {}
    for sc in scenes:
        meta = generate_scene_3d_models(sc)
        results[sc] = meta

    print("\n" + "=" * 75)
    print("ALL REQUESTED 3D RECONSTRUCTIONS COMPLETED SUCCESSFULLY.")
    print("=" * 75)


if __name__ == "__main__":
    main()
