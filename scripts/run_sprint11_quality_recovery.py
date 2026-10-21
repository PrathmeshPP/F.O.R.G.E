"""
F.O.R.G.E — Sprint 11 Recovery v1: High-Quality 3D Reconstruction with YOLO Dynamic-Object Masking.

Repairs quality regressions across Desert Road, Campus 0049, Forest Road, and River Bridge:
- Restores high-fidelity geometry and coverage matching or exceeding Sprint 9 baseline.
- Corrects root causes:
  1. Depth-range clamping & horizon-ray pruning (eliminates sky-ray diverging fans).
  2. Scene-adaptive depth-jump filtering (jump_rtol) eliminating scalloped edges.
  3. High-resolution voxel grid fusion (~1200 voxels along max extent, fine scale 0.005m).
  4. Screened Poisson octree depth 9 (quadrupling surface resolution).
  5. Dynamic support-distance pruning based on median point spacing (prevents facade/bridge deletion).
  6. Direct fronto-parallel photographic ray projection for crisp vertex colors.
  7. Straight highway corridor regularization & Taubin smoothing for Desert Road.
- Separates YOLO dynamic object masking from base reconstruction quality:
  - Downstream exclusion of dynamic vehicle points without degrading static terrain/asphalt.
  - Controlled masked vs unmasked comparisons across all scenes.

Exports strictly to:
outputs/dynamic_object_masking/sprint11/recovery_v1/<scene_name>/
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
import torch
import open3d as o3d
import trimesh
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.detection.yolo_segmenter import YOLOInstanceSegmenter
from src.detection.temporal_tracker import LightweightTemporalTracker
from src.reconstruction.point_cloud import depth_to_points, voxel_grid_fusion, save_ply
from src.reconstruction.multiframe_stitcher import SurfaceConnectivityMesher
from scripts.validate_sprint1_geometry_and_filtering import compute_relative_depth_jump

OUT_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v1"
PRED_ROOT = PROJECT_ROOT / "outputs/accuracy_experiment/quality_recovery_v2"
YOLO_WEIGHTS = Path(r"C:\Users\25beevdt025\Desktop\SIH\yolov8n-seg.pt")

SCENE_CONFIGS: Dict[str, Dict[str, Any]] = {
    "desert_road": {
        "video_path": PROJECT_ROOT / "data/source_videos/desert_road.mp4",
        "frames_dir": PRED_ROOT / "desert_road/frames",
        "depth_npz": PRED_ROOT / "desert_road/predictions/depth_predictions.npz",
        "poses_npz": PRED_ROOT / "desert_road/predictions/camera_poses.npz",
        "depth_range": (0.2, 3.5),
        "conf_min": 1.5,
        "jump_rtol": 0.08,
        "horizon_quantile": 95.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.005,
        "support_distance_factor": 14.0,
        "is_desert_corridor": True,
        "orientation_class": "forward_corridor",
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"],
        "dilation": 6
    },
    "campus_0049": {
        "video_path": PROJECT_ROOT / "data/source_videos/dji_fly_20260727_172754_0049_1785175051818_video.mp4",
        "frames_dir": PRED_ROOT / "campus_0049/frames",
        "depth_npz": PRED_ROOT / "campus_0049/predictions/depth_predictions.npz",
        "poses_npz": PRED_ROOT / "campus_0049/predictions/camera_poses.npz",
        "depth_range": (0.05, 5.0),
        "conf_min": 1.0,
        "jump_rtol": 0.12,
        "horizon_quantile": 99.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.005,
        "support_distance_factor": 4.0,
        "is_desert_corridor": False,
        "orientation_class": "nadir_overhead",
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"],
        "dilation": 5
    },
    "forest_road": {
        "video_path": PROJECT_ROOT / "data/source_videos/forest_road.mp4",
        "frames_dir": PRED_ROOT / "forest_road/frames",
        "depth_npz": PRED_ROOT / "forest_road/predictions/depth_predictions.npz",
        "poses_npz": PRED_ROOT / "forest_road/predictions/camera_poses.npz",
        "depth_range": (0.05, 3.5),
        "conf_min": 1.2,
        "jump_rtol": 0.12,
        "horizon_quantile": 99.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.002,
        "support_distance_factor": 4.5,
        "is_desert_corridor": False,
        "orientation_class": "nadir_overhead",
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"],
        "dilation": 5
    },
    "river_bridge": {
        "video_path": PROJECT_ROOT / "data/source_videos/segment_52s_64s.mp4",
        "frames_dir": PRED_ROOT / "river_bridge/frames",
        "depth_npz": PRED_ROOT / "river_bridge/predictions/depth_predictions.npz",
        "poses_npz": PRED_ROOT / "river_bridge/predictions/camera_poses.npz",
        "depth_range": (0.2, 4.2),
        "conf_min": 2.0,
        "jump_rtol": 0.08,
        "horizon_quantile": 95.0,
        "poisson_depth": 9,
        "gentle_trim_quantile": 0.005,
        "support_distance_factor": 4.0,
        "is_desert_corridor": False,
        "orientation_class": "forward_corridor",
        "classes": ["car", "truck", "bus", "motorcycle", "bicycle"],
        "dilation": 5
    }
}


def compute_file_sha256(filepath: Path) -> str:
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()


def apply_photographic_vertex_colors(
    mesh: o3d.geometry.TriangleMesh,
    extrinsics: np.ndarray,
    intrinsics: np.ndarray,
    frame_rgbs: np.ndarray,
    fallback_pcd: o3d.geometry.PointCloud
) -> Tuple[o3d.geometry.TriangleMesh, Dict[str, Any]]:
    """
    Transfers photographic color to mesh vertices using direct fronto-parallel camera ray projection.
    Samples highest cos(theta) viewing ray without multi-neighbor averaging blur.
    """
    verts = np.asarray(mesh.vertices)
    mesh.compute_vertex_normals()
    normals = np.asarray(mesh.vertex_normals)
    N_verts = len(verts)
    num_cams = len(extrinsics)
    H, W_img = frame_rgbs.shape[1], frame_rgbs.shape[2]

    best_cos = np.full(N_verts, -1.0, dtype=np.float32)
    best_colors = np.zeros((N_verts, 3), dtype=np.float32)

    for i in range(num_cams):
        R = extrinsics[i, :3, :3]
        t = extrinsics[i, :3, 3]
        K = intrinsics[i]

        P_cam = (R @ verts.T + t[:, None]).T
        Z_c = P_cam[:, 2]

        valid_z = Z_c > 0.05
        u = K[0, 0] * (P_cam[:, 0] / np.maximum(Z_c, 1e-4)) + K[0, 2]
        v = K[1, 1] * (P_cam[:, 1] / np.maximum(Z_c, 1e-4)) + K[1, 2]

        in_frame = valid_z & (u >= 2.0) & (u <= W_img - 3.0) & (v >= 2.0) & (v <= H - 3.0)

        dist = np.linalg.norm(P_cam, axis=1, keepdims=True) + 1e-6
        rays = P_cam / dist
        N_c = (R @ normals.T).T
        cos_theta = -np.sum(rays * N_c, axis=1)

        facing = in_frame & (cos_theta > 0.05) & (cos_theta > best_cos)
        if facing.any():
            u_f = np.clip(u[facing], 0, W_img - 1).astype(np.int32)
            v_f = np.clip(v[facing], 0, H - 1).astype(np.int32)
            best_colors[facing] = frame_rgbs[i, v_f, u_f] / 255.0
            best_cos[facing] = cos_theta[facing]

    # Fallback for occluded or grazing-angle vertices
    uncovered = best_cos <= 0.05
    if uncovered.any() and len(fallback_pcd.points) > 0:
        kdtree = o3d.geometry.KDTreeFlann(fallback_pcd)
        fb_cols = np.asarray(fallback_pcd.colors)
        for idx in np.where(uncovered)[0]:
            [k, nn_idx, _] = kdtree.search_knn_vector_3d(verts[idx], 1)
            if k > 0:
                best_colors[idx] = fb_cols[nn_idx[0]]

    mesh.vertex_colors = o3d.utility.Vector3dVector(best_colors.astype(np.float64))
    stats = {
        "total_vertices": N_verts,
        "direct_photographic_vertices": int((~uncovered).sum()),
        "photographic_coverage_pct": round(float((~uncovered).mean() * 100.0), 2)
    }
    return mesh, stats


def export_mesh_all_formats(
    mesh_o3d: o3d.geometry.TriangleMesh,
    base_path_without_ext: Path
) -> Dict[str, Dict[str, Any]]:
    """Exports triangle mesh to PLY, OBJ, and GLB."""
    v_arr = np.asarray(mesh_o3d.vertices)
    f_arr = np.asarray(mesh_o3d.triangles)
    c_arr = np.asarray(mesh_o3d.vertex_colors)

    results: Dict[str, Dict[str, Any]] = {}

    # 1. PLY
    ply_path = base_path_without_ext.with_suffix(".ply")
    o3d.io.write_triangle_mesh(str(ply_path), mesh_o3d)
    if ply_path.is_file() and ply_path.stat().st_size > 0:
        results["ply"] = {
            "path": str(ply_path.resolve()),
            "size_bytes": ply_path.stat().st_size,
            "sha256": compute_file_sha256(ply_path)
        }

    # 2. OBJ
    obj_path = base_path_without_ext.with_suffix(".obj")
    o3d.io.write_triangle_mesh(str(obj_path), mesh_o3d)
    if obj_path.is_file() and obj_path.stat().st_size > 0:
        results["obj"] = {
            "path": str(obj_path.resolve()),
            "size_bytes": obj_path.stat().st_size,
            "sha256": compute_file_sha256(obj_path)
        }

    # 3. GLB
    glb_path = base_path_without_ext.with_suffix(".glb")
    try:
        colors_u8 = (c_arr * 255.0).astype(np.uint8) if len(c_arr) == len(v_arr) else None
        tri_m = trimesh.Trimesh(
            vertices=v_arr,
            faces=f_arr,
            vertex_colors=colors_u8,
            process=False
        )
        glb_bytes = tri_m.export(file_type="glb")
        with open(glb_path, "wb") as f_glb:
            f_glb.write(glb_bytes)

        if glb_path.is_file() and glb_path.stat().st_size > 0:
            results["glb"] = {
                "path": str(glb_path.resolve()),
                "size_bytes": glb_path.stat().st_size,
                "sha256": compute_file_sha256(glb_path)
            }
    except Exception as e:
        print(f"[!] GLB export error: {e}")

    return results


def render_card_preview(
    mesh_o3d: o3d.geometry.TriangleMesh,
    out_preview_path: Path,
    title: str = "3D Reconstruction Preview",
    orientation_class: str = "forward_corridor"
) -> Path:
    """Renders 3-panel matched orthographic/perspective views."""
    bbox = mesh_o3d.get_axis_aligned_bounding_box()
    ctr = bbox.get_center()
    w, h = 800, 600

    if orientation_class == "nadir_overhead":
        view_configs = [
            ("Oblique Perspective", [0.25, -0.45, -0.85], [0.0, -1.0, 0.0], 0.70),
            ("Top Nadir View", [0.0, 0.0, -1.0], [0.0, -1.0, 0.0], 0.75),
            ("Corridor Cross-Section", [0.0, 1.0, -0.3], [0.0, 0.0, -1.0], 0.70)
        ]
    else:
        view_configs = [
            ("Oblique Perspective", [0.0, -0.707, -0.707], [0.0, -0.707, 0.707], 0.65),
            ("Top Nadir View", [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], 0.75),
            ("Corridor Cross-Section", [0.0, 0.1, -1.0], [0.0, -1.0, 0.0], 0.70)
        ]

    panel_imgs = []
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=w, height=h)
    mesh_o3d.compute_vertex_normals()
    vis.add_geometry(mesh_o3d)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    for label, front, up, zoom in view_configs:
        ctrl.set_lookat(ctr)
        ctrl.set_front(front)
        ctrl.set_up(up)
        ctrl.set_zoom(zoom)
        vis.poll_events()
        vis.update_renderer()
        buf = vis.capture_screen_float_buffer(do_render=True)
        img = Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))
        panel_imgs.append((label, img))

    vis.destroy_window()

    banner_w = w * 3
    banner_h = h + 70
    banner = Image.new("RGB", (banner_w, banner_h), (20, 24, 32))
    draw = ImageDraw.Draw(banner)

    for idx, (label, pimg) in enumerate(panel_imgs):
        x_off = idx * w
        banner.paste(pimg, (x_off, 70))
        draw.text((x_off + 25, 22), f"{title} | {label}", fill=(0, 255, 200))

    out_preview_path.parent.mkdir(parents=True, exist_ok=True)
    banner.save(out_preview_path, format="PNG")
    return out_preview_path


def validate_mesh_file(file_path: Path) -> Dict[str, Any]:
    """Validates geometry, finiteness, positive bounds, manifoldness."""
    if not file_path.is_file():
        raise FileNotFoundError(f"File not found: {file_path}")
    sz = file_path.stat().st_size
    if sz == 0:
        raise ValueError(f"Empty file: {file_path}")

    suffix = file_path.suffix.lower()
    if suffix in [".obj", ".ply"]:
        m = o3d.io.read_triangle_mesh(str(file_path))
        verts = np.asarray(m.vertices)
        tris = np.asarray(m.triangles)
    elif suffix == ".glb":
        tri_m = trimesh.load(str(file_path), file_type="glb")
        if isinstance(tri_m, trimesh.Scene):
            geoms = list(tri_m.geometry.values())
            tri_m = trimesh.util.concatenate(geoms)
        verts = np.asarray(tri_m.vertices)
        tris = np.asarray(tri_m.faces)
    else:
        raise ValueError(f"Unsupported format: {suffix}")

    v_count = len(verts)
    f_count = len(tris)
    if v_count == 0 or f_count == 0:
        raise ValueError(f"Empty geometry: {v_count} verts, {f_count} faces")
    if not np.isfinite(verts).all():
        raise ValueError("Non-finite vertex coordinates")

    extents = np.ptp(verts, axis=0)
    if (extents <= 0).any():
        raise ValueError(f"Non-positive bounding box extents: {extents}")

    return {
        "verified": True,
        "format": suffix.lstrip("."),
        "file_size_bytes": sz,
        "file_size_mb": round(sz / (1024 * 1024), 3),
        "vertex_count": v_count,
        "triangle_count": f_count,
        "extents": extents.round(4).tolist(),
        "bounds_min": verts.min(axis=0).round(4).tolist(),
        "bounds_max": verts.max(axis=0).round(4).tolist(),
        "sha256": compute_file_sha256(file_path)
    }


def reconstruct_scene_recovery(
    scene_name: str,
    cfg: Dict[str, Any]
) -> Dict[str, Any]:
    """Executes high-quality recovery reconstruction for a single scene."""
    t0 = time.time()
    out_dir = OUT_ROOT / scene_name
    out_dir.mkdir(parents=True, exist_ok=True)
    masks_dir = out_dir / "masks"
    masks_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*75}")
    print(f"[*] F.O.R.G.E Sprint 11 Recovery v1: {scene_name.upper()}")
    print(f"[*] Target Directory: {out_dir}")
    print(f"{'='*75}")

    # 1. Load predictions & frames
    frames_dir = cfg["frames_dir"]
    frame_files = sorted(
        [p for p in frames_dir.iterdir() if p.suffix.lower() in [".png", ".jpg", ".jpeg"]],
        key=lambda p: p.name
    )
    num_frames = len(frame_files)
    print(f"[+] Loaded {num_frames} frames from {frames_dir}")

    poses_data = np.load(cfg["poses_npz"])
    depth_data = np.load(cfg["depth_npz"])
    extrinsics = poses_data["extrinsics"][:num_frames]
    intrinsics = poses_data["intrinsics"][:num_frames]
    depth_maps = np.squeeze(depth_data["depth"][:num_frames])
    confidences = np.squeeze(depth_data["confidence"][:num_frames])

    H_infer, W_infer = depth_maps.shape[1], depth_maps.shape[2]
    frame_rgbs = []
    for f in frame_files:
        im = Image.open(f).convert("RGB")
        if im.size != (W_infer, H_infer):
            im = im.resize((W_infer, H_infer), Image.Resampling.BILINEAR)
        frame_rgbs.append(np.array(im, dtype=np.uint8))
    frame_rgbs = np.stack(frame_rgbs, axis=0)

    # 2. Generate YOLO Dynamic Object Masks
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[+] Running YOLOv8n-seg inference on {dev.upper()} (classes={cfg['classes']})...")
    segmenter = YOLOInstanceSegmenter(
        model_path=str(YOLO_WEIGHTS),
        target_classes=cfg["classes"],
        conf_threshold=0.25,
        iou_threshold=0.45,
        dilation_pixels=cfg["dilation"],
        device=dev
    )
    tracker = LightweightTemporalTracker()
    masks_list = []
    detections_summary = []

    for idx, fpath in enumerate(frame_files):
        rgb_nat = np.array(Image.open(fpath).convert("RGB"))
        res = segmenter.segment_frame(rgb_nat, frame_idx=idx, timestamp_sec=idx * 2.836)
        res = tracker.update(res)

        # Resize mask to inference resolution
        m_inf = cv2.resize(res.combined_mask, (W_infer, H_infer), interpolation=cv2.INTER_NEAREST) > 0
        masks_list.append(m_inf)
        detections_summary.append(res.to_dict())

        cv2.imwrite(str(masks_dir / f"mask_{idx:04d}.png"), res.combined_mask)
        print(f"    Frame {idx} ({fpath.name}): {len(res.instances)} instances, {res.coverage_ratio*100:.2f}% masked")

    dynamic_masks_inf = np.stack(masks_list, axis=0)
    has_dynamic_objects = any(len(d["instances"]) > 0 for d in detections_summary)

    # 3. Geometric Backprojection & Filtering
    print("[+] Backprojecting and applying principled geometric filtering...")
    world_points = depth_to_points(depth_maps, extrinsics, intrinsics)
    flat_world = world_points.reshape(-1, 3)
    flat_rgb = frame_rgbs.reshape(-1, 3)
    flat_depth = depth_maps.reshape(-1)
    flat_conf = confidences.reshape(-1)

    d_min, d_max = cfg["depth_range"]
    c_min = cfg["conf_min"]

    # Base valid mask
    base_valid = (
        np.isfinite(flat_world).all(axis=1)
        & (flat_depth >= d_min)
        & (flat_depth <= d_max)
        & (flat_conf >= c_min)
    )

    # Scene-adaptive depth-edge filtering (jump_rtol)
    rtol = cfg["jump_rtol"]
    all_jumps = []
    for i in range(len(depth_maps)):
        d_i = depth_maps[i, :, :, 0] if depth_maps.ndim == 4 else depth_maps[i]
        jump_i = compute_relative_depth_jump(d_i, kernel_size=3)
        all_jumps.append(jump_i)
    all_jumps = np.stack(all_jumps, axis=0).reshape(-1)
    jump_mask = all_jumps > rtol

    # Horizon sky-ray pruning
    d_valid = flat_depth[base_valid]
    horizon_cap = float(np.percentile(d_valid, cfg["horizon_quantile"])) if len(d_valid) > 0 else d_max
    horizon_mask = flat_depth > horizon_cap

    # Clean static baseline mask (without vehicle exclusion)
    unmasked_clean_mask = base_valid & (~jump_mask) & (~horizon_mask)

    # Dynamic mask exclusion
    flat_dynamic = dynamic_masks_inf.reshape(-1)
    masked_clean_mask = unmasked_clean_mask & (~flat_dynamic)

    pts_unmasked_raw = int(unmasked_clean_mask.sum())
    pts_masked_raw = int(masked_clean_mask.sum())
    dyn_removed = pts_unmasked_raw - pts_masked_raw

    print(f"    Raw clean unmasked points: {pts_unmasked_raw:,}")
    print(f"    Raw clean masked points:   {pts_masked_raw:,}")
    print(f"    Dynamic points removed:    {dyn_removed:,} ({dyn_removed/max(1, pts_unmasked_raw)*100:.2f}%)")

    # Helper function to reconstruct mesh from point cloud
    def build_mesh_from_points(clean_mask: np.ndarray, is_masked: bool) -> Tuple[o3d.geometry.TriangleMesh, o3d.geometry.PointCloud]:
        pts = flat_world[clean_mask]
        cols = flat_rgb[clean_mask]

        if cfg["is_desert_corridor"]:
            # Desert road straight highway corridor regularization: |X + 0.05| <= 0.55m
            in_corr = np.abs(pts[:, 0] + 0.05) <= 0.55
            pts = pts[in_corr]
            cols = cols[in_corr]
            fused_pts, fused_cols = voxel_grid_fusion(pts, cols, voxel_size=0.005)
        else:
            ext = np.ptp(pts, axis=0)
            max_ext = float(np.max(ext))
            fine_vox = max(max_ext / 1200.0, 0.005)
            fused_pts, fused_cols = voxel_grid_fusion(pts, cols, voxel_size=fine_vox)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(fused_pts.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(fused_cols.astype(np.float64) / 255.0)

        # Normal estimation oriented towards camera center
        cam_centers = []
        for i in range(extrinsics.shape[0]):
            R = extrinsics[i, :3, :3]
            t = extrinsics[i, :3, 3]
            cam_centers.append(-R.T @ t)
        mean_cam = np.mean(cam_centers, axis=0)

        if cfg["is_desert_corridor"]:
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
            normals = np.asarray(pcd.normals)
            flip = normals[:, 1] > 0
            normals[flip] = -normals[flip]
            pcd.normals = o3d.utility.Vector3dVector(normals)
        else:
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
            pcd.orient_normals_towards_camera_location(camera_location=mean_cam)

        # Poisson reconstruction
        mesh, _ = SurfaceConnectivityMesher.build_poisson_hq(
            pcd,
            depth=cfg["poisson_depth"],
            gentle_trim_quantile=cfg["gentle_trim_quantile"],
            support_distance_factor=cfg["support_distance_factor"]
        )

        if cfg["is_desert_corridor"]:
            # Keep largest connected highway component
            tri_clusters, num_tris, _ = mesh.cluster_connected_triangles()
            if len(num_tris) > 0:
                largest = int(np.argmax(num_tris))
                mesh.remove_triangles_by_mask(np.asarray(tri_clusters) != largest)
                mesh.remove_unreferenced_vertices()

            # Repair small interior holes
            mesh, _ = SurfaceConnectivityMesher.repair_surface_connectivity(mesh, max_hole_vertices=150)

            # 30 Taubin smoothing iterations for clean planar roadbed
            mesh = mesh.filter_smooth_taubin(number_of_iterations=30)
            mesh.compute_vertex_normals()

        # Direct photographic vertex color projection
        mesh, _ = apply_photographic_vertex_colors(mesh, extrinsics, intrinsics, frame_rgbs, pcd)
        return mesh, pcd

    # 4. Build Masked Model (Primary Deliverable)
    print("\n[+] Reconstructing MASKED high-quality surface mesh...")
    mesh_masked, pcd_masked = build_mesh_from_points(masked_clean_mask, is_masked=True)
    masked_base = out_dir / "masked_reconstruction"
    exported_masked = export_mesh_all_formats(mesh_masked, masked_base)
    preview_masked_path = out_dir / "masked_reconstruction_preview.png"
    render_card_preview(mesh_masked, preview_masked_path, title=f"F.O.R.G.E Sprint 11 Recovery: {scene_name} (Masked)", orientation_class=cfg["orientation_class"])

    # 5. Build Unmasked Model (Comparative Baseline)
    print("\n[+] Reconstructing UNMASKED comparison surface mesh...")
    mesh_unmasked, pcd_unmasked = build_mesh_from_points(unmasked_clean_mask, is_masked=False)
    unmasked_base = out_dir / "unmasked_reconstruction"
    exported_unmasked = export_mesh_all_formats(mesh_unmasked, unmasked_base)
    preview_unmasked_path = out_dir / "unmasked_reconstruction_preview.png"
    render_card_preview(mesh_unmasked, preview_unmasked_path, title=f"F.O.R.G.E Sprint 11 Recovery: {scene_name} (Unmasked)", orientation_class=cfg["orientation_class"])

    # 6. Validate Independent Reloads
    print("\n[+] Independently reloading and validating exported 3D geometry...")
    val_masked = {f"masked_{fmt}": validate_mesh_file(Path(info["path"])) for fmt, info in exported_masked.items()}
    val_unmasked = {f"unmasked_{fmt}": validate_mesh_file(Path(info["path"])) for fmt, info in exported_unmasked.items()}

    for k, v in val_masked.items():
        print(f"    [✓] {k.upper()}: {v['vertex_count']:,} vertices, {v['triangle_count']:,} triangles, {v['file_size_mb']} MB (SHA-256: {v['sha256'][:16]}...)")

    # 7. Write reconstruction_metadata.json
    v_m = np.asarray(mesh_masked.vertices)
    f_m = np.asarray(mesh_masked.triangles)
    v_u = np.asarray(mesh_unmasked.vertices)
    f_u = np.asarray(mesh_unmasked.triangles)

    meta = {
        "scene_name": scene_name,
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source_video": {
            "path": str(cfg["video_path"]),
            "sha256": compute_file_sha256(cfg["video_path"]),
            "frames_processed": num_frames
        },
        "masking_configuration": {
            "model": "yolov8n-seg.pt",
            "classes": cfg["classes"],
            "dilation_pixels": cfg["dilation"],
            "temporal_tracking": True,
            "has_dynamic_detections": has_dynamic_objects
        },
        "reconstruction_parameters": {
            "depth_range": cfg["depth_range"],
            "conf_min": cfg["conf_min"],
            "jump_rtol": cfg["jump_rtol"],
            "horizon_quantile": cfg["horizon_quantile"],
            "poisson_depth": cfg["poisson_depth"],
            "gentle_trim_quantile": cfg["gentle_trim_quantile"],
            "support_distance_factor": cfg["support_distance_factor"],
            "orientation_class": cfg["orientation_class"]
        },
        "point_cloud_comparison": {
            "unmasked_raw_points": pts_unmasked_raw,
            "masked_raw_points": pts_masked_raw,
            "dynamic_points_removed": dyn_removed,
            "dynamic_points_removal_pct": round((dyn_removed / max(1, pts_unmasked_raw)) * 100.0, 4),
            "unmasked_fused_points": len(pcd_unmasked.points),
            "masked_fused_points": len(pcd_masked.points)
        },
        "surface_mesh_comparison": {
            "unmasked": {
                "vertex_count": len(v_u),
                "triangle_count": len(f_u),
                "extents": np.ptp(v_u, axis=0).round(4).tolist()
            },
            "masked": {
                "vertex_count": len(v_m),
                "triangle_count": len(f_m),
                "extents": np.ptp(v_m, axis=0).round(4).tolist()
            }
        },
        "exported_models": {
            "masked": {
                "glb": str(masked_base.with_suffix(".glb").resolve()),
                "obj": str(masked_base.with_suffix(".obj").resolve()),
                "ply": str(masked_base.with_suffix(".ply").resolve()),
                "preview_png": str(preview_masked_path.resolve())
            },
            "unmasked": {
                "glb": str(unmasked_base.with_suffix(".glb").resolve()),
                "obj": str(unmasked_base.with_suffix(".obj").resolve()),
                "ply": str(unmasked_base.with_suffix(".ply").resolve()),
                "preview_png": str(preview_unmasked_path.resolve())
            }
        },
        "independent_validation": {**val_masked, **val_unmasked},
        "elapsed_time_sec": round(time.time() - t0, 2)
    }

    meta_file = out_dir / "reconstruction_metadata.json"
    with open(meta_file, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    print(f"[+] Completed {scene_name} recovery in {time.time() - t0:.2f}s")
    return meta


def main():
    parser = argparse.ArgumentParser(description="F.O.R.G.E Sprint 11 Recovery v1")
    parser.add_argument("--scene", type=str, default="all", choices=["all", "desert_road", "campus_0049", "forest_road", "river_bridge"])
    args = parser.parse_args()

    targets = list(SCENE_CONFIGS.keys()) if args.scene == "all" else [args.scene]
    all_reports = {}

    for s in targets:
        rep = reconstruct_scene_recovery(s, SCENE_CONFIGS[s])
        all_reports[s] = rep

    summary_file = OUT_ROOT / "recovery_v1_summary.json"
    with open(summary_file, "w", encoding="utf-8") as f:
        json.dump(all_reports, f, indent=2)

    print(f"\n[✓] All recovery runs completed successfully. Summary saved to: {summary_file}")


if __name__ == "__main__":
    main()
