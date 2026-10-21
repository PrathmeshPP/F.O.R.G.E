"""
F.O.R.G.E — Spatial Reconstruction Module
Converts VGGT-Omega predicted metric depth maps and camera poses into dense 3D colored point clouds.
"""

import os
import time
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union, Any
import numpy as np
from PIL import Image


def depth_to_points(
    depth_map: np.ndarray,
    extrinsics: np.ndarray,
    intrinsics: np.ndarray
) -> np.ndarray:
    """
    Backprojects metric depth map(s) to 3D world coordinates using camera intrinsics and extrinsics.

    Args:
        depth_map: Array of shape (N, H, W, 1), (N, H, W), (H, W, 1), or (H, W).
        extrinsics: Camera-from-world extrinsics matrix [R | T] in OpenCV convention, shape (N, 3, 4) or (3, 4).
        intrinsics: Camera pinhole calibration matrix, shape (N, 3, 3) or (3, 3).

    Returns:
        Array of 3D world coordinates, shape (N, H, W, 3) or (H, W, 3).
    """
    is_single_frame = (depth_map.ndim == 2) or (depth_map.ndim == 3 and depth_map.shape[-1] == 1)

    if depth_map.ndim == 2:
        depth = depth_map[None, ...]
    elif depth_map.ndim == 3 and depth_map.shape[-1] == 1:
        depth = depth_map[..., 0][None, ...]
    elif depth_map.ndim == 4 and depth_map.shape[-1] == 1:
        depth = depth_map[..., 0]
    else:
        depth = depth_map

    ext = np.asarray(extrinsics)
    if ext.ndim == 2:
        ext = ext[None, ...]

    intr = np.asarray(intrinsics)
    if intr.ndim == 2:
        intr = intr[None, ...]

    num_frames, height, width = depth.shape

    # Construct pixel coordinate grids (u: horizontal, v: vertical)
    v, u = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    u_grid = np.broadcast_to(u[None, ...], (num_frames, height, width))
    v_grid = np.broadcast_to(v[None, ...], (num_frames, height, width))

    # Focal lengths and principal points
    fx = intr[:, 0, 0][:, None, None]
    fy = intr[:, 1, 1][:, None, None]
    cx = intr[:, 0, 2][:, None, None]
    cy = intr[:, 1, 2][:, None, None]

    # Camera coordinate system (OpenCV convention: +X right, +Y down, +Z forward)
    # X_cam = (u - cx) * depth / fx
    # Y_cam = (v - cy) * depth / fy
    # Z_cam = depth
    finite_mask = np.isfinite(depth)
    depth_clean = np.where(finite_mask, depth, 0.0)
    camera_points = np.stack(
        [
            (u_grid - cx) / fx * depth_clean,
            (v_grid - cy) / fy * depth_clean,
            depth_clean,
        ],
        axis=-1,
    )  # Shape: (N, H, W, 3)
    camera_points[~finite_mask] = np.nan

    # In OpenCV camera convention: p_cam = R * p_world + T
    # Therefore: p_world = R^T * (p_cam - T)
    rotation = ext[:, :3, :3]       # Shape: (N, 3, 3)
    translation = ext[:, :3, 3]     # Shape: (N, 3)

    # R^T is np.transpose(rotation, (0, 2, 1))
    world_points = np.einsum(
        "sij,shwj->shwi",
        np.transpose(rotation, (0, 2, 1)),
        camera_points - translation[:, None, None, :],
    )

    if is_single_frame:
        return world_points[0]
    return world_points


def save_ply(
    points: np.ndarray,
    colors: np.ndarray,
    output_path: str,
    binary: bool = True
) -> int:
    """
    Exports 3D points and RGB colors to a standard PLY file.

    Args:
        points: (M, 3) float32 coordinates.
        colors: (M, 3) uint8 or float [0, 1] RGB colors.
        output_path: Target .ply file path.
        binary: If True, writes binary_little_endian (fast & compact); else ASCII.

    Returns:
        File size in bytes.
    """
    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    points = np.asarray(points, dtype=np.float32)
    if colors.dtype != np.uint8:
        if colors.max() <= 1.0:
            colors = (colors * 255.0).clip(0, 255).astype(np.uint8)
        else:
            colors = colors.clip(0, 255).astype(np.uint8)

    num_vertices = len(points)
    if len(colors) != num_vertices:
        raise ValueError(f"Points count ({num_vertices}) must match colors count ({len(colors)})")

    if binary:
        # Create structured array for fast vectorized binary serialization
        vertex_dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
        ])
        vertex_data = np.empty(num_vertices, dtype=vertex_dtype)
        vertex_data['x'] = points[:, 0]
        vertex_data['y'] = points[:, 1]
        vertex_data['z'] = points[:, 2]
        vertex_data['red'] = colors[:, 0]
        vertex_data['green'] = colors[:, 1]
        vertex_data['blue'] = colors[:, 2]

        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {num_vertices}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n"
        ).encode('ascii')

        with open(out_file, 'wb') as f:
            f.write(header)
            vertex_data.tofile(f)
    else:
        with open(out_file, 'w', encoding='ascii') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {num_vertices}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for p, c in zip(points, colors):
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {int(c[0])} {int(c[1])} {int(c[2])}\n")

    return out_file.stat().st_size


def voxel_grid_fusion(
    points: np.ndarray,
    colors: np.ndarray,
    voxel_size: Optional[float] = None
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Consolidates nearby 3D points and averages their RGB colors using a regular 3D voxel grid.

    Note on Coordinate Units:
        The voxel_size parameter operates in the model's coordinate units (nominally meters
        in VGGT-Omega convention). These units are derived directly from model predictions
        and have not been independently verified against ground truth survey measurements or GCPs.

    Args:
        points: (M, 3) float array of 3D coordinates.
        colors: (M, 3) array of RGB colors (uint8 or float [0, 1]).
        voxel_size: Grid cell dimension in model coordinate units (e.g., 0.005).
                    If None or <= 0, points and colors are returned without consolidation.

    Returns:
        Tuple of (fused_points, fused_colors) where colors are uint8 [0, 255].
    """
    if voxel_size is None or voxel_size <= 0:
        if colors.dtype != np.uint8:
            if colors.max() <= 1.0:
                colors = (colors * 255.0).clip(0, 255).astype(np.uint8)
            else:
                colors = colors.clip(0, 255).astype(np.uint8)
        return points.astype(np.float32), colors

    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        if colors.dtype == np.uint8 or colors.max() > 1.0:
            c_float = colors.astype(np.float64) / 255.0
        else:
            c_float = colors.astype(np.float64)
        pcd.colors = o3d.utility.Vector3dVector(c_float)

        down_pcd = pcd.voxel_down_sample(voxel_size=float(voxel_size))
        fused_pts = np.asarray(down_pcd.points, dtype=np.float32)
        fused_cols = (np.asarray(down_pcd.colors, dtype=np.float32) * 255.0).clip(0, 255).astype(np.uint8)
        return fused_pts, fused_cols
    except Exception:
        # Vectorized NumPy voxel averaging fallback
        voxel_coords = np.floor(points / float(voxel_size)).astype(np.int64)
        _, inverse_indices, counts = np.unique(voxel_coords, axis=0, return_inverse=True, return_counts=True)
        num_voxels = len(counts)

        sum_pts = np.zeros((num_voxels, 3), dtype=np.float64)
        np.add.at(sum_pts, inverse_indices, points.astype(np.float64))
        fused_pts = (sum_pts / counts[:, None]).astype(np.float32)

        sum_cols = np.zeros((num_voxels, 3), dtype=np.float64)
        if colors.dtype == np.uint8 or colors.max() > 1.0:
            c_arr = colors.astype(np.float64)
        else:
            c_arr = colors.astype(np.float64) * 255.0
        np.add.at(sum_cols, inverse_indices, c_arr)
        fused_cols = (sum_cols / counts[:, None]).clip(0, 255).astype(np.uint8)
        return fused_pts, fused_cols


def filter_by_multiview_support(
    points: np.ndarray,
    colors: np.ndarray,
    extrinsics: np.ndarray,
    intrinsics: np.ndarray,
    depth_maps: np.ndarray,
    confidence_maps: Optional[np.ndarray] = None,
    min_camera_support: int = 2,
    rtol: float = 0.08,
    min_confidence: float = 1.5,
    return_support_counts: bool = False
) -> Union[Tuple[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    Filters 3D points based on principled multi-view camera support and depth consistency.
    A point P is supported by camera i if:
    1. It projects into camera i's sensor bounds [0, W) x [0, H).
    2. Camera depth Z_cam > 0.
    3. Relative depth error |Z_cam - d_i(u, v)| / d_i(u, v) <= rtol.
    4. If confidence_maps provided, c_i(u, v) >= min_confidence.

    Points with camera support count < min_camera_support are rejected as unsupported
    single-view fringe artifacts.
    """
    pts = np.asarray(points, dtype=np.float64)
    cols = np.asarray(colors)
    N_pts = len(pts)
    if N_pts == 0:
        if return_support_counts:
            return pts.astype(np.float32), cols, np.zeros(0, dtype=np.int32)
        return pts.astype(np.float32), cols

    d_arr = np.squeeze(depth_maps)
    if d_arr.ndim == 2:
        d_arr = d_arr[None, ...]
    num_cams, H, W = d_arr.shape

    if confidence_maps is not None:
        c_arr = np.squeeze(confidence_maps)
        if c_arr.ndim == 2:
            c_arr = c_arr[None, ...]
    else:
        c_arr = None

    ext = np.asarray(extrinsics)
    if ext.ndim == 2:
        ext = ext[None, ...]

    intr = np.asarray(intrinsics)
    if intr.ndim == 2:
        intr = intr[None, ...]

    support_counts = np.zeros(N_pts, dtype=np.int32)

    for i in range(num_cams):
        R = ext[i, :3, :3]
        t = ext[i, :3, 3]
        K = intr[i]
        d_map = d_arr[i]
        c_map = c_arr[i] if c_arr is not None else None

        P_cam = (R @ pts.T + t[:, None]).T
        Z_cam = P_cam[:, 2]
        in_front = Z_cam > 0.05

        u = np.round(K[0, 0] * P_cam[:, 0] / np.maximum(Z_cam, 1e-6) + K[0, 2]).astype(np.int64)
        v = np.round(K[1, 1] * P_cam[:, 1] / np.maximum(Z_cam, 1e-6) + K[1, 2]).astype(np.int64)

        in_bounds = in_front & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        valid_idx = np.where(in_bounds)[0]

        if len(valid_idx) == 0:
            continue

        u_v = u[valid_idx]
        v_v = v[valid_idx]
        pred_d = d_map[v_v, u_v]
        z_v = Z_cam[valid_idx]

        rel_diff = np.abs(z_v - pred_d) / np.maximum(pred_d, 1e-6)
        consistent = rel_diff <= rtol

        if c_map is not None:
            pred_c = c_map[v_v, u_v]
            consistent = consistent & (pred_c >= min_confidence)

        support_counts[valid_idx[consistent]] += 1

    mask = support_counts >= min_camera_support
    filtered_pts = pts[mask].astype(np.float32)
    filtered_cols = cols[mask]

    if return_support_counts:
        return filtered_pts, filtered_cols, support_counts[mask]
    return filtered_pts, filtered_cols



def create_point_cloud(
    depth_npz_path: str,
    poses_npz_path: str,
    frames_dir: str,
    output_ply_path: str,
    metadata_output_path: Optional[str] = None,
    min_depth: float = 0.1,
    max_depth: Optional[float] = 100.0,
    min_confidence: float = 0.1,
    subsample_step: int = 1,
    voxel_size: Optional[float] = None,
    save_unfused_ply_path: Optional[str] = None
) -> Dict[str, Any]:
    """
    Loads depth predictions and camera poses, aligns frame colors, backprojects into 3D,
    applies configurable confidence filtering and optional voxel-grid fusion, and saves
    the resulting colored point cloud to a PLY file.

    Note on Coordinate Units:
        Depth, bounding box dimensions, and voxel_size are expressed in model coordinate
        units (nominally meters in VGGT-Omega convention). These units are model-predicted
        and have not been independently verified against ground truth survey data or GCPs.

    Args:
        depth_npz_path: Path to predicted depth NPZ (containing "depth" and optional "confidence").
        poses_npz_path: Path to camera poses NPZ (containing "extrinsics" and "intrinsics").
        frames_dir: Directory containing input video frames.
        output_ply_path: Destination path for the primary output PLY.
        metadata_output_path: Optional destination path for reconstruction metadata JSON.
        min_depth: Minimum valid depth in model units (default: 0.1).
        max_depth: Maximum valid depth in model units (default: 100.0). Configurable; not hardcoded.
        min_confidence: Minimum depth confidence threshold (default: 0.1). Preserves all valid points
                        by default; set higher (e.g. 1.2 or 1.5) or use adaptive filtering only when justified.
        subsample_step: Pixel sampling stride (default: 1).
        voxel_size: Optional voxel grid size in model units (e.g. 0.005) to consolidate nearby
                    points and average their colors. If None, points remain un-fused.
        save_unfused_ply_path: Optional path to save the confidence-filtered point cloud prior
                               to voxel fusion for direct comparison.

    Returns:
        Dictionary of reconstruction statistics and metadata.
    """
    t_start = time.perf_counter()

    # 1. Load predicted depth and confidence
    depth_data = np.load(depth_npz_path)
    depth_maps = depth_data["depth"]        # Shape: (N, H, W, 1) or (N, H, W)
    if "confidence" in depth_data:
        confidence_maps = depth_data["confidence"]  # Shape: (N, H, W)
    else:
        confidence_maps = np.ones(depth_maps.shape[:3], dtype=np.float32)

    # 2. Load camera poses
    poses_data = np.load(poses_npz_path)
    extrinsics = poses_data["extrinsics"]    # Shape: (N, 3, 4)
    intrinsics = poses_data["intrinsics"]    # Shape: (N, 3, 3)

    num_frames = len(extrinsics)
    if depth_maps.ndim == 4:
        h, w = depth_maps.shape[1], depth_maps.shape[2]
    else:
        h, w = depth_maps.shape[1], depth_maps.shape[2]

    # 3. Load and resize frame images for color mapping
    frames_path = Path(frames_dir)
    frame_files = sorted(list(frames_path.glob("*.png")) + list(frames_path.glob("*.jpg")))
    if len(frame_files) < num_frames:
        raise ValueError(f"Found {len(frame_files)} frames in {frames_dir}, but expected at least {num_frames}")

    frame_colors_list = []
    for i in range(num_frames):
        img = Image.open(frame_files[i]).convert("RGB")
        if img.size != (w, h):
            img = img.resize((w, h), Image.Resampling.BILINEAR)
        frame_colors_list.append(np.array(img, dtype=np.uint8))
    frame_colors = np.stack(frame_colors_list, axis=0)  # Shape: (N, H, W, 3)

    # 4. Backproject depth to 3D world points (official VGGT-Omega camera convention)
    t_backproject_start = time.perf_counter()
    world_points = depth_to_points(depth_maps, extrinsics, intrinsics)  # Shape: (N, H, W, 3)
    backproject_time = time.perf_counter() - t_backproject_start

    # 5. Filter valid points
    if depth_maps.ndim == 4:
        depth_flat = depth_maps[..., 0]
    else:
        depth_flat = depth_maps

    valid_mask = np.isfinite(depth_flat) & (depth_flat > min_depth)
    if max_depth is not None:
        valid_mask &= (depth_flat <= max_depth)
    if min_confidence is not None and min_confidence > 0:
        valid_mask &= (confidence_maps >= min_confidence)

    # Subsampling if requested
    if subsample_step > 1:
        step_mask = np.zeros_like(valid_mask, dtype=bool)
        step_mask[:, ::subsample_step, ::subsample_step] = True
        valid_mask &= step_mask

    raw_filtered_points = world_points[valid_mask]  # Shape: (M, 3)
    raw_filtered_colors = frame_colors[valid_mask]  # Shape: (M, 3)

    raw_points_count = len(raw_filtered_points)
    if raw_points_count == 0:
        raise RuntimeError("No valid 3D points survived the filtering criteria!")

    # Optionally persist the unfused point cloud for comparison
    unfused_ply_size_bytes = None
    if save_unfused_ply_path:
        unfused_ply_size_bytes = save_ply(raw_filtered_points, raw_filtered_colors, save_unfused_ply_path, binary=True)

    # 6. Optional voxel-grid fusion
    t_fusion_start = time.perf_counter()
    if voxel_size is not None and voxel_size > 0:
        fused_points, fused_colors = voxel_grid_fusion(raw_filtered_points, raw_filtered_colors, voxel_size)
    else:
        fused_points, fused_colors = raw_filtered_points, raw_filtered_colors
    fusion_time = time.perf_counter() - t_fusion_start

    total_points = len(fused_points)
    if total_points == 0:
        raise RuntimeError("No valid 3D points remained after voxel-grid fusion!")

    # 7. Compute spatial bounding box and color statistics
    bbox_min = fused_points.min(axis=0).tolist()
    bbox_max = fused_points.max(axis=0).tolist()
    bbox_extents = (fused_points.max(axis=0) - fused_points.min(axis=0)).tolist()
    centroid = fused_points.mean(axis=0).tolist()

    color_mean = fused_colors.mean(axis=0).tolist()
    color_std = fused_colors.std(axis=0).tolist()

    # 8. Save primary output PLY file
    t_io_start = time.perf_counter()
    ply_size_bytes = save_ply(fused_points, fused_colors, output_ply_path, binary=True)
    io_time = time.perf_counter() - t_io_start

    total_time = time.perf_counter() - t_start

    result_metadata = {
        "module": "F.O.R.G.E Spatial Reconstruction (Phase 2B - Controlled Accuracy)",
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "coordinate_units_note": "Model coordinate units (nominally meters); not independently verified against ground truth survey/GCPs.",
        "input": {
            "depth_predictions_path": str(Path(depth_npz_path).resolve()),
            "camera_poses_path": str(Path(poses_npz_path).resolve()),
            "frames_directory": str(Path(frames_dir).resolve()),
            "num_frames": num_frames,
            "depth_resolution": {"height": h, "width": w}
        },
        "filtering": {
            "min_depth": min_depth,
            "max_depth": max_depth,
            "min_confidence": min_confidence,
            "subsample_step": subsample_step,
            "voxel_size": voxel_size
        },
        "reconstruction": {
            "raw_filtered_points": raw_points_count,
            "total_points": total_points,
            "fusion_reduction_ratio": round(raw_points_count / total_points, 3) if total_points > 0 else 1.0,
            "bounding_box": {
                "min": [round(v, 4) for v in bbox_min],
                "max": [round(v, 4) for v in bbox_max],
                "extents": [round(v, 4) for v in bbox_extents],
                "centroid": [round(v, 4) for v in centroid]
            },
            "color_statistics": {
                "mean_rgb": [round(float(v), 2) for v in color_mean],
                "std_rgb": [round(float(v), 2) for v in color_std]
            },
            "output_ply_path": str(Path(output_ply_path).resolve()),
            "output_ply_size_bytes": ply_size_bytes,
            "output_ply_size_mb": round(ply_size_bytes / (1024**2), 2),
            "unfused_ply_path": str(Path(save_unfused_ply_path).resolve()) if save_unfused_ply_path else None,
            "unfused_ply_size_bytes": unfused_ply_size_bytes
        },
        "performance": {
            "backprojection_time_sec": round(backproject_time, 4),
            "fusion_time_sec": round(fusion_time, 4),
            "ply_io_time_sec": round(io_time, 4),
            "total_reconstruction_time_sec": round(total_time, 4)
        },
        "status": "SUCCESS"
    }

    # Save reconstruction metadata if requested
    if metadata_output_path:
        meta_file = Path(metadata_output_path)
        meta_file.parent.mkdir(parents=True, exist_ok=True)
        with open(meta_file, "w", encoding="utf-8") as f:
            json.dump(result_metadata, f, indent=2)

    return result_metadata
