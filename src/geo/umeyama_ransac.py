"""F.O.R.G.E — Umeyama–RANSAC 7-DoF Geospatial Similarity Alignment Module.

Implements robust 7-DoF similarity transformation (scale, 3D rotation, 3D translation)
between reconstructed camera trajectories (model space) and GPS telemetry (local metric ENU):

    p_enu = s * R * p_model + t

Features:
1. Closed-form Umeyama (1991) least-squares similarity estimator with SVD reflection correction.
2. RANSAC hypothesis-and-verification loop to reject noisy GPS fixes and transient jumps.
3. Degenerate configuration detection (collinear points, insufficient points, zero variance).
4. Rigorous pose and geometry transformation:
   - Point clouds & mesh vertices: p_new = s * R * p + t
   - Surface normals: n_new = R * n
   - Camera extrinsics: R_new = R_cam * R^T, t_new = s * t_cam - R_cam * R^T * t
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Tuple, List, Optional, Dict, Any, Union

import numpy as np
import open3d as o3d


@dataclass
class AlignmentResult:
    """Encapsulates the estimated 7-DoF similarity transformation and fit statistics."""
    success: bool
    scale: float
    rotation_matrix: List[List[float]]
    translation_vector: List[float]
    inlier_mask: List[bool]
    num_total_correspondences: int
    num_inliers: int
    inlier_ratio: float
    rmse_m: float
    mae_m: float
    max_error_m: float
    residuals_m: List[float]
    inlier_threshold_m: float
    ref_geodetic_anchor: Dict[str, float]
    status_message: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def umeyama_alignment(
    src_pts: np.ndarray,
    dst_pts: np.ndarray
) -> Tuple[float, np.ndarray, np.ndarray]:
    """
    Computes the optimal 7-DoF similarity transformation (s, R, t) that maps
    src_pts to dst_pts in the least-squares sense:

        dst_pts ≈ s * (src_pts @ R.T) + t

    Algorithm Reference:
        Shinji Umeyama, "Least-squares estimation of transformation parameters
        between two point patterns", IEEE TPAMI, Vol. 13, No. 4, 1991.

    Args:
        src_pts: (N, 3) source points (reconstructed camera centers in model space).
        dst_pts: (N, 3) destination points (measured camera centers in local ENU meters).

    Returns:
        (scale, rotation_3x3, translation_3)
    """
    src_pts = np.asarray(src_pts, dtype=np.float64)
    dst_pts = np.asarray(dst_pts, dtype=np.float64)

    if len(src_pts) != len(dst_pts):
        raise ValueError(f"Point count mismatch: src={len(src_pts)}, dst={len(dst_pts)}")

    n = len(src_pts)
    if n < 3:
        raise ValueError(f"At least 3 point correspondences are required for 3D alignment; got {n}")

    # Check finite coordinates
    if not (np.isfinite(src_pts).all() and np.isfinite(dst_pts).all()):
        raise ValueError("Source or destination coordinates contain non-finite values (NaN / Inf).")

    mu_x = src_pts.mean(axis=0)
    mu_y = dst_pts.mean(axis=0)

    X = src_pts - mu_x
    Y = dst_pts - mu_y

    var_x = float(np.mean(np.sum(X**2, axis=1)))
    if var_x < 1e-12:
        raise ValueError("Degenerate source point configuration: all source points are identical or variance < 1e-12.")

    # Cross-covariance matrix
    Sigma = (Y.T @ X) / n

    U, D, Vt = np.linalg.svd(Sigma)

    # Reflection-check matrix S
    S = np.eye(3, dtype=np.float64)
    det_rot = np.linalg.det(U) * np.linalg.det(Vt)
    if det_rot < 0:
        S[2, 2] = -1.0

    R = U @ S @ Vt

    # Ensure clean orthonormal rotation in SO(3)
    if np.linalg.det(R) < 0:
        R[2, :] *= -1.0

    scale = float(np.sum(D * np.diag(S)) / var_x)
    if scale <= 0:
        raise ValueError(f"Computed negative or zero scale ({scale}); alignment failed.")

    translation = mu_y - scale * (R @ mu_x)

    return scale, R, translation


def check_triangle_collinearity(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, min_area: float = 1e-6) -> bool:
    """Checks if three 3D points form a non-degenerate triangle (area > min_area)."""
    v1 = p2 - p1
    v2 = p3 - p1
    cross_prod = np.cross(v1, v2)
    area = 0.5 * np.linalg.norm(cross_prod)
    return area < min_area


def umeyama_ransac_alignment(
    src_pts: np.ndarray,
    dst_pts: np.ndarray,
    inlier_threshold_m: float = 2.5,
    max_iterations: int = 2000,
    confidence: float = 0.99,
    ref_geodetic_anchor: Optional[Dict[str, float]] = None,
    random_seed: int = 42
) -> AlignmentResult:
    """
    Robustly fits a 7-DoF similarity transformation using RANSAC and Umeyama estimation.

    Args:
        src_pts: (N, 3) reconstructed camera centers in model coordinates.
        dst_pts: (N, 3) GPS camera centers in local metric ENU coordinates.
        inlier_threshold_m: Maximum residual distance (meters) to classify as an inlier.
                            Defaults to 2.5m, reflecting typical consumer drone GPS uncertainty.
        max_iterations: Maximum RANSAC trials.
        confidence: Desired probability of selecting at least one clean sample subset.
        ref_geodetic_anchor: Optional dictionary with ref_lat, ref_lon, ref_alt.
        random_seed: Deterministic random seed for reproducible sample drawing.

    Returns:
        AlignmentResult dataclass containing transformation parameters and diagnostics.
    """
    src_pts = np.asarray(src_pts, dtype=np.float64)
    dst_pts = np.asarray(dst_pts, dtype=np.float64)

    if ref_geodetic_anchor is None:
        ref_geodetic_anchor = {"latitude_deg": 0.0, "longitude_deg": 0.0, "altitude_m": 0.0}

    n = len(src_pts)
    if n < 3:
        return AlignmentResult(
            success=False,
            scale=1.0,
            rotation_matrix=np.eye(3).tolist(),
            translation_vector=[0.0, 0.0, 0.0],
            inlier_mask=[False] * n,
            num_total_correspondences=n,
            num_inliers=0,
            inlier_ratio=0.0,
            rmse_m=999.0,
            mae_m=999.0,
            max_error_m=999.0,
            residuals_m=[999.0] * n,
            inlier_threshold_m=inlier_threshold_m,
            ref_geodetic_anchor=ref_geodetic_anchor,
            status_message=f"FAILED: Insufficient correspondences ({n} < 3)."
        )

    # Check finite numbers
    valid_coords = np.isfinite(src_pts).all(axis=1) & np.isfinite(dst_pts).all(axis=1)
    if not valid_coords.all():
        src_pts = src_pts[valid_coords]
        dst_pts = dst_pts[valid_coords]
        n = len(src_pts)
        if n < 3:
            return AlignmentResult(
                success=False,
                scale=1.0,
                rotation_matrix=np.eye(3).tolist(),
                translation_vector=[0.0, 0.0, 0.0],
                inlier_mask=[False] * len(valid_coords),
                num_total_correspondences=len(valid_coords),
                num_inliers=0,
                inlier_ratio=0.0,
                rmse_m=999.0,
                mae_m=999.0,
                max_error_m=999.0,
                residuals_m=[999.0] * len(valid_coords),
                inlier_threshold_m=inlier_threshold_m,
                ref_geodetic_anchor=ref_geodetic_anchor,
                status_message="FAILED: Too many non-finite coordinates."
            )

    rng = np.random.RandomState(random_seed)
    best_inliers: Optional[np.ndarray] = None
    best_num_inliers = 0
    best_residual_sum = float("inf")

    # If small number of correspondences (e.g. 3 to 10 frames), test combinations
    num_samples = min(max_iterations, 3000)

    for _ in range(num_samples):
        subset_idx = rng.choice(n, size=3, replace=False)
        p_src = src_pts[subset_idx]
        p_dst = dst_pts[subset_idx]

        # Check for collinearity
        if check_triangle_collinearity(p_src[0], p_src[1], p_src[2]):
            continue

        try:
            s_cand, R_cand, t_cand = umeyama_alignment(p_src, p_dst)
        except Exception:
            continue

        if s_cand <= 0 or not np.isfinite(s_cand):
            continue

        # Evaluate all points
        pred_dst = s_cand * (src_pts @ R_cand.T) + t_cand
        res = np.linalg.norm(dst_pts - pred_dst, axis=1)

        inlier_mask = res <= inlier_threshold_m
        num_inl = int(np.sum(inlier_mask))
        res_sum = float(np.sum(res[inlier_mask]))

        if (num_inl > best_num_inliers) or (num_inl == best_num_inliers and res_sum < best_residual_sum):
            best_num_inliers = num_inl
            best_inliers = inlier_mask
            best_residual_sum = res_sum

    # If no inlier set found, fallback to direct Umeyama on all points
    if best_inliers is None or best_num_inliers < 3:
        try:
            scale, R, translation = umeyama_alignment(src_pts, dst_pts)
            pred_dst = scale * (src_pts @ R.T) + translation
            residuals = np.linalg.norm(dst_pts - pred_dst, axis=1)
            inlier_mask = residuals <= inlier_threshold_m
            num_inliers = int(np.sum(inlier_mask))
            rmse = float(np.sqrt(np.mean(residuals**2)))
            mae = float(np.mean(residuals))
            max_err = float(np.max(residuals))

            return AlignmentResult(
                success=True,
                scale=scale,
                rotation_matrix=R.tolist(),
                translation_vector=translation.tolist(),
                inlier_mask=inlier_mask.tolist(),
                num_total_correspondences=n,
                num_inliers=num_inliers,
                inlier_ratio=round(num_inliers / n, 4),
                rmse_m=round(rmse, 4),
                mae_m=round(mae, 4),
                max_error_m=round(max_err, 4),
                residuals_m=residuals.round(4).tolist(),
                inlier_threshold_m=inlier_threshold_m,
                ref_geodetic_anchor=ref_geodetic_anchor,
                status_message="SUCCESS: Direct Umeyama fit without RANSAC pruning."
            )
        except Exception as e:
            return AlignmentResult(
                success=False,
                scale=1.0,
                rotation_matrix=np.eye(3).tolist(),
                translation_vector=[0.0, 0.0, 0.0],
                inlier_mask=[False] * n,
                num_total_correspondences=n,
                num_inliers=0,
                inlier_ratio=0.0,
                rmse_m=999.0,
                mae_m=999.0,
                max_error_m=999.0,
                residuals_m=[999.0] * n,
                inlier_threshold_m=inlier_threshold_m,
                ref_geodetic_anchor=ref_geodetic_anchor,
                status_message=f"FAILED: Umeyama estimation failed: {str(e)}"
            )

    # Refit Umeyama on the complete consensus inlier set
    try:
        scale, R, translation = umeyama_alignment(src_pts[best_inliers], dst_pts[best_inliers])
    except Exception as e:
        # Fallback to direct fit if inlier refit fails
        scale, R, translation = umeyama_alignment(src_pts, dst_pts)

    pred_dst = scale * (src_pts @ R.T) + translation
    residuals = np.linalg.norm(dst_pts - pred_dst, axis=1)
    final_inliers = residuals <= inlier_threshold_m
    num_final_inliers = int(np.sum(final_inliers))

    inlier_residuals = residuals[final_inliers] if num_final_inliers > 0 else residuals
    rmse = float(np.sqrt(np.mean(inlier_residuals**2)))
    mae = float(np.mean(inlier_residuals))
    max_err = float(np.max(inlier_residuals))

    return AlignmentResult(
        success=True,
        scale=round(float(scale), 6),
        rotation_matrix=R.round(6).tolist(),
        translation_vector=translation.round(6).tolist(),
        inlier_mask=final_inliers.tolist(),
        num_total_correspondences=n,
        num_inliers=num_final_inliers,
        inlier_ratio=round(num_final_inliers / n, 4),
        rmse_m=round(rmse, 4),
        mae_m=round(mae, 4),
        max_error_m=round(max_err, 4),
        residuals_m=residuals.round(4).tolist(),
        inlier_threshold_m=inlier_threshold_m,
        ref_geodetic_anchor=ref_geodetic_anchor,
        status_message=f"SUCCESS: {num_final_inliers}/{n} inliers retained (RMSE: {rmse:.3f}m)."
    )


def apply_similarity_transform(
    points: np.ndarray,
    scale: float,
    rotation_matrix: Union[np.ndarray, List[List[float]]],
    translation_vector: Union[np.ndarray, List[float]]
) -> np.ndarray:
    """
    Applies the similarity transformation p_new = s * R * p + t to a 3D point array.

    Args:
        points: (N, 3) input coordinates.
        scale: Uniform scale factor.
        rotation_matrix: (3, 3) rotation matrix.
        translation_vector: (3,) translation offset.

    Returns:
        (N, 3) transformed coordinates.
    """
    pts = np.asarray(points, dtype=np.float64)
    R = np.asarray(rotation_matrix, dtype=np.float64)
    t = np.asarray(translation_vector, dtype=np.float64)

    if len(pts) == 0:
        return pts

    return scale * (pts @ R.T) + t


def transform_camera_poses(
    extrinsics: np.ndarray,
    scale: float,
    rotation_matrix: Union[np.ndarray, List[List[float]]],
    translation_vector: Union[np.ndarray, List[float]]
) -> np.ndarray:
    """
    Transforms camera extrinsics matrices under the similarity transformation:

        p_new = s * R * p_old + t

    Maintains standard camera projection consistency:
        R_cam_new = R_cam_old @ R^T
        t_cam_new = s * t_cam_old - R_cam_old @ R^T @ t

    So that:
        C_new = - R_cam_new^T @ t_cam_new = s * R * C_old + t

    Args:
        extrinsics: (N, 3, 4) or (N, 4, 4) camera extrinsics matrices.
        scale: Estimated scale factor.
        rotation_matrix: (3, 3) estimated similarity rotation matrix.
        translation_vector: (3,) estimated similarity translation vector.

    Returns:
        Transformed extrinsics array with matching shape.
    """
    ext = np.asarray(extrinsics, dtype=np.float64).copy()
    R = np.asarray(rotation_matrix, dtype=np.float64)
    t = np.asarray(translation_vector, dtype=np.float64)

    is_4x4 = (ext.shape[-2:] == (4, 4))
    num_cams = len(ext)
    out_ext = np.zeros_like(ext)

    for i in range(num_cams):
        R_old = ext[i, :3, :3]
        t_old = ext[i, :3, 3]

        # Transformed camera orientation
        R_new = R_old @ R.T

        # Transformed camera translation
        t_new = scale * t_old - R_new @ t

        out_ext[i, :3, :3] = R_new
        out_ext[i, :3, 3] = t_new
        if is_4x4:
            out_ext[i, 3, :3] = 0.0
            out_ext[i, 3, 3] = 1.0

    return out_ext


def transform_mesh(
    mesh: o3d.geometry.TriangleMesh,
    scale: float,
    rotation_matrix: Union[np.ndarray, List[List[float]]],
    translation_vector: Union[np.ndarray, List[float]]
) -> o3d.geometry.TriangleMesh:
    """
    Transforms a triangle mesh: vertices scaled and translated, normals rotated.
    """
    R = np.asarray(rotation_matrix, dtype=np.float64)
    t = np.asarray(translation_vector, dtype=np.float64)

    aligned_mesh = o3d.geometry.TriangleMesh(mesh)
    verts = np.asarray(aligned_mesh.vertices)
    if len(verts) > 0:
        new_verts = scale * (verts @ R.T) + t
        aligned_mesh.vertices = o3d.utility.Vector3dVector(new_verts)

    if aligned_mesh.has_vertex_normals():
        normals = np.asarray(aligned_mesh.vertex_normals)
        new_normals = normals @ R.T
        aligned_mesh.vertex_normals = o3d.utility.Vector3dVector(new_normals)

    return aligned_mesh
