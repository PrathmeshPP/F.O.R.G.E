"""
F.O.R.G.E — Point Cloud Generation Script
Command-line runner to backproject depth predictions and camera poses into a 3D PLY point cloud.

Usage:
    python scripts/generate_point_cloud.py
"""

import sys
import argparse
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.reconstruction.point_cloud import create_point_cloud


def main():
    parser = argparse.ArgumentParser(description="Generate 3D point cloud from VGGT-Omega outputs")
    parser.add_argument(
        "--depth",
        default=str(PROJECT_ROOT / "outputs" / "predictions" / "depth_predictions.npz"),
        help="Path to depth_predictions.npz"
    )
    parser.add_argument(
        "--poses",
        default=str(PROJECT_ROOT / "outputs" / "predictions" / "camera_poses.npz"),
        help="Path to camera_poses.npz"
    )
    parser.add_argument(
        "--frames",
        default=str(PROJECT_ROOT / "outputs" / "frames"),
        help="Directory containing extracted frame images"
    )
    parser.add_argument(
        "--output-ply",
        default=str(PROJECT_ROOT / "outputs" / "reconstruction" / "point_cloud.ply"),
        help="Path for exported PLY file"
    )
    parser.add_argument(
        "--output-meta",
        default=str(PROJECT_ROOT / "outputs" / "reconstruction" / "reconstruction_metadata.json"),
        help="Path for reconstruction metadata JSON"
    )
    parser.add_argument("--min-depth", type=float, default=0.1, help="Minimum depth in meters (default: 0.1)")
    parser.add_argument("--max-depth", type=float, default=100.0, help="Maximum depth in meters (default: 100.0)")
    parser.add_argument("--min-confidence", type=float, default=0.1, help="Minimum depth confidence threshold (default: 0.1)")
    parser.add_argument("--step", type=int, default=1, help="Pixel subsampling step (default: 1)")

    args = parser.parse_args()

    print("=" * 65)
    print("F.O.R.G.E Phase 2B — Point Cloud Reconstruction")
    print("=" * 65)
    print(f"Depth input:       {args.depth}")
    print(f"Poses input:       {args.poses}")
    print(f"Frames directory:  {args.frames}")
    print(f"Output PLY:        {args.output_ply}")
    print(f"Output Metadata:   {args.output_meta}")
    print(f"Min Confidence:    {args.min_confidence}")
    print(f"Depth range:       [{args.min_depth}m, {args.max_depth}m]")
    print("-" * 65)

    meta = create_point_cloud(
        depth_npz_path=args.depth,
        poses_npz_path=args.poses,
        frames_dir=args.frames,
        output_ply_path=args.output_ply,
        metadata_output_path=args.output_meta,
        min_depth=args.min_depth,
        max_depth=args.max_depth,
        min_confidence=args.min_confidence,
        subsample_step=args.step
    )

    recon = meta["reconstruction"]
    bbox = recon["bounding_box"]
    perf = meta["performance"]

    print("\n[Reconstruction Results]")
    print(f"  Total Valid 3D Points: {recon['total_points']:,}")
    print(f"  PLY File Size:         {recon['output_ply_size_bytes']:,} bytes ({recon['output_ply_size_mb']:.2f} MB)")
    print("\n[Bounding Box Dimensions (Meters)]")
    print(f"  Min [X, Y, Z]:         {bbox['min']}")
    print(f"  Max [X, Y, Z]:         {bbox['max']}")
    print(f"  Extents [dX, dY, dZ]:  {bbox['extents']}")
    print(f"  Centroid [X, Y, Z]:    {bbox['centroid']}")
    print("\n[Execution Timing]")
    print(f"  Backprojection Time:   {perf['backprojection_time_sec']:.4f}s")
    print(f"  PLY Serialization:     {perf['ply_io_time_sec']:.4f}s")
    print(f"  Total Elapsed Time:    {perf['total_reconstruction_time_sec']:.4f}s")
    print("=" * 65)
    print("RECONSTRUCTION: SUCCESS")
    print("=" * 65)


if __name__ == "__main__":
    main()
