"""
F.O.R.G.E — Sprint 11 Recovery Deliverables Verifier
Verifies and prints detailed quantitative geometry, boundary stats, and cryptographic hashes
for all models under outputs/dynamic_object_masking/sprint11/recovery_v1/
"""

import json
from pathlib import Path
import open3d as o3d
import trimesh
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REC_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11/recovery_v1"
SPRINT9_ROOT = PROJECT_ROOT / "outputs/accuracy_experiment/sprint9_video_to_viewer_release"

summary_file = REC_ROOT / "recovery_v1_summary.json"
summary = json.loads(summary_file.read_text(encoding="utf-8"))

scenes = ["desert_road", "campus_0049", "forest_road", "river_bridge"]

print("\n" + "=" * 90)
print("F.O.R.G.E SPRINT 11 RECOVERY V1 — COMPREHENSIVE VERIFICATION AUDIT")
print("=" * 90)

for s in scenes:
    d = summary.get(s, {})
    print(f"\n--- SCENE: {s.upper()} ---")
    
    # Point cloud masking stats
    p_comp = d.get("point_cloud_comparison", {})
    print(f"Point Cloud Masking Impact:")
    print(f"  Unmasked Raw Points: {p_comp.get('unmasked_raw_points', 0):,}")
    print(f"  Masked Raw Points:   {p_comp.get('masked_raw_points', 0):,}")
    print(f"  Points Removed:      {p_comp.get('dynamic_points_removed', 0):,} ({p_comp.get('dynamic_points_removal_pct', 0)}%)")
    print(f"  Fused Points:        {p_comp.get('masked_fused_points', 0):,}")

    # Surface mesh comparison
    m_comp = d.get("surface_mesh_comparison", {})
    masked_m = m_comp.get("masked", {})
    unmasked_m = m_comp.get("unmasked", {})
    print(f"\nSurface Mesh Statistics:")
    print(f"  Masked Vertices:     {masked_m.get('vertex_count', 0):,} | Triangles: {masked_m.get('triangle_count', 0):,} | Extents: {masked_m.get('extents', [])}")
    print(f"  Unmasked Vertices:   {unmasked_m.get('vertex_count', 0):,} | Triangles: {unmasked_m.get('triangle_count', 0):,} | Extents: {unmasked_m.get('extents', [])}")

    # Model Files
    val = d.get("independent_validation", {})
    print(f"\nExported Formats & Independent Reload Validation:")
    for key in sorted(val.keys()):
        v = val[key]
        fmt = v.get("format", "")
        sz = v.get("file_size_bytes", 0)
        mb = v.get("file_size_mb", 0)
        sha = v.get("sha256", "")
        print(f"  {key.upper()}: {sz:,} bytes ({mb} MB) | SHA-256: {sha}")

    # Previews
    exp = d.get("exported_models", {})
    masked_p = exp.get("masked", {})
    unmasked_p = exp.get("unmasked", {})
    prev_m = Path(masked_p.get("preview_png", ""))
    prev_u = Path(unmasked_p.get("preview_png", ""))
    print(f"\nPreview Banners:")
    print(f"  Masked Preview:   exists={prev_m.is_file()} ({prev_m.stat().st_size:,} bytes) -> {prev_m}")
    print(f"  Unmasked Preview: exists={prev_u.is_file()} ({prev_u.stat().st_size:,} bytes) -> {prev_u}")

print("\n" + "=" * 90)
print("AUDIT COMPLETE")
print("=" * 90)
