import json
from pathlib import Path

p = Path("outputs/accuracy_experiment/generalization_disaster_validation_v1/evaluations/generalization_disaster_validation_metrics.json")
with open(p, "r", encoding="utf-8") as f:
    d = json.load(f)

for sname, sdata in d["scenes"].items():
    print("=" * 70)
    print(f"SCENE: {sname.upper()}")
    print("=" * 70)
    pcd = sdata["point_cloud"]
    mesh = sdata["surface_mesh"]
    traj = sdata["trajectory_diagnostics"]
    dis = sdata["disaster_readiness_evaluation"]
    print(f"  Active Points:      {pcd['active_points']:,} (Voxel size: {pcd['voxel_size']})")
    print(f"  Extents [dX,dY,dZ]: {pcd['extents']}")
    print(f"  Roughness Residual: {pcd['surface_roughness_residual']}")
    print(f"  Mesh Topo:          {mesh['vertex_count']:,} verts, {mesh['triangle_count']:,} faces (Edge Manifold: {mesh['is_edge_manifold']})")
    print(f"  Motion Type:        {traj['motion_type']} (Baseline Extent: {traj['trajectory_extent']})")
    print("  Disaster Verdicts:")
    for v_k, v_val in dis["operational_readiness_verdict"].items():
        if v_k != "operational_warnings":
            print(f"    - {v_k}: {v_val}")
    print("  Operational Limitations & Warnings:")
    for w in dis["operational_readiness_verdict"]["operational_warnings"]:
        print(f"    * {w}")
    print()
