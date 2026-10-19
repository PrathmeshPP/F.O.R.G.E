import json
from pathlib import Path

base = Path("outputs/accuracy_experiment/generalization_disaster_validation_v1/forest_road")
for s in ["baseline", "uniform_span", "viewpoint_diversity"]:
    p = base / f"frames_{s}" / "frame_selection_metadata.json"
    if p.exists():
        with open(p, "r", encoding="utf-8") as f:
            d = json.load(f)
        cov = d["trajectory_coverage_ratio"] * 100
        span = d["temporal_span_sec"]
        dur = d["duration_sec"]
        print(f"=== STRATEGY: {s.upper()} ===")
        print(f"  Temporal Coverage: {cov:.1f}% ({span:.1f}s / {dur:.1f}s)")
        print(f"  Mean Sharpness:    {d['mean_sharpness']:.2f} (min: {d['min_sharpness']:.2f}, max: {d['max_sharpness']:.2f})")
        print(f"  Timestamps (s):    {[round(x['timestamp_sec'], 3) for x in d['frames']]}")
        print()
