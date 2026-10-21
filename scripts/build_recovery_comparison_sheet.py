"""
F.O.R.G.E — Sprint 11 Recovery Comparison Sheet Generator
Generates comprehensive 3-way matched comparison cards:
Column 1: Sprint 9 Verified Baseline
Column 2: Initial Sprint 11 Output (Regressed)
Column 3: Sprint 11 Recovery v1 (Repaired & Masked)
"""

from __future__ import annotations

from pathlib import Path
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent

SPRINT9_ROOT = PROJECT_ROOT / "outputs/accuracy_experiment/sprint9_video_to_viewer_release"
SPRINT11_INIT_ROOT = PROJECT_ROOT / "outputs/dynamic_object_masking/sprint11"
RECOVERY_V1_ROOT = SPRINT11_INIT_ROOT / "recovery_v1"


def render_mesh(mesh_path: Path, front: list, up: list, lookat: list, zoom: float, width: int = 800, height: int = 600) -> Image.Image:
    if not mesh_path.is_file():
        img = Image.new("RGB", (width, height), (30, 30, 30))
        d = ImageDraw.Draw(img)
        d.text((width // 4, height // 2), f"Missing: {mesh_path.name}", fill=(255, 100, 100))
        return img

    m = o3d.io.read_triangle_mesh(str(mesh_path))
    m.compute_vertex_normals()

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    vis.add_geometry(m)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.10, 0.14])
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Color
    opt.light_on = False

    ctrl = vis.get_view_control()
    ctrl.set_lookat(lookat)
    ctrl.set_front(front)
    ctrl.set_up(up)
    ctrl.set_zoom(zoom)

    vis.poll_events()
    vis.update_renderer()
    buf = vis.capture_screen_float_buffer(do_render=True)
    vis.destroy_window()

    return Image.fromarray((np.asarray(buf) * 255).astype(np.uint8))


def main():
    print("[*] Generating Recovery v1 Master Visual Comparison Sheets...")

    scenes = {
        "desert_road": {
            "title": "Desert Road (Corridor Geometry & Moving Vehicle Exclusion)",
            "s9": SPRINT9_ROOT / "desert_road/candidate_a/surface_mesh_repaired.ply",
            "init11": SPRINT11_INIT_ROOT / "desert_road/masked_reconstruction.ply",
            "rec11": RECOVERY_V1_ROOT / "desert_road/masked_reconstruction.ply",
            "front": [0.0, -0.707, -0.707],
            "up": [0.0, -0.707, 0.707],
            "zoom": 0.65
        },
        "campus_0049": {
            "title": "Campus 0049 (Architectural Facades, Roofs & Walkways)",
            "s9": SPRINT9_ROOT / "campus_0049/surface_mesh_repaired.ply",
            "init11": SPRINT11_INIT_ROOT / "campus_0049/masked_reconstruction.ply",
            "rec11": RECOVERY_V1_ROOT / "campus_0049/masked_reconstruction.ply",
            "front": [0.25, -0.45, -0.85],
            "up": [0.0, -1.0, 0.0],
            "zoom": 0.70
        },
        "forest_road": {
            "title": "Forest Road (Canopy Voids & Winding Asphalt Roadway)",
            "s9": SPRINT9_ROOT / "forest_road/surface_mesh_repaired.ply",
            "init11": SPRINT11_INIT_ROOT / "forest_road/masked_reconstruction.ply",
            "rec11": RECOVERY_V1_ROOT / "forest_road/masked_reconstruction.ply",
            "front": [0.25, -0.45, -0.85],
            "up": [0.0, -1.0, 0.0],
            "zoom": 0.70
        },
        "river_bridge": {
            "title": "River/Bridge (Bridge Deck, Railings, Piers & Channel)",
            "s9": SPRINT9_ROOT / "river_bridge/surface_mesh_repaired.ply",
            "init11": SPRINT11_INIT_ROOT / "river_bridge/masked_reconstruction.ply",
            "rec11": RECOVERY_V1_ROOT / "river_bridge/masked_reconstruction.ply",
            "front": [0.0, -0.707, -0.707],
            "up": [0.0, -0.707, 0.707],
            "zoom": 0.65
        }
    }

    card_w, card_h = 640, 420
    header_h = 75
    gap_y = 40
    num_scenes = len(scenes)

    master_w = card_w * 3
    master_h = header_h + num_scenes * (card_h + gap_y)

    master_canvas = Image.new("RGB", (master_w, master_h), (14, 18, 24))
    m_draw = ImageDraw.Draw(master_canvas)

    m_draw.text((30, 16), "F.O.R.G.E Sprint 11 Recovery Audit: 3-Way Controlled Baseline Comparison", fill=(255, 255, 255))
    m_draw.text((30, 42), "Col 1: Sprint 9 Verified Baseline | Col 2: Initial Sprint 11 Run (Regressed) | Col 3: Sprint 11 Recovery v1 (Repaired & YOLO-Masked)", fill=(180, 210, 240))

    for s_idx, (sname, scfg) in enumerate(scenes.items()):
        print(f"  Rendering comparison for {sname}...")
        m_s9 = o3d.io.read_triangle_mesh(str(scfg["s9"]))
        lookat = m_s9.get_axis_aligned_bounding_box().get_center()

        im_s9 = render_mesh(scfg["s9"], scfg["front"], scfg["up"], lookat, scfg["zoom"], card_w, card_h)
        im_init = render_mesh(scfg["init11"], scfg["front"], scfg["up"], lookat, scfg["zoom"], card_w, card_h)
        im_rec = render_mesh(scfg["rec11"], scfg["front"], scfg["up"], lookat, scfg["zoom"], card_w, card_h)

        y_top = header_h + s_idx * (card_h + gap_y)

        # Draw scene title bar
        m_draw.rectangle([0, y_top - 28, master_w, y_top], fill=(22, 28, 38))
        m_draw.text((30, y_top - 22), f"Scene {s_idx + 1}: {scfg['title']}", fill=(0, 255, 200))

        master_canvas.paste(im_s9, (0, y_top))
        master_canvas.paste(im_init, (card_w, y_top))
        master_canvas.paste(im_rec, (card_w * 2, y_top))

        # Column badges
        m_draw.rectangle([10, y_top + 10, 310, y_top + 45], fill=(10, 14, 20))
        m_draw.text((18, y_top + 16), "Sprint 9 Verified Baseline", fill=(100, 255, 160))

        m_draw.rectangle([card_w + 10, y_top + 10, card_w + 310, y_top + 45], fill=(10, 14, 20))
        m_draw.text((card_w + 18, y_top + 16), "Sprint 11 Initial (Coarse/Fragmented)", fill=(255, 160, 80))

        m_draw.rectangle([card_w * 2 + 10, y_top + 10, card_w * 2 + 310, y_top + 45], fill=(10, 14, 20))
        m_draw.text((card_w * 2 + 18, y_top + 16), "Sprint 11 Recovery v1 (Repaired)", fill=(0, 255, 220))

        # Save individual scene comparison
        scene_canvas = Image.new("RGB", (card_w * 3, card_h + 60), (14, 18, 24))
        sc_draw = ImageDraw.Draw(scene_canvas)
        sc_draw.text((25, 14), f"F.O.R.G.E Sprint 11 Recovery Comparison — {scfg['title']}", fill=(255, 255, 255))
        sc_draw.text((25, 36), "Sprint 9 Baseline (Left) vs Initial Sprint 11 (Center) vs Recovery v1 Masked (Right)", fill=(180, 210, 240))
        scene_canvas.paste(im_s9, (0, 60))
        scene_canvas.paste(im_init, (card_w, 60))
        scene_canvas.paste(im_rec, (card_w * 2, 60))

        scene_out_img = RECOVERY_V1_ROOT / sname / "baseline_vs_recovery_comparison.png"
        scene_canvas.save(scene_out_img)
        print(f"    Saved scene comparison: {scene_out_img}")

    master_out = RECOVERY_V1_ROOT / "all_scenes_recovery_comparison.png"
    master_canvas.save(master_out)
    print(f"[+] Saved master comparison sheet: {master_out}")


if __name__ == "__main__":
    main()
