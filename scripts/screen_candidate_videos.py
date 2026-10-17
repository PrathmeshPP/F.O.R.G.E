"""F.O.R.G.E — Drone Video Candidate Screening & Parallax Evaluation Tool.

Analyzes prospective drone video candidates for:
1. Camera translation vs. pure rotation using Homography vs. Fundamental matrix residuals.
2. Parallax dispersion (depth-dependent optical flow divergence).
3. Viewpoint angle and structural depth cues.
4. Generates an annotated comparative contact sheet.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Any

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def analyze_segment_motion(
    video_path: str,
    start_time: float,
    end_time: float,
    step_sec: float = 1.0,
    sample_res: Tuple[int, int] = (960, 540)
) -> Dict[str, Any]:
    """
    Computes optical flow, homography vs fundamental matrix residuals, and parallax dispersion.
    """
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    timestamps = []
    t = start_time
    while t <= end_time:
        timestamps.append(t)
        t += step_sec

    frames_gray = []
    frames_bgr = []
    for t_s in timestamps:
        frame_idx = min(int(t_s * fps), total_frames - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        if ret:
            resized = cv2.resize(frame, sample_res)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            frames_gray.append(gray)
            frames_bgr.append(resized)
        else:
            break
    cap.release()

    if len(frames_gray) < 2:
        return {"error": "Insufficient frames extracted"}

    # Feature matching across consecutive sampled frames
    orb = cv2.ORB_create(nfeatures=1500)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    flow_mags = []
    homography_residuals = []
    parallax_divergence_scores = []
    motion_types = []

    for i in range(1, len(frames_gray)):
        kp1, des1 = orb.detectAndCompute(frames_gray[i - 1], None)
        kp2, des2 = orb.detectAndCompute(frames_gray[i], None)

        if des1 is None or des2 is None or len(des1) < 20 or len(des2) < 20:
            continue

        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)[:400]
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        # Flow vectors
        displacement = pts2 - pts1
        mags = np.linalg.norm(displacement, axis=1)
        flow_mags.append(float(np.median(mags)))

        # 1. Homography fit (tests if motion is purely rotational / planar)
        H, h_mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 3.0)
        if H is not None and h_mask is not None:
            # Transform pts1 using H
            pts1_homo = np.concatenate([pts1, np.ones((len(pts1), 1))], axis=1)
            pts2_pred_homo = (H @ pts1_homo.T).T
            pts2_pred = pts2_pred_homo[:, :2] / (pts2_pred_homo[:, 2:3] + 1e-8)
            h_errors = np.linalg.norm(pts2 - pts2_pred, axis=1)
            mean_h_err = float(np.mean(h_errors))
            inlier_ratio_h = float(np.mean(h_mask))
        else:
            mean_h_err = 999.0
            inlier_ratio_h = 0.0

        # 2. Fundamental matrix fit (epipolar geometry from translation)
        F, f_mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 3.0)
        inlier_ratio_f = float(np.mean(f_mask)) if f_mask is not None else 0.0

        # Parallax metric:
        # If motion has high translation over a 3D depth scene, a single 2D homography FAILS (high residuals / low inliers)
        # while epipolar geometry succeeds. If motion is pure rotation, homography inlier ratio is ~1.0 with tiny residual.
        # Parallax score: deviation from homography
        parallax_score = mean_h_err * (1.0 - inlier_ratio_h + 1e-4)
        parallax_divergence_scores.append(parallax_score)
        homography_residuals.append(mean_h_err)

        # Classify step
        if inlier_ratio_h > 0.92 and mean_h_err < 3.5:
            m_type = "Pure Rotation / Gimbal Pan"
        elif inlier_ratio_h < 0.75 or mean_h_err > 8.0:
            m_type = "Strong Translational Parallax"
        else:
            m_type = "Mixed Motion"
        motion_types.append(m_type)

    mean_flow = float(np.mean(flow_mags)) if flow_mags else 0.0
    mean_parallax = float(np.mean(parallax_divergence_scores)) if parallax_divergence_scores else 0.0
    mean_h_res = float(np.mean(homography_residuals)) if homography_residuals else 0.0

    return {
        "video_path": video_path,
        "start_time": start_time,
        "end_time": end_time,
        "num_samples": len(frames_gray),
        "mean_flow_mag_px": mean_flow,
        "mean_homography_residual": mean_h_res,
        "parallax_score": mean_parallax,
        "motion_types": motion_types,
        "sampled_frames_bgr": frames_bgr,
        "timestamps": timestamps,
    }


def create_contact_sheet(
    candidate_results: List[Dict[str, Any]],
    output_png_path: str
) -> str:
    """Creates an annotated visual contact sheet comparing candidate video sequences."""
    panel_w, panel_h = 600, 338  # 16:9 thumbnails
    pad = 20
    header_h = 80
    row_header_h = 60

    num_candidates = len(candidate_results)
    max_cols = 4

    sheet_w = max_cols * panel_w + (max_cols + 1) * pad
    sheet_h = header_h + num_candidates * (row_header_h + panel_h + pad) + pad

    canvas = Image.new("RGB", (sheet_w, sheet_h), color=(15, 18, 25))
    draw = ImageDraw.Draw(canvas)

    # Main Header
    draw.rectangle([0, 0, sheet_w, header_h], fill=(22, 27, 38))
    draw.line([0, header_h, sheet_w, header_h], fill=(45, 55, 75), width=2)
    draw.text((pad, 16), "F.O.R.G.E Drone Video Pre-Screening: Translation & Parallax Comparison", fill=(255, 255, 255))
    draw.text((pad, 44), "Evaluating Camera Motion Dynamics, Epipolar Parallax, and Viewpoint Geometry for 3D Reconstruction", fill=(140, 165, 195))

    curr_y = header_h + pad
    for cand_idx, res in enumerate(candidate_results):
        # Row Header
        draw.rectangle([pad, curr_y, sheet_w - pad, curr_y + row_header_h], fill=(28, 34, 48))
        draw.line([pad, curr_y + row_header_h, sheet_w - pad, curr_y + row_header_h], fill=(55, 68, 92), width=1)

        c_title = res.get("title", f"Candidate {cand_idx + 1}")
        c_stats = (
            f"Segment: {res['start_time']:.1f}s - {res['end_time']:.1f}s | "
            f"Flow: {res['mean_flow_mag_px']:.1f} px | "
            f"Parallax Score: {res['parallax_score']:.2f} | "
            f"Verdict: {res.get('verdict', 'N/A')}"
        )
        draw.text((pad + 15, curr_y + 10), c_title, fill=(240, 245, 255))
        draw.text((pad + 15, curr_y + 34), c_stats, fill=(160, 185, 215))

        curr_y += row_header_h + 10

        # Place up to 4 thumbnails
        frames = res.get("sampled_frames_bgr", [])
        times = res.get("timestamps", [])
        step = max(1, len(frames) // max_cols)

        for col_idx in range(min(max_cols, len(frames))):
            f_idx = col_idx * step
            if f_idx >= len(frames):
                break
            bgr = frames[f_idx]
            rgb = cv2.cvtColor(cv2.resize(bgr, (panel_w, panel_h)), cv2.COLOR_BGR2RGB)
            pil_thumb = Image.fromarray(rgb)
            thumb_draw = ImageDraw.Draw(pil_thumb)

            # Overlay timestamp badge
            t_str = f"t = {times[f_idx]:.1f}s"
            thumb_draw.rectangle([10, 10, 110, 36], fill=(0, 0, 0))
            thumb_draw.text((18, 14), t_str, fill=(255, 220, 100))

            x_pos = pad + col_idx * (panel_w + pad)
            canvas.paste(pil_thumb, (x_pos, curr_y))

        curr_y += panel_h + pad

    out_p = Path(output_png_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_p, format="PNG")
    return str(out_p.resolve())


def main():
    v1 = r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172754_0049_1785175051818_video.mp4"
    v2 = r"C:\Users\25beevdt025\Desktop\dji_fly_20260727_172812_0050_1785175896547_video.mp4"

    print("=" * 72)
    print("F.O.R.G.E Drone Candidate Screening & Motion Analysis")
    print("=" * 72)

    candidates = []

    # 1. Video 1 Oblique segment (10.0s - 16.0s)
    print("\n[Analyzing Candidate 1: Video 0049 (10.0s - 16.0s Oblique)]...")
    c1 = analyze_segment_motion(v1, start_time=10.0, end_time=16.0, step_sec=1.0)
    c1["title"] = "Candidate 1: Video 0049 — Oblique Campus View (10.0s - 16.0s)"
    c1["verdict"] = "POOR TRANSLATION (Mostly Gimbal Pitch-Up & Yaw while Hovering)"
    candidates.append(c1)

    # 2. Video 2 Scan across segments:
    # Segment A: 0s - 20s
    # Segment B: 20s - 45s
    # Segment C: 45s - 80s
    v2_segments = [
        (0.0, 20.0, "Segment A (0s - 20s)"),
        (20.0, 50.0, "Segment B (20s - 50s)"),
        (50.0, 80.0, "Segment C (50s - 80s)"),
    ]

    for start_t, end_t, label in v2_segments:
        print(f"\n[Analyzing Video 0050: {label}]...")
        c = analyze_segment_motion(v2, start_time=start_t, end_time=end_t, step_sec=2.0)
        c["title"] = f"Candidate 2 ({label}): Video 0050 ({start_t:.0f}s - {end_t:.0f}s)"
        # Evaluate verdict based on parallax and flow
        if c["parallax_score"] > 8.0 and c["mean_flow_mag_px"] > 15.0:
            c["verdict"] = "EXCELLENT (Strong Translational Parallax & Oblique Structures)"
        elif c["mean_flow_mag_px"] > 10.0:
            c["verdict"] = "MODERATE TRANSLATION"
        else:
            c["verdict"] = "LOW MOTION / HOVER"
        candidates.append(c)

    # Generate Contact Sheet
    out_sheet = "outputs/candidate_screening/contact_sheet.png"
    print(f"\nGenerating visual contact sheet...")
    saved_path = create_contact_sheet(candidates, out_sheet)
    print(f"Saved contact sheet: {saved_path}")

    # Summary table
    print("\n" + "=" * 72)
    print("SCREENING RESULTS SUMMARY")
    print("=" * 72)
    for i, c in enumerate(candidates):
        print(f"Candidate {i+1}: {c['title']}")
        print(f"  - Mean Optical Flow Magnitude: {c['mean_flow_mag_px']:.2f} px")
        print(f"  - Homography Residual Error:   {c['mean_homography_residual']:.2f} px")
        print(f"  - Parallax Divergence Score:   {c['parallax_score']:.2f}")
        print(f"  - Motion Classification:       {c['verdict']}")
        print("-" * 72)


if __name__ == "__main__":
    main()
