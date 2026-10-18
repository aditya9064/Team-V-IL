"""
B1-B3 check on held-out synthetic frames: YOLO detections -> plan_sort geometry -> error
against the simulator's true object poses (saved per frame by generate_v2.py).

    python -m synth.eval_perception <frames dir> --weights best.pt --out report.json [--limit N]

<frames dir> holds <stem>__<camera>.png (the detector crop), .json (generate_v2 meta).
The crop is pasted back into a full-resolution canvas at its ROI so the calibrated
camera model applies unchanged. Matching mirrors plan_sort.score_observations:
one-to-one by table distance, accepted within 25 mm.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

import scene_config as C
from backproject import CameraModel
from cube_pose import square_yaw
from perceive_yolo import perceive
from plan_sort import build_plan

MATCH_MM = 25.0
ON_TABLE_Z = 0.001  # metres; higher cubes are stacked in towers
# plan_sort rejections caused by perception; task-level ones (e.g. the stack task's
# fixed 2-red/2-yellow/2-blue colour set) do not count against B1-B3.
PERCEPTION_REJECTS = {"cube_pose_missing_or_invalid", "cube_pose_low_confidence",
                      "cube_pose_fit_uncertain", "localisation_uncertain",
                      "cube_colour_unknown", "outside_work_surface"}


def full_frame(crop, roi, cam):
    canvas = np.zeros((cam.H, cam.W, 3), np.uint8)
    x0, y0, x1, y1 = roi
    canvas[y0:y1, x0:x1] = crop[: y1 - y0, : x1 - x0]
    return canvas


def visible_truth(meta, cam, roi):
    """True objects that are visible and project inside the detector crop."""
    x0, y0, x1, y1 = roi
    out = []
    for g in meta["scene"]["objects"]:
        if not g.get("visible", True):
            continue
        uv = cam.project([g["x"], g["y"], cam.table_z + C.CUBE_EDGE / 2 + g.get("z", 0.0)])
        if uv is not None and x0 <= uv[0] < x1 and y0 <= uv[1] < y1:
            out.append(g)
    return out


def match(preds, truths):
    """[(pred, truth, dist_mm)] one-to-one within MATCH_MM; also unmatched counts."""
    if not preds or not truths:
        return [], len(preds), len(truths)
    dist = np.array([[np.hypot(p["xy"][0] - g["x"], p["xy"][1] - g["y"]) * 1000 for g in truths]
                     for p in preds])
    pairs = [(i, j) for i, j in zip(*linear_sum_assignment(dist)) if dist[i, j] <= MATCH_MM]
    return ([(preds[i], truths[j], float(dist[i, j])) for i, j in pairs],
            len(preds) - len(pairs), len(truths) - len(pairs))


def cube_rows(plan, truths):
    preds = [o for o in plan["observations"] if o.get("category") == "cube" and "xy" in o]
    cubes = [g for g in truths if g["category"] == "cube"]
    pairs, fp, fn = match(preds, cubes)
    rows = []
    for p, g, mm in pairs:
        row = {"colour_ok": p.get("colour") == g["colour"], "pos_mm": mm,
               "stacked": g.get("z", 0.0) > ON_TABLE_Z,
               "pose_ready": p.get("skip_reason") not in PERCEPTION_REJECTS,
               "reject": p.get("skip_reason")}
        if "yaw_deg" in p:
            row["yaw_err"] = abs(square_yaw(p["yaw_deg"] - g["yaw_deg"]))
        rows.append(row)
    return rows, fp, fn


def other_rows(plan, truths):
    """Caps and vials: position only (orientation needs the crop models)."""
    out = {}
    for category in ("cap", "vial"):
        preds = [o for o in plan["observations"] if o.get("category") == category and "xy_raw" in o]
        preds = [dict(o, xy=o["xy_raw"]) for o in preds]
        pairs, fp, fn = match(preds, [g for g in truths if g["category"] == category])
        out[category] = {"pos_mm": [mm for _, _, mm in pairs], "fp": fp, "fn": fn}
    return out


def evaluate_frame(model, png):
    meta = json.loads(png.with_suffix(".json").read_text())
    camera = png.stem.rsplit("__", 1)[-1]
    cam = CameraModel(meta["camera"])
    roi = meta["roi"]
    img = full_frame(cv2.imread(str(png)), roi, cam)
    start = time.time()
    det = perceive(model, img, cam, camera)
    stack_plan = build_plan(det, cam, img, task="cube_stack")
    sort_plan = build_plan(det, cam, img, task="sort")
    seconds = time.time() - start
    truths = visible_truth(meta, cam, roi)
    cubes, fp, fn = cube_rows(stack_plan, truths)
    return {"frame": png.stem, "camera": camera, "mode": meta["scene"].get("mode"),
            "seconds": seconds, "detect_s": det["timing_s"]["detect"], "cubes": cubes,
            "cube_fp": fp, "cube_fn": fn, "other": other_rows(sort_plan, truths)}


def pct(values, q):
    return round(float(np.percentile(values, q)), 2) if values else None


def summarise(frames):
    out = {}
    for camera in sorted({f["camera"] for f in frames}):
        fs = [f for f in frames if f["camera"] == camera]
        rows = [r for f in fs for r in f["cubes"]]
        table = [r for r in rows if not r["stacked"]]
        tp, fp, fn = len(rows), sum(f["cube_fp"] for f in fs), sum(f["cube_fn"] for f in fs)
        pos = [r["pos_mm"] for r in table]
        yaw = [r["yaw_err"] for r in table if "yaw_err" in r]
        out[camera] = {
            "frames": len(fs), "cubes_true": tp + fn,
            "cube_recall": round(tp / max(tp + fn, 1), 3), "cube_precision": round(tp / max(tp + fp, 1), 3),
            # stacked cubes share one table xy, so position matching can pair them with the
            # wrong true cube; judge colour on single cubes and report stacked ones separately
            "colour_accuracy": round(sum(r["colour_ok"] for r in table) / max(len(table), 1), 3),
            "stacked_cubes_detected": len(rows) - len(table),
            "on_table_cubes": len(table), "pos_mm_median": pct(pos, 50), "pos_mm_p90": pct(pos, 90),
            "pos_within_2mm": round(sum(p <= 2 for p in pos) / max(len(pos), 1), 3),
            "yaw_deg_median": pct(yaw, 50), "yaw_deg_p90": pct(yaw, 90),
            "yaw_within_5deg": round(sum(y <= 5 for y in yaw) / max(len(yaw), 1), 3),
            "pose_ready_share": round(sum(r["pose_ready"] for r in table) / max(len(table), 1), 3),
            "sec_per_frame": round(float(np.mean([f["seconds"] for f in fs])), 2),
            "detect_sec_per_frame": round(float(np.mean([f["detect_s"] for f in fs])), 3),
        }
        for category in ("cap", "vial"):
            mm = [m for f in fs for m in f["other"][category]["pos_mm"]]
            out[camera][f"{category}_pos_mm_median"] = pct(mm, 50)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, help="max frames per camera")
    ap.add_argument("--cameras", default="table,side,wrist")
    args = ap.parse_args()
    from ultralytics import YOLO

    model = YOLO(args.weights)
    frames = []
    for camera in args.cameras.split(","):
        pngs = sorted(Path(args.frames).glob(f"*__{camera}.png"))[: args.limit]
        for i, png in enumerate(pngs, 1):
            frames.append(evaluate_frame(model, png))
            print(f"[eval] {camera} {i}/{len(pngs)} {png.stem} {frames[-1]['seconds']:.1f}s", flush=True)
    summary = summarise(frames)
    Path(args.out).write_text(json.dumps({"summary": summary, "frames": frames}, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
