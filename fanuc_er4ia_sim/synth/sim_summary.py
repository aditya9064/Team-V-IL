"""
Summarise digital-twin episodes written by rosie/jobs/sim_job.sh.

    python -m synth.sim_summary <runs/sim>

One block per model folder: tower success, pickups / placements per attempt,
simulated cycle time (rendered frames at 30 fps), perception errors against ground
truth (plan_sort's perception_evaluation.json) and the reasons episodes failed.
"""

import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

FPS = 30


def load(path):
    return json.loads(path.read_text()) if path.exists() else None


def episode(run):
    report = load(run / "execution_report.json")
    plan = load(run / "sort_plan.json") or {}
    evaluation = load(run / "perception_evaluation.json") or {}
    cubes = [m for m in evaluation.get("matches", []) if m.get("category") == "cube"]
    row = {"seed": run.name, "stage": "executed" if report else
           "planned" if plan else "perceived" if (run / "detections.json").exists() else
           "captured" if (run / "world_camera.png").exists() else "none",
           "planned_picks": len(plan.get("picks", [])),
           "pos_mm": [m["position_error_mm"] for m in cubes],
           "yaw_err": [m["yaw_error_deg_mod90"] for m in cubes if "yaw_error_deg_mod90" in m],
           "colour_ok": sum(m.get("colour_correct", False) for m in cubes), "matched": len(cubes),
           "missed": len(evaluation.get("missed_objects", []))}
    if report:
        row.update(complete=bool(report.get("task_complete")),
                   pickups=report.get("verified_pickups", 0),
                   placements=report.get("verified_placements", 0),
                   attempts=len(report.get("picks", [])),
                   seconds=report.get("frames", 0) / FPS,
                   abort=report.get("abort_reason"))
    return row


def block(model, rows):
    done = [r for r in rows if "complete" in r]
    attempts = sum(r["attempts"] for r in done)
    pos = [p for r in rows for p in r["pos_mm"]]
    yaw = [y for r in rows for y in r["yaw_err"]]
    matched = sum(r["matched"] for r in rows)
    lines = [f"=== {model}: {len(rows)} episodes, {len(done)} executed",
             f"tower complete        : {sum(r['complete'] for r in done)}/{len(done)}",
             f"picks per attempt     : {sum(r['pickups'] for r in done)}/{attempts}",
             f"placements per attempt: {sum(r['placements'] for r in done)}/{attempts}"]
    if done:
        lines.append(f"sim time per episode  : {np.mean([r['seconds'] for r in done]):.1f} s "
                     f"({np.mean([r['seconds'] / max(r['attempts'], 1) for r in done]):.1f} s per pick)")
    if pos:
        lines.append(f"cube position error   : median {np.median(pos):.2f} mm, p90 {np.percentile(pos, 90):.2f} mm")
    if yaw:
        lines.append(f"cube yaw error        : median {np.median(yaw):.2f} deg, p90 {np.percentile(yaw, 90):.2f} deg")
    lines.append(f"cube colour correct   : {sum(r['colour_ok'] for r in rows)}/{matched}; "
                 f"missed objects {sum(r['missed'] for r in rows)}")
    stuck = Counter(r["stage"] for r in rows if r["stage"] != "executed")
    if stuck:
        lines.append(f"stopped before robot  : {dict(stuck)}")
    aborts = Counter(r["abort"] for r in done if r.get("abort"))
    for reason, n in aborts.most_common(5):
        lines.append(f"  failure x{n}: {reason}")
    return "\n".join(lines)


def main():
    root = Path(sys.argv[1])
    for model_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        runs = sorted(model_dir.glob("seed_*"), key=lambda p: int(p.name.split("_")[1]))
        if runs:
            print(block(model_dir.name, [episode(r) for r in runs]) + "\n")


if __name__ == "__main__":
    main()
