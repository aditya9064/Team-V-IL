"""Audit supplied CSI files without changing source data or fitting a model."""

import argparse
import csv
import json
import statistics
from collections import Counter
from pathlib import Path


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as source:
        return list(csv.DictReader(source))


def audit(root):
    report = {"dataset": root.name, "cameras": {}, "issues": []}
    videos = read_csv(root / "videos.csv")
    report["runs"] = videos
    report["manifest_vial_count"] = sum(int(row["n_vials"]) for row in videos)
    for camera in ("cap_seal", "tank_level", "vial_fill"):
        folder = root / camera
        index = read_csv(folder / "image_index.csv")
        names = [row["filename"] for row in index]
        missing = [name for name in names if not (folder / "images" / name).is_file()]
        missing_videos = [row["video_id"] for row in videos
                          if not (folder / "videos" / f"{camera}_video_{row['video_id']}.mp4").is_file()]
        duplicate_count = len(names) - len(set(names))
        report["cameras"][camera] = {
            "indexed_images": len(index),
            "images_by_run": dict(sorted(Counter(row["video_id"] for row in index).items())),
            "video_files": len(list((folder / "videos").glob("*.mp4"))),
            "missing_indexed_images": len(missing),
            "duplicate_filenames": duplicate_count,
            "missing_video_ids": missing_videos,
        }
        if missing or missing_videos or duplicate_count:
            report["issues"].append(f"{camera}: missing or duplicate media references")

    labels = read_csv(root / "vial_fill/labels/video5_vial_recipes.csv")
    frames = read_csv(root / "vial_fill/labels/video5_frame_labels.csv")
    json_labels = json.loads((root / "vial_fill/labels/video5_vial_recipes.json").read_text())
    invalid = []
    previous_end = None
    for row in labels:
        components = [float(row[key] or 0) for key in ("t100_ml", "t200_ml", "t400_ml")]
        total = float(row["total_ml"])
        start, end = float(row["start_s"]), float(row["end_s"])
        reasons = []
        if abs(sum(components) - total) > 0.001:
            reasons.append("component total")
        if not 0 < total <= 80 or components[1] > 50 or any(0 < volume < 10 for volume in components):
            reasons.append("recipe volume constraint")
        if end <= start or abs(end - start - float(row["duration_s"])) > 0.002:
            reasons.append("fill duration")
        if previous_end is not None and start < previous_end:
            reasons.append("overlapping interval")
        if any(abs(float(row[key]) - seconds * 30) > 1 for key, seconds in
               (("start_frame", start), ("end_frame", end))):
            reasons.append("frame/time mapping")
        if reasons:
            invalid.append({"vial": row["vial"], "reasons": reasons})
        previous_end = end
    if invalid:
        report["issues"].append("Invalid per-vial labels")
    if len(json_labels) != len(labels):
        report["issues"].append("CSV/JSON label count differs")
    report["labels"] = {
        "vials": len(labels),
        "recipes": dict(sorted(Counter(row["recipe_id"] for row in labels).items(), key=lambda item: int(item[0]))),
        "frame_rows": len(frames),
        "frame_fill_state_counts": dict(Counter(row["in_fill"] for row in frames)),
        "fill_duration_s": {
            "min": min(float(row["duration_s"]) for row in labels),
            "median": statistics.median(float(row["duration_s"]) for row in labels),
            "max": max(float(row["duration_s"]) for row in labels),
        },
        "invalid_rows": invalid,
        "per_vial_columns": list(labels[0]),
        "quality_or_cause_ground_truth": "No explicit defect, pass/fail, or carryover-cause field in supplied per-vial schema",
    }
    report["total_indexed_images"] = sum(item["indexed_images"] for item in report["cameras"].values())
    report["total_video_files"] = sum(item["video_files"] for item in report["cameras"].values())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = audit(args.dataset)
    result = json.dumps(report, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(result + "\n")
    print(result)
    return 1 if report["issues"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
