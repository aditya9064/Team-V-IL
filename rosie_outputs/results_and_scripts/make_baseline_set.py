"""Pick a fixed, seeded sample of validation images for the Cosmos-vs-YOLO baseline.

    python3 make_baseline_set.py <datasets dir> <out dir> [--extended --exclude old/manifest.csv]

Copies images + YOLO labels (dereferencing symlinks) and writes manifest.csv with
the source set (v1_easy / v2_hard) and camera (table / side / wrist) of each image.
Stdlib only, so it runs on the Rosie login node.
"""
import argparse
import csv
import json
import random
import shutil
from pathlib import Path

SEED = 20261016
QUOTA = {("v1_easy", "table"): 20, ("v2_hard", "table"): 30,
         ("v2_hard", "side"): 15, ("v2_hard", "wrist"): 15}
# --extended: a larger, disjoint set for firmer baseline numbers (more side/wrist frames)
EXTENDED_SEED = 20261017
EXTENDED_QUOTA = {("v1_easy", "table"): 40, ("v2_hard", "table"): 110,
                  ("v2_hard", "side"): 75, ("v2_hard", "wrist"): 75}
SOURCES = {"v1_easy": "cubes_all", "v2_hard": "cubes_v2"}


def camera_of(image):
    """Camera recorded in the dataset.json of the chunk the image came from."""
    meta = image.resolve().parent.parent / "dataset.json"
    return json.loads(meta.read_text()).get("camera", "table") if meta.exists() else "table"


def candidates(datasets, source):
    root = datasets / SOURCES[source]
    by_camera = {}
    for image in sorted((root / "images" / "val").glob("*.png")):
        by_camera.setdefault(camera_of(image), []).append(image)
    return root, by_camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("datasets")
    ap.add_argument("out")
    ap.add_argument("--extended", action="store_true")
    ap.add_argument("--exclude", help="manifest.csv of a set whose images must not be reused")
    args = ap.parse_args()
    datasets, out = Path(args.datasets), Path(args.out)
    quota, seed = (EXTENDED_QUOTA, EXTENDED_SEED) if args.extended else (QUOTA, SEED)
    used = set()
    if args.exclude:
        used = {r["image"] for r in csv.DictReader(open(args.exclude))}
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    rows = []
    for source in SOURCES:
        root, by_camera = candidates(datasets, source)
        for (src, camera), n in quota.items():
            if src != source:
                continue
            pool = [p for p in by_camera.get(camera, []) if f"{source}_{p.name}" not in used]
            for image in rng.sample(pool, min(n, len(pool))):
                name = f"{source}_{image.name}"
                shutil.copyfile(image.resolve(), out / "images" / name)
                label = root / "labels" / "val" / f"{image.stem}.txt"
                shutil.copyfile(label.resolve(), out / "labels" / f"{Path(name).stem}.txt")
                rows.append({"image": name, "set": source, "camera": camera})
    with open(out / "manifest.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["image", "set", "camera"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"{len(rows)} images -> {out}")


if __name__ == "__main__":
    main()
