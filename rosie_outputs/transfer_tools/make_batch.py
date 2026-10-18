"""Select, white-balance and shard v2 images for a Cosmos Transfer batch (run on Rosie).

    python make_batch.py <datasets dir> <out dir> [--shards 2]
    python make_batch.py <datasets dir> <out dir> --seed 1017 --train table=700,wrist=300 \
        --heldout none --exclude <earlier batch dir>      (a second, disjoint batch)

Training images come from cubes_v2's train split, held-out images from its val split,
so Transfer outputs can be judged on frames no model trained on. Each image is
white-balanced (white_balance.balance) and saved as high-quality JPEG with its label,
named <stem>__<camera>.jpg so the prompt matches the camera.
"""
import argparse
import json
import os
import random
import shutil
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image

from white_balance import balance

SEED = 1016
TRAIN_QUOTA = {"table": 1200, "side": 400, "wrist": 400}
HELDOUT_QUOTA = {"table": 120, "side": 40, "wrist": 40}


def by_camera(split_dir):
    groups = {}
    for png in sorted(split_dir.glob("*.png")):
        chunk = png.resolve().parent.parent
        camera = json.loads((chunk / "dataset.json").read_text())["camera"]
        groups.setdefault(camera, []).append(png)
    return groups


def write(png, labels_dir, dest, camera):
    name = f"{png.stem}__{camera}"
    target = dest / "images" / f"{name}.jpg"
    if target.exists() and (dest / "labels" / f"{name}.txt").exists():
        try:
            Image.open(target).load()  # a cancelled run can leave a truncated file
            return  # resumable: keep complete images written by an earlier run
        except OSError:
            pass
    rgb, _ = balance(np.asarray(Image.open(png).convert("RGB")))
    Image.fromarray(rgb).save(dest / "images" / f"{name}.jpg", quality=95, subsampling=0)
    shutil.copyfile(labels_dir / f"{png.stem}.txt", dest / "labels" / f"{name}.txt")


def pick(groups, quota, rng, used=frozenset()):
    out = []
    for camera, n in quota.items():
        pool = [p for p in groups.get(camera, []) if f"{p.stem}__{camera}" not in used]
        out += [(png, camera) for png in rng.sample(pool, min(n, len(pool)))]
    return out


def parse_quota(text, default):
    if text is None:
        return default
    if text == "none":
        return {}
    return {k: int(v) for k, v in (part.split("=") for part in text.split(","))}


def already_used(folder):
    """Frame names (<stem>__<camera>) present anywhere in an earlier batch."""
    return frozenset(p.stem for p in Path(folder).rglob("images/*.jpg")) if folder else frozenset()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("datasets")
    ap.add_argument("out")
    ap.add_argument("--shards", type=int, default=2)
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--train", help="camera=count,... (default: 1200 table, 400 side, 400 wrist)")
    ap.add_argument("--heldout", help="camera=count,... or none (default: 120/40/40)")
    ap.add_argument("--exclude", help="earlier batch dir whose frames must not be reused")
    args = ap.parse_args()
    root, out, rng = Path(args.datasets) / "cubes_v2", Path(args.out), random.Random(args.seed)
    used = already_used(args.exclude)
    quotas = (("train", parse_quota(args.train, TRAIN_QUOTA), "train"),
              ("val", parse_quota(args.heldout, HELDOUT_QUOTA), "heldout"))
    jobs = []
    for split, quota, tag in quotas:
        if not quota:
            continue
        for i, (png, camera) in enumerate(pick(by_camera(root / "images" / split), quota, rng, used)):
            jobs.append((png, root / "labels" / split, f"shard{i % args.shards}", tag, camera))
    for _, _, shard, tag, _ in jobs:
        (out / shard / tag / "images").mkdir(parents=True, exist_ok=True)
        (out / shard / tag / "labels").mkdir(parents=True, exist_ok=True)
    workers = int(os.environ.get("SLURM_CPUS_PER_TASK", os.cpu_count() or 4))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for future in [pool.submit(write, png, labels, out / shard / tag, camera)
                       for png, labels, shard, tag, camera in jobs]:
            future.result()  # re-raise any worker error
    for shard in sorted(out.glob("shard*")):
        counts = {t.name: len(list((t / "images").glob("*.jpg"))) for t in sorted(shard.iterdir())}
        print(shard.name, counts)


if __name__ == "__main__":
    main()
