"""
Assemble generated chunks into a YOLO dataset with a stable train/val split.

    .venv/bin/python synth/split_dataset.py synth_out/a synth_out/b --out datasets/cubes_v1

A frame's split is decided by a hash of its name, not by shuffling, so adding
more chunks later never moves an existing frame between train and val.
Every frame is a distinct randomised scene, so no scene appears in both splits.
Real photos of the line stay out of this dataset: event rules allow them for
testing only.
"""

import argparse
import hashlib
import json
import shutil
from pathlib import Path


def split_of(stem, val_fraction):
    digest = hashlib.sha1(stem.encode()).digest()
    return "val" if int.from_bytes(digest[:4], "big") / 2**32 < val_fraction else "train"


def collect(chunks):
    """(stem, image, label) triples; rejects duplicate frames across chunks."""
    frames, classes = {}, None
    for chunk in map(Path, chunks):
        manifest = json.loads((chunk / "dataset.json").read_text())
        if classes not in (None, manifest["classes"]):
            raise ValueError(f"{chunk}: class list differs from earlier chunks")
        classes = manifest["classes"]
        for image in sorted((chunk / "images").glob("*.png")):
            label = chunk / "labels" / f"{image.stem}.txt"
            if not label.is_file():
                # A job killed between the image and label writes; skip, don't fail.
                print(f"[split] skipping {image} (no label: interrupted frame)")
                continue
            if image.stem in frames:
                raise ValueError(f"Frame {image.stem} appears in two chunks")
            frames[image.stem] = (image, label)
    if not frames:
        raise ValueError("No frames found; check the chunk directories")
    return classes, frames


def build(chunks, out, val_fraction, link, max_frames=None):
    """max_frames keeps the lowest frame numbers, so smaller sets nest in larger ones."""
    classes, frames = collect(chunks)
    if max_frames is not None:
        frames = dict(sorted(frames.items())[:max_frames])
    out = Path(out).resolve()
    counts = {"train": 0, "val": 0}
    for stem, (image, label) in sorted(frames.items()):
        split = split_of(stem, val_fraction)
        for kind, src in (("images", image), ("labels", label)):
            dst = out / kind / split / src.name
            dst.parent.mkdir(parents=True, exist_ok=True)
            if dst.exists() or dst.is_symlink():
                dst.unlink()
            if link:
                dst.symlink_to(src.resolve())
            else:
                shutil.copy2(src, dst)
        counts[split] += 1
    yaml = [f"path: {out}", "train: images/train", "val: images/val", "names:"]
    yaml += [f"  {i}: {name}" for i, name in enumerate(classes)]
    (out / "data.yaml").write_text("\n".join(yaml) + "\n")
    return counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("chunks", nargs="+", help="generate_synthetic.py output dirs")
    ap.add_argument("--out", required=True)
    ap.add_argument("--val-fraction", type=float, default=0.15)
    ap.add_argument("--copy", action="store_true", help="copy instead of symlink")
    ap.add_argument("--max-frames", type=int, default=None, help="data-scaling subsets")
    args = ap.parse_args()
    counts = build(
        args.chunks, args.out, args.val_fraction, link=not args.copy, max_frames=args.max_frames
    )
    print(f"[split] train {counts['train']}  val {counts['val']}  -> {args.out}/data.yaml")


if __name__ == "__main__":
    main()
