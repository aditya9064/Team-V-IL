"""
Automatic label-alignment gate: does every cube box contain that cube's colour?

    python synth/check_labels.py synth_out/chunk_0000

A misaligned projection, a wrong ROI crop or a colour/class mix-up all show up
as boxes that miss their cube's pixels. The pipeline runs this on the test
chunk and stops before the full dataset if it fails. Exit code 0 = pass.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# OpenCV HSV: hue 0-179. Wide bands: lighting is randomised.
HUE_BANDS = {
    "cube_red": [(0, 10), (165, 179)],
    "cube_yellow": [(18, 40)],
    "cube_blue": [(95, 135)],
}
MIN_SATURATION, MIN_VALUE = 70, 40


def colour_fractions(hsv_crop):
    """Fraction of crop pixels in each cube colour band."""
    h, s, v = hsv_crop[..., 0], hsv_crop[..., 1], hsv_crop[..., 2]
    vivid = (s >= MIN_SATURATION) & (v >= MIN_VALUE)
    out = {}
    for name, bands in HUE_BANDS.items():
        in_band = np.zeros_like(vivid)
        for lo, hi in bands:
            in_band |= (h >= lo) & (h <= hi)
        out[name] = float((vivid & in_band).mean()) if vivid.size else 0.0
    return out


def check_box(hsv, line, classes, min_fraction):
    """(class name, passed, expected-colour fraction) for one cube label, else None."""
    cls, cx, cy, bw, bh = line.split()
    name = classes[int(cls)]
    if name not in HUE_BANDS:
        return None
    H, W = hsv.shape[:2]
    x0 = max(int((float(cx) - float(bw) / 2) * W), 0)
    x1 = min(int(np.ceil((float(cx) + float(bw) / 2) * W)), W)
    y0 = max(int((float(cy) - float(bh) / 2) * H), 0)
    y1 = min(int(np.ceil((float(cy) + float(bh) / 2) * H)), H)
    fractions = colour_fractions(hsv[y0:y1, x0:x1])
    expected = fractions[name]
    passed = expected >= min_fraction and expected == max(fractions.values())
    return name, passed, expected


CLEAR_VIEW = 0.8


def _visibility(chunk, stem, n):
    """Per-label visible fraction from v2 meta; v1 frames count as fully visible."""
    meta = chunk / "meta" / f"{stem}.json"
    if meta.is_file():
        vis = json.loads(meta.read_text()).get("visibility")
        if vis is not None and len(vis) == n:
            return vis
    return [1.0] * n


def check_chunk(chunk, min_fraction=0.2, min_pass_rate=0.9, min_cubes=20):
    chunk = Path(chunk)
    manifest = json.loads((chunk / "dataset.json").read_text())
    classes, roi = manifest["classes"], manifest["roi"]
    results, problems = [], []
    for image in sorted((chunk / "images").glob("*.png")):
        img = cv2.imread(str(image))
        if img is None or img.shape[1::-1] != (roi[2] - roi[0], roi[3] - roi[1]):
            problems.append(f"{image.name}: unreadable or not the ROI size")
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        label = chunk / "labels" / f"{image.stem}.txt"
        lines = label.read_text().splitlines() if label.is_file() else []
        for line, vis in zip(lines, _visibility(chunk, image.stem, len(lines))):
            if vis < CLEAR_VIEW:
                continue  # partly hidden (arm, neighbour): less colour is expected, not a bug
            r = check_box(hsv, line, classes, min_fraction)
            if r:
                results.append((image.name, *r))
    passed = sum(r[2] for r in results)
    rate = passed / len(results) if results else 0.0
    ok = not problems and len(results) >= min_cubes and rate >= min_pass_rate
    return ok, rate, results, problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("chunk")
    ap.add_argument("--min-fraction", type=float, default=0.2)
    ap.add_argument("--min-pass-rate", type=float, default=0.9)
    ap.add_argument("--min-cubes", type=int, default=20)
    args = ap.parse_args()
    ok, rate, results, problems = check_chunk(args.chunk, args.min_fraction, args.min_pass_rate, args.min_cubes)
    for p in problems:
        print(f"[check] {p}")
    for image, name, good, frac in sorted(results, key=lambda r: r[3])[:10]:
        print(f"[check] {'ok  ' if good else 'FAIL'} {image} {name}: {frac:.0%} expected colour")
    print(f"[check] {len(results)} cube boxes, {rate:.1%} aligned -> {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
