"""Quality gate for Cosmos Transfer outputs: an output may only be used for training if
every labelled cube keeps its colour and no new cube-coloured blobs appear elsewhere.

    python transfer_qc.py <in dir with images/ labels/> <transfer jpg dir> [--json out.json]

Colours are judged on the centre half of each box with HSV hue bins, so edges, shadows
and antialiasing do not decide the vote. Thresholds are deliberately strict.
"""
import argparse
import colorsys
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

CUBE_COLOURS = {0: "red", 1: "yellow", 2: "blue"}
MIN_BOX_PX = 8                # cubes narrower than this are too small to judge
MAX_NEW_COLOUR_FRACTION = 0.002  # new cube-coloured area outside boxes, share of image


def colour_name(r, g, b):
    h, s, v = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)
    if s < 0.35 or v < 0.2:
        return "grey"
    deg = h * 360
    if deg < 20 or deg > 330:
        return "red"
    if 35 <= deg < 75:
        return "yellow"
    if 185 <= deg < 260:
        return "blue"
    return "other"


def colour_map(image):
    """Per-pixel colour name as a small integer array (0 grey/other, 1 red, 2 yellow, 3 blue)."""
    hsv = np.asarray(image.convert("HSV"), dtype=np.float32) / 255.0
    h, s, v = hsv[..., 0] * 360, hsv[..., 1], hsv[..., 2]
    vivid = (s >= 0.35) & (v >= 0.2)
    out = np.zeros(h.shape, dtype=np.uint8)
    out[vivid & ((h < 20) | (h > 330))] = 1
    out[vivid & (h >= 35) & (h < 75)] = 2
    out[vivid & (h >= 185) & (h < 260)] = 3
    return out


def box_pixels(size, box, shrink):
    w, h = size
    cx, cy, bw, bh = box
    return (int((cx - bw * shrink / 2) * w), int((cy - bh * shrink / 2) * h),
            int((cx + bw * shrink / 2) * w), int((cy + bh * shrink / 2) * h))


def box_colour(image, box):
    x0, y0, x1, y1 = box_pixels(image.size, box, 0.5)
    if x1 <= x0 or y1 <= y0:
        return None
    votes = Counter(colour_name(*p) for p in image.crop((x0, y0, x1, y1)).convert("RGB").getdata())
    return votes.most_common(1)[0][0]


def read_labels(path):
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) == 5:
            rows.append((int(parts[0]), tuple(map(float, parts[1:]))))
    return rows


def new_colour_fraction(original, generated, labels):
    """Share of the image that became red/yellow/blue outside every label box (padded)."""
    before, after = colour_map(original), colour_map(generated)
    outside = np.ones(before.shape, dtype=bool)
    for _, box in labels:
        x0, y0, x1, y1 = box_pixels(original.size, box, 1.6)
        outside[max(y0, 0):max(y1, 0), max(x0, 0):max(x1, 0)] = False
    appeared = (after > 0) & (after != before) & outside
    return float(appeared.mean())


def realism_proxies(image, labels):
    """Grain and colour cast of the empty table (outside padded label boxes).

    grain: std of the high-pass (image minus 5x5 box blur) luminance, in grey levels.
    cast:  mean |a|+|b| chroma in CIE-Lab-like opponent space; 0 = neutral grey table.
    """
    rgb = np.asarray(image, dtype=np.float32)
    grey = rgb.mean(2)
    padded = np.pad(grey, 2, mode="edge")
    blur = sum(padded[dy:dy + grey.shape[0], dx:dx + grey.shape[1]]
               for dy in range(5) for dx in range(5)) / 25.0
    spread = rgb.max(2) - rgb.min(2)
    mask = spread < 40  # low-saturation pixels: the table, not the parts
    for _, box in labels:
        x0, y0, x1, y1 = box_pixels(image.size, box, 1.6)
        mask[max(y0, 0):max(y1, 0), max(x0, 0):max(x1, 0)] = False
    if mask.sum() < 100:
        return {"grain": None, "cast": None}
    r, g, b = rgb[..., 0][mask], rgb[..., 1][mask], rgb[..., 2][mask]
    cast = np.abs(r - g).mean() / 2 + np.abs((r + g) / 2 - b).mean() / 2
    return {"grain": float((grey - blur)[mask].std()), "cast": float(cast)}


def check(original_path, generated_path, label_path):
    original = Image.open(original_path).convert("RGB")
    generated = Image.open(generated_path).convert("RGB").resize(original.size)
    labels = read_labels(label_path)
    kept = changed = 0
    for cls, box in labels:
        if cls not in CUBE_COLOURS or box[2] * original.size[0] < MIN_BOX_PX:
            continue
        if box_colour(original, box) != CUBE_COLOURS[cls]:
            continue  # colour not readable in the render itself; cannot judge
        if box_colour(generated, box) == CUBE_COLOURS[cls]:
            kept += 1
        else:
            changed += 1
    new_frac = new_colour_fraction(original, generated, labels)
    return {"kept": kept, "changed": changed, "new_colour_fraction": new_frac,
            "pass": changed == 0 and new_frac <= MAX_NEW_COLOUR_FRACTION,
            "before": realism_proxies(original, labels), "after": realism_proxies(generated, labels)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("generated")
    ap.add_argument("--json")
    args = ap.parse_args()
    src, gen = Path(args.src), Path(args.generated)
    results = {}
    for original in sorted([*(src / "images").glob("*.png"), *(src / "images").glob("*.jpg")]):
        out = gen / f"{original.stem}.jpg"
        if out.exists():
            results[original.stem] = check(original, out, src / "labels" / f"{original.stem}.txt")
    kept = sum(r["kept"] for r in results.values())
    changed = sum(r["changed"] for r in results.values())
    passed = sum(r["pass"] for r in results.values())

    def mean(stage, key):
        values = [r[stage][key] for r in results.values() if r[stage][key] is not None]
        return sum(values) / max(len(values), 1)

    print(f"{len(results)} images | cube colour kept {kept}/{kept + changed} "
          f"({kept / max(kept + changed, 1):.0%}) | pass gate {passed}/{len(results)} | "
          f"grain {mean('before', 'grain'):.1f}->{mean('after', 'grain'):.1f} | "
          f"cast {mean('before', 'cast'):.1f}->{mean('after', 'cast'):.1f}")
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
