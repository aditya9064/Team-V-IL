"""
Draw YOLO labels over generated images to check box alignment by eye.

    .venv/bin/python synth/preview_labels.py synth_out/a --limit 10
"""

import argparse
import json
from pathlib import Path

import cv2

COLOURS = [(40, 40, 230), (0, 210, 240), (230, 120, 30), (60, 200, 60), (200, 60, 200)]


def draw(image_path, label_path, classes):
    img = cv2.imread(str(image_path))
    if img is None:
        raise FileNotFoundError(image_path)
    h, w = img.shape[:2]
    for line in label_path.read_text().splitlines():
        cls, cx, cy, bw, bh = line.split()
        cls = int(cls)
        cx, cy, bw, bh = float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
        p0 = (round(cx - bw / 2), round(cy - bh / 2))
        p1 = (round(cx + bw / 2), round(cy + bh / 2))
        cv2.rectangle(img, p0, p1, COLOURS[cls % len(COLOURS)], 1)
        cv2.putText(img, classes[cls], (p0[0], p0[1] - 3), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, COLOURS[cls % len(COLOURS)], 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("chunk", help="generate_synthetic.py output dir")
    ap.add_argument("--limit", type=int, default=10)
    args = ap.parse_args()
    chunk = Path(args.chunk)
    classes = json.loads((chunk / "dataset.json").read_text())["classes"]
    out = chunk / "preview"
    out.mkdir(exist_ok=True)
    for image in sorted((chunk / "images").glob("*.png"))[: args.limit]:
        cv2.imwrite(str(out / image.name), draw(image, chunk / "labels" / f"{image.stem}.txt", classes))
    print(f"[preview] wrote {out}")


if __name__ == "__main__":
    main()
