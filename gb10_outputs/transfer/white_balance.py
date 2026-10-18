"""Grey-world white balance on the table background, before Cosmos Transfer.

    python white_balance.py <in dir with images/ labels/> <out dir>

Scales R, G, B so the median of low-saturation pixels (the table) becomes neutral grey.
Only global colour balance changes; geometry and labels are untouched (labels are copied).
"""
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def balance(rgb):
    pixels = rgb.reshape(-1, 3).astype(np.float32)
    spread = pixels.max(1) - pixels.min(1)
    background = pixels[spread < np.percentile(spread, 60)]  # least colourful 60%: the table
    gains = background.mean() / np.maximum(np.median(background, 0), 1.0)
    return np.clip(rgb.astype(np.float32) * gains, 0, 255).astype(np.uint8), gains


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    (out / "images").mkdir(parents=True, exist_ok=True)
    shutil.copytree(src / "labels", out / "labels", dirs_exist_ok=True)
    for png in sorted((src / "images").glob("*.png")):
        balanced, gains = balance(np.asarray(Image.open(png).convert("RGB")))
        Image.fromarray(balanced).save(out / "images" / png.name)
        print(f"{png.stem}: gains R{gains[0]:.2f} G{gains[1]:.2f} B{gains[2]:.2f}")


if __name__ == "__main__":
    main()
