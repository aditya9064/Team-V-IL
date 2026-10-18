"""
Procedural table-mat textures for v2 (speckle, grid, stripes, printed
markings, scratches, mottling). Light-toned so cubes stay distinguishable,
but busy enough that the detector stops treating "plain white" as a cue.
"""

from pathlib import Path

import cv2
import numpy as np

KINDS = ("speckle", "grid", "stripes", "markings", "scratches", "mottled")


def _base(rng, size):
    # Near-neutral like real mats: a grey level plus at most +/-8 per channel,
    # so the table never takes on a cube class colour.
    tone = rng.uniform(175, 235) + rng.uniform(-8, 8, 3)
    return np.ones((size, size, 3), np.float32) * tone


def _draw(kind, img, rng):
    size = img.shape[0]
    dark = tuple(float(v) for v in rng.uniform(60, 140) + rng.uniform(-6, 6, 3))  # neutral ink
    if kind == "speckle":
        img += rng.normal(0, 18, img.shape[:2])[..., None]
    elif kind == "grid":
        step = int(rng.integers(40, 120))
        for k in range(0, size, step):
            cv2.line(img, (k, 0), (k, size), dark, 2)
            cv2.line(img, (0, k), (size, k), dark, 2)
    elif kind == "stripes":
        phase = np.sin(np.arange(size) / rng.uniform(6, 30) + rng.normal(0, 0.3, size).cumsum() * 0.05)
        img += (phase[None, :, None] * 14).astype(np.float32)
    elif kind == "markings":
        for _ in range(int(rng.integers(6, 14))):
            p0 = tuple(int(v) for v in rng.integers(0, size, 2))
            p1 = tuple(int(v) for v in rng.integers(0, size, 2))
            cv2.rectangle(img, p0, p1, dark, int(rng.integers(2, 6)))
        text = np.clip(img, 0, 255).astype(np.uint8)  # putText needs 8-bit in OpenCV 5
        cv2.putText(text, "ZONE A-3", (int(size * 0.1), int(size * 0.9)),
                    cv2.FONT_HERSHEY_SIMPLEX, size / 512, dark, max(1, size // 256))
        img = text.astype(np.float32)
    elif kind == "scratches":
        for _ in range(int(rng.integers(40, 90))):
            p0 = rng.integers(0, size, 2)
            p1 = p0 + rng.integers(-120, 120, 2)
            cv2.line(img, tuple(int(v) for v in p0), tuple(int(v) for v in p1), dark, 1)
    else:  # mottled
        blobs = cv2.GaussianBlur(rng.normal(0, 1, (size // 16, size // 16)).astype(np.float32), (0, 0), 2)
        img += cv2.resize(blobs, (size, size))[..., None] * 25
    return img


def make_textures(out_dir, seed=0, size=1024):
    """Write one PNG per kind; returns their paths in KINDS order."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    paths = []
    for kind in KINDS:
        img = np.clip(_draw(kind, _base(rng, size), rng), 0, 255).astype(np.uint8)
        path = out / f"mat_{kind}.png"
        cv2.imwrite(str(path), img)
        paths.append(str(path))
    return paths
