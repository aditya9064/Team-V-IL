"""Side-by-side sheets: original render | Cosmos Transfer output, both with the original
YOLO labels drawn, so realism and label alignment can be judged by eye.

    python compare_sheet.py <in dir with images/ labels/> <transfer jpg dir> <out dir>
"""
import sys
from pathlib import Path

from PIL import Image, ImageDraw

COLOURS = {0: (255, 40, 40), 1: (255, 220, 0), 2: (40, 120, 255), 3: (255, 140, 0), 4: (0, 230, 230)}
WIDTH = 960  # per panel


def draw_labels(image, label_file):
    draw = ImageDraw.Draw(image)
    w, h = image.size
    for line in label_file.read_text().splitlines():
        cls, cx, cy, bw, bh = line.split()
        cx, cy, bw, bh = float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
        draw.rectangle([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2],
                       outline=COLOURS.get(int(cls), (255, 255, 255)), width=max(2, w // 400))
    return image


def panel(path, label_file):
    image = draw_labels(Image.open(path).convert("RGB"), label_file)
    return image.resize((WIDTH, round(image.height * WIDTH / image.width)))


def main():
    src, gen, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    for original in sorted((src / "images").glob("*.png")):
        generated = gen / f"{original.stem}.jpg"
        if not generated.exists():
            continue
        label = src / "labels" / f"{original.stem}.txt"
        left, right = panel(original, label), panel(generated, label)
        sheet = Image.new("RGB", (WIDTH * 2 + 10, left.height), "white")
        sheet.paste(left, (0, 0))
        sheet.paste(right, (WIDTH + 10, 0))
        sheet.save(out / f"{original.stem}.jpg", quality=88)
    print(f"{len(list(out.glob('*.jpg')))} sheets -> {out}")


if __name__ == "__main__":
    main()
