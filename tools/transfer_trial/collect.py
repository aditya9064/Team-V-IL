"""Pull the single generated frame out of each Cosmos Transfer output video as a PNG.

    python collect.py <raw output dir> <png dir>
"""
import subprocess
import sys
from pathlib import Path


def main():
    raw, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    videos = [v for v in sorted(raw.rglob("*.mp4")) if "control" not in v.name and "edge" not in v.name]
    for video in videos:
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(video), "-frames:v", "1",
                        str(out / f"{video.stem}.png")], check=True)
    images = list(raw.rglob("*.png")) + list(raw.rglob("*.jpg"))
    print(f"{len(videos)} videos -> {out}; {len(images)} images already in {raw}")


if __name__ == "__main__":
    main()
