"""Turn each PNG into a 1-frame MP4 + a Cosmos Transfer image2image spec (edge control).

    python make_specs.py <png dir> <spec dir> <prompts.json>

prompts.json maps camera -> prompt; the camera comes from the filename
(<name>__<camera>.png). keep_input_resolution (the default) keeps 1664x896,
so the original YOLO labels stay valid for the output image.
"""
import json
import subprocess
import sys
from pathlib import Path


def to_mp4(png, mp4):
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(png), "-frames:v", "1",
                    "-c:v", "libx264", "-crf", "8", "-pix_fmt", "yuv420p", str(mp4)], check=True)


def main():
    pngs, specs, prompts = Path(sys.argv[1]), Path(sys.argv[2]), json.loads(Path(sys.argv[3]).read_text())
    specs.mkdir(parents=True, exist_ok=True)
    for png in sorted(pngs.glob("*.png")):
        camera = png.stem.rsplit("__", 1)[-1]
        mp4 = specs / f"{png.stem}.mp4"
        to_mp4(png, mp4)
        spec = {"name": png.stem, "prompt": prompts[camera], "video_path": str(mp4),
                "num_video_frames_per_chunk": 1, "max_frames": 1, "seed": 1,
                "edge": {"control_weight": 1.0}}
        (specs / f"{png.stem}.json").write_text(json.dumps(spec, indent=2))
    print(f"{len(list(specs.glob('*.json')))} specs in {specs}")


if __name__ == "__main__":
    main()
