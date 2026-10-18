"""Write Cosmos Transfer specs for a parameter sweep (one folder per config).

    python sweep_specs.py <png dir> <out dir> <prompts.json> [--only name,name]

Each PNG becomes a 1-frame MP4 once; every config gets its own spec folder, so a
config is one batch (model loaded once). Filenames carry the camera: <stem>__<camera>.png
"""
import argparse
import json
from pathlib import Path

from make_specs import to_mp4

CONFIGS = {
    # name: (controls, sigma_max) - sigma_max None = start from pure noise (trial #1)
    "edge_s80": ({"edge": {"control_weight": 1.0}}, "80"),
    "edgevis_s80": ({"edge": {"control_weight": 1.0},
                     "vis": {"control_weight": 1.0, "preset_blur_strength": "very_low"}}, "80"),
    "edgevis_s120": ({"edge": {"control_weight": 1.0},
                      "vis": {"control_weight": 1.0, "preset_blur_strength": "very_low"}}, "120"),
    "edgevis_s50": ({"edge": {"control_weight": 1.0},
                     "vis": {"control_weight": 1.0, "preset_blur_strength": "very_low"}}, "50"),
    "edgevis_noise": ({"edge": {"control_weight": 1.0},
                       "vis": {"control_weight": 1.0, "preset_blur_strength": "very_low"}}, None),
    # Round 2: vis carries only coarse colour (stronger blur, lower weight) so lighting,
    # texture and tint can change while cube colours stay anchored.
    "vis05_med": ({"edge": {"control_weight": 1.0},
                   "vis": {"control_weight": 0.5, "preset_blur_strength": "medium"}}, None),
    "vis03_med": ({"edge": {"control_weight": 1.0},
                   "vis": {"control_weight": 0.3, "preset_blur_strength": "medium"}}, None),
    "vis05_high": ({"edge": {"control_weight": 1.0},
                    "vis": {"control_weight": 0.5, "preset_blur_strength": "high"}}, None),
}


def spec(png, mp4, prompt, controls, sigma_max):
    out = {"name": png.stem, "prompt": prompt, "video_path": str(mp4),
           "num_video_frames_per_chunk": 1, "max_frames": 1, "seed": 1, **controls}
    if sigma_max is not None:
        out["sigma_max"] = sigma_max
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pngs")
    ap.add_argument("out")
    ap.add_argument("prompts")
    ap.add_argument("--only", help="comma-separated config names (default: all)")
    args = ap.parse_args()
    pngs, out, prompts = Path(args.pngs), Path(args.out), json.loads(Path(args.prompts).read_text())
    names = args.only.split(",") if args.only else list(CONFIGS)
    videos = out / "videos"
    videos.mkdir(parents=True, exist_ok=True)
    for png in sorted([*pngs.glob("*.png"), *pngs.glob("*.jpg")]):
        mp4 = videos / f"{png.stem}.mp4"
        if not mp4.exists():
            to_mp4(png, mp4)
        camera = png.stem.rsplit("__", 1)[-1]
        for name in names:
            controls, sigma = CONFIGS[name]
            folder = out / name
            folder.mkdir(exist_ok=True)
            (folder / f"{png.stem}.json").write_text(
                json.dumps(spec(png, mp4, prompts[camera], controls, sigma), indent=2))
    print(f"configs {names} x {len(list(videos.glob('*.mp4')))} images -> {out}")


if __name__ == "__main__":
    main()
