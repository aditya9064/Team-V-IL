"""
Train the first cube/cap/vial detector on the synthetic dataset.

    python synth/train_detector.py --data datasets/cubes_v1/data.yaml --name cubes_v1

Runs on any CUDA machine (Brev, Rosie, the CSI workstation); install
requirements-train.txt there. Results land in runs_detect/<name>/ with
results.csv, plots and weights/best.pt.

Augmentation choices follow from the task, not the library defaults:
  - hsv_h small: colour IS the class. Rendered cube hues measured on the T4
    (OpenCV 0-179) are red 2, yellow 25, blue 110; +/-1.8 covers camera
    white balance without moving one class into another.
  - scale kept small: cubes are only ~25-40 px in the 4K crop; heavy
    down-scaling would erase them.
  - imgsz matches the crop's long side so the network sees native pixels.
"""

import argparse
import json
from pathlib import Path

AUGMENT = dict(
    hsv_h=0.01,
    hsv_s=0.3,
    hsv_v=0.35,
    scale=0.15,
    translate=0.05,
    degrees=0.0,
    fliplr=0.5,
    flipud=0.5,
    mosaic=1.0,
)


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="data.yaml from split_dataset.py")
    ap.add_argument("--model", default="yolo11m.pt", help="pretrained checkpoint")
    ap.add_argument("--name", default="cubes")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=1664)  # 4K crop is 1664x896
    ap.add_argument("--batch", type=int, default=-1, help="-1 = auto-fit GPU memory")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--device", default="0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--project", default="runs_detect", help="parent dir for run folders")
    ap.add_argument(
        "--cache", default="ram", choices=["ram", "disk", "none"],
        help="ram: decode each PNG once (~45 GB for 10k 1664x896 frames)",
    )
    return ap.parse_args()


def main():
    args = parse_args()
    from ultralytics import YOLO  # heavy import; keep --help fast

    project = Path(args.project).resolve()
    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
        seed=args.seed,
        deterministic=True,
        patience=30,
        cache=False if args.cache == "none" else args.cache,
        project=str(project),
        name=args.name,
        exist_ok=False,
        **AUGMENT,
    )
    metrics = model.val(data=args.data, imgsz=args.imgsz, split="val")
    summary = {
        "args": vars(args),
        "augment": AUGMENT,
        "mAP50": float(metrics.box.map50),
        "mAP50_95": float(metrics.box.map),
        "per_class_mAP50_95": dict(
            zip(metrics.names.values(), map(float, metrics.box.maps))
        ),
        "weights": str(Path(model.trainer.save_dir) / "weights" / "best.pt"),
    }
    out = Path(model.trainer.save_dir) / "summary.json"
    out.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
