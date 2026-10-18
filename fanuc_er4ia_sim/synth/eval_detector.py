"""
Cross-evaluate detectors on several validation sets (e.g. v1 easy vs v2 hard).

    python synth/eval_detector.py --model v1=runs/.../best.pt --model v2=... \
        --data easy=datasets/cubes_all/data.yaml --data hard=datasets/cubes_v2/data.yaml

Prints and saves a model x dataset table of mAP50-95. Validation images are
synthetic; real-camera accuracy needs real test photos.
"""

import argparse
import json
from pathlib import Path


def pairs(values):
    out = {}
    for value in values:
        name, _, path = value.partition("=")
        if not path:
            raise SystemExit(f"expected name=path, got {value!r}")
        out[name] = path
    return out


def evaluate(model_path, data, imgsz, project):
    from ultralytics import YOLO

    metrics = YOLO(model_path).val(data=data, split="val", imgsz=imgsz, batch=16,
                                   plots=False, project=project, name="eval", exist_ok=True)
    return {"mAP50": float(metrics.box.map50), "mAP50_95": float(metrics.box.map),
            "per_class_mAP50_95": dict(zip(metrics.names.values(), map(float, metrics.box.maps)))}


def table(results, datasets):
    lines = ["Cross-evaluation, mAP50-95 (synthetic validation sets)", ""]
    lines.append(f"{'model':<14}" + "".join(f"{d:>14}" for d in datasets))
    for model, row in results.items():
        lines.append(f"{model:<14}" + "".join(f"{row[d]['mAP50_95']:>14.3f}" for d in datasets))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", required=True, help="name=path/to/best.pt")
    ap.add_argument("--data", action="append", required=True, help="name=path/to/data.yaml")
    ap.add_argument("--imgsz", type=int, default=1664)
    ap.add_argument("--out", required=True, help="output stem (.json and .txt written)")
    args = ap.parse_args()
    models, datasets = pairs(args.model), pairs(args.data)
    project = str(Path(args.out).parent / "eval_runs")
    results = {m: {d: evaluate(p, y, args.imgsz, project) for d, y in datasets.items()}
               for m, p in models.items()}
    Path(f"{args.out}.json").write_text(json.dumps(results, indent=2))
    text = table(results, list(datasets))
    Path(f"{args.out}.txt").write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
