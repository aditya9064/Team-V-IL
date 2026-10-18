"""Cosmos-vs-YOLO baseline on a fixed labelled image set (run on the GB10).

    python run_baseline.py <starter dir> <set dir> --detector cosmos
    python run_baseline.py <starter dir> <set dir> --detector yolo --weights models/cubes_all.pt
    python run_baseline.py <starter dir> <set dir> --summary

Each image's result is appended to <set dir>/results.jsonl as soon as it is scored,
so an interrupted run resumes where it stopped. --summary writes BASELINE.txt.
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import baseline_metrics as bm  # noqa: E402


def disable_triton_overrides():
    # torch 2.14 routes bmm & co. to Triton kernels that need Python.h at runtime;
    # the GB10 has no python3-dev (and no sudo), so use the stock CUDA kernels.
    import torch._native.registry as native_ops
    native_ops.deregister_op_overrides(
        disable_op_symbols=["bmm", "_foreach_mm", "linalg_polar", "linalg_polar.out"])


def cosmos_detector(starter):
    sys.path.insert(0, starter)
    import perceive

    model, processor = perceive.load_model()

    def detect(image):
        reply = perceive.ask(model, processor, str(image), perceive.PROMPT)
        parsed = perceive.parse_json(reply)
        return bm.cosmos_boxes(parsed), parsed is not None, reply
    return detect


def yolo_detector(weights):
    from ultralytics import YOLO

    model = YOLO(weights)

    def detect(image):
        result = model.predict(str(image), imgsz=1664, conf=0.25, verbose=False)[0]
        order = result.boxes.conf.argsort(descending=True)  # match best boxes first
        boxes = [(int(result.boxes.cls[i]), tuple(result.boxes.xyxyn[i].tolist())) for i in order]
        return boxes, True, None
    return detect


def score(detector_name, detect, row, set_dir):
    truths = bm.yolo_label_boxes((set_dir / "labels" / f"{Path(row['image']).stem}.txt").read_text())
    start = time.time()
    preds, parsed, reply = detect(set_dir / "images" / row["image"])
    seconds = time.time() - start
    tp, fp, fn = bm.match(preds, truths)
    loc_tp, _, _ = bm.match(preds, truths, use_class=False)
    return {"detector": detector_name, **row, "tp": tp, "fp": fp, "fn": fn, "loc_tp": loc_tp,
            "seconds": seconds, "parsed": parsed, "n_true": len(truths), "n_pred": len(preds),
            "reply": reply}


def run(args):
    set_dir = Path(args.set_dir)
    results = set_dir / "results.jsonl"
    name = args.name or args.detector
    done = set()
    if results.exists():
        done = {r["image"] for r in map(json.loads, results.read_text().splitlines())
                if r["detector"] == name}
    rows = [r for r in csv.DictReader(open(set_dir / "manifest.csv")) if r["image"] not in done]
    print(f"[baseline] {name}: {len(rows)} images to score ({len(done)} already scored)", flush=True)
    if not rows:
        return
    disable_triton_overrides()
    detect = cosmos_detector(args.starter) if args.detector == "cosmos" else yolo_detector(args.weights)
    for i, row in enumerate(rows, 1):
        out = score(name, detect, row, set_dir)
        with open(results, "a") as f:
            f.write(json.dumps(out) + "\n")
        print(f"[baseline] {name} {i}/{len(rows)} {row['image']} tp={out['tp']} fp={out['fp']} "
              f"fn={out['fn']} {out['seconds']:.1f}s", flush=True)


def summary(args):
    set_dir = Path(args.set_dir)
    rows = [json.loads(line) for line in (set_dir / "results.jsonl").read_text().splitlines()]
    text = ("Cosmos3-Nano (starter prompt) vs YOLO, synthetic held-out val images, IoU>=0.5\n"
            "found = share of true objects boxed at all (class ignored); bad = unparseable replies\n\n"
            + bm.table(bm.summarise(rows)))
    (set_dir / "BASELINE.txt").write_text(text + "\n")
    print(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("starter")
    ap.add_argument("set_dir")
    ap.add_argument("--detector", choices=["cosmos", "yolo"])
    ap.add_argument("--weights")
    ap.add_argument("--name", help="result label, e.g. yolo_v1 (default: detector)")
    ap.add_argument("--summary", action="store_true")
    args = ap.parse_args()
    summary(args) if args.summary else run(args)


if __name__ == "__main__":
    main()
