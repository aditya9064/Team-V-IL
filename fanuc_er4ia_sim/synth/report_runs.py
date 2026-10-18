"""
Collect every training run's summary.json into one comparison table.

    python synth/report_runs.py runs/detect
"""

import json
import sys
from pathlib import Path


def rows(root):
    for summary in sorted(Path(root).glob("*/summary.json")):
        s = json.loads(summary.read_text())
        dataset = Path(s["args"]["data"]).parent
        train = len(list((dataset / "images" / "train").glob("*.png")))
        yield summary.parent.name, train, s["mAP50"], s["mAP50_95"], s["per_class_mAP50_95"], s["weights"]


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else "runs_detect"
    found = list(rows(root))
    if not found:
        sys.exit(f"No finished training runs under {root}")
    print("Synthetic validation results (twin images only; not real-camera accuracy)\n")
    print(f"{'run':<28}{'train imgs':>11}{'mAP50':>8}{'mAP50-95':>10}")
    for name, train, m50, m, _, _ in found:
        print(f"{name:<28}{train:>11}{m50:>8.3f}{m:>10.3f}")
    print("\nPer-class mAP50-95 and weights:")
    for name, _, _, _, per_class, weights in found:
        classes = "  ".join(f"{k}={v:.3f}" for k, v in per_class.items())
        print(f"- {name}: {classes}\n  {weights}")


if __name__ == "__main__":
    main()
