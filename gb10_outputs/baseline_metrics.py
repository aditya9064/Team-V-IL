"""Scoring for the Cosmos-vs-YOLO baseline: one detection format, IoU matching, P/R/F1.

A detection is (class_id, (x0, y0, x1, y1)) in normalised image coordinates.
Class ids follow the dataset: 0 cube_red, 1 cube_yellow, 2 cube_blue, 3 cap, 4 vial.
"""
from collections import defaultdict

CLASSES = ["cube_red", "cube_yellow", "cube_blue", "cap", "vial"]
CUBE_COLOURS = {"red": 0, "yellow": 1, "blue": 2}
IOU_THRESHOLD = 0.5
WRONG_CLASS = -1  # a cube Cosmos gave a colour that is not in the dataset


def yolo_label_boxes(text):
    """YOLO label lines 'cls cx cy w h' -> detections."""
    boxes = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cls, cx, cy, w, h = int(parts[0]), *map(float, parts[1:])
        boxes.append((cls, (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)))
    return boxes


def cosmos_class(obj):
    category = str(obj.get("category", "")).lower()
    if category == "cap":
        return 3
    if category == "vial":
        return 4
    if category == "cube":
        return CUBE_COLOURS.get(str(obj.get("colour", "")).lower(), WRONG_CLASS)
    return None


def cosmos_boxes(parsed):
    """Starter-prompt JSON reply -> detections (objects with unusable boxes dropped)."""
    boxes = []
    for obj in (parsed or {}).get("objects", []):
        cls, bbox = cosmos_class(obj), obj.get("bbox")
        if cls is None or not isinstance(bbox, list) or len(bbox) != 4:
            continue
        try:
            x0, y0, x1, y1 = (min(max(float(v), 0.0), 1.0) for v in bbox)
        except (TypeError, ValueError):
            continue
        if x1 > x0 and y1 > y0:
            boxes.append((cls, (x0, y0, x1, y1)))
    return boxes


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def match(preds, truths, use_class=True):
    """Greedy one-to-one matching in prediction order -> (tp, fp, fn)."""
    unmatched = list(truths)
    tp = 0
    for cls, box in preds:
        best, best_iou = None, IOU_THRESHOLD
        for truth in unmatched:
            if use_class and truth[0] != cls:
                continue
            overlap = iou(box, truth[1])
            if overlap >= best_iou:
                best, best_iou = truth, overlap
        if best is not None:
            unmatched.remove(best)
            tp += 1
    return tp, len(preds) - tp, len(unmatched)


def prf(tp, fp, fn):
    p = tp / (tp + fp) if tp + fp else 0.0
    r = tp / (tp + fn) if tp + fn else 0.0
    return p, r, (2 * p * r / (p + r) if p + r else 0.0)


def summarise(rows):
    """rows: dicts with detector, set, camera, tp, fp, fn, loc_tp, seconds, parsed."""
    groups = defaultdict(list)
    for row in rows:
        groups[(row["detector"], f"{row['set']}/{row['camera']}")].append(row)
        groups[(row["detector"], "ALL")].append(row)
    out = {}
    for key, items in sorted(groups.items()):
        tp, fp, fn = (sum(r[k] for r in items) for k in ("tp", "fp", "fn"))
        loc_tp = sum(r["loc_tp"] for r in items)
        p, r, f1 = prf(tp, fp, fn)
        out[key] = {"images": len(items), "precision": p, "recall": r, "f1": f1,
                    "found_any_class": loc_tp / (tp + fn) if tp + fn else 0.0,
                    "sec_per_image": sum(x["seconds"] for x in items) / len(items),
                    "unparsed": sum(1 for x in items if not x["parsed"])}
    return out


def table(summary):
    head = (f"{'detector':<10}{'group':<16}{'n':>4}{'prec':>7}{'recall':>8}{'F1':>7}"
            f"{'found':>8}{'s/img':>8}{'bad':>5}")
    lines = [head, "-" * len(head)]
    for (detector, group), m in summary.items():
        lines.append(f"{detector:<10}{group:<16}{m['images']:>4}{m['precision']:>7.3f}"
                     f"{m['recall']:>8.3f}{m['f1']:>7.3f}{m['found_any_class']:>8.3f}"
                     f"{m['sec_per_image']:>8.2f}{m['unparsed']:>5}")
    return "\n".join(lines)
