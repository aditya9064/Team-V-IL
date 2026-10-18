"""
Stage 2, YOLO variant: a trained detector in place of Cosmos3-Nano.

    python perceive_yolo.py --run runs/example --weights models/cubes_v2.pt

Writes the same detections.json as `perceive.py --stack-cubes`, so plan_sort.py
and sort_scene.py run unchanged. The detector sees the same fixed workspace crop
it was trained on (table camera) or the full frame (side/wrist cameras); boxes
are mapped back to full-frame pixels. Cube yaw comes from the starter's
image-only silhouette fit (cube_pose.refine_pose) instead of a VLM query.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

import scene_config as C
from backproject import CameraModel
from cube_pose import refine_pose, square_yaw
from synth.labels import workspace_roi
from synth.layout import Randomization

# Dataset class id -> (category, colour), as used by perceive.py / plan_sort.py
CLASS_TO_OBJECT = {0: ("cube", "red"), 1: ("cube", "yellow"), 2: ("cube", "blue"),
                   3: ("cap", "yellow"), 4: ("vial", "clear")}
DEFAULT_CONF = 0.25
POSE_MIN_FIT = 0.85  # silhouette IoU below this leaves the cube pose unconfirmed
IMGSZ = 1664


def detector_roi(cam, camera):
    """Crop the detector was trained on: table workspace crop, else the full frame."""
    if camera != "table":
        return 0, 0, cam.W, cam.H
    r = Randomization()
    return workspace_roi(cam, r.x_range, r.y_range)


def detect(model, img, roi, conf):
    """YOLO on the crop -> [(class_id, score, (x0, y0, x1, y1) full-frame pixels)]."""
    x0, y0, x1, y1 = roi
    result = model.predict(img[y0:y1, x0:x1], imgsz=IMGSZ, conf=conf, verbose=False)[0]
    boxes = result.boxes
    return [(int(c), float(s), (b[0] + x0, b[1] + y0, b[2] + x0, b[3] + y0))
            for b, c, s in zip(boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist())]


def to_object(cls, score, box, width, height):
    category, colour = CLASS_TO_OBJECT[cls]
    x0, y0, x1, y1 = box
    return {"label": f"{colour} {category}", "category": category, "colour": colour,
            "center": [(x0 + x1) / 2 / width, (y0 + y1) / 2 / height],
            "bbox": [x0 / width, y0 / height, x1 / width, y1 / height],
            "confidence": round(score, 4), "graspable": True}


def top_edge(cam, xy, yaw):
    """Normalised endpoints of the cube's top edge, the form edge_pose() reads."""
    t = np.deg2rad(yaw)
    rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    points = [cam.project([*(xy + rot @ np.array([a, -1]) * C.CUBE_EDGE / 2),
                           cam.table_z + C.CUBE_EDGE]) for a in (-1, 1)]
    return (np.array(points) / [cam.W, cam.H]).tolist()


def add_cube_pose(img, cam, obj):
    """Image-only yaw for one cube; confidence is the silhouette fit (IoU)."""
    u, v = np.array(obj["center"]) * [cam.W, cam.H]
    point = cam.to_plane(u, v, z=cam.table_z + C.CUBE_EDGE / 2)
    if point is None:  # ray never meets the table (e.g. above the side camera's horizon)
        obj.update(pose_confidence=0.0, top_edge_endpoints=None,
                   pose_reason="Detection does not lie on the table plane")
        return
    xy, yaw, fit = refine_pose(img, cam, obj, point[:2], 0, search_half_width=45)
    obj.update(pose_source="yolo_box_plus_image_silhouette_fit",
               pose_silhouette_iou=round(float(fit), 4), top_edge_endpoints=None,
               pose_confidence=round(float(fit), 4) if fit >= POSE_MIN_FIT else 0.0)
    if fit < POSE_MIN_FIT:
        obj["pose_reason"] = "Image geometry fit is uncertain"
        return
    obj["top_edge_endpoints"] = top_edge(cam, np.asarray(xy), yaw)
    obj["pose_world"] = dict(position_m=[*map(float, xy), cam.table_z + C.CUBE_EDGE / 2],
                             yaw_deg=float(square_yaw(yaw)), roll_deg=0, pitch_deg=0,
                             yaw_symmetry_deg=90)


def perceive(model, img, cam, camera="table", conf=DEFAULT_CONF):
    """Full-frame BGR image -> detections dict in the perceive.py format."""
    start = time.time()
    raw = detect(model, img, detector_roi(cam, camera), conf)
    detect_s = time.time() - start
    objects = [to_object(c, s, b, cam.W, cam.H) for c, s, b in raw]
    for obj in objects:
        if obj["category"] == "cube":
            add_cube_pose(img, cam, obj)
    return {"objects": objects, "bins": [], "model": "YOLO", "task": "cube_stack",
            "timing_s": {"detect": round(detect_s, 4), "total": round(time.time() - start, 4)}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--image", help="default: <run>/world_camera.png")
    ap.add_argument("--camera", default="table", choices=["table", "side", "wrist"])
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF)
    args = ap.parse_args()
    from ultralytics import YOLO

    run = Path(args.run)
    image = Path(args.image) if args.image else run / "world_camera.png"
    img = cv2.imread(str(image))
    if img is None:
        raise SystemExit(f"cannot read {image}")
    cam = CameraModel(str(run / "camera.json"))
    det = perceive(YOLO(args.weights), img, cam, args.camera, args.conf)
    det["model"] = f"YOLO:{Path(args.weights).name}"
    (run / "detections.json").write_text(json.dumps(det, indent=2) + "\n")
    cubes = [o for o in det["objects"] if o["category"] == "cube"]
    posed = sum(o["pose_confidence"] > 0 for o in cubes)
    print(f"[perceive_yolo] {len(det['objects'])} objects ({len(cubes)} cubes, {posed} posed) "
          f"in {det['timing_s']['total']:.2f}s")


if __name__ == "__main__":
    main()
