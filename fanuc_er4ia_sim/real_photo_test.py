"""
Real-cell photo test: YOLO + the starter's calibrated geometry on photos from the CSI cell.

    python real_photo_test.py photo1.jpg [photo2.png ...] --weights best.pt --out csi_out [--camera table]

For each photo: scale it to the calibrated resolution (any resolution with the same
16:9 shape works), run perceive_yolo, then plan_sort.build_plan (cube tower task), and write
  <name>_overlay.jpg   boxes, colour, confidence, table position (mm) and yaw per object
  <name>_result.json   detections + plan (positions in the robot base frame)
  <name>_picklist.csv  ordered picks for MENTOR REVIEW (base frame and controller UF0 frame)
The camera model comes from scene_config (v0.5 ArUco calibration). The starter notes the table
and wrist mounts were not revalidated after the testbed rebuild: confirm with the mentor that
the cameras have not moved before trusting millimetres. Nothing here talks to the robot.
"""

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np

import scene_config as C
from backproject import CameraModel
from perceive_yolo import perceive
from plan_sort import build_plan

CALIBRATED = {"table": (C.TABLE_CAM, 2), "side": (C.SIDE_CAM, 1)}  # (config, resolution scale)
UF0_ABOVE_BASE_M = 0.330  # docs/CALIBRATION_V05.md: controller UF0 is 330 mm above the base plane
ASPECT_TOLERANCE = 0.01


def camera_json(camera):
    """Pinhole model in the USD row convention used by backproject.CameraModel."""
    cfg, scale = CALIBRATED[camera]
    eye, target, up = (np.asarray(cfg[k], float) for k in ("eye", "look_at", "up"))
    back = (eye - target) / np.linalg.norm(eye - target)
    right = np.cross(up, back)
    right /= np.linalg.norm(right)
    width, height = (scale * v for v in cfg["resolution"])
    return {"resolution": [width, height], "focal_length_mm": cfg["focal_length"],
            "horizontal_aperture_mm": cfg["horiz_aperture"],
            "vertical_aperture_mm": cfg["horiz_aperture"] * height / width,
            "camera_to_world": [[*right, 0.0], [*np.cross(back, right), 0.0], [*back, 0.0], [*eye, 1.0]],
            "table_top_z": float(C.TABLE_TOP_Z + C.SURFACE_DZ)}


def fit_to_camera(img, cam):
    """Scale a photo to the calibrated resolution; refuse a different shape (crop/rotation)."""
    h, w = img.shape[:2]
    if abs(w / h - cam.W / cam.H) > ASPECT_TOLERANCE * cam.W / cam.H:
        raise ValueError(f"photo is {w}x{h}, calibration expects {cam.W}x{cam.H} (16:9); "
                         "use an uncropped, unrotated frame from the cell camera")
    if (w, h) == (cam.W, cam.H):
        return img, 1.0
    return cv2.resize(img, (cam.W, cam.H), interpolation=cv2.INTER_CUBIC), cam.W / w


READY, OTHER, REJECTED = (60, 190, 60), (150, 150, 150), (40, 40, 230)  # BGR


def status(obj):
    """ready = cube the planner can pick; other = not part of the tower task; rejected = cube refused."""
    if obj.get("category") != "cube":
        return "other", OTHER
    if obj.get("skip_reason") in (None, "incomplete_cube_colour_set", "duplicate_detection"):
        return "ready", READY
    return f'rejected: {obj.get("skip_reason")}', REJECTED


def label(obj):
    text = f'{obj.get("colour", "")} {obj.get("category", "")} {obj.get("confidence", 0):.2f}'
    if "xy" in obj:
        text += f'  x={obj["xy"][0] * 1000:.0f} y={obj["xy"][1] * 1000:.0f} mm'
    if obj.get("category") == "cube" and "yaw_deg" in obj:
        text += f'  yaw {obj["yaw_deg"]:.0f} deg'
    return text


def draw(img, plan):
    """Short #n tags on the objects plus a legend panel, so close objects stay readable."""
    out = img.copy()
    rows = []
    for n, obj in enumerate([o for o in plan["observations"] if o.get("bbox")], 1):
        x0, y0, x1, y1 = (int(v * s) for v, s in zip(obj["bbox"], (img.shape[1], img.shape[0]) * 2))
        state, colour = status(obj)
        cv2.rectangle(out, (x0, y0), (x1, y1), colour, 3)
        cv2.putText(out, f"#{n}", (x0, max(y0 - 8, 24)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)
        rows.append((f"#{n} {label(obj)}  [{state}]", colour))
    line = 34
    cv2.rectangle(out, (0, 0), (1500, 20 + line * (len(rows) + 1)), (30, 30, 30), -1)
    cv2.putText(out, "green = cube ready to pick, grey = not in tower task, red = cube rejected",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (230, 230, 230), 2)
    for i, (text, colour) in enumerate(rows, 2):
        cv2.putText(out, text, (12, line * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
    return out


def pick_rows(plan):
    """Ordered picks in the robot base frame and the controller UF0 frame (mm, deg)."""
    rows = []
    for order, p in enumerate(plan.get("picks", []), 1):
        x, y, z = p["xy"][0], p["xy"][1], p["pick_z"]
        rows.append({"order": order, "colour": p.get("colour"), "category": p.get("category"),
                     "pick_x_mm": round(x * 1000, 1), "pick_y_mm": round(y * 1000, 1),
                     "tip_z_base_mm": round(z * 1000, 1),
                     "tip_z_uf0_mm": round((z - UF0_ABOVE_BASE_M) * 1000, 1),
                     "cube_yaw_deg": round(p.get("yaw_deg", 0.0), 1),
                     "place_x_mm": round(p.get("place_xy", [0, 0])[0] * 1000, 1),
                     "place_y_mm": round(p.get("place_xy", [0, 0])[1] * 1000, 1),
                     "stack_level": p.get("stack_level")})
    return rows


def run_photo(model, path, camera, out):
    cam_json = camera_json(camera)
    cam = CameraModel(cam_json)
    img, scale = fit_to_camera(cv2.imread(str(path)), cam)
    start = time.time()
    det = perceive(model, img, cam, camera)
    plan = build_plan(det, cam, img, task="cube_stack")
    seconds = time.time() - start
    stem = out / path.stem
    cv2.imwrite(f"{stem}_overlay.jpg", draw(img, plan), [cv2.IMWRITE_JPEG_QUALITY, 88])
    rows = pick_rows(plan)
    Path(f"{stem}_result.json").write_text(json.dumps(
        {"photo": str(path), "scaled_by": scale, "camera": cam_json, "seconds": seconds,
         "detections": det, "plan": plan}, indent=2, default=float))
    if rows:
        with open(f"{stem}_picklist.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return det, plan, rows, seconds


def summary_line(path, det, plan, rows, seconds):
    found = {}
    for o in det["objects"]:
        key = f'{o["colour"]} {o["category"]}' if o["category"] == "cube" else o["category"]
        found[key] = found.get(key, 0) + 1
    ready = sum(1 for o in plan["observations"] if o.get("category") == "cube"
                and o.get("skip_reason") in (None, "incomplete_cube_colour_set"))
    return (f"{path.name}: {dict(sorted(found.items()))} | cubes pose-ready {ready} | "
            f"tower picks planned {len(rows)} | {seconds:.1f}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("photos", nargs="+")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", default="csi_out")
    ap.add_argument("--camera", default="table", choices=sorted(CALIBRATED))
    args = ap.parse_args()
    from ultralytics import YOLO

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.weights)
    for name in args.photos:
        path = Path(name)
        try:
            print(summary_line(path, *run_photo(model, path, args.camera, out)))
        except (ValueError, cv2.error, AttributeError) as e:
            print(f"{path.name}: SKIPPED - {e}")
    print(f"overlays, results and pick lists in {out}/  (pick lists are for MENTOR REVIEW only)")


if __name__ == "__main__":
    main()
