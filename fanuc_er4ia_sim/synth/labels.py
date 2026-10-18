"""
Bounding-box labels from known object geometry and the rendered camera model.

Because the generator sets every object pose itself, a tight 2D box is the
projection of the object's 3D hull: eight corners for a cube, two rim circles
for a cap or a lying vial. No segmentation pass is needed, and the maths runs
on the host so it is unit-tested against backproject.CameraModel.

Limitation: hulls ignore occlusion by taller neighbours. Layout separation keeps
footprints apart, which leaves only small oblique-view overlaps.
"""

import math

import numpy as np

import scene_config as C

CLASS_NAMES = ("cube_red", "cube_yellow", "cube_blue", "cap", "vial")
RING_POINTS = 24


def class_id(obj):
    name = f"cube_{obj.colour}" if obj.category == "cube" else obj.category
    return CLASS_NAMES.index(name)


def _ring(centre, radius, u, v):
    angles = np.linspace(0.0, 2 * math.pi, RING_POINTS, endpoint=False)
    return np.array(centre) + radius * (
        np.outer(np.cos(angles), u) + np.outer(np.sin(angles), v)
    )


def hull_points(obj, table_z):
    """World-space points whose projection bounds the rendered object."""
    yaw = math.radians(obj.yaw_deg)
    axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    side = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    # v2 items carry a stack height and variants; v1 ObjectPose has neither.
    base = np.array([obj.x, obj.y, table_z + getattr(obj, "z", 0.0)])
    if obj.category == "cube":
        h = C.CUBE_EDGE / 2
        return np.array(
            [
                base + sx * h * axis + sy * h * side + z * up
                for sx in (-1, 1)
                for sy in (-1, 1)
                for z in (0.0, C.CUBE_EDGE)
            ]
        )
    if obj.category == "cap":
        x_dir = np.array([1.0, 0.0, 0.0])
        y_dir = np.array([0.0, 1.0, 0.0])
        return np.vstack(
            [_ring(base + z * up, C.CAP_RADIUS, x_dir, y_dir) for z in (0.0, C.CAP_HEIGHT)]
        )
    if getattr(obj, "variant", "") == "upright":
        x_dir, y_dir = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
        return np.vstack(
            [_ring(base + z * up, C.VIAL_RADIUS, x_dir, y_dir) for z in (0.0, C.VIAL_LENGTH)]
        )
    centre = base + C.VIAL_RADIUS * up
    ends = (centre - axis * C.VIAL_LENGTH / 2, centre + axis * C.VIAL_LENGTH / 2)
    return np.vstack([_ring(end, C.VIAL_RADIUS, side, up) for end in ends])


def project_box(cam, points):
    """(x0, y0, x1, y1) in full-image pixels, or None if behind the camera."""
    uv = [cam.project(p) for p in points]
    if any(p is None for p in uv):
        return None
    uv = np.array(uv)
    return (*uv.min(axis=0), *uv.max(axis=0))


def workspace_roi(cam, x_range, y_range, pad_px=96, multiple=32):
    """Fixed crop (x0, y0, x1, y1) around the sampled table area."""
    z = cam.table_z
    corners = [(x, y, zz) for x in x_range for y in y_range for zz in (z, z + 0.02)]
    x0, y0, x1, y1 = project_box(cam, corners)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    w = math.ceil((x1 - x0 + 2 * pad_px) / multiple) * multiple
    h = math.ceil((y1 - y0 + 2 * pad_px) / multiple) * multiple
    left = int(min(max(cx - w / 2, 0), cam.W - w))
    top = int(min(max(cy - h / 2, 0), cam.H - h))
    return left, top, left + w, top + h


def yolo_line(box, roi, cls, min_visible=0.5, min_px=4.0):
    """YOLO 'cls cx cy w h' normalised to the ROI crop, or None if mostly cut."""
    x0, y0, x1, y1 = box
    rx0, ry0, rx1, ry1 = roi
    cx0, cy0 = max(x0, rx0), max(y0, ry0)
    cx1, cy1 = min(x1, rx1), min(y1, ry1)
    full = (x1 - x0) * (y1 - y0)
    if cx1 - cx0 < min_px or cy1 - cy0 < min_px or full <= 0:
        return None
    if (cx1 - cx0) * (cy1 - cy0) / full < min_visible:
        return None
    w, h = rx1 - rx0, ry1 - ry0
    return (
        f"{cls} {((cx0 + cx1) / 2 - rx0) / w:.6f} {((cy0 + cy1) / 2 - ry0) / h:.6f} "
        f"{(cx1 - cx0) / w:.6f} {(cy1 - cy0) / h:.6f}"
    )


def label_lines(sample, cam, roi):
    """All YOLO label lines for one rendered scene."""
    lines = []
    for obj in sample.objects:
        if not obj.visible:
            continue
        box = project_box(cam, hull_points(obj, cam.table_z))
        line = None if box is None else yolo_line(box, roi, class_id(obj))
        if line:
            lines.append(line)
    return lines


def label_lines_v2(scene, cam, roi, depth):
    """Occlusion-aware labels: (lines, per-line visibility) for one v2 frame.

    Objects whose camera-facing surface is less than MIN_VISIBLE visible in the
    rendered depth (hidden by the arm, a neighbour, a tower top, a distractor)
    are left unlabelled rather than labelled from geometry alone.
    """
    from synth.visibility import MIN_VISIBLE, visible_fraction

    lines, vis = [], []
    for obj in scene.objects:
        if not obj.visible:
            continue
        box = project_box(cam, hull_points(obj, cam.table_z))
        frac = visible_fraction(cam, depth, obj, cam.table_z) if box else 0.0
        line = yolo_line(box, roi, class_id(obj)) if frac >= MIN_VISIBLE else None
        if line:
            lines.append(line)
            vis.append(round(frac, 3))
    return lines, vis
