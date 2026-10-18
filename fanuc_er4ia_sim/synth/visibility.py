"""
Occlusion-aware labelling from the rendered depth image.

For each labelled object we sample points on its camera-facing surfaces and
compare the depth we expect there with the depth Isaac rendered. Where the
render is clearly nearer, something (arm, neighbour, tower top, distractor)
hides that point. The visible fraction decides whether the object is labelled.
Pure numpy: depth maps are synthesised in the host tests.
"""

import math

import numpy as np

import scene_config as C

MIN_VISIBLE = 0.35  # below this an object is too hidden to label


def _grid(n):
    return np.linspace(-0.8, 0.8, n)


def _cube_samples(obj, base):
    yaw = math.radians(obj.yaw_deg)
    ax = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    sd = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    h = C.CUBE_EDGE / 2
    centre = base + h * up
    pts, nrm = [], []
    for normal, u, v in ((up, ax, sd), (ax, sd, up), (-ax, sd, up), (sd, ax, up), (-sd, ax, up)):
        for a in _grid(4):
            for b in _grid(4):
                pts.append(centre + h * (normal + a * u + b * v))
                nrm.append(normal)
    return np.array(pts), np.array(nrm)


def _disc_samples(centre, radius):
    pts = [centre]
    for r in (0.4, 0.8):
        for t in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            pts.append(centre + radius * r * np.array([math.cos(t), math.sin(t), 0.0]))
    return np.array(pts), np.tile([0.0, 0.0, 1.0], (len(pts), 1))


def _vial_samples(obj, base):
    r, length = C.VIAL_RADIUS, C.VIAL_LENGTH
    if obj.variant == "upright":
        return _disc_samples(base + np.array([0, 0, length]), r)
    yaw = math.radians(obj.yaw_deg)
    ax = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    sd = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    pts, nrm = [], []
    for t in np.linspace(-0.45, 0.45, 9):
        for ang in np.linspace(0, math.pi, 7):  # upper half of the tube
            n = math.cos(ang) * sd + math.sin(ang) * up
            pts.append(base + r * up + t * length * ax + r * n)
            nrm.append(n)
    return np.array(pts), np.array(nrm)


def surface_samples(obj, table_z):
    """(points, outward normals) on the object's visible-from-above surfaces."""
    base = np.array([obj.x, obj.y, table_z + getattr(obj, "z", 0.0)])
    if obj.category == "cube":
        return _cube_samples(obj, base)
    if obj.category == "cap":
        top = base + np.array([0, 0, C.CAP_HEIGHT])
        return _disc_samples(top, C.CAP_PLUG_TIP_RADIUS)  # inside both cap faces
    return _vial_samples(obj, base)


def visible_fraction(cam, depth, obj, table_z, tol=0.004, rel_tol=0.003):
    """Share of camera-facing, in-frame samples not hidden by nearer geometry."""
    pts, nrm = surface_samples(obj, table_z)
    facing = np.einsum("ij,ij->i", nrm, cam.eye - pts) > 0
    seen = total = 0
    for p in pts[facing]:
        uv = cam.project(p)
        if uv is None:
            continue
        u, v = int(uv[0]), int(uv[1])
        if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
            continue
        expected = -(cam.R @ (p - cam.eye))[2]
        total += 1
        seen += depth[v, u] >= expected - (tol + rel_tol * expected)
    return seen / total if total else 0.0
