"""
Domain-randomised scene sampling for synthetic detector training data.

Pure Python so the sampling rules can be unit-tested on any host. The Isaac
script (generate_synthetic.py) applies each SceneSample to the stage.

Every object is approximated by discs on the table for separation checks:
cubes by their half diagonal, caps by their radius, vials by a row of discs
along the tube axis. Objects never overlap, so a footprint label always
describes one physical object.
"""

import math
import random
from dataclasses import dataclass

import scene_config as C

CUBE_SLOTS = len(C.SORT_CUBES)
CAP_SLOTS = len(C.SORT_CAPS)
VIAL_SLOTS = len(C.SORT_VIALS)
CUBE_COLOURS = tuple(C.CUBE_MATS)
LIGHT_NAMES = ("DomeLight", "KeyLight", "FillLight", "RimLight")


@dataclass(frozen=True)
class Randomization:
    """Ranges for one dataset. Tune after comparing renders with real photos."""

    x_range: tuple = (0.20, 0.50)  # robot-base metres, table camera coverage
    y_range: tuple = (-0.25, 0.32)
    cube_presence: float = 0.85
    cap_presence: float = 0.7
    vial_presence: float = 0.6
    separation_margin: float = 0.003
    dome_scale: tuple = (0.6, 1.4)
    rect_light_scale: tuple = (0.4, 1.6)
    light_tint: float = 0.08  # +/- per RGB channel before renormalising
    camera_eye_sigma: float = 0.004  # metres; covers calibration error
    camera_target_sigma: float = 0.003


@dataclass(frozen=True)
class ObjectPose:
    name: str
    category: str  # cube | cap | vial
    colour: str
    x: float
    y: float
    yaw_deg: float
    visible: bool


@dataclass(frozen=True)
class LightSample:
    name: str
    intensity_scale: float
    tint: tuple


@dataclass(frozen=True)
class SceneSample:
    index: int
    seed: int
    objects: tuple
    lights: tuple
    camera_eye_offset: tuple
    camera_target_offset: tuple


def footprint_discs(category, x, y, yaw_deg):
    """(cx, cy, radius) discs covering an object's table footprint."""
    if category == "cube":
        return [(x, y, C.CUBE_EDGE / math.sqrt(2))]
    if category == "cap":
        return [(x, y, C.CAP_RADIUS)]
    axis = (math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)))
    half = C.VIAL_LENGTH / 2
    steps = 9
    return [
        (x + t * axis[0], y + t * axis[1], C.VIAL_RADIUS)
        for t in (-half + i * 2 * half / (steps - 1) for i in range(steps))
    ]


def _clear(discs, placed, margin):
    return all(
        math.dist((a[0], a[1]), (b[0], b[1])) >= a[2] + b[2] + margin
        for a in discs
        for b in placed
    )


def _place(rng, category, cfg, placed, attempts=200):
    """Sample a non-overlapping (x, y, yaw) or None when the table is full."""
    for _ in range(attempts):
        x = rng.uniform(*cfg.x_range)
        y = rng.uniform(*cfg.y_range)
        yaw = rng.uniform(0.0, 180.0 if category == "vial" else 90.0)
        discs = footprint_discs(category, x, y, yaw)
        if _clear(discs, placed, cfg.separation_margin):
            placed.extend(discs)
            return x, y, yaw
    return None


def _objects(rng, cfg):
    """Largest objects first so small cubes fill the remaining space."""
    slots = (
        [("vial", f"SortVial{i}", "clear", cfg.vial_presence) for i in range(VIAL_SLOTS)]
        + [("cap", f"SortCap{i}", "yellow", cfg.cap_presence) for i in range(CAP_SLOTS)]
        + [("cube", f"SortCube{i}", None, cfg.cube_presence) for i in range(CUBE_SLOTS)]
    )
    placed, objects = [], []
    for category, name, colour, presence in slots:
        colour = colour or rng.choice(CUBE_COLOURS)
        pose = _place(rng, category, cfg, placed) if rng.random() < presence else None
        x, y, yaw = pose if pose else (0.0, 0.0, 0.0)
        objects.append(ObjectPose(name, category, colour, x, y, yaw, pose is not None))
    return tuple(objects)


def _lights(rng, cfg):
    samples = []
    for name in LIGHT_NAMES:
        scale = cfg.dome_scale if name == "DomeLight" else cfg.rect_light_scale
        tint = [1.0 + rng.uniform(-cfg.light_tint, cfg.light_tint) for _ in range(3)]
        peak = max(tint)
        samples.append(LightSample(name, rng.uniform(*scale), tuple(c / peak for c in tint)))
    return tuple(samples)


def sample_scene(index, base_seed, cfg=Randomization()):
    """Deterministic per index, so generation can be split across machines."""
    seed = base_seed * 1_000_003 + index
    rng = random.Random(seed)
    objects = _objects(rng, cfg)
    lights = _lights(rng, cfg)
    eye = tuple(rng.gauss(0.0, cfg.camera_eye_sigma) for _ in range(3))
    target = tuple(rng.gauss(0.0, cfg.camera_target_sigma) for _ in range(3))
    return SceneSample(index, seed, objects, lights, eye, target)
