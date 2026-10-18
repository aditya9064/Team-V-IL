"""
V2 scene sampling: the hard cases v1 never showed the detector.

Pure Python (no Isaac) so every rule is unit-tested on the host. Each frame
draws a scene *mode* (scatter, cluster, tower, crowded, sparse, edge) plus
independent toggles: arm in view, unlabelled distractors, flat look-alike
tape, table appearance, light direction, camera zoom and sensor effects.

Labels stay geometry-exact: labelled objects never interpenetrate, stacked
cubes sit exactly one edge apart, and distractors never imitate a labelled
class (no 7-14 mm cubes in cube colours, no cap-sized yellow cylinders).
"""

import math
import random
from dataclasses import dataclass, field

import scene_config as C
from synth.layout import CUBE_COLOURS, LIGHT_NAMES, footprint_discs

CUBE_SLOTS = 12
CAMERAS = ("table", "side", "wrist")
MODES = {"scatter": 0.25, "cluster": 0.2, "tower": 0.2, "crowded": 0.15, "sparse": 0.1, "edge": 0.1}
SHAPES = ("cube", "sphere", "cylinder", "cone", "capsule")
# Distractor palette: (name, rgb, imitates a cube class colour?)
PALETTE = (
    ("red", (0.75, 0.08, 0.06), True), ("yellow", (0.85, 0.72, 0.1), True),
    ("blue", (0.08, 0.2, 0.7), True), ("green", (0.1, 0.55, 0.2), False),
    ("purple", (0.45, 0.15, 0.6), False), ("white", (0.9, 0.9, 0.88), False),
    ("black", (0.05, 0.05, 0.05), False), ("grey", (0.45, 0.45, 0.47), False),
    ("brown", (0.4, 0.26, 0.15), False), ("cyan", (0.1, 0.6, 0.65), False),
    ("pink", (0.85, 0.45, 0.6), False), ("steel", (0.62, 0.63, 0.66), False),
)
DISTRACTOR_SLOTS = {shape: 3 for shape in SHAPES}
TAPE_SLOTS = 6
MAT_TEXTURES = 6


@dataclass(frozen=True)
class V2Config:
    x_range: tuple = (0.20, 0.50)
    y_range: tuple = (-0.25, 0.32)
    edge_x: tuple = (0.12, 0.58)  # "edge" mode: partly outside the crop
    edge_y: tuple = (-0.36, 0.44)
    margin: float = 0.003
    arm_in_view: dict = field(default_factory=lambda: {"table": 0.5, "side": 0.6, "wrist": 1.0})
    camera_eye_sigma: float = 0.012
    camera_target_sigma: float = 0.008
    focal_jitter: float = 0.05


@dataclass(frozen=True)
class Item:
    """A labelled object (cube/cap/vial) or an unlabelled distractor/tape."""

    name: str
    category: str  # cube | cap | vial | distractor | tape
    colour: str
    x: float
    y: float
    yaw_deg: float
    visible: bool
    z: float = 0.0  # base height above the work surface (stacked cubes)
    variant: str = ""  # cap: narrow_up|wide_up; vial: lying|upright; distractor: shape
    size: float = 0.0  # distractor/tape edge length or diameter (m)


@dataclass(frozen=True)
class SceneV2:
    index: int
    seed: int
    camera: str
    mode: str
    objects: tuple  # labelled slots (cubes, caps, vials)
    distractors: tuple
    lights: tuple  # (name, intensity_scale, tint, rotation jitter deg)
    mat: tuple  # (texture index or -1 for the stock mat, rgb tint)
    arm: dict  # joint name -> degrees, plus "jaw" in metres per side
    camera_eye_offset: tuple
    camera_target_offset: tuple
    focal_scale: float
    sensor: dict


def _discs(item):
    if item.category in ("cube", "cap"):
        return footprint_discs(item.category, item.x, item.y, item.yaw_deg)
    if item.category == "vial":
        if item.variant == "upright":
            return [(item.x, item.y, C.VIAL_RADIUS)]
        return footprint_discs("vial", item.x, item.y, item.yaw_deg)
    if item.category == "tape":
        return []  # flat; objects may rest on top of tape
    return [(item.x, item.y, item.size / 2 * 1.42)]


def _clear(discs, placed, margin):
    return all(
        math.dist(a[:2], b[:2]) >= a[2] + b[2] + margin for a in discs for b in placed
    )


def _place(rng, item, placed, area, margin, attempts=150):
    """Return item moved to a free spot in area, or None."""
    for _ in range(attempts):
        x, y = rng.uniform(*area[0]), rng.uniform(*area[1])
        cand = _with(item, x=x, y=y)
        discs = _discs(cand)
        if _clear(discs, placed, margin):
            placed.extend(discs)
            return cand
    return None


def _with(item, **changes):
    values = {k: getattr(item, k) for k in item.__dataclass_fields__}
    values.update(changes)
    return Item(**values)


CUBE_COUNTS = {"scatter": (3, 8), "cluster": (4, 10), "tower": (4, 12), "crowded": (9, 12),
               "sparse": (0, 2), "edge": (3, 8)}
DISTRACTOR_COUNTS = {"scatter": (0, 5), "cluster": (0, 4), "tower": (0, 4), "crowded": (6, 12),
                     "sparse": (0, 2), "edge": (0, 5)}


def _cube(rng, i, **kw):
    base = dict(name=f"SortCube{i}", category="cube", colour=rng.choice(CUBE_COLOURS),
                x=0.0, y=0.0, yaw_deg=rng.uniform(0, 90), visible=True)
    base.update(kw)
    return Item(**base)


def _tower(rng, slots, placed, area, cfg):
    """2-6 cubes stacked one edge apart, each slightly offset and twisted."""
    height = min(rng.randint(2, 6), len(slots))
    base = _place(rng, _cube(rng, slots[0]), placed, area, cfg.margin)
    if base is None:
        return []
    out = [base]
    for level, i in enumerate(slots[1:height], start=1):
        jx, jy = rng.uniform(-8e-4, 8e-4), rng.uniform(-8e-4, 8e-4)
        out.append(_cube(rng, i, x=base.x + jx, y=base.y + jy, z=level * C.CUBE_EDGE,
                         yaw_deg=base.yaw_deg + rng.uniform(-8, 8)))
    return out


def _cluster(rng, slots, placed, area, cfg):
    """2-4 cubes sharing a yaw on a small grid with 0-2 mm gaps (touching)."""
    n = min(rng.randint(2, 4), len(slots))
    cells = rng.sample([(0, 0), (1, 0), (0, 1), (1, 1), (-1, 0), (0, -1)], n)
    yaw, gap = rng.uniform(0, 90), rng.uniform(0.0, 0.002)
    step = C.CUBE_EDGE + gap
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    reach = max(math.hypot(i, j) for i, j in cells) * step + C.CUBE_EDGE / math.sqrt(2)
    for _ in range(150):
        cx, cy = rng.uniform(*area[0]), rng.uniform(*area[1])
        if _clear([(cx, cy, reach)], placed, cfg.margin):
            placed.append((cx, cy, reach))
            return [_cube(rng, k, x=cx + step * (i * c - j * s), y=cy + step * (i * s + j * c),
                          yaw_deg=yaw) for k, (i, j) in zip(slots, cells)]
    return []


def _cubes(rng, mode, placed, area, cfg):
    total = rng.randint(*CUBE_COUNTS[mode])
    slots, cubes = list(range(CUBE_SLOTS)), []
    groups = rng.randint(1, 2) if mode in ("tower", "cluster") else 0
    for _ in range(groups):
        if total - len(cubes) < 2:
            break  # a tower or cluster needs at least two cubes
        maker = _tower if mode == "tower" else _cluster
        group = maker(rng, slots[: total - len(cubes)], placed, area, cfg)
        cubes += group
        slots = slots[len(group):]
    while len(cubes) < total and slots:
        item = _place(rng, _cube(rng, slots.pop(0)), placed, area, cfg.margin)
        if item:
            cubes.append(item)
    used = {c.name for c in cubes}
    hidden = [_cube(rng, i, visible=False) for i in range(CUBE_SLOTS) if f"SortCube{i}" not in used]
    return cubes + hidden


def _caps_vials(rng, placed, area, cfg):
    out = []
    for i in range(len(C.SORT_VIALS)):
        variant = "upright" if rng.random() < 0.3 else "lying"
        item = Item(f"SortVial{i}", "vial", "clear", 0, 0, rng.uniform(0, 180), True, variant=variant)
        placed_item = _place(rng, item, placed, area, cfg.margin) if rng.random() < 0.6 else None
        out.append(placed_item or _with(item, visible=False))
    for i in range(len(C.SORT_CAPS)):
        variant = rng.choice(("narrow_up", "wide_up"))
        item = Item(f"SortCap{i}", "cap", "yellow", 0, 0, 0.0, True, variant=variant)
        placed_item = _place(rng, item, placed, area, cfg.margin) if rng.random() < 0.6 else None
        out.append(placed_item or _with(item, visible=False))
    return out


def _distractor_colour(rng, shape, size):
    """Never imitate a labelled class: no cube-sized cubes in cube colours, no cap-like yellow."""
    choices = list(PALETTE)
    if shape == "cube" and 0.007 <= size <= 0.014:
        choices = [p for p in PALETTE if not p[2]]
    if shape == "cylinder" and 0.024 <= size <= 0.04:
        choices = [p for p in PALETTE if p[0] != "yellow"]
    return rng.choice(choices)[0]


def _distractors(rng, mode, placed, area, cfg):
    wanted = rng.randint(*DISTRACTOR_COUNTS[mode])
    slots = [(shape, k) for shape in SHAPES for k in range(DISTRACTOR_SLOTS[shape])]
    rng.shuffle(slots)
    out = []
    for n, (shape, k) in enumerate(slots):
        size = rng.uniform(0.005, 0.04)
        item = Item(f"Distractor_{shape}{k}", "distractor", _distractor_colour(rng, shape, size),
                    0, 0, rng.uniform(0, 360), True, variant=shape, size=size)
        placed_item = _place(rng, item, placed, area, cfg.margin) if n < wanted else None
        out.append(placed_item or _with(item, visible=False))
    for k in range(TAPE_SLOTS):
        visible = rng.random() < 0.35
        out.append(Item(f"Tape{k}", "tape", rng.choice(PALETTE)[0], rng.uniform(*area[0]),
                        rng.uniform(*area[1]), rng.uniform(0, 180), visible,
                        size=rng.uniform(0.01, 0.06)))
    return out


def _lights(rng):
    out = []
    for name in LIGHT_NAMES:
        if name == "DomeLight":
            scale, rot = rng.uniform(0.4, 1.6), (0.0, 0.0, 0.0)
        else:
            scale = 0.0 if rng.random() < 0.15 else rng.uniform(0.3, 1.8)  # light off: hard shadows
            rot = tuple(rng.uniform(-25, 25) for _ in range(3))
        tint = [1.0 + rng.uniform(-0.12, 0.12) for _ in range(3)]
        peak = max(tint)
        out.append((name, scale, tuple(t / peak for t in tint), rot))
    return tuple(out)


def _mat(rng):
    roll = rng.random()
    if roll < 0.45:
        return (-1, (1.0, 1.0, 1.0))  # stock white mat
    grey = rng.uniform(0.62, 0.95)
    tint = tuple(grey * rng.uniform(0.95, 1.05) for _ in range(3))  # near-neutral
    return (rng.randrange(MAT_TEXTURES) if roll < 0.8 else -2, tint)  # -2: solid tint


ARM_JITTER = {"wrist": (40, 10, 10, 30, 15, 180), "other": (70, 20, 20, 90, 30, 180)}


def _arm(rng, camera, cfg):
    if rng.random() >= cfg.arm_in_view[camera]:
        pose = dict(C.CAPTURE_POSE_DEG)
    else:
        jitter = ARM_JITTER["wrist" if camera == "wrist" else "other"]
        pose = {}
        for (joint, centre), spread in zip(C.WRIST_PHOTO_POSE_DEG.items(), jitter):
            lo, hi = C.JOINT_LIMITS_DEG[joint]
            pose[joint] = min(max(centre + rng.uniform(-spread, spread), lo), hi)
    pose["jaw"] = rng.uniform(0.0, C.JAW_OPEN)
    return pose


def _sensor(rng):
    blur = rng.choices(("none", "gaussian", "motion"), weights=(0.5, 0.3, 0.2))[0]
    return {
        "gain": rng.uniform(0.75, 1.3),
        "gamma": rng.uniform(0.85, 1.2),
        "wb": tuple(rng.uniform(0.9, 1.1) for _ in range(3)),
        "noise": rng.uniform(0, 6) if rng.random() < 0.7 else 0.0,
        "blur": blur,
        "blur_sigma": rng.uniform(0.4, 1.3),
        "motion_len": rng.randint(3, 9),
        "motion_angle": rng.uniform(0, 180),
        "jpeg": rng.randint(55, 95) if rng.random() < 0.5 else 0,
        "vignette": rng.uniform(0, 0.35),
    }


def sample_v2(index, base_seed, camera, cfg=V2Config()):
    """Deterministic per (index, seed, camera); chunks can run anywhere."""
    if camera not in CAMERAS:
        raise ValueError(f"unknown camera {camera!r}")
    seed = base_seed * 1_000_003 + index
    rng = random.Random(seed)
    mode = rng.choices(list(MODES), weights=list(MODES.values()))[0]
    area = (cfg.edge_x, cfg.edge_y) if mode == "edge" else (cfg.x_range, cfg.y_range)
    placed = []
    objects = _cubes(rng, mode, placed, area, cfg) + _caps_vials(rng, placed, area, cfg)
    distractors = _distractors(rng, mode, placed, area, cfg)
    return SceneV2(
        index=index, seed=seed, camera=camera, mode=mode,
        objects=tuple(objects), distractors=tuple(distractors),
        lights=_lights(rng), mat=_mat(rng), arm=_arm(rng, camera, cfg),
        camera_eye_offset=tuple(rng.gauss(0, cfg.camera_eye_sigma) for _ in range(3)),
        camera_target_offset=tuple(rng.gauss(0, cfg.camera_target_sigma) for _ in range(3)),
        focal_scale=rng.uniform(1 - cfg.focal_jitter, 1 + cfg.focal_jitter),
        sensor=_sensor(rng),
    )
