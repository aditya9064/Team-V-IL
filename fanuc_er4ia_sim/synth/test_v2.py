"""V2 scenarios, occlusion-aware visibility and sensor effects (host-only)."""

import json
import math
import unittest
from itertools import combinations

import numpy as np

import scene_config as C
from test_support import camera_model
import tempfile
from pathlib import Path

import cv2

from synth import labels, mat_textures, scenarios, sensor_fx, visibility

CAM = camera_model(C.TABLE_CAM)  # 1920x1080 keeps synthetic depth maps cheap


def square(obj):
    yaw = math.radians(obj.yaw_deg)
    ax = np.array([math.cos(yaw), math.sin(yaw)])
    sd = np.array([-math.sin(yaw), math.cos(yaw)])
    h, c = C.CUBE_EDGE / 2, np.array([obj.x, obj.y])
    return [c + h * (a * ax + b * sd) for a in (-1, 1) for b in (-1, 1)]


def squares_overlap(a, b, eps=1e-7):
    """Separating-axis test for two upright cubes' footprints."""
    pa, pb = square(a), square(b)
    for poly in (pa, pb):
        for i in range(4):
            edge = poly[(i + 1) % 4 if i % 2 == 0 else (i + 2) % 4] - poly[i]
            n = np.array([-edge[1], edge[0]])
            if not n.any():
                continue
            ra, rb = [p @ n for p in pa], [p @ n for p in pb]
            if max(ra) <= min(rb) + eps * abs(n).sum() or max(rb) <= min(ra) + eps * abs(n).sum():
                return False
    return True


def plane_depth(cam, z):
    """Depth image of a horizontal plane at height z (distance to image plane)."""
    H, W = cam.H, cam.W
    vv, uu = np.mgrid[0:H, 0:W].astype(float)
    x = (uu + 0.5) / W * 2 - 1
    y = 1 - (vv + 0.5) / H * 2
    d_cam = np.stack([x * cam.tan_h, y * cam.tan_v, -np.ones_like(x)], -1)
    d_world = d_cam @ cam.R
    t = (z - cam.eye[2]) / d_world[..., 2]
    return t  # d_cam has unit -z component, so t is the image-plane distance


class ScenarioTests(unittest.TestCase):
    def scenes(self, n=150):
        for cam in scenarios.CAMERAS:
            for i in range(n):
                yield scenarios.sample_v2(i, 11, cam)

    def test_reproducible(self):
        self.assertEqual(scenarios.sample_v2(5, 3, "side"), scenarios.sample_v2(5, 3, "side"))

    def test_ground_cubes_never_overlap(self):
        for s in self.scenes():
            ground = [o for o in s.objects if o.visible and o.category == "cube" and o.z == 0]
            for a, b in combinations(ground, 2):
                self.assertFalse(squares_overlap(a, b), (s.index, s.camera, a.name, b.name))

    def test_towers_are_exact_stacks(self):
        towers = 0
        for s in self.scenes():
            for o in (o for o in s.objects if o.visible and o.z > 0):
                level = o.z / C.CUBE_EDGE
                self.assertAlmostEqual(level, round(level), places=9)
                below = [b for b in s.objects if b.visible and b.category == "cube"
                         and abs(b.z - (o.z - C.CUBE_EDGE)) < 1e-9 and math.dist((b.x, b.y), (o.x, o.y)) < 0.002]
                self.assertTrue(below, f"{o.name} floats")
                towers += 1
        self.assertGreater(towers, 50)

    def test_distractors_never_imitate_a_label(self):
        class_like = {p[0] for p in scenarios.PALETTE if p[2]}
        for s in self.scenes():
            for d in s.distractors:
                if d.variant == "cube" and 0.007 <= d.size <= 0.014:
                    self.assertNotIn(d.colour, class_like)
                if d.variant == "cylinder" and 0.024 <= d.size <= 0.04:
                    self.assertNotEqual(d.colour, "yellow")

    def test_all_modes_and_limits(self):
        modes = set()
        for s in self.scenes():
            modes.add(s.mode)
            names = [o.name for o in s.objects + s.distractors]
            self.assertEqual(len(names), len(set(names)))
            for joint, (lo, hi) in C.JOINT_LIMITS_DEG.items():
                self.assertTrue(lo <= s.arm[joint] <= hi)
        self.assertEqual(modes, set(scenarios.MODES))


class LabelV2Tests(unittest.TestCase):
    def test_stacked_cube_projects_higher_than_its_base(self):
        base = scenarios.Item("a", "cube", "red", 0.35, 0.05, 0, True)
        top = scenarios.Item("b", "cube", "red", 0.35, 0.05, 0, True, z=2 * C.CUBE_EDGE)
        y_base = labels.project_box(CAM, labels.hull_points(base, CAM.table_z))[1]
        y_top = labels.project_box(CAM, labels.hull_points(top, CAM.table_z))[1]
        self.assertLess(y_top, y_base)

    def test_upright_vial_box_is_tall(self):
        vial = scenarios.Item("v", "vial", "clear", 0.35, 0.05, 0, True, variant="upright")
        x0, y0, x1, y1 = labels.project_box(CAM, labels.hull_points(vial, CAM.table_z))
        self.assertGreater(y1 - y0, x1 - x0)


class VisibilityTests(unittest.TestCase):
    def setUp(self):
        self.depth = plane_depth(CAM, CAM.table_z)
        self.cube = scenarios.Item("c", "cube", "red", 0.35, 0.05, 20, True)

    def test_free_cube_is_visible(self):
        self.assertGreater(visibility.visible_fraction(CAM, self.depth, self.cube, CAM.table_z), 0.9)

    def test_cube_under_an_occluder_is_hidden(self):
        x0, y0, x1, y1 = (int(v) for v in labels.project_box(CAM, labels.hull_points(self.cube, CAM.table_z)))
        depth = self.depth.copy()
        depth[y0 - 5:y1 + 5, x0 - 5:x1 + 5] = 0.2  # e.g. the arm, much nearer the camera
        self.assertLess(visibility.visible_fraction(CAM, depth, self.cube, CAM.table_z), 0.05)

    def test_half_occluded_cube_is_partial(self):
        x0, y0, x1, y1 = (int(v) for v in labels.project_box(CAM, labels.hull_points(self.cube, CAM.table_z)))
        depth = self.depth.copy()
        depth[:, : (x0 + x1) // 2] = 0.2
        f = visibility.visible_fraction(CAM, depth, self.cube, CAM.table_z)
        self.assertTrue(0.15 < f < 0.85, f)


class LabelLinesV2Tests(unittest.TestCase):
    def test_occluded_objects_get_no_label(self):
        roi = (0, 0, CAM.W, CAM.H)
        scene = scenarios.sample_v2(4, 5, "table")
        depth = plane_depth(CAM, CAM.table_z)
        free, _ = labels.label_lines_v2(scene, CAM, roi, depth)
        hidden, _ = labels.label_lines_v2(scene, CAM, roi, np.full_like(depth, 0.2))
        self.assertGreater(len(free), 0)
        self.assertEqual(hidden, [])


class CheckLabelsV2Tests(unittest.TestCase):
    def test_partly_hidden_cubes_are_not_judged(self):
        from synth import check_labels
        from synth.test_synth import CheckLabelsTests
        with tempfile.TemporaryDirectory() as tmp:
            chunk = CheckLabelsTests().make_chunk(tmp, shift_px=60)  # every box misaligned
            self.assertFalse(check_labels.check_chunk(chunk)[0])
            for label in (chunk / "labels").glob("*.txt"):
                n = len(label.read_text().splitlines())
                (chunk / "meta").mkdir(exist_ok=True)
                (chunk / "meta" / f"{label.stem}.json").write_text(json.dumps({"visibility": [0.5] * n}))
            ok, rate, results, _ = check_labels.check_chunk(chunk, min_cubes=0)
            self.assertEqual(results, [])  # nothing clearly visible -> nothing judged


class MatTextureTests(unittest.TestCase):
    def test_one_light_texture_per_kind(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = mat_textures.make_textures(tmp, size=256)
            self.assertEqual(len(paths), len(mat_textures.KINDS))
            self.assertEqual(scenarios.MAT_TEXTURES, len(mat_textures.KINDS))
            for path in paths:
                img = cv2.imread(path)
                self.assertEqual(img.shape, (256, 256, 3))
                self.assertGreater(img.mean(), 120)  # light enough for dark-on-light cubes


class SensorFxTests(unittest.TestCase):
    def test_identity_leaves_image_unchanged(self):
        img = np.random.default_rng(0).integers(0, 256, (40, 60, 3), dtype=np.uint8)
        np.testing.assert_array_equal(sensor_fx.apply(img, sensor_fx.IDENTITY, 1), img)

    def test_every_effect_keeps_shape_and_is_reproducible(self):
        img = np.full((90, 160, 3), 128, np.uint8)
        for i in range(30):
            params = scenarios.sample_v2(i, 2, "table").sensor
            out = sensor_fx.apply(img, params, i)
            self.assertEqual((out.shape, out.dtype), (img.shape, img.dtype))
            np.testing.assert_array_equal(out, sensor_fx.apply(img, params, i))


if __name__ == "__main__":
    unittest.main()
