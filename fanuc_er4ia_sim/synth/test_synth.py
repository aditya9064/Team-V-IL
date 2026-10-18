"""Scene sampling, projected labels and dataset splitting (host-only)."""

import json, math, tempfile, unittest
from collections import Counter
from pathlib import Path

import numpy as np

import scene_config as C
from test_support import camera_model
import cv2

from synth import check_labels, labels, layout, split_dataset

CFG = layout.Randomization()
CAM_4K = camera_model(dict(C.TABLE_CAM, resolution=(3840, 2160)))


def parse(line):
    cls, *vals = line.split()
    return int(cls), [float(v) for v in vals]


class LayoutTests(unittest.TestCase):
    def test_same_index_is_reproducible(self):
        self.assertEqual(layout.sample_scene(7, 1), layout.sample_scene(7, 1))
        self.assertNotEqual(layout.sample_scene(7, 1), layout.sample_scene(8, 1))

    def test_visible_objects_stay_in_range_and_apart(self):
        for i in range(200):
            discs = []
            for obj in layout.sample_scene(i, 3).objects:
                if not obj.visible:
                    continue
                self.assertTrue(CFG.x_range[0] <= obj.x <= CFG.x_range[1])
                self.assertTrue(CFG.y_range[0] <= obj.y <= CFG.y_range[1])
                own = layout.footprint_discs(obj.category, obj.x, obj.y, obj.yaw_deg)
                for a in own:
                    for b in discs:
                        gap = math.dist(a[:2], b[:2]) - a[2] - b[2]
                        self.assertGreaterEqual(gap, CFG.separation_margin - 1e-9)
                discs.extend(own)

    def test_colour_counts_vary_between_scenes(self):
        counts = set()
        for i in range(100):
            cubes = [o for o in layout.sample_scene(i, 5).objects if o.category == "cube"]
            counts.add(tuple(sorted(Counter(o.colour for o in cubes if o.visible).items())))
        self.assertGreater(len(counts), 10)


class LabelTests(unittest.TestCase):
    def test_cube_box_matches_projected_corners(self):
        cube = layout.ObjectPose("SortCube0", "cube", "red", 0.35, 0.05, 30.0, True)
        x0, y0, x1, y1 = labels.project_box(CAM_4K, labels.hull_points(cube, CAM_4K.table_z))
        centre = CAM_4K.project([0.35, 0.05, CAM_4K.table_z + C.CUBE_EDGE / 2])
        self.assertTrue(x0 < centre[0] < x1 and y0 < centre[1] < y1)
        self.assertTrue(20 < x1 - x0 < 50 and 20 < y1 - y0 < 50)

    def test_class_ids_follow_colour_and_category(self):
        make = lambda cat, colour: layout.ObjectPose("n", cat, colour, 0, 0, 0, True)
        self.assertEqual(labels.class_id(make("cube", "blue")), 2)
        self.assertEqual(labels.class_id(make("cap", "yellow")), 3)
        self.assertEqual(labels.class_id(make("vial", "clear")), 4)

    def test_mostly_cropped_or_tiny_boxes_are_dropped(self):
        roi = (100, 100, 500, 400)
        self.assertIsNone(labels.yolo_line((80, 150, 110, 180), roi, 0))
        self.assertIsNone(labels.yolo_line((200, 200, 202, 230), roi, 0))
        cls, vals = parse(labels.yolo_line((200, 200, 240, 240), roi, 1))
        self.assertEqual(cls, 1)
        np.testing.assert_allclose(vals, [0.3, 0.4, 0.1, 40 / 300], atol=1e-6)

    def test_roi_is_aligned_and_inside_image(self):
        x0, y0, x1, y1 = labels.workspace_roi(CAM_4K, CFG.x_range, CFG.y_range)
        self.assertEqual(((x1 - x0) % 32, (y1 - y0) % 32), (0, 0))
        self.assertTrue(0 <= x0 < x1 <= CAM_4K.W and 0 <= y0 < y1 <= CAM_4K.H)

    def test_every_visible_cube_gets_a_valid_label(self):
        roi = labels.workspace_roi(CAM_4K, CFG.x_range, CFG.y_range)
        for i in range(100):
            sample = layout.sample_scene(i, 9)
            lines = labels.label_lines(sample, CAM_4K, roi)
            cubes = sum(o.visible and o.category == "cube" for o in sample.objects)
            self.assertGreaterEqual(len(lines), cubes)
            for line in lines:
                cls, vals = parse(line)
                self.assertIn(cls, range(len(labels.CLASS_NAMES)))
                self.assertTrue(all(0.0 < v <= 1.0 for v in vals), line)


class SplitTests(unittest.TestCase):
    def make_chunk(self, root, name, stems):
        chunk = Path(root) / name
        for sub in ("images", "labels"):
            (chunk / sub).mkdir(parents=True)
        for stem in stems:
            (chunk / "images" / f"{stem}.png").write_bytes(b"png")
            (chunk / "labels" / f"{stem}.txt").write_text("0 0.5 0.5 0.1 0.1\n")
        manifest = {"classes": list(labels.CLASS_NAMES)}
        (chunk / "dataset.json").write_text(json.dumps(manifest))
        return chunk

    def test_split_is_stable_and_roughly_proportional(self):
        stems = [f"{i:06d}" for i in range(2000)]
        first = [split_dataset.split_of(s, 0.15) for s in stems]
        self.assertEqual(first, [split_dataset.split_of(s, 0.15) for s in stems])
        self.assertAlmostEqual(first.count("val") / len(stems), 0.15, delta=0.03)

    def test_build_links_frames_and_writes_yaml(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = self.make_chunk(tmp, "a", [f"{i:06d}" for i in range(40)])
            b = self.make_chunk(tmp, "b", [f"{i:06d}" for i in range(40, 80)])
            out = Path(tmp) / "ds"
            counts = split_dataset.build([a, b], out, 0.2, link=True)
            self.assertEqual(sum(counts.values()), 80)
            yaml = (out / "data.yaml").read_text()
            self.assertIn("4: vial", yaml)
            images = list((out / "images").rglob("*.png"))
            labels_ = list((out / "labels").rglob("*.txt"))
            self.assertEqual((len(images), len(labels_)), (80, 80))

    def test_max_frames_subsets_nest_with_same_split(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = self.make_chunk(tmp, "a", [f"{i:06d}" for i in range(60)])
            small, large = Path(tmp) / "s", Path(tmp) / "l"
            self.assertEqual(sum(split_dataset.build([a], small, 0.2, True, 20).values()), 20)
            split_dataset.build([a], large, 0.2, True, 50)
            for img in (small / "images").rglob("*.png"):
                self.assertTrue((large / img.relative_to(small)).is_symlink())

    def test_interrupted_frame_without_label_is_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = self.make_chunk(tmp, "a", ["000001", "000002"])
            (a / "labels" / "000002.txt").unlink()
            counts = split_dataset.build([a], Path(tmp) / "ds", 0.2, link=True)
            self.assertEqual(sum(counts.values()), 1)

    def test_duplicate_frames_across_chunks_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = self.make_chunk(tmp, "a", ["000001"])
            b = self.make_chunk(tmp, "b", ["000001"])
            with self.assertRaises(ValueError):
                split_dataset.build([a, b], Path(tmp) / "ds", 0.2, link=True)


class CheckLabelsTests(unittest.TestCase):
    BGR = {"cube_red": (0, 0, 200), "cube_yellow": (0, 210, 230), "cube_blue": (200, 60, 0)}

    def make_chunk(self, root, shift_px=0):
        roi = labels.workspace_roi(CAM_4K, CFG.x_range, CFG.y_range)
        chunk = Path(root)
        for sub in ("images", "labels"):
            (chunk / sub).mkdir(parents=True, exist_ok=True)
        for i in range(8):
            sample = layout.sample_scene(i, 11)
            img = np.full((roi[3] - roi[1], roi[2] - roi[0], 3), 230, np.uint8)
            for obj in sample.objects:
                if obj.visible and obj.category == "cube":
                    uv = [CAM_4K.project(p) for p in labels.hull_points(obj, CAM_4K.table_z)]
                    pts = (np.array(uv) - roi[:2]).astype(np.int32)
                    colour = self.BGR[labels.CLASS_NAMES[labels.class_id(obj)]]
                    cv2.fillConvexPoly(img, cv2.convexHull(pts), colour)
            cv2.imwrite(str(chunk / "images" / f"{i:06d}.png"), img)
            lines = labels.label_lines(sample, CAM_4K, roi)
            if shift_px:
                w = roi[2] - roi[0]
                lines = [
                    " ".join([p[0], f"{float(p[1]) + shift_px / w:.6f}", *p[2:]])
                    for p in (l.split() for l in lines)
                ]
            (chunk / "labels" / f"{i:06d}.txt").write_text("\n".join(lines) + "\n")
        manifest = {"classes": list(labels.CLASS_NAMES), "roi": list(roi)}
        (chunk / "dataset.json").write_text(json.dumps(manifest))
        return chunk

    def test_aligned_labels_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            ok, rate, results, problems = check_labels.check_chunk(self.make_chunk(tmp))
            self.assertTrue(ok, (rate, problems))
            self.assertGreaterEqual(len(results), 20)

    def test_shifted_labels_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            ok, rate, _, _ = check_labels.check_chunk(self.make_chunk(tmp, shift_px=60))
            self.assertFalse(ok)
            self.assertLess(rate, 0.5)


if __name__ == "__main__":
    unittest.main()
