import json
import unittest

import numpy as np

import perceive_yolo as py
from backproject import CameraModel
from cube_pose import edge_pose
from synth.eval_perception import full_frame, match


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Exactly orthonormal camera_to_world (USD row convention: rows = x, y, z basis, eye)."""
    back = np.asarray(eye, float) - target
    back /= np.linalg.norm(back)
    right = np.cross(up, back)
    right /= np.linalg.norm(right)
    rows = [right, np.cross(back, right), back]
    return [[*r, 0.0] for r in rows] + [[*eye, 1.0]]


CAMERA = {  # 4K table camera above and behind the work area, looking down at it
    "resolution": [3840, 2160], "focal_length_mm": 11.354464530944824,
    "horizontal_aperture_mm": 20.954999923706055, "vertical_aperture_mm": 11.787187576293945,
    "camera_to_world": look_at([0.72, 0.02, 0.64], np.array([0.35, 0.03, 0.0])),
    "table_top_z": 0.0}


class FakeBoxes:
    def __init__(self, xyxy, cls, conf):
        self.xyxy, self.cls, self.conf = (_T(xyxy), _T(cls), _T(conf))


class _T(list):
    def tolist(self):
        return list(self)


class FakeModel:
    def __init__(self, boxes):
        self.boxes, self.seen = boxes, None

    def predict(self, image, **_):
        self.seen = image.shape
        return [type("R", (), {"boxes": self.boxes})()]


class PerceiveYoloTest(unittest.TestCase):
    def setUp(self):
        self.cam = CameraModel(json.loads(json.dumps(CAMERA)))

    def test_side_and_wrist_use_full_frame(self):
        self.assertEqual(py.detector_roi(self.cam, "side"), (0, 0, 3840, 2160))

    def test_table_roi_matches_training_crop_size(self):
        x0, y0, x1, y1 = py.detector_roi(self.cam, "table")
        self.assertEqual((x1 - x0) % 32, 0)
        self.assertTrue(0 <= x0 < x1 <= 3840 and 0 <= y0 < y1 <= 2160)

    def test_detect_maps_crop_boxes_to_full_frame(self):
        model = FakeModel(FakeBoxes([[10, 20, 30, 40]], [2], [0.9]))
        out = py.detect(model, np.zeros((2160, 3840, 3), np.uint8), (100, 200, 1764, 1096), 0.25)
        self.assertEqual(model.seen, (896, 1664, 3))
        self.assertEqual(out, [(2, 0.9, (110, 220, 130, 240))])

    def test_to_object_matches_perceive_format(self):
        obj = py.to_object(4, 0.5, (384, 216, 768, 432), 3840, 2160)
        self.assertEqual((obj["category"], obj["colour"]), ("vial", "clear"))
        np.testing.assert_allclose(obj["center"], [0.15, 0.15])
        np.testing.assert_allclose(obj["bbox"], [0.1, 0.1, 0.2, 0.2])

    def test_top_edge_round_trips_through_edge_pose(self):
        for yaw in (-30.0, 0.0, 17.5, 40.0):
            edge = py.top_edge(self.cam, np.array([0.35, 0.05]), yaw)
            self.assertAlmostEqual(edge_pose(self.cam, edge), yaw, places=4)

    def test_cube_off_the_table_plane_gets_no_pose(self):
        obj = {"category": "cube", "colour": "red", "center": [0.5, 0.0]}
        cam = CameraModel(dict(CAMERA, camera_to_world=[[0, 1, 0, 0], [0, 0, 1, 0],
                                                        [1, 0, 0, 0], [0, 0, 0.5, 1]]))
        py.add_cube_pose(np.zeros((2160, 3840, 3), np.uint8), cam, obj)
        self.assertEqual(obj["pose_confidence"], 0.0)
        self.assertIsNone(obj["top_edge_endpoints"])

    def test_full_frame_pastes_crop_at_roi(self):
        canvas = full_frame(np.full((4, 6, 3), 7, np.uint8), (10, 20, 16, 24), self.cam)
        self.assertEqual(canvas.shape, (2160, 3840, 3))
        self.assertEqual(int(canvas[20:24, 10:16].min()), 7)
        self.assertEqual(int(canvas.sum()), 7 * 4 * 6 * 3)

    def test_match_is_one_to_one_within_25mm(self):
        truths = [{"x": 0.30, "y": 0.00}, {"x": 0.40, "y": 0.00}]
        preds = [{"xy": [0.301, 0.0]}, {"xy": [0.302, 0.0]}, {"xy": [0.50, 0.0]}]
        pairs, fp, fn = match(preds, truths)
        self.assertEqual((len(pairs), fp, fn), (1, 2, 1))
        self.assertAlmostEqual(pairs[0][2], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
