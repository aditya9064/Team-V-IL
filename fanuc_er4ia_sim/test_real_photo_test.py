import unittest

import numpy as np

import real_photo_test as rpt
from backproject import CameraModel


class RealPhotoTestTest(unittest.TestCase):
    def test_table_camera_is_4k_and_orthonormal(self):
        cam = rpt.camera_json("table")
        self.assertEqual(cam["resolution"], [3840, 2160])
        R = np.array(cam["camera_to_world"])[:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)

    def test_camera_looks_at_its_configured_target(self):
        cam = CameraModel(rpt.camera_json("table"))
        cfg = rpt.C.TABLE_CAM
        u, v = cam.project(cfg["look_at"])
        self.assertAlmostEqual(u, cam.W / 2, delta=1.0)
        self.assertAlmostEqual(v, cam.H / 2, delta=1.0)

    def test_lower_resolution_photo_is_scaled_up(self):
        cam = CameraModel(rpt.camera_json("table"))
        img, scale = rpt.fit_to_camera(np.zeros((1080, 1920, 3), np.uint8), cam)
        self.assertEqual(img.shape[:2], (2160, 3840))
        self.assertEqual(scale, 2.0)

    def test_cropped_photo_is_refused(self):
        cam = CameraModel(rpt.camera_json("table"))
        with self.assertRaises(ValueError):
            rpt.fit_to_camera(np.zeros((2160, 3000, 3), np.uint8), cam)

    def test_pick_rows_convert_to_millimetres_and_uf0(self):
        plan = {"picks": [{"colour": "red", "category": "cube", "xy": [0.3, -0.05], "pick_z": -0.0026,
                           "yaw_deg": 12.34, "place_xy": [0.28, 0.1], "stack_level": 4}]}
        row = rpt.pick_rows(plan)[0]
        self.assertEqual((row["pick_x_mm"], row["pick_y_mm"]), (300.0, -50.0))
        self.assertEqual(row["tip_z_uf0_mm"], -332.6)
        self.assertEqual((row["order"], row["stack_level"], row["cube_yaw_deg"]), (1, 4, 12.3))


if __name__ == "__main__":
    unittest.main()
