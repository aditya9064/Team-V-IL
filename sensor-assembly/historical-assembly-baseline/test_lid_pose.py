import inspect
import json
from pathlib import Path
import unittest
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from lid_pose import estimate, render_mask, placement_from_held
from offline_overlay import load_inputs

EVIDENCE = Path(__file__).parent/'fit-fixtures'/'286486'
MESH = Path(__file__).with_name('nest-mesh.json')


def feet(mesh):
    # Actual CAD support vertices, not hypothetical AABB floor corners.
    v = np.asarray(mesh['vertices_mm'], float)*.001
    return v[np.isclose(v[:, 2], 0)]


def max_corrected_foot_error(fit, truth_p, truth_R, mesh):
    p, R = np.asarray(fit['position_world_m']), np.asarray(fit['rotation_world'])
    actual = np.einsum('ij,kj->ik', feet(mesh), truth_R)+truth_p
    corrected = np.einsum('ij,jk->ik', actual-p, R)
    return float(np.max(np.abs(corrected[:, 2]))*1000)


class LidPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.images, cls.models, cls.mesh, cls.nominal = load_inputs(EVIDENCE, MESH)

    def test_fit_signature_accepts_no_truth(self):
        self.assertEqual(list(inspect.signature(estimate).parameters), ['images', 'models', 'visual_mesh', 'nominal_position'])

    def test_actual_archived_rgb_and_same_input_determinism(self):
        # Truth is read only AFTER the visual estimate has been produced.
        first = estimate(self.images, self.models, self.mesh, self.nominal)
        second = estimate(self.images, self.models, self.mesh, self.nominal)
        np.testing.assert_array_equal(first['position_world_m'], second['position_world_m'])
        np.testing.assert_array_equal(first['rotation_world'], second['rotation_world'])
        report = json.loads((EVIDENCE/'report.json').read_text())
        truth = next(c for c in report['checkpoints'] if c['phase']=='3_cover_carry')['poses']['cover']
        truth_R = Rotation.from_quat(np.roll(truth['orientation_wxyz'], -1)).as_matrix()
        self.assertLess(max_corrected_foot_error(first, np.asarray(truth['position_world_m']), truth_R, self.mesh), .05)
        self.assertFalse(first['uncertainty_diagnostics']['clearance_qualified'])

    def test_actual_cad_rendered_variants(self):
        for offset, angles in [([.35, -.2, -.2], [-.2, -.4, -.2]), ([-.4, .3, -.5], [1.2, -1., .4])]:
            with self.subTest(offset=offset):
                p = self.nominal+np.asarray(offset)*.001
                R = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
                images = {}
                for name, model in self.models.items():
                    mask = render_mask(self.mesh, p, R, model)
                    image = np.full((*mask.shape, 3), 180, np.uint8)
                    image[mask>0] = [150, 175, 190]
                    images[name] = image
                fit = estimate(images, self.models, self.mesh, self.nominal)
                self.assertLess(max_corrected_foot_error(fit, p, R, self.mesh), .05)

    def test_blank_and_missing_view_rejected(self):
        blank = {n: np.full_like(im, 180) for n, im in self.images.items()}
        with self.assertRaises(ValueError): estimate(blank, self.models, self.mesh, self.nominal)
        with self.assertRaises(ValueError): estimate({'inspection_x': self.images['inspection_x']}, self.models, self.mesh, self.nominal)

    def test_same_camera_rejected(self):
        models = {n: self.models['inspection_x'] for n in self.models}
        with self.assertRaises(ValueError): estimate(self.images, models, self.mesh, self.nominal)

    def test_obscured_feet_rejected(self):
        images = {n: im.copy() for n, im in self.images.items()}
        for im in images.values(): im[1000:, :, :] = 180
        with self.assertRaises(ValueError): estimate(images, self.models, self.mesh, self.nominal)

    def test_wrong_cad_rejected(self):
        wrong = json.loads(MESH.read_text())['parts']['base']['visual']
        with self.assertRaises(ValueError): estimate(self.images, self.models, wrong, self.nominal)

    def test_rigid_compensation_on_non_nominal_tool(self):
        p = self.nominal; R = Rotation.from_euler('xyz', [.4, -.3, .2], degrees=True).as_matrix()
        T = Rotation.from_euler('xyz', [2., 89., 1.], degrees=True).as_matrix(); tip = p+[0, 0, .0375]
        dest = np.array([.3, .1, .9009]); fit = placement_from_held(p, R, tip, T, dest, self.mesh['vertices_mm'])
        newT = np.asarray(fit['tool_rotation_world']); local = T.T@(p-tip)
        np.testing.assert_allclose(np.asarray(fit['tip_world_m'])+newT@local, dest+[0, 0, .0001], atol=1e-12)
        np.testing.assert_allclose(newT@(T.T@R), np.eye(3), atol=1e-12)


if __name__ == '__main__': unittest.main()
