"""Execute the real nested measurement function; never import Isaac Sim.

Only RGB acquisition/estimation and simulator interfaces are mocked. Actual CAD
foot-gap computation and rigid rotations remain real. This proves the runtime
uses the returned image estimate even when truth-based precision scores fail.
"""
import ast
import copy
import os
from pathlib import Path
import sys
import types
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

DEFAULT_SOURCE = Path(__file__).resolve().parent
if not (DEFAULT_SOURCE / 'nest-mesh.json').exists():
    DEFAULT_SOURCE = Path('/private/tmp/team-v-original-lid-placement-v58b-20261022/snapshot')
SOURCE = Path(os.environ.get('LID_PLACEMENT_SOURCE', str(DEFAULT_SOURCE)))
sys.path.insert(0, str(SOURCE))
from foot_audit import all_foot_gaps
from payload_checks import rotation
import json

GEOMETRY = json.loads((SOURCE / 'nest-mesh.json').read_text())
DESIGN = json.loads((SOURCE / 'nest-design.json').read_text())
RGB_POSITION = np.array([.3, .1, 1.1])


class Attribute:
    def Get(self):
        return 35.


class CameraModel:
    def __getattr__(self, name):
        return lambda: Attribute()


class Transform:
    def ComputeLocalToWorldTransform(self, time):
        return np.eye(4)


def measurement_scope(actual_delta=(0., 0., 0.), actual_angles=(0., 0., 0.), estimates=None, estimator_error=None):
    tree = ast.parse((SOURCE / 'original_lid_placement_trial.py').read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'measure_lid')
    rgb = {'position_world_m': RGB_POSITION.tolist(), 'rotation_world': np.eye(3).tolist()}
    supplied = copy.deepcopy(estimates if estimates is not None else [rgb, rgb])
    used = []

    def estimate(*args):
        if estimator_error is not None:
            raise estimator_error
        value = supplied[len(used)]
        used.append(value)
        return value

    actual_q = Rotation.from_euler('xyz', actual_angles, degrees=True).as_quat()
    actual = {'position_world_m': (RGB_POSITION + actual_delta).tolist(), 'orientation_wxyz': actual_q[[3, 0, 1, 2]].tolist()}
    report = {'incoming_lid_stability': {'reference_freeze_count': 0}}
    state = {'step': 400, 'incoming_lid_capture': True}
    qualified = []
    scope = {
        'np': np, 'Rotation': Rotation, 'geometry': GEOMETRY, 'design': DESIGN,
        'C': types.SimpleNamespace(TABLE_TOP_Z=.9),
        'save': lambda label: {'tip_robot_m': [.3, .1, .2]},
        'cameras': {name: types.SimpleNamespace(get_rgba=lambda: np.zeros((2, 2, 4), dtype=np.uint8)) for name in ['inspection_x', 'inspection_y']},
        'stage': types.SimpleNamespace(GetPrimAtPath=lambda path: path),
        'UsdGeom': types.SimpleNamespace(Camera=lambda prim: CameraModel(), Xformable=lambda prim: Transform()),
        'estimate_lid': estimate, 'report': report, 'state': state, 'persist': lambda: None,
        'world': types.SimpleNamespace(current_time=400/60),
        'poses': lambda: {'cover': copy.deepcopy(actual)}, 'mass_rotation': rotation,
        'all_foot_gaps': all_foot_gaps,
        'fk': types.SimpleNamespace(tool_pose=lambda q, tool: (np.eye(3), np.array([.3, .1, .2]))),
        'arm': types.SimpleNamespace(get_joint_positions=lambda: np.zeros(6)),
        'ai': np.arange(6), 'tool': [.293, 0., 0.],
        'qualify_incoming_hold': lambda label: qualified.append(label),
    }
    exec(compile(ast.Module(body=[node], type_ignores=[]), 'extracted-runtime-measure_lid', 'exec'), scope)
    return scope, actual, used, qualified


class OraclePrecisionAdvisory(unittest.TestCase):
    def check_case(self, delta, angles, failed_key):
        for label in ['3_cover_held_inspection', '3_cover_leveled_inspection']:
            with self.subTest(label=label, precision=failed_key):
                s, actual, used, qualified = measurement_scope(delta, angles)
                result = s['measure_lid'](label)
                self.assertIs(result, used[-1])
                np.testing.assert_array_equal(result['position_world_m'], RGB_POSITION)
                np.testing.assert_array_equal(result['rotation_world'], np.eye(3))
                self.assertIs(s['state']['lid_fk_reference']['estimate'], result)
                self.assertEqual(len(used), 2)
                self.assertTrue(s['report']['lid_pose_repeatability'][-1]['pass'])
                score = s['report']['lid_pose_independent_scores'][-1]
                advisory = score['historical_precision_advisory']
                self.assertIs(advisory[failed_key], False)
                self.assertIs(advisory['terminates_this_diagnostic'], False)
                self.assertIs(advisory['truth_used_for_target_correction'], False)
                if label == '3_cover_leveled_inspection':
                    self.assertEqual(qualified, ['final_measurement_result'])
                    self.assertEqual(s['report']['incoming_lid_stability']['reference_freeze_count'], 1)
                    self.assertEqual(s['state']['lid_drift_reference']['actual_pose'], actual)
                else:
                    self.assertEqual(qualified, [])

    def test_foot_precision_exceedance_continues(self):
        self.check_case((0., 0., .00006), (0., 0., 0.), 'foot_height_pass')

    def test_position_precision_exceedance_continues(self):
        self.check_case((.0002, 0., 0.), (0., 0., 0.), 'position_pass')

    def test_angle_precision_exceedance_continues(self):
        self.check_case((0., 0., 0.), (0., 0., .5), 'orientation_pass')

    def test_combined_precision_exceedance_continues(self):
        self.check_case((.0003, 0., .0002), (.5, 0., .5), 'foot_height_pass')

    def test_observed_v58a_foot_threshold_regression(self):
        self.check_case((0., 0., .0000508526053), (0., 0., 0.), 'foot_height_pass')

    def test_actual_truth_cannot_change_returned_rgb_target(self):
        results = []
        for delta, angles in [((0., 0., .00006), (0., 0., 0.)), ((.0003, 0., .0002), (.5, 0., .5))]:
            s, _, _, _ = measurement_scope(delta, angles)
            result = s['measure_lid']('3_cover_leveled_inspection')
            results.append((result['position_world_m'], result['rotation_world']))
        self.assertEqual(results[0], results[1])

    def test_invalid_image_estimator_rejection_propagates(self):
        s, _, _, _ = measurement_scope(estimator_error=ValueError('Lid image-fit uncertain: synthetic invalid image evidence'))
        with self.assertRaisesRegex(ValueError, 'image-fit uncertain'):
            s['measure_lid']('3_cover_leveled_inspection')
        self.assertNotIn('lid_fk_reference', s['state'])

    def test_two_frame_position_disagreement_still_stops(self):
        base = {'position_world_m': RGB_POSITION.tolist(), 'rotation_world': np.eye(3).tolist()}
        changed = copy.deepcopy(base)
        changed['position_world_m'][0] += .0002
        s, _, _, _ = measurement_scope(estimates=[base, changed])
        with self.assertRaisesRegex(RuntimeError, 'two-frame agreement'):
            s['measure_lid']('3_cover_leveled_inspection')
        self.assertNotIn('lid_fk_reference', s['state'])

    def test_two_frame_angle_disagreement_still_stops(self):
        base = {'position_world_m': RGB_POSITION.tolist(), 'rotation_world': np.eye(3).tolist()}
        changed = copy.deepcopy(base)
        changed['rotation_world'] = Rotation.from_euler('z', .3, degrees=True).as_matrix().tolist()
        s, _, _, _ = measurement_scope(estimates=[base, changed])
        with self.assertRaisesRegex(RuntimeError, 'two-frame agreement'):
            s['measure_lid']('3_cover_leveled_inspection')
        self.assertNotIn('lid_fk_reference', s['state'])


if __name__ == '__main__':
    unittest.main()
