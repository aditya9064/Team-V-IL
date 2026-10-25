import copy
import math
from pathlib import Path
import unittest

from foot_audit import (all_foot_gaps, audit_raw_contacts, error_budget_at_feet,
                        load_mesh, predict_from_held, qualify_prediction,
                        quaternion_rotation, rigid_rotation, usd_matrix_pose,
                        tcp_tracking_bounds_at_feet, observed_tcp_tracking_foot_errors,
                        direct_error_budget)

MESH = str(Path(__file__).with_name('nest-mesh.json'))
IDENTITY = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]


def pose(p, r=IDENTITY):
    return {'position_world_m': list(p), 'rotation_world': r}


class FootAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mesh, cls.digest = load_mesh(MESH)

    def test_all_four_actual_cad_feet_level_positive_gap(self):
        result = all_foot_gaps(self.mesh, pose([0, 0, .0031]), pose([0, 0, 0]))
        self.assertEqual(len(result['feet']), 4)
        self.assertTrue(result['all_feet_over_pads'])
        for foot in result['feet']:
            self.assertAlmostEqual(foot['minimum_gap_mm'], .1, places=10)
            self.assertAlmostEqual(foot['minimum_pad_edge_margin_mm'], 4.75, places=10)

    def test_roll_analytic_opposite_feet_and_worst_corner(self):
        theta = math.radians(.6)
        r = [[1, 0, 0], [0, math.cos(theta), -math.sin(theta)],
             [0, math.sin(theta), math.cos(theta)]]
        result = all_foot_gaps(self.mesh, pose([0, 0, .0031], r), pose([0, 0, 0]))
        expected = .1-21.25*math.sin(theta)
        self.assertAlmostEqual(result['minimum_gap_mm'], expected, places=10)
        self.assertLess(result['feet'][0]['maximum_gap_mm'], 0)
        self.assertGreater(result['feet'][1]['minimum_gap_mm'], 0)

    def test_global_translation_invariance(self):
        a = all_foot_gaps(self.mesh, pose([0, 0, .0031]), pose([0, 0, 0]))
        b = all_foot_gaps(self.mesh, pose([.3, -.2, .9031]), pose([.3, -.2, .9]))
        self.assertAlmostEqual(a['minimum_gap_mm'], b['minimum_gap_mm'], places=10)

    def test_global_rotation_invariance(self):
        r = quaternion_rotation([math.cos(.2), 0, math.sin(.2), 0])
        p = [r[i][2]*.0031 for i in range(3)]
        result = all_foot_gaps(self.mesh, pose(p, r), pose([0, 0, 0], r))
        self.assertAlmostEqual(result['minimum_gap_mm'], .1, places=10)

    def test_gap_alone_cannot_approve_missed_pad(self):
        result = all_foot_gaps(self.mesh, pose([.005, 0, .0031]), pose([0, 0, 0]))
        self.assertAlmostEqual(result['minimum_gap_mm'], .1, places=10)
        self.assertFalse(result['all_feet_over_pads'])
        self.assertFalse(qualify_prediction(result, {'maximum_height_error_mm': 0})['geometric_dispatch_eligible'])

    def test_rigid_prediction_preserves_true_held_offset(self):
        # A 0.2 mm longer held offset must not disappear into a nominal CAD TCP.
        result = predict_from_held(pose([0, 0, .9628]), [0, 0, 1.], IDENTITY,
                                   [0, 0, .0403], IDENTITY)
        self.assertAlmostEqual(result['position_world_m'][2], .0031, places=12)

    def test_quarter_degree_rotation_alone_exceeds_seating_gap(self):
        budget = error_budget_at_feet(self.mesh, [0, 0, 0], .25, 0, 0, 0, 0)
        self.assertGreater(budget['maximum_height_error_mm'], .14)
        gaps = all_foot_gaps(self.mesh, pose([0, 0, .0031]), pose([0, 0, 0]))
        self.assertFalse(qualify_prediction(gaps, budget)['geometric_dispatch_eligible'])

    def test_exact_budget_boundary_rejects(self):
        gaps = all_foot_gaps(self.mesh, pose([0, 0, .0031]), pose([0, 0, 0]))
        result = qualify_prediction(gaps, {'maximum_height_error_mm': gaps['minimum_gap_mm']})
        self.assertEqual(result['positive_clearance_lower_bound_mm'], 0)
        self.assertFalse(result['geometric_dispatch_eligible'])

    def test_shear_with_determinant_one_rejects(self):
        with self.assertRaises(ValueError):
            rigid_rotation([[1, .01, 0], [0, 1, 0], [0, 0, 1]])

    def test_missing_or_negative_budget_rejects(self):
        with self.assertRaises(TypeError):
            error_budget_at_feet(self.mesh, [0, 0, 0], .01, 0, 0)
        with self.assertRaises(ValueError):
            error_budget_at_feet(self.mesh, [0, 0, 0], .01, -1, 0, 0, 0)

    def test_tcp_rotation_uses_full_tcp_to_foot_lever(self):
        result = tcp_tracking_bounds_at_feet(self.mesh, [0, 0, -.0374252], IDENTITY, .032279, .0176004)
        self.assertGreater(result['feet'][0]['foot_to_tcp_radius_mm'], 49)
        self.assertGreater(result['maximum_height_error_mm'], .047)
        self.assertLess(result['maximum_height_error_mm'], .048)

    def test_exact_observed_tcp_tracking_z_error(self):
        result = observed_tcp_tracking_foot_errors(self.mesh, pose([0, 0, .0031]),
            [0, 0, .0405], IDENTITY, [0, 0, .04047], IDENTITY, pose([0, 0, 0]))
        self.assertAlmostEqual(result['maximum_absolute_height_error_mm'], .03, places=10)
        self.assertAlmostEqual(result['maximum_position_error_mm'], .03, places=10)

    def test_horizontal_fk_error_cannot_hide_in_height_only(self):
        result = observed_tcp_tracking_foot_errors(self.mesh, pose([0, 0, .0031]),
            [0, 0, .0405], IDENTITY, [.0002, 0, .0405], IDENTITY, pose([0, 0, 0]))
        self.assertAlmostEqual(result['maximum_absolute_height_error_mm'], 0, places=10)
        self.assertAlmostEqual(result['maximum_position_error_mm'], .2, places=10)

    def test_bounded_precontact_budget_cannot_qualify_final_gap(self):
        radius = math.hypot(25.25, 21.25)
        fit_xy = .15+2*radius*math.sin(math.radians(.25)/2)
        budget = direct_error_budget(self.mesh, .05, 0, .02, .35, 1., .05,
                                     .75, fit_xy+.05+.02,
                                     {'scope': 'simulation exact camera; empirical two-frame fit; stricter live vetoes required'})
        final = all_foot_gaps(self.mesh, pose([0, 0, .0031]), pose([0, 0, 0]))
        hover = all_foot_gaps(self.mesh, pose([0, 0, .0041]), pose([0, 0, 0]))
        self.assertGreater(budget['maximum_height_error_mm'], 1.06)
        self.assertLess(budget['maximum_height_error_mm'], 1.07)
        self.assertFalse(qualify_prediction(final, budget)['geometric_dispatch_eligible'])
        self.assertTrue(qualify_prediction(hover, budget)['geometric_dispatch_eligible'])
        self.assertGreater(budget['support_pad_angular_displacement_mm'], 1.38)
        self.assertLess(budget['support_pad_angular_displacement_mm'], 1.40)

    def test_support_yaw_reduces_horizontal_margin_not_height_budget(self):
        a = direct_error_budget(self.mesh, .05, 0, .02, .35, 1., .05, .75, .364,
                                {'scope': 'test'}, support_yaw_deg=0)
        b = direct_error_budget(self.mesh, .05, 0, .02, .35, 1., .05, .75, .364,
                                {'scope': 'test'}, support_yaw_deg=1)
        self.assertEqual(a['maximum_height_error_mm'], b['maximum_height_error_mm'])
        self.assertGreater(b['maximum_horizontal_error_mm'], a['maximum_horizontal_error_mm'])

    def test_uncertainty_can_invalidate_pad_containment(self):
        gaps = all_foot_gaps(self.mesh, pose([.0047, 0, .0041]), pose([0, 0, 0]))
        self.assertTrue(gaps['all_feet_over_pads'])
        self.assertFalse(qualify_prediction(gaps, {'maximum_height_error_mm': .01,
                                                  'maximum_horizontal_error_mm': .1})['geometric_dispatch_eligible'])

    def test_fake_cad_support_and_wrong_hash_reject(self):
        mesh = copy.deepcopy(self.mesh)
        mesh['parts']['cover']['colliders'][5]['vertices_mm'][0][2] += .01
        with self.assertRaises(ValueError):
            all_foot_gaps(mesh, pose([0, 0, .0031]), pose([0, 0, 0]))
        with self.assertRaises(ValueError):
            load_mesh(MESH, '0'*64)

    def test_usd_row_matrix_convention(self):
        value = usd_matrix_pose([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [.3, .1, .9, 1]])
        self.assertEqual(value, pose([.3, .1, .9]))

    def test_raw_failure_survives_pre_post_reconstruction(self):
        # Distinct times must be reported, never select the one closest to raw.
        def snap(step, z):
            return {'step': step, 'actor_world_matrices': {
                '/World/Assembly_base': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                '/World/Assembly_cover': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, z, 1]]}}
        report = {'limits': {'maximum_intended_penetration_mm': .25},
                  'unexpected_contacts': [{'step': 42, 'phase': 'lower',
                    'colliders': ['/World/Assembly_cover/C_005_ear', '/World/Assembly_base/C_009_floor'],
                    'minimum_separation_mm': -.26}],
                  'contact_timing_diagnostics': {'pre_step_trace': [snap(42, .00275)],
                                                  'post_step': snap(43, .00272)}}
        result = audit_raw_contacts(self.mesh, report)['contacts'][0]
        self.assertTrue(result['original_raw_failure_retained'])
        self.assertEqual(len(result['timed_reconstructions']), 2)
        self.assertAlmostEqual(result['timed_reconstructions'][0]['foot_plane_depth_mm'], .25)
        self.assertAlmostEqual(result['timed_reconstructions'][1]['foot_plane_depth_mm'], .28)


if __name__ == '__main__':
    unittest.main()
