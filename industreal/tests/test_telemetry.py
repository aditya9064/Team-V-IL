import json
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from team_v_industreal.contacts import ContactAudit
from team_v_industreal.telemetry import DiagnosticTrace, measured_sample, peg_metrics


class TelemetryTests(unittest.TestCase):
    def test_rotated_socket_depth_and_tilt_are_not_world_z_metrics(self):
        q = [math.sqrt(.5), 0., math.sqrt(.5), 0.]
        metrics = peg_metrics([1.02, 2., 3.], q, [1., 2., 3.], q)
        self.assertAlmostEqual(metrics['depth_error_m'], .02)
        self.assertAlmostEqual(metrics['radial_error_m'], 0., places=12)
        self.assertAlmostEqual(metrics['tilt_rad'], 0.)
        upright = peg_metrics([0., 0., 0.], [1., 0., 0., 0.], [0., 0., 0.], q)
        self.assertAlmostEqual(upright['tilt_rad'], math.pi/2)

    def test_malformed_nonfinite_or_zero_pose_cannot_be_scored(self):
        for position, quaternion in [([0., 0.], [1., 0., 0., 0.]),
                                     ([float('nan'), 0., 0.], [1., 0., 0., 0.]),
                                     ([0., 0., 0.], [0., 0., 0., 0.])]:
            with self.assertRaises(ValueError):
                peg_metrics(position, quaternion, [0., 0., 0.], [1., 0., 0., 0.])

    def test_auto_reset_and_opening_request_do_not_certify_release(self):
        env = SimpleNamespace(held_pos=[[0., 0., .01]], held_quat=[[1., 0., 0., 0.]],
            fixed_pos=[[0., 0., 0.]], fixed_quat=[[1., 0., 0., 0.]],
            fingertip_midpoint_pos=[[0., 0., .05]], fingertip_midpoint_quat=[[0., 1., 0., 0.]],
            joint_pos=[[0.]*8], ctrl_target_joint_pos=[[.001]*8],
            release_requested=True, contact_audit=ContactAudit())
        row = measured_sample(env, step=1, elapsed_s=1/15, terminated=True)
        self.assertEqual(row['state_phase'], 'AUTO_RESET_POST_STEP_STATE')
        self.assertTrue(row['release_requested'])
        self.assertFalse(row['release_verified'])
        self.assertNotEqual(row['measured_joint_positions'], row['commanded_joint_positions'])

    def test_trace_retains_flushed_samples_before_failure_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'trace.jsonl'
            trace = DiagnosticTrace(path, {'source_sha256': 'example'})
            trace.write({'kind': 'measured_state', 'step': 1})
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(trace.samples, 1)
            with self.assertRaises(FileExistsError):
                DiagnosticTrace(path, {})
            with self.assertRaises(ValueError):
                trace.write({'kind': 'measured_state', 'value': float('nan')})
            self.assertEqual(trace.samples, 1)
            trace.close()
