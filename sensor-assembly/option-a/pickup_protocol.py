"""Pure, finite, authored-CAD pickup recipe. Never reads simulator poses."""
import json
import math
from pathlib import Path
import numpy as np

PARTS = ('base', 'carrier', 'cover')
DT = 1 / 60
LIMITS = {'endpoint_mm': 2., 'endpoint_samples': 6,
          'camera_speed_rad_s': .02, 'camera_samples': 6,
          'minimum_lift_mm': 35., 'maximum_pickup_xy_slip_mm': 2.,
          'maximum_pickup_tilt_deg': 5., 'maximum_pickup_yaw_deg': 3.,
          'maximum_intended_penetration_mm': .25}
SDK_PINS = {'cell.py': 'cfde65b149d4c87add24793734d65d968b79ea70563c169e2f46b458b32a4260',
 'scene_config.py': '6d5d0c7df867986e78ff02b2888236dca6c00cc3bc6b7ecf60f9a49d75facc1b',
 'gripper_geometry.py': '5ee8ff8c87d9a44f96f8af738b46fd9d2a586fa2ad3b8e0968c86161355007dc',
 'ik.py': '6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795',
 'fk.py': 'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed'}


def protocol(design, part):
    if part not in PARTS or design['scale'] != 2 or design['gripper_changed']:
        raise ValueError('Approved CAD/part scope differs')
    positions = {'base': [.58, .32, .8952], 'carrier': [.30, .36, .8952],
                 'cover': [.62, -.35, .8952], 'nest': [.10, .63, .8949]}
    positions[part] = [.36, -.12, .8952]
    positions['pcb'] = [*positions['carrier'][:2], positions['carrier'][2] + .004]
    grip = design['grip'][part]
    home = np.array([.30, -.10, -.0051 + .16])
    contact = np.array([.36, -.12, -.0051]) + np.array([
        *grip['handle_centre_xy_mm'], grip['tcp_above_part_bottom_mm']]) * .001
    offset = np.array([grip['side_entry_mm'] * .001, 0, 0])
    targets = [home, contact + offset + [0, 0, .065], contact + offset,
               contact, contact, contact + [0, 0, .04]]
    roles = ['orient', 'above', 'approach', 'side_entry', 'close', 'lift']
    durations = [1., 2., 2., 1.5, 3., 1.5]
    phases = [{'name': part + '_' + role, 'role': role, 'duration_s': duration,
               'target_robot_m': goal.tolist(), 'jaw_state': 'closed' if role in ('close', 'lift') else 'open'}
              for role, duration, goal in zip(roles, durations, targets)]
    spawns = {name: {'position_world_m': xyz, 'orientation_wxyz': [1., 0, 0, 0]}
              for name, xyz in positions.items()}
    return {'schema': 'option_a_known_fixture_pickup_v1', 'selected_part': part, 'phases': phases, 'hold_phase': part + '_hold',
            'inspection_phase': part + '_hold', 'hold_checkpoint': part + '_hold_end',
            'hold_steps': 120, 'hold_seconds': 2., 'lift_command_mm': 40.,
            'home_robot_m': home.tolist(), 'source_body_robot_m': [.36, -.12, -.0051],
            'world_table_z': .9, 'tool_offset_m': [.293, 0., 0.],
            'jaw_targets': {'open': {'jaw_l': 0., 'jaw_r': 0.},
                            'closed': {'jaw_l': -.006, 'jaw_r': .006}},
            'authored_spawn': spawns, 'limits': dict(design['limits_unchanged']),
            'payload_floor_m': .004, 'payload_limits': dict(design['payload_limits_unchanged']),
            'fixture_spawn_gap_m': .0003, 'known_fixture_targets': True,
            'simulator_truth_used_for_targets': False, 'camera_guided': False,
            'pixel_freshness_certified': False, 'physical_hardware_test': False}


def finite_vector(value, length):
    a = np.asarray(value, float)
    if a.shape != (length,) or not np.isfinite(a).all():
        raise ValueError('Malformed finite vector')
    return a


def pickup_score(initial, current):
    a = finite_vector(initial['position_world_m'], 3)
    b = finite_vector(current['position_world_m'], 3)
    value = {'lift_mm': float((b[2]-a[2])*1000),
             'xy_slip_mm': float(np.linalg.norm(b[:2]-a[:2])*1000),
             'tilt_deg': float(current['tilt_deg']), 'yaw_deg': abs(float(current['yaw_deg']))}
    if not all(math.isfinite(v) for v in value.values()):
        raise ValueError('Nonfinite pickup score')
    value['pass'] = (value['lift_mm'] >= LIMITS['minimum_lift_mm'] and
                     value['xy_slip_mm'] <= LIMITS['maximum_pickup_xy_slip_mm'] and
                     value['tilt_deg'] <= LIMITS['maximum_pickup_tilt_deg'] and
                     value['yaw_deg'] <= LIMITS['maximum_pickup_yaw_deg'])
    return value
