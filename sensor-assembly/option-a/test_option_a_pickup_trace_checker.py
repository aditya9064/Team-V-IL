"""Synthetic checker controls only. These are not native simulation reports."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import zlib
import struct
import subprocess
import shutil

import numpy as np

import option_a_pickup_trace_checker as C

HERE = Path(__file__).resolve().parent
CAD = HERE if (HERE/'design.json').is_file() else HERE
SDK = HERE/'pinned-sdk' if (HERE/'pinned-sdk').is_dir() else CAD.parent/'pinned-sdk'
assert hashlib.sha256((SDK/'ik.py').read_bytes()).hexdigest() == '6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'
sys.path.insert(0, str(SDK))
import ik


def mkpose(p):
    return {'position_world_m': list(map(float, p)), 'orientation_wxyz': [1., 0., 0., 0.]}


def raw_matrix(p):
    point, rot = C.pose(p)
    m = np.eye(4)
    m[:3, :3] = rot.T
    m[3, :3] = point
    return m.tolist()


def refresh_payload(r):
    scores = [C.payload_metrics(x['poses']) for x in r['full_motion_observations']]
    keys = {'xy_mm': 'maximum_xy_mm', 'z_mm': 'maximum_z_mm',
            'tilt_deg': 'maximum_relative_tilt_deg', 'yaw_deg': 'maximum_relative_yaw_deg'}
    r['payload_motion_audit'] = {'samples': len(scores), **{v: max(x[k] for x in scores) for k, v in keys.items()}}


def update_raw_pose(r, index, part):
    row = r['full_motion_observations'][index]
    post = r['diagnostic_trace'][index+1] if index+1 < len(r['full_motion_observations']) else r['full_motion_last_snapshot']
    post['actor_world_matrices']['/World/Assembly_'+part] = raw_matrix(row['poses'][part])


def fixture(part='base'):
    d = json.loads((CAD/'design.json').read_text())
    names = [part+'_'+x for x in C.ROLES]
    spec = {'selected_part': part, 'phases': names, 'phase_roles': C.ROLES,
            'durations_s': C.DURATIONS, 'hold_phase': part+'_hold',
            'hold_checkpoint': part+'_hold_camera',
            'source_body_robot_m': [.36, -.12, -.0051], 'home_robot_m': [.3, -.1, .1549]}
    protocol = {'schema': 'option_a_known_fixture_pickup_v1', 'cases': {part: spec},
                'binary_jaw_joint_targets_m': {'open': {'jaw_l': 0., 'jaw_r': 0.},
                                              'closed': {'jaw_l': -.006, 'jaw_r': .006}}}
    # The synthetic rows need only test evidence contracts; instantaneous ideal
    # pose changes below make no claim about a physically achievable trajectory.
    poses = {n: mkpose([.65, .2, .8949]) for n in C.BODIES}
    poses[part] = mkpose([.36, -.12, .8949])
    if part == 'carrier':
        poses['pcb'] = mkpose([.36, -.12, .8989])
    else:
        poses['pcb'] = mkpose([.65, .2, .8989])
    q = np.deg2rad([0, 35, -25, 0, -80, 0])
    solutions = []
    for goal in C.goals_for(d, spec):
        q, ok = ik.solve(q, goal, [0, 0, -1], C.JOINT_LIMITS,
                         iters=500, tool_offset=C.TOOL_OFFSET, spread_dir=[0, 1, 0])
        assert ok
        solutions.append(np.asarray(q).tolist())
    r = {'selected_part': part, 'status': 'DIAGNOSTIC_PASS',
         'limits': d['limits_unchanged'], 'payload_limits': d['payload_limits_unchanged'],
         'physics': {'solver': 'PGS', 'physics_dt': C.DT, 'render_dt': 1/30,
                     'position_iterations': 32, 'velocity_iterations': 1},
         'kinematic_binding': {'arm_joint_names': ['J'+str(i) for i in range(1, 7)],
                               'arm_joint_indices': list(range(6)), 'joint_dtype': 'float32',
                               'tool_offset_m': C.TOOL_OFFSET, 'table_top_z_m': C.TABLE_Z},
         'pose_readback_dtypes': {n: {'position': 'float64', 'orientation': 'float64'} for n in C.BODIES},
         'pose_readback_dtype_errors': [], 'full_motion_observations': [],
         'pose_readback_dtype_capture_counts': {},
         'diagnostic_trace': [], 'checkpoints': [], 'endpoint_traces': {},
         'motion_sampling': {'command_hz': 60, 'physics_steps_per_command': 1, 'phases': []}}
    rows = r['full_motion_observations']

    def add(phase, command, count, closed=False):
        for _ in range(count):
            rot, tip = C.independent_fk(command)
            step = len(rows)+1
            rows.append({'phase': phase, 'step': step, 'sim_time': step/60,
                         'actual_q_rad': command+[0., 0.], 'actual_qdot_rad_s': [0.]*8,
                         'issued_arm_q_rad': command,
                         'issued_jaw_targets_m': protocol['binary_jaw_joint_targets_m']['closed' if closed else 'open'],
                         'tip_world_m': tip.tolist(), 'tool_rotation_world': rot.tolist(),
                         'poses': copy.deepcopy(poses)})

    def shot(name):
        step = len(rows)
        r['checkpoints'].append({'phase': name, 'step': step, 'sim_time': step/60,
                                 'poses': copy.deepcopy(rows[-1]['poses']),
                                 'stillness_trace': [{'step': s, 'sim_time': s/60,
                                                      'maximum_raw_joint_speed_rad_s': 0.}
                                                     for s in range(step-10, step+1, 2)]})

    add('initial_settling', solutions[0], 132)
    shot('initial')
    for i, (name, duration, command) in enumerate(zip(names, C.DURATIONS, solutions)):
        if i == 5:
            poses[part]['position_world_m'][2] += .04
            if part == 'carrier':
                poses['pcb']['position_world_m'][2] += .04
        add(name, command, round(duration*60)+18, closed=i >= 4)
        entries = []
        for row in rows[-18:-12]:
            rot = np.asarray(row['tool_rotation_world'])
            entries.append({'step': row['step'],
                            'tip_error_mm': float(np.linalg.norm(np.asarray(row['tip_world_m'])-[0, 0, .9]-C.goals_for(d, spec)[i])*1000),
                            'tool_axis_error_deg': C.angle(rot[:, 0] @ [0, 0, -1]),
                            'spread_axis_error_deg': C.angle(rot[:, 1] @ [0, 1, 0])})
        r['endpoint_traces'][name] = entries
        r['motion_sampling']['phases'].append({'phase': name, 'requested_duration_s': duration,
                                               'commands': round(duration*60), 'actual_duration_s': duration,
                                               'maximum_command_interval_s': 1/60})
        shot(name)
    start = len(rows)
    add(spec['hold_phase'], solutions[-1], 132, closed=True)
    shot(spec['hold_checkpoint'])
    r['pickup_hold'] = {'requested_steps': 120, 'requested_duration_s': 2.,
                        'start_step': start, 'end_step': start+120,
                        'start_sim_time': start/60, 'end_sim_time': (start+120)/60}
    for i, row in enumerate(rows):
        prev = rows[max(0, i-1)]
        r['diagnostic_trace'].append({'step': i+1, 'sim_time': i/60,
                                      'commanded_arm_joints': row['issued_arm_q_rad'],
                                      'issued_jaw_joint_targets_m': row['issued_jaw_targets_m'],
                                      'joint_positions': prev['actual_q_rad'], 'joint_velocities': prev['actual_qdot_rad_s'],
                                      'actor_world_matrices': {'/World/Assembly_'+n: raw_matrix(prev['poses'][n]) for n in C.BODIES}})
    r['full_motion_last_snapshot'] = {'step': len(rows), 'sim_time': len(rows)/60,
                                      'joint_positions': rows[-1]['actual_q_rad'],
                                      'joint_velocities': rows[-1]['actual_qdot_rad_s'],
                                      'actor_world_matrices': {'/World/Assembly_'+n: raw_matrix(rows[-1]['poses'][n]) for n in C.BODIES}}
    r['step_timing_audit'] = {'samples': len(rows), 'maximum_step_error_s': 0., 'maximum_render_advance_s': 0.}
    r['pose_readback_dtype_capture_counts'] = {n: len(rows) for n in C.BODIES}
    refresh_payload(r)
    return r, protocol, d


class TraceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = fixture()

    def setUp(self):
        self.r, self.p, self.d = copy.deepcopy(self.saved)

    def audit(self):
        return C.audit_traces(self.r, self.p, self.d, expected_selected_part='base')

    def test_complete_trace_component_never_claims_whole_audit(self):
        v = self.audit()
        self.assertTrue(v['pickup_hold_trace_complete'] and v['measured_trace_gates_pass'])
        self.assertFalse(v['whole_audit_pass'])

    def test_other_two_case_identities(self):
        for part in ['carrier', 'cover']:
            r, p, d = fixture(part)
            v = C.audit_traces(r, p, d, expected_selected_part=part)
            self.assertTrue(v['measured_trace_gates_pass'])
            self.assertEqual(v['selected_part'], part)

    def test_selected_part_replay_rejected(self):
        self.r['selected_part'] = 'cover'
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_raw_terminal_snapshot_missing_rejected(self):
        del self.r['full_motion_last_snapshot']['actor_world_matrices']['/World/Assembly_pcb']
        with self.assertRaises((C.EvidenceError, KeyError)): self.audit()

    def test_post_step_gap_rejected(self):
        self.r['full_motion_observations'][200]['step'] += 1
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_q_forgery_rejected_by_independent_fk(self):
        self.r['full_motion_observations'][200]['actual_q_rad'][0] += .1
        self.r['diagnostic_trace'][201]['joint_positions'][0] += .1
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_all_constant_wrong_home_commands_rejected(self):
        for x in self.r['full_motion_observations']:
            x['issued_arm_q_rad'] = [.123]*6
        for x in self.r['diagnostic_trace']:
            x['commanded_arm_joints'] = [.123]*6
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_preload_jaw_commands_rejected_even_raw_agrees(self):
        for x in self.r['full_motion_observations']:
            if x['phase'] in ['base_close', 'base_lift', 'base_hold']:
                x['issued_jaw_targets_m'] = {'jaw_l': .002, 'jaw_r': -.002}
        for x, row in zip(self.r['diagnostic_trace'], self.r['full_motion_observations']):
            x['issued_jaw_joint_targets_m'] = row['issued_jaw_targets_m']
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_wrong_endpoint_claim_rejected(self):
        self.r['endpoint_traces']['base_lift'][0]['tip_error_mm'] = 9.
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_three_dimensional_hold_slip_recomputed(self):
        index = self.r['pickup_hold']['start_step']+20
        self.r['full_motion_observations'][index]['poses']['base']['position_world_m'][0] += .003
        update_raw_pose(self.r, index, 'base')
        v = self.audit()
        self.assertFalse(v['measured_trace_gates_pass'])
        self.assertGreater(v['maximum_hold_relative_slip_mm'], 2.9)

    def test_terminal_payload_violation_not_hidden_by_runtime_pass(self):
        index = len(self.r['full_motion_observations'])-2
        self.r['full_motion_observations'][index]['poses']['pcb']['position_world_m'][0] += .001
        update_raw_pose(self.r, index, 'pcb')
        refresh_payload(self.r)
        v = self.audit()
        self.assertFalse(v['measured_trace_gates_pass'])
        self.assertTrue(any(x['kind']=='loose_PCB' for x in v['observed_trace_violations']))

    def test_missing_final_payload_guard_count_rejected(self):
        self.r['payload_motion_audit']['samples'] -= 1
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_hold_reference_reset_rejected(self):
        self.r['pickup_hold']['start_step'] += 1
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_forged_stillness_rejected(self):
        self.r['checkpoints'][-1]['stillness_trace'][0]['maximum_raw_joint_speed_rad_s'] = .03
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_hold_target_drift_rejected(self):
        index = self.r['pickup_hold']['start_step']+20
        q = list(self.r['full_motion_observations'][index]['issued_arm_q_rad'])
        q[0] += .001
        self.r['full_motion_observations'][index]['issued_arm_q_rad'] = q
        self.r['diagnostic_trace'][index]['commanded_arm_joints'] = q
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_complete_manifest_rejects_extra_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp); (p/'a.py').write_text('data')
            (p/'snapshot-sha256.json').write_text(json.dumps({'a.py': hashlib.sha256(b'data').hexdigest()}))
            pin = hashlib.sha256((p/'snapshot-sha256.json').read_bytes()).hexdigest()
            C.seal_source(p, pin)
            (p/'unreviewed.py').write_text('extra')
            with self.assertRaises(C.EvidenceError): C.seal_source(p, pin)

    def test_actual_per_case_protocol_and_distinct_inspection_label(self):
        spec = self.p['cases']['base']
        actual = {'schema': 'option_a_known_fixture_pickup_v1',
                  'selected_part': 'base', 'hold_phase': 'base_hold', 'hold_steps': 120,
                  'inspection_phase': 'base_hold_inspection', 'hold_checkpoint': 'base_hold_end',
                  'hold_seconds': 2., 'lift_command_mm': 40., 'payload_floor_m': .004,
                  'world_table_z': .9, 'tool_offset_m': C.TOOL_OFFSET,
                  'known_fixture_targets': True, 'simulator_truth_used_for_targets': False,
                  'camera_guided': False, 'pixel_freshness_certified': False,
                  'physical_hardware_test': False,
                  'phases': [{'name': n, 'role': role, 'duration_s': dur,
                              'jaw_state': 'closed' if i >= 4 else 'open',
                              'target_robot_m': goal.tolist()}
                             for i, (n, role, dur, goal) in enumerate(zip(spec['phases'], C.ROLES, C.DURATIONS, C.goals_for(self.d, spec)))],
                  'home_robot_m': spec['home_robot_m'], 'source_body_robot_m': spec['source_body_robot_m'],
                  'jaw_targets': self.p['binary_jaw_joint_targets_m']}
        for row in self.r['full_motion_observations'][-12:]: row['phase'] = 'base_hold_inspection'
        self.r['checkpoints'][-1]['phase'] = 'base_hold_end'
        self.r['hold'] = {'steps': 120, 'start_step': self.r['pickup_hold']['start_step'],
                          'end_step': self.r['pickup_hold']['end_step'], 'duration_s': 2.,
                          'start_time': self.r['pickup_hold']['start_sim_time'],
                          'end_time': self.r['pickup_hold']['end_sim_time']}
        v = C.audit_traces(self.r, actual, self.d, expected_selected_part='base')
        self.assertTrue(v['pickup_hold_trace_complete'] and v['measured_trace_gates_pass'])
        actual['inspection_phase'] = 'base_hold'
        for row in self.r['full_motion_observations'][-12:]: row['phase'] = 'base_hold'
        v = C.audit_traces(self.r, actual, self.d, expected_selected_part='base')
        self.assertTrue(v['pickup_hold_trace_complete'] and v['measured_trace_gates_pass'])


class ContactAndMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = fixture()
        cls.parts = json.loads((CAD/'parts.json').read_text())

    def setUp(self):
        self.r, self.p, self.d = copy.deepcopy(self.saved)
        self.r.update(contact_monitor_setup={'api_and_subscription_before_reset': True,
                                              'runtime_checks_active_during_reset': True},
                      contact_records=[], contact_summary={}, unexpected_contacts=[], contact_monitor_errors=[],
                      support_pairs=[[['carrier', role], ['pcb', 'substrate']] for role in ['floor', 'wall', 'pcb_keeper']])

    def contact(self, step, *, violation=False, zero=False, positive=False):
        row = self.r['full_motion_observations'][step-1]
        first_lift = next(x['step'] for x in self.r['full_motion_observations'] if x['phase'] == 'base_lift')
        phase = row['phase']
        setup = phase == 'initial_settling'
        in_lift = phase == 'base_lift'
        table = False if setup or phase == 'base_hold' else (step-first_lift+1 <= 18 if in_lift else True)
        active = None if setup else 'base'
        grip = phase in ['base_close', 'base_lift', 'base_hold']
        index = len(self.r['contact_records'])+1
        sep = -.0003 if violation else (.0001 if positive else 0.)
        actors = ['/World/Assembly_base', '/World/Mat']
        colliders = ['/World/Assembly_base/C_000_floor', '/World/Mat']
        roles = [('base', 'floor'), None]
        allowed, category = C.contact_classification(actors, colliders, roles, active, grip, table, set())
        bad = ((not allowed and (sep < -1e-6 or .001 > 1e-8)) or (allowed and sep < -.00025))
        rec = {'callback': index, 'header_ordinal': 0, 'step': step,
               'sim_time': row['sim_time'], 'state': {'step': step, 'phase': phase,
               'active': active, 'allow_grip': grip, 'allow_active_table': table},
               'actors': actors, 'colliders': colliders, 'roles': [list(x) if x else None for x in roles],
               'num_contact_data': 0 if zero else 1,
               'points': [] if zero else [{'position_world_m': [.36, -.12, .8949],
                                           'separation_m': sep, 'normal_world': [0., 0., 1.],
                                           'impulse_Ns': [0., 0., .001]}]}
        if not zero:
            rec.update(allowed_pair=allowed, category=category, violation=bad)
            key = phase+'|'+category+'|'+str(roles)
            self.r['contact_summary'][key] = {'events': 1, 'minimum_separation_mm': sep*1000,
                                              'maximum_impulse_Ns': .001}
            if bad: self.r['unexpected_contacts'].append(copy.deepcopy(rec))
        self.r['contact_records'].append(rec)
        self.r['relevant_contact_headers_seen'] = len(self.r['contact_records'])
        self.r['contact_headers_seen'] = len(self.r['contact_records'])
        self.r['contact_callback_count'] = len(self.r['contact_records'])
        return rec

    def audit(self):
        return C.audit_contacts(self.r, self.p, self.parts, expected_selected_part='base')

    def test_initial_active_none_and_raw_positive_separation(self):
        self.contact(100, positive=True)
        self.assertTrue(self.audit()['original_contact_gates_pass'])

    def test_positive_raw_separation_must_not_be_clipped_to_zero(self):
        self.contact(100, positive=True)
        next(iter(self.r['contact_summary'].values()))['minimum_separation_mm'] = 0.
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_zero_point_header_preserved_and_not_fake_summary(self):
        self.contact(100, zero=True)
        self.assertTrue(self.audit()['original_contact_gates_pass'])

    def test_midlift_table_allowance_boundary(self):
        first = next(x['step'] for x in self.r['full_motion_observations'] if x['phase'] == 'base_lift')
        self.contact(first+17)
        self.assertTrue(self.audit()['original_contact_gates_pass'])
        self.contact(first+18)
        self.assertFalse(self.audit()['original_contact_gates_pass'])

    def test_forged_midlift_state_cannot_allow_table(self):
        first = next(x['step'] for x in self.r['full_motion_observations'] if x['phase'] == 'base_lift')
        rec = self.contact(first+18)
        rec['state']['allow_active_table'] = True
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_allowed_contact_original_depth_limit(self):
        self.contact(100, violation=True)
        v = self.audit()
        self.assertFalse(v['original_contact_gates_pass'])
        self.assertAlmostEqual(v['observed_contact_violations'][0]['minimum_separation_mm'], -.3)

    def test_summary_and_header_omission_rejected(self):
        self.contact(100)
        self.r['relevant_contact_headers_seen'] += 1
        with self.assertRaises(C.EvidenceError): self.audit()

    def test_monitor_error_is_never_physics_pass(self):
        self.contact(100)
        self.r['contact_monitor_errors'] = ['callback read failed']
        self.assertFalse(self.audit()['original_contact_gates_pass'])

    def mass_report(self):
        from scipy.spatial.transform import Rotation
        r = {'part_attachment_joints': [], 'separate_body_audit': {}, 'part_masses_kg': {},
             'physical_collider_counts': {}, 'installed_full_body_mass_properties': {}}
        for n, mp in self.d['mass_properties'].items():
            r['separate_body_audit'][n] = {'root': '/World/Assembly_'+n, 'dynamic': n != 'nest',
                                         'fixture_static': n == 'nest', 'kinematic': False,
                                         'gravity_disabled': False, 'mass_kg': mp['mass_kg']}
            r['part_masses_kg'][n] = mp['mass_kg']
            r['physical_collider_counts'][n] = len(self.parts['parts'][n]['colliders'])
            if n == 'nest': continue
            values, vectors = np.linalg.eigh(mp['inertia_tensor_kg_m2'])
            if np.linalg.det(vectors) < 0: vectors[:, 0] *= -1
            quat = Rotation.from_matrix(vectors).as_quat()[[3, 0, 1, 2]]
            r['installed_full_body_mass_properties'][n] = {
                'validated': True, 'physics_time_advanced_s': 0., 'mass_kg': mp['mass_kg'],
                'centre_of_mass_local_m': mp['centre_of_mass_local_m'],
                'principal_axes_wxyz': quat.tolist(), 'diagonal_inertia_kg_m2': values.tolist()}
        return r

    def test_nontrivial_principal_axis_full_inertia_readback(self):
        r = self.mass_report()
        self.assertTrue(C.audit_mass_and_body_modes(r, self.d, self.parts)['mass_properties_verified'])

    def test_incorrect_mass_or_glued_pcb_rejected(self):
        for kind in ['mass', 'joint', 'kinematic']:
            r = self.mass_report()
            if kind == 'mass': r['installed_full_body_mass_properties']['pcb']['mass_kg'] *= 2
            if kind == 'joint': r['part_attachment_joints'] = ['/World/glued_PCB']
            if kind == 'kinematic': r['separate_body_audit']['pcb']['kinematic'] = True
            with self.assertRaises(C.EvidenceError): C.audit_mass_and_body_modes(r, self.d, self.parts)

    def test_original_installed_material_and_jaw_drive_parameters(self):
        prefix = '/World/ER4iA/J6_link/Gripper/'
        r = {'installed_physics_materials': {
            '/World/Physics/'+n: {'static_friction': v[0], 'dynamic_friction': v[1], 'restitution': v[2]}
            for n, v in [('ObjectMaterial', [.9, .8, .05]), ('TapeMaterial', [1.5, 1.3, 0]), ('SoftTipMaterial', [1.5, 1.3, 0])]},
            'installed_jaw_drives': {prefix+n: {'axis': 'Y', 'lower_limit': lo, 'upper_limit': hi,
                'stiffness': 40000., 'damping': 3000., 'max_force': 180.}
                for n, lo, hi in [('jaw_l', -.006, 0.), ('jaw_r', 0., .006)]}}
        self.assertTrue(C.audit_drives_and_materials(r)['source_physical_parameters_match'])
        for kind in ['friction', 'force', 'stroke']:
            bad = copy.deepcopy(r)
            if kind == 'friction': bad['installed_physics_materials']['/World/Physics/ObjectMaterial']['static_friction'] = 1.5
            if kind == 'force': bad['installed_jaw_drives'][prefix+'jaw_l']['max_force'] = 500.
            if kind == 'stroke': bad['installed_jaw_drives'][prefix+'jaw_l']['lower_limit'] = -.012
            with self.assertRaises(C.EvidenceError): C.audit_drives_and_materials(bad)

    def test_sealed_new_geometry_probe_rays_and_original_surface_tolerance(self):
        recipe = [{'part': n, 'label': 'synthetic_probe', 'start_m': [0, 0, .01],
                   'direction': [0, 0, -1], 'maximum_distance_m': .02,
                   'hit': True, 'expected_distance_mm': 2.} for n in C.BODIES]
        r = {'finger_cooking': {'mode': 'source_fidelity_cooking', 'collider_count': 138,
                               'source_profile_dimensions_unchanged': True, 'min_thickness_m': 0.,
                               'hull_vertex_limit': 255, 'maximum_world_source_vertex_change_m': 0.},
             'finger_surface_audit': {'source_surface_match': True, 'maximum_allowed_absolute_error_mm': .03,
                                      'probes': [{'same_jaw_hit': True, 'inward_error_mm': 0.} for _ in range(12)]},
             'cooked_geometry_probes': [{**e, 'actual': {'collider': '/World/Assembly_'+e['part']+'/C_000_floor',
                                                        'distance_m': .002}} for e in recipe]}
        self.assertTrue(C.audit_cooking_probes(r, recipe)['finger_and_part_surface_gates_pass'])
        bad = copy.deepcopy(r);bad['cooked_geometry_probes'][0]['actual']['distance_m'] = .00204
        with self.assertRaises(C.EvidenceError): C.audit_cooking_probes(bad, recipe)
        bad = copy.deepcopy(r);bad['cooked_geometry_probes'][0]['expected_distance_mm'] = 2.04
        with self.assertRaises(C.EvidenceError): C.audit_cooking_probes(bad, recipe)


def write_png(path, size):
    def chunk(tag, data):
        return struct.pack('>I', len(data))+tag+data+struct.pack('>I', zlib.crc32(tag+data)&0xffffffff)
    w, h = size
    header = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    pixels = (b'\0'+b'\x22\x55\xaa'*w)*h
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', header)+chunk(b'IDAT', zlib.compress(pixels))+chunk(b'IEND', b''))


class MediaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        encoder = shutil.which('ffmpeg')
        if encoder is None:
            try:
                import imageio_ffmpeg
                encoder = imageio_ffmpeg.get_ffmpeg_exe()
            except ImportError:
                pass
        if encoder is None: raise unittest.SkipTest('Local CPU encoder unavailable; remote media verification still required')
        path = cls.root/'synthetic-fixture.mp4'
        subprocess.run([encoder, '-v', 'error', '-f', 'lavfi', '-i', 'testsrc=size=1280x960:rate=30',
                        '-frames:v', '6', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-threads', '1', str(path)], check=True)
        cls.report = {'status': 'FAILED', 'checkpoints': [{'phase': 'initial', 'images': {}}],
                      'full_motion_observations': [{'step': i, 'sim_time': i/60, 'phase': 'initial_settling'} for i in range(1, 13)],
                      'video': {'download_to_laptop': False, 'fps': 30, 'resolution': [1280, 960],
                                'path': str(path), 'bytes': path.stat().st_size,
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                'frames': 6, 'duration_s': .2,
                                'frame_ledger': [{'frame_index': i, 'step': 2*i, 'sim_time': 2*i/60,
                                                  'phase': 'initial_settling', 'frozen_tail': False}
                                                 for i in range(1, 7)]}}
        for camera, size in [('overview', (1280, 960)), ('table', (640, 360)), ('side', (640, 360)), ('wrist', (640, 360))]:
            p = cls.root/('initial-'+camera+'.png');write_png(p, size)
            cls.report['checkpoints'][0]['images'][camera] = {'path': p.name,
                'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'native_pixel_age_certified': False}

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, 'tmp'): cls.tmp.cleanup()

    def test_real_cpu_codec_fixture_decode_count_identity(self):
        r = C.verify_media(self.root/'report.json', self.report)
        self.assertTrue(r['actual_media_identity_verified'])
        self.assertEqual(r['video']['frames'], 6)
        self.assertFalse(r['video_visually_reviewed'])

    def test_video_metadata_cannot_forge_decoded_frame_count(self):
        r = copy.deepcopy(self.report);r['video']['frames'] = 7
        with self.assertRaises(C.EvidenceError): C.verify_media(self.root/'report.json', r)

    def test_hash_valid_but_png_crc_corrupt_is_rejected(self):
        r = copy.deepcopy(self.report)
        original = self.root/'initial-table.png'
        data = original.read_bytes()
        try:
            modified = bytearray(data);modified[-1] ^= 1;original.write_bytes(modified)
            r['checkpoints'][0]['images']['table']['sha256'] = hashlib.sha256(modified).hexdigest()
            with self.assertRaises(C.EvidenceError): C.verify_media(self.root/'report.json', r)
        finally: original.write_bytes(data)

    def test_valid_png_with_wrong_dimensions_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'small.png';write_png(p, (16, 16))
            with self.assertRaises(C.EvidenceError): C.verify_png(p, (640, 360))

    def test_frame_ledger_repetition_is_not_extra_physics_video(self):
        r = copy.deepcopy(self.report)
        r['video']['frame_ledger'][-1]['step'] = 10
        with self.assertRaises(C.EvidenceError): C.audit_video_frame_ledger(r)

    def test_valid_short_video_does_not_prove_completed_pickup(self):
        r = copy.deepcopy(self.report)
        r['status'] = C.COMPLETE
        r['protocol'] = {'phases': [{'name': 'base_lift'}], 'hold_phase': 'base_hold'}
        with self.assertRaises(C.EvidenceError): C.audit_video_frame_ledger(r)

    def test_cli_rejects_unreviewed_source_even_with_real_valid_movie(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp);(source/'snapshot-sha256.json').write_text('{}')
            report_path = self.root/'native-synthetic-report.json'
            report_path.write_text(json.dumps(self.report))
            output = self.root/'synthetic-independent-audit.json'
            from contextlib import redirect_stdout
            import io
            with redirect_stdout(io.StringIO()):
                code = C.main(['--source', str(source), '--report', str(report_path),
                               '--manifest-sha256', '0'*64, '--part', 'base', '--output', str(output)])
            result = json.loads(output.read_text())
            self.assertEqual(code, 1)
            self.assertFalse(result['pickup_hold_verified'])
            self.assertFalse(result['notification_ready_after_actual_visual_review'])


if __name__ == '__main__':
    unittest.main()
