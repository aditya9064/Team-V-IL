"""Pure CPU evidence checks for Option A known-fixture pickups.

No simulator imports, report edits, source imports, video downloads or truth-based
command correction. ``audit_traces`` is a component, not an admission decision or
complete physics/source certificate. The caller must independently seal source,
runtime, CAD, protocol, physics/cooking and actual video before notification.
"""
import hashlib
import math
from pathlib import Path

import numpy as np

CAD_DESIGN_SHA = 'd88ac777e0f57c48255278894cafc2cc6386a7479cdeb8fe5a40c1f598f25cb7'
CAD_PARTS_SHA = '819e147f08a1a992ab5b7d24a8d9443f619d3989e000095306b88381b00892b9'
FINGER_SHA = '5ee8ff8c87d9a44f96f8af738b46fd9d2a586fa2ad3b8e0968c86161355007dc'
BODIES = {'base', 'carrier', 'pcb', 'cover', 'nest'}
DT = 1 / 60
TOOL_OFFSET = [.293, 0., 0.]
TABLE_Z = .9
TARGET_R = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
ROLES = ['orient', 'above', 'approach', 'side_entry', 'close', 'lift']
DURATIONS = [1., 2., 2., 1.5, 3., 1.5]
COMPLETE = 'DIAGNOSTIC_PASS'
JOINT_LIMITS = np.deg2rad([[-170, 170], [-110, 120], [-69, 205],
                         [-190, 190], [-120, 120], [-360, 360]])


class EvidenceError(ValueError):
    """The available data cannot support the requested conclusion."""


def need(ok, message):
    if not ok:
        raise EvidenceError(message)


def number(value):
    need(type(value) in (int, float) and math.isfinite(value), 'Nonfinite/non-numeric scalar')
    return float(value)


def array(value, shape):
    a = np.asarray(value, float)
    need(a.shape == shape and np.isfinite(a).all(), 'Nonfinite/wrong-shaped evidence')
    return a


def integer(value):
    need(type(value) is int and value >= 0, 'Invalid step/count')
    return value


def near(a, b, atol=1e-8):
    return np.allclose(a, b, rtol=0, atol=atol)


def rotation(q, normalize=True):
    w, x, y, z = array(q, (4,))
    need(abs(np.linalg.norm([w, x, y, z]) - 1) <= 1e-3, 'Invalid quaternion')
    # Normalize only to reconstruct the represented rigid orientation, as the
    # existing scipy quaternion reader does; this is not a geometry correction.
    if normalize:
        w, x, y, z = np.array([w, x, y, z]) / np.linalg.norm([w, x, y, z])
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose(record):
    return array(record['position_world_m'], (3,)), rotation(record['orientation_wxyz'])


def matrix(value):
    m = array(value, (4, 4))
    need(near(m[:, 3], [0, 0, 0, 1]) and near(m[:3, :3] @ m[:3, :3].T, np.eye(3), 1e-5)
         and abs(np.linalg.det(m[:3, :3])-1) < 1e-5, 'Nonrigid raw actor matrix')
    return m


def independent_fk(q):
    """Explicit original ER4iA chain; raw joint trigonometry uses float32."""
    q = array(q, (6,)).astype('float32')
    r, p = np.eye(3), np.zeros(3)
    chain = [((0, 0, .33), 'z', 1), ((0, 0, 0), 'y', 1),
             ((0, 0, .26), 'y', -1), ((0, 0, .02), 'x', -1),
             ((.29, 0, 0), 'y', -1), ((0, 0, 0), 'x', -1)]
    for value, (offset, axis, sign) in zip(q, chain):
        p += r @ np.asarray(offset)
        a = value * sign
        c, s = np.cos(a), np.sin(a)
        turn = (np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) if axis == 'x' else
                np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) if axis == 'y' else
                np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))
        r = r @ turn
    return r, p + r @ TOOL_OFFSET + [0, 0, TABLE_Z]


def angle(dot):
    return math.degrees(math.acos(float(np.clip(dot, -1, 1))))


def body_metrics(record, initial_position):
    p, r = pose(record)
    delta = (p - initial_position) * 1000
    return {'lift_mm': float(delta[2]), 'xy_slip_mm': float(np.linalg.norm(delta[:2])),
            'tilt_deg': angle(r[2, 2]),
            'yaw_deg': abs(math.degrees(math.atan2(r[1, 0], r[0, 0])))}


def payload_metrics(poses):
    carrier = array(poses['carrier']['position_world_m'], (3,))
    pcb = array(poses['pcb']['position_world_m'], (3,))
    # Match unchanged payload_checks.rotation exactly: it validates quaternion
    # norm but does not renormalize the native values before the PCB gate.
    rc = rotation(poses['carrier']['orientation_wxyz'], normalize=False)
    rp = rotation(poses['pcb']['orientation_wxyz'], normalize=False)
    local = rc.T @ (pcb-carrier)
    relative = rc.T @ rp
    return {'xy_mm': float(np.linalg.norm(local[:2])*1000),
            'z_mm': float(abs(local[2]-.004)*1000),
            'tilt_deg': angle(relative[2, 2]),
            'yaw_deg': abs(math.degrees(math.atan2(relative[1, 0], relative[0, 0])))}


def seal_source(source, manifest_sha, manifest_name='snapshot-sha256.json'):
    """Check a complete flat/recursive manifest before importing source modules."""
    import json
    root = Path(source).resolve()
    manifest_path = root / manifest_name
    need(hashlib.sha256(manifest_path.read_bytes()).hexdigest() == manifest_sha,
         'Unreviewed source manifest')
    manifest = json.loads(manifest_path.read_text())
    need(isinstance(manifest, dict) and manifest, 'Invalid manifest')
    actual = set()
    for p in root.rglob('*'):
        need(not p.is_symlink(), 'Symlink in source')
        if p.is_file():
            actual.add(str(p.relative_to(root)))
    need(actual == set(manifest) | {manifest_name}, 'Missing/extra source files')
    for name, digest in manifest.items():
        p = root / name
        need(p.resolve().is_relative_to(root) and str(p.relative_to(root)) == name
             and hashlib.sha256(p.read_bytes()).hexdigest() == digest, 'Source mismatch: '+name)
    return manifest


def goals_for(design, case):
    part = case['selected_part']
    source = array(case['source_body_robot_m'], (3,))
    need(abs(source[2] + .0051) < 1e-12, 'Wrong nominal support plane')
    g = design['grip'][part]
    grasp = source + np.r_[g['handle_centre_xy_mm'], g['tcp_above_part_bottom_mm']] * .001
    entry = np.array([g['side_entry_mm']*.001, 0, 0])
    home = array(case['home_robot_m'], (3,))
    need(near(home, [.30, -.10, .1549], 1e-12), 'Home goal changed')
    return [home, grasp+entry+[0, 0, .065], grasp+entry,
            grasp, grasp, grasp+[0, 0, .04]]


def normalized_contract(protocol):
    """Parse the sealed lean protocol into the trace component's contract.

    This only describes policy. It never changes/relabels a native report or
    supplies missing native measurements. The caller pins original protocol bytes.
    """
    if protocol.get('schema') == 'option_a_known_fixture_pickup_v1' and 'cases' in protocol:
        return protocol
    need(protocol.get('schema') == 'option_a_known_fixture_pickup_v1', 'Unknown native protocol schema')
    part = protocol['selected_part']
    need(part in ['base', 'carrier', 'cover'], 'Unknown selected case')
    need(protocol['hold_steps'] == 120 and protocol['hold_seconds'] == 2
         and protocol['lift_command_mm'] == 40 and protocol['payload_floor_m'] == .004,
         'Lift/hold/PCB policy changed')
    need(protocol['world_table_z'] == TABLE_Z and protocol['tool_offset_m'] == TOOL_OFFSET
         and protocol['known_fixture_targets'] is True
         and protocol['simulator_truth_used_for_targets'] is False
         and protocol['camera_guided'] is False and protocol['pixel_freshness_certified'] is False
         and protocol['physical_hardware_test'] is False, 'Wrong mechanical/camera scope')
    phases = protocol['phases']
    need([p['role'] for p in phases] == ROLES and [p['duration_s'] for p in phases] == DURATIONS,
         'Six-phase policy changed')
    need(all(p['jaw_state'] == ('closed' if p['role'] in ['close', 'lift'] else 'open') for p in phases),
         'Phase binary states changed')
    return {'schema': 'option_a_known_fixture_pickup_v1',
            'cases': {part: {'selected_part': part, 'phases': [p['name'] for p in phases],
                             'phase_roles': ROLES, 'durations_s': DURATIONS,
                             'hold_phase': protocol['hold_phase'],
                             'inspection_phase': protocol['inspection_phase'],
                             'hold_checkpoint': protocol['hold_checkpoint'],
                             'source_body_robot_m': protocol['source_body_robot_m'],
                             'home_robot_m': protocol['home_robot_m']}},
            'binary_jaw_joint_targets_m': protocol['jaw_targets']}


def check_raw_ledger(report):
    rows = report['full_motion_observations']
    need(rows and len(rows) == report['step_timing_audit']['samples'], 'Truncated post-step ledger')
    raw = report['diagnostic_trace']
    need(len(raw) == len(rows), 'Truncated pre-step raw ledger')
    by = {integer(x['step']): x for x in raw}
    need(len(by) == len(raw) and set(by) == set(range(1, len(rows)+1)), 'Raw step duplicate/gap')
    last = report['full_motion_last_snapshot']
    need(integer(last['step']) == len(rows), 'Terminal snapshot step mismatch')
    dtypes = report['pose_readback_dtypes']
    need(set(dtypes) == BODIES and not report['pose_readback_dtype_errors'], 'Missing/changed SDK pose dtype')
    counts = report['pose_readback_dtype_capture_counts']
    need(set(counts) == BODIES and all(type(n) is int and n >= len(rows) for n in counts.values()),
         'Incomplete SDK dtype capture coverage')
    for dtype in dtypes.values():
        need(set(dtype) == {'position', 'orientation'} and all(v in ['float32', 'float64'] for v in dtype.values()), 'Unknown pose dtype')
    for i, row in enumerate(rows, 1):
        need(integer(row['step']) == i and abs(number(row['sim_time'])-i*DT) <= 2e-6, 'Post-step gap/time drift')
        q = array(row['actual_q_rad'], (8,))
        qdot = array(row['actual_qdot_rad_s'], (8,))
        issued = array(row['issued_arm_q_rad'], (6,))
        need(np.all(issued >= JOINT_LIMITS[:, 0]) and np.all(issued <= JOINT_LIMITS[:, 1]), 'Issued arm outside original joint limits')
        r, tip = independent_fk(q[:6])
        need(near(tip, array(row['tip_world_m'], (3,)), 1e-8)
             and near(r, array(row['tool_rotation_world'], (3, 3)), 1e-7), 'Raw actual FK mismatch')
        pre, post = by[i], by[i+1] if i < len(rows) else last
        need(abs(number(pre['sim_time'])-(i-1)*DT) <= 2e-6
             and abs(number(post['sim_time'])-row['sim_time']) <= 2e-6, 'Raw snapshot time mismatch')
        need(near(issued, pre['commanded_arm_joints'], 0)
             and row['issued_jaw_targets_m'] == pre['issued_jaw_joint_targets_m'], 'Issued command not raw pre-step command')
        need(near(q, array(post['joint_positions'], (8,)), 0)
             and near(qdot, array(post['joint_velocities'], (8,)), 0), 'Actual feedback not raw post-step feedback')
        need(set(row['poses']) == BODIES, 'Missing physical body')
        for name, record in row['poses'].items():
            p, rr = pose(record)
            m = matrix(post['actor_world_matrices']['/World/Assembly_'+name])
            need(near(m[3, :3].astype(dtypes[name]['position']).astype(float), p, 1e-8)
                 and near(m[:3, :3].T, rr, 1e-7), 'Pose not raw actor matrix: '+name)
    need(number(report['step_timing_audit']['maximum_step_error_s']) <= 2e-6
         and number(report['step_timing_audit']['maximum_render_advance_s']) <= 1e-9, 'Physics/render timing invalid')
    return rows


def audit_traces(report, protocol, design, *, expected_selected_part):
    """Recompute measurements; never promote this component into a whole audit.

    ``protocol`` and ``design`` must already be externally sealed and reviewed.
    Raw contacts, cooked physics and actual media are separate required modules.
    """
    part = expected_selected_part
    need(part in ['base', 'carrier', 'cover'] and report['selected_part'] == part, 'Wrong/replayed selected part')
    need(report['status'] in ['FAILED', COMPLETE], 'Nonterminal report')
    protocol = normalized_contract(protocol)
    need(protocol['schema'] == 'option_a_known_fixture_pickup_v1', 'Unreviewed protocol schema')
    case = protocol['cases'][part]
    need(case['selected_part'] == part, 'Wrong case binding')
    need(report['limits'] == design['limits_unchanged'] and report['payload_limits'] == design['payload_limits_unchanged'], 'Physical limits changed')
    need(report['physics'] == {'solver': 'PGS', 'physics_dt': DT, 'render_dt': 1/30,
                              'position_iterations': 32, 'velocity_iterations': 1}, 'Physics changed')
    expected_binding = {
        'arm_joint_names': ['J'+str(i) for i in range(1, 7)],
        'arm_joint_indices': list(range(6)), 'joint_dtype': 'float32',
        'tool_offset_m': TOOL_OFFSET, 'table_top_z_m': TABLE_Z}
    need(all(report['kinematic_binding'][k] == v for k, v in expected_binding.items()), 'FK binding changed')
    need(report['motion_sampling']['command_hz'] == 60
         and report['motion_sampling']['physics_steps_per_command'] == 1, 'Control cadence changed')
    names = case['phases']
    need(len(names) == 6 and len(set(names)) == 6 and case['phase_roles'] == ROLES
         and case['durations_s'] == DURATIONS, 'Motion protocol changed')
    hold_phase = case['hold_phase']
    sequence = ['initial_settling'] + names + [hold_phase]
    if 'inspection_phase' in case and case['inspection_phase'] != hold_phase:
        sequence.append(case['inspection_phase'])
    need(len(set(sequence)) == len(sequence), 'Phase aliases/duplicates')
    rows = check_raw_ledger(report)
    blocks = list(dict.fromkeys(x['phase'] for x in rows))
    need(blocks == sequence[:len(blocks)] and all(x['phase'] in sequence for x in rows), 'Extra/reordered motion')
    index = [sequence.index(x['phase']) for x in rows]
    need(index == sorted(index), 'Returned to an earlier phase')
    # Binary maps are reviewed externally from exact cell source. They must be
    # endpoint pairs, not an arbitrary grasp-specific preload or interpolation.
    jawmap = protocol['binary_jaw_joint_targets_m']
    need(jawmap == {'open': {'jaw_l': 0., 'jaw_r': 0.},
                    'closed': {'jaw_l': -.006, 'jaw_r': .006}}, 'Not original binary jaw endpoints')
    violations = []
    for row in rows:
        closed = sequence.index(row['phase']) >= sequence.index(names[4])
        need(row['issued_jaw_targets_m'] == jawmap['closed' if closed else 'open'], 'Intermediate/nonbinary jaw command')
    goals = dict(zip(names, goals_for(design, case)))
    setup = [x for x in rows if x['phase'] == 'initial_settling']
    need(setup, 'No initial home setup')
    first_command = setup[0]['issued_arm_q_rad']
    commanded_r, commanded_tip = independent_fk(first_command)
    need(np.linalg.norm(commanded_tip-[0, 0, TABLE_Z]-goals[names[0]]) <= .0001
         and angle((np.trace(TARGET_R @ commanded_r.T)-1)/2) <= math.degrees(.01),
         'Issued home command does not solve pinned full home pose')
    need(all(x['issued_arm_q_rad'] == first_command for x in setup), 'Initial home target changed')
    shots = report['checkpoints']
    shotnames = [s['phase'] for s in shots]
    expected_shots = ['initial'] + names + [case['hold_checkpoint']]
    need(shotnames == expected_shots[:len(shotnames)] and len(set(shotnames)) == len(shotnames), 'Checkpoint order/truncation')
    need(shots and shots[0]['phase'] == 'initial', 'No settled initial reference')
    shotmap = {s['phase']: s for s in shots}
    for shot in shots:
        step = integer(shot['step'])
        need(1 <= step <= len(rows) and abs(number(shot['sim_time'])-rows[step-1]['sim_time']) <= 2e-6,
             'Checkpoint step/time not raw')
        need(shot['poses'] == rows[step-1]['poses'], 'Checkpoint poses not raw')
        still = shot['stillness_trace']
        need(len(still) == 6 and [x['step'] for x in still] == list(range(step-10, step+1, 2)), 'Not six consecutive rendered stillness samples')
        for item in still:
            raw = rows[integer(item['step'])-1]
            speed = max(abs(v) for v in raw['actual_qdot_rad_s'][:6])
            need(abs(number(item['sim_time'])-raw['sim_time']) <= 2e-6
                 and abs(number(item['maximum_raw_joint_speed_rad_s'])-speed) < 1e-7, 'Stillness not measured feedback')
            if speed >= .02:
                violations.append({'kind': 'stillness', 'step': raw['step'], 'speed_rad_s': speed})
    phases = report['motion_sampling']['phases']
    need([p['phase'] for p in phases] == names[:len(phases)] and len(phases) <= 6, 'Motion summary order')
    endpoint_metrics = {}
    for phase, duration in zip(phases, DURATIONS):
        name = phase['phase']
        need(phase['requested_duration_s'] == duration and phase['commands'] == round(duration*60)
             and abs(number(phase['actual_duration_s'])-duration) <= 2e-6
             and abs(number(phase['maximum_command_interval_s'])-DT) <= 2e-6, 'Motion timing changed')
        entries = report['endpoint_traces'][name]
        need(len(entries) == 6 and [x['step'] for x in entries] == list(range(entries[-1]['step']-5, entries[-1]['step']+1)), 'Endpoint raw window gap')
        metrics = []
        for item in entries:
            raw = rows[integer(item['step'])-1]
            need(raw['phase'] == name, 'Endpoint wrong phase')
            cr, cp = independent_fk(raw['issued_arm_q_rad'])
            need(np.linalg.norm(cp-[0, 0, TABLE_Z]-goals[name]) <= .0001
                 and angle((np.trace(TARGET_R @ cr.T)-1)/2) <= math.degrees(.01),
                 'Issued endpoint command does not solve pinned full goal pose')
            rr = array(raw['tool_rotation_world'], (3, 3))
            m = {'tip_error_mm': float(np.linalg.norm(array(raw['tip_world_m'], (3,))-[0, 0, TABLE_Z]-goals[name])*1000),
                 'tool_axis_error_deg': angle(rr[:, 0] @ [0, 0, -1]),
                 'spread_axis_error_deg': angle(rr[:, 1] @ [0, 1, 0])}
            need(all(abs(number(item[k])-v) < 1e-5 for k, v in m.items()), 'Forged endpoint score')
            if m['tip_error_mm'] > 2 or m['tool_axis_error_deg'] > 1 or m['spread_axis_error_deg'] > 1:
                violations.append({'kind': 'endpoint', 'step': raw['step'], **m})
            metrics.append(m)
        endpoint_metrics[name] = metrics
    # Every observed step, including a contact-stop step, contributes PCB data.
    payload = [payload_metrics(row['poses']) for row in rows]
    maxima = {key: max(s[key] for s in payload) for key in payload[0]}
    monitor = report['payload_motion_audit']
    need(integer(monitor['samples']) == len(rows), 'Missing terminal PCB guard sample')
    keys = {'xy_mm': 'maximum_xy_mm', 'z_mm': 'maximum_z_mm',
            'tilt_deg': 'maximum_relative_tilt_deg', 'yaw_deg': 'maximum_relative_yaw_deg'}
    need(all(abs(number(monitor[keys[k]])-v) < 1e-5 for k, v in maxima.items()), 'PCB summary not actual poses')
    for row, score in zip(rows, payload):
        if any(score[k] > lim for k, lim in [('xy_mm', .75), ('z_mm', .35), ('tilt_deg', 1), ('yaw_deg', 1)]):
            violations.append({'kind': 'loose_PCB', 'step': row['step'], **score})
    initial = pose(shots[0]['poses'][part])[0]
    pickup, hold_metrics = None, []
    if names[-1] in shotmap:
        lift = shotmap[names[-1]]
        pickup = body_metrics(lift['poses'][part], initial)
        if pickup['lift_mm'] < 35 or pickup['xy_slip_mm'] > 2 or pickup['tilt_deg'] > 5 or pickup['yaw_deg'] > 3:
            violations.append({'kind': 'pickup', 'step': lift['step'], **pickup})
        if part == 'carrier':
            pcb_lift = (pose(lift['poses']['pcb'])[0][2]-pose(shots[0]['poses']['pcb'])[0][2])*1000
            pickup['separate_PCB_lift_mm'] = float(pcb_lift)
            if pcb_lift < 35:
                violations.append({'kind': 'PCB_not_lifted', 'step': lift['step'], 'lift_mm': float(pcb_lift)})
        reference = rows[lift['step']-1]
        rp, _ = pose(reference['poses'][part])
        ref_relative = rp-array(reference['tip_world_m'], (3,))
        ref_local = array(reference['tool_rotation_world'], (3, 3)).T @ ref_relative
        hold_rows = [x for x in rows if x['phase'] in [hold_phase, case.get('inspection_phase')]]
        need(not hold_rows or hold_rows[0]['step'] == lift['step']+1, 'Hold reference reset/delayed')
        fixed_arm = reference['issued_arm_q_rad']
        for row in hold_rows:
            need(row['issued_arm_q_rad'] == fixed_arm and row['issued_jaw_targets_m'] == jawmap['closed'], 'Adaptive hold command')
            p, _ = pose(row['poses'][part])
            local = array(row['tool_rotation_world'], (3, 3)).T @ (p-array(row['tip_world_m'], (3,)))
            m = body_metrics(row['poses'][part], initial)
            m['held_relative_slip_mm'] = float(np.linalg.norm(p-array(row['tip_world_m'], (3,))-ref_relative)*1000)
            m['tool_local_relative_slip_mm_diagnostic'] = float(np.linalg.norm(local-ref_local)*1000)
            hold_metrics.append({'step': row['step'], **m})
            if m['lift_mm'] < 35 or m['held_relative_slip_mm'] > 2 or m['tilt_deg'] > 5 or m['yaw_deg'] > 3:
                violations.append({'kind': 'hold', 'step': row['step'], **m})
    # Continuous lift/held drift uses the actual close checkpoint, never a
    # convenient later reference after a possible loss of grip during lift.
    if names[4] in shotmap:
        close = shotmap[names[4]]
        reference = rows[close['step']-1]
        cp, _ = pose(reference['poses'][part])
        reference_relative = cp-array(reference['tip_world_m'], (3,))
        # The declared diagnostic has a close->lift guard and a separate fixed
        # post-lift hold reference. Neither reference may move within its window.
        lift_end = shotmap[names[-1]]['step'] if names[-1] in shotmap else len(rows)
        for row in rows[close['step']:lift_end]:
            bp, _ = pose(row['poses'][part])
            relative = bp-array(row['tip_world_m'], (3,))
            drift = float(np.linalg.norm(relative-reference_relative)*1000)
            m = body_metrics(row['poses'][part], initial)
            if drift > 2 or m['tilt_deg'] > 5 or m['yaw_deg'] > 3:
                violations.append({'kind': 'continuous_lift_held', 'step': row['step'],
                                   'held_relative_slip_mm': drift, **m})
    complete = report['status'] == COMPLETE
    if complete:
        need(blocks == sequence and len(phases) == 6 and shotnames == expected_shots, 'Claimed pass omits required motion')
        native_h = report.get('hold')
        if native_h is not None:
            need(abs(number(native_h['duration_s'])-2) <= 2e-6, 'Native hold duration changed')
        h = ({'requested_steps': native_h['steps'], 'requested_duration_s': 2.,
              'start_step': native_h['start_step'], 'end_step': native_h['end_step'],
              'start_sim_time': native_h['start_time'], 'end_sim_time': native_h['end_time']}
             if native_h is not None else report['pickup_hold'])
        lift = shotmap[names[-1]]
        need(h['requested_steps'] == 120 and h['requested_duration_s'] == 2
             and h['start_step'] == lift['step'] and h['end_step'] == lift['step']+120
             and abs(number(h['start_sim_time'])-lift['sim_time']) <= 2e-6
             and abs(number(h['end_sim_time'])-h['start_sim_time']-2) <= 2e-6, 'Fixed hold timing/count changed')
        need(len(hold_metrics) >= 120 and [x['step'] for x in hold_metrics[:120]] == list(range(h['start_step']+1, h['end_step']+1)), 'Missing fixed120 raw hold samples')
        if case.get('inspection_phase') not in [None, hold_phase]:
            need(sum(row['phase'] == hold_phase for row in rows) == 120, 'Native fixed hold extended/truncated')
    else:
        need(isinstance(report.get('error'), str) and report['error'], 'Missing stopping cause')
    return {'component': 'raw_motion_pickup_hold_and_continuous_PCB',
            'trace_integrity_valid': True, 'runtime_status': report['status'],
            'measured_trace_gates_pass': not violations,
            'pickup_hold_trace_complete': complete and len(hold_metrics) >= 120,
            'selected_part': part, 'pickup': pickup, 'hold_samples': len(hold_metrics),
            'maximum_hold_relative_slip_mm': max((m['held_relative_slip_mm'] for m in hold_metrics), default=None),
            'continuous_PCB_maxima': maxima, 'endpoint_metrics': endpoint_metrics,
            'observed_trace_violations': violations,
            'whole_audit_pass': False, 'contact_physics_source_video_audits_required': True,
            'full_assembly_accepted': False, 'physical_or_print_qualified': False}


def contact_classification(actors, colliders, roles, active, allow_grip, allow_table, support_pairs):
    """Original classify_contact semantics, independently replayed from identities."""
    for actor, role in zip(actors, roles):
        need(not actor.startswith('/World/Assembly_') or role is not None, 'Unmapped assembly contact')
    if actors[0] == actors[1]:
        return True, 'same_rigid_body'
    if all(role is not None for role in roles):
        allowed = tuple(sorted(roles)) in support_pairs
        return allowed, 'part_support' if allowed else 'part_interference'
    for side, role in enumerate(roles):
        if role is None:
            continue
        other = 1-side
        if colliders[other] == '/World/Mat' and role[1] in ('floor', 'ear', 'panel', 'ring'):
            allowed = role[0] != active or allow_table
            return allowed, 'table_support' if allowed else 'active_part_table'
        if actors[other].startswith('/World/ER4iA/Jaw'):
            allowed = role[0] == active and allow_grip and role[1] in ('grip_stem', 'grip_cap', 'grip_support')
            return allowed, 'grip_contact' if allowed else 'finger_interference'
    return False, 'unintended'


def audit_contacts(report, protocol, parts, *, expected_selected_part):
    """Requires every relevant, nonempty raw header; no summary-only pass."""
    part = expected_selected_part
    need(report['selected_part'] == part, 'Contact case replay')
    protocol = normalized_contract(protocol)
    case = protocol['cases'][part]
    phases = ['initial_settling'] + case['phases'] + [case['hold_phase']]
    inspection = case.get('inspection_phase')
    if inspection is not None and inspection not in phases:
        phases.append(inspection)
    roles = {'/World/Assembly_'+name+f'/C_{i:03d}_'+c['role']: (name, c['role'])
             for name, item in parts['parts'].items() for i, c in enumerate(item['colliders'])}
    supports = {tuple(sorted([('carrier', role), ('pcb', 'substrate')]))
                for role in ['floor', 'pcb_keeper', 'wall']}
    need({tuple(sorted(tuple(x) for x in pair)) for pair in report['support_pairs']} == supports,
         'Original isolated loose-PCB support policy changed')
    setup = report['contact_monitor_setup']
    need(setup['api_and_subscription_before_reset'] is True
         and setup['runtime_checks_active_during_reset'] is True, 'Contact guard not active before reset')
    records = report['contact_records']
    need(records and integer(report['relevant_contact_headers_seen']) == len(records)
         and integer(report['contact_headers_seen']) >= len(records), 'Raw contact coverage missing')
    count = integer(report['contact_callback_count'])
    seen, grouped, violations = set(), {}, []
    last_identity = (-1, -1)
    rows = report['full_motion_observations']
    for rec in records:
        identity = (integer(rec['callback']), integer(rec['header_ordinal']))
        need(identity not in seen and identity > last_identity and 1 <= identity[0] <= count, 'Duplicate/reordered contact header')
        seen.add(identity);last_identity = identity
        state = rec['state']
        step, phase = integer(rec['step']), state['phase']
        need(state['step'] == step, 'Raw contact state step mismatch')
        need(phase in phases and 0 <= step <= len(rows), 'Contact outside observed phase/step')
        if step:
            need(rows[step-1]['phase'] == phase, 'Contact phase not raw step')
        else:
            need(phase == 'initial_settling', 'Non-setup contact before first step')
        actors, colliders = rec['actors'], rec['colliders']
        need(len(actors) == len(colliders) == 2 and all(isinstance(v, str) for v in actors+colliders), 'Invalid raw contact identity')
        pair_roles = [roles.get(c) for c in colliders]
        for actor, role in zip(actors, pair_roles):
            if role:
                need(actor == '/World/Assembly_'+role[0], 'Actor/collider identity mismatch')
        role = 'setup' if phase == 'initial_settling' else ('hold' if phase in [case['hold_phase'], inspection]
                                                          else ROLES[case['phases'].index(phase)])
        expected_active = None if role == 'setup' else part
        allow_grip = role in ['close', 'lift', 'hold']
        if role == 'setup' or role == 'hold':
            allow_table = False
        elif role == 'lift':
            first = next(x['step'] for x in rows if x['phase'] == phase)
            elapsed_steps = step-first+1
            allow_table = elapsed_steps * DT <= DURATIONS[5] * .2 + 1e-12
        else:
            allow_table = True
        need(state['active'] == expected_active and state['allow_grip'] is allow_grip
             and state['allow_active_table'] is allow_table, 'Raw contact state not independently derived control policy')
        if 'roles' in rec:
            need(rec['roles'] == [list(x) if x else None for x in pair_roles]
                 or rec['roles'] == pair_roles, 'Raw contact roles not source collider identities')
        allowed, category = contact_classification(actors, colliders, pair_roles,
                                                   expected_active, allow_grip, allow_table, supports)
        points = rec['points']
        need(integer(rec['num_contact_data']) == len(points), 'Raw contact points truncated')
        if not points:
            continue
        separations, impulses = [], []
        for pt in points:
            separations.append(number(pt['separation_m'])*1000)
            impulses.append(float(np.linalg.norm(array(pt['impulse_Ns'], (3,)))))
            array(pt['normal_world'], (3,))
            array(pt['position_world_m'], (3,))
        minimum, maximum = min(separations), max(impulses)
        violation = ((not allowed and (minimum < -.001 or maximum > 1e-8))
                     or (allowed and minimum < -.25))
        if violation:
            violations.append({'kind': 'contact', 'step': step, 'phase': phase,
                               'category': category, 'actors': actors, 'colliders': colliders,
                               'minimum_separation_mm': minimum, 'maximum_impulse_Ns': maximum})
        if 'allowed_pair' in rec:
            need(rec['allowed_pair'] is allowed, 'Forged raw allowed-contact field')
        if 'category' in rec:
            need(rec['category'] == category, 'Forged raw contact category')
        # Same canonical key as the unchanged original callback.
        key = phase+'|'+category+'|'+str(pair_roles)
        value = grouped.setdefault(key, {'events': 0, 'allowed_pair': allowed,
                                         'minimum_separation_mm': math.inf, 'maximum_impulse_Ns': 0.})
        value['events'] += 1
        value['minimum_separation_mm'] = min(value['minimum_separation_mm'], minimum)
        value['maximum_impulse_Ns'] = max(value['maximum_impulse_Ns'], maximum)
    need(set(grouped) == set(report['contact_summary']), 'Summary contact groups omit/add raw events')
    for key, expected in grouped.items():
        actual = report['contact_summary'][key]
        need(actual['events'] == expected['events']
             and abs(number(actual['minimum_separation_mm'])-expected['minimum_separation_mm']) < 1e-7
             and abs(number(actual['maximum_impulse_Ns'])-expected['maximum_impulse_Ns']) < 1e-10,
             'Contact summary not reproduced from raw points')
    errors = report['contact_monitor_errors']
    need(isinstance(errors, list), 'Malformed contact-monitor errors')
    need(bool(report['unexpected_contacts']) == bool(violations), 'Reported contact stop disagrees with raw points')
    return {'component': 'original_raw_contact_gates', 'contact_evidence_integrity_valid': True,
            'raw_relevant_headers': len(records), 'original_contact_gates_pass': not violations and not errors,
            'observed_contact_violations': violations, 'contact_monitor_errors': errors,
            'force_or_static_equilibrium_certified': False, 'whole_audit_pass': False}


def audit_mass_and_body_modes(report, design, parts):
    """Recompute full inertia from installed diagonal/principal-axis readbacks."""
    need(not report['part_attachment_joints'], 'Product attached/glued by joint')
    need(set(report['separate_body_audit']) == BODIES and set(report['part_masses_kg']) == BODIES,
         'Missing separate bodies/masses')
    physical = design['mass_properties']
    readbacks = report['installed_full_body_mass_properties']
    need(set(readbacks) == BODIES-{'nest'}, 'Missing dynamic full-mass readback')
    for name in BODIES:
        body = report['separate_body_audit'][name]
        need(body['root'] == '/World/Assembly_'+name and body['dynamic'] is (name != 'nest')
             and body['fixture_static'] is (name == 'nest') and body['kinematic'] is False
             and body['gravity_disabled'] is False, 'Invalid body flags: '+name)
        expected = physical[name]
        need(abs(number(body['mass_kg'])-expected['mass_kg']) < 1e-8
             and abs(number(report['part_masses_kg'][name])-expected['mass_kg']) < 1e-8,
             'Wrong geometry-derived mass: '+name)
        need(report['physical_collider_counts'][name] == len(parts['parts'][name]['colliders']), 'Collider count changed')
        if name == 'nest':
            continue
        actual = readbacks[name]
        need(actual['validated'] is True and number(actual['physics_time_advanced_s']) == 0,
             'Invalid installed mass readback')
        r = rotation(actual['principal_axes_wxyz'])
        inertia = r @ np.diag(array(actual['diagonal_inertia_kg_m2'], (3,))) @ r.T
        need(np.linalg.eigvalsh(inertia).min() > 0
             and abs(number(actual['mass_kg'])-expected['mass_kg']) < 1e-8
             and np.linalg.norm(array(actual['centre_of_mass_local_m'], (3,))-expected['centre_of_mass_local_m']) <= 1e-7
             and np.allclose(inertia, expected['inertia_tensor_kg_m2'], rtol=1e-5, atol=1e-12),
             'Installed mass/COM/inertia mismatch: '+name)
    return {'component': 'geometry_derived_mass_and_separate_body_modes',
            'mass_properties_verified': True, 'separate_dynamic_PCB_verified': True,
            'whole_audit_pass': False, 'cooking_probe_source_and_video_checks_still_required': True}


def audit_cooking_probes(report, recipe):
    """Replay unchanged numerical surface gates against a sealed new-CAD recipe."""
    cooking = report['finger_cooking']
    need(cooking['mode'] == 'source_fidelity_cooking' and cooking['collider_count'] == 138
         and cooking['source_profile_dimensions_unchanged'] is True
         and cooking['min_thickness_m'] == 0 and cooking['hull_vertex_limit'] == 255
         and number(cooking['maximum_world_source_vertex_change_m']) <= 1e-7,
         'Supplied finger cooking changed')
    finger = report['finger_surface_audit']
    need(finger['source_surface_match'] is True and len(finger['probes']) == 12
         and finger['maximum_allowed_absolute_error_mm'] == .03, 'Missing finger surface probes')
    for p in finger['probes']:
        need(p['same_jaw_hit'] is True and abs(number(p['inward_error_mm'])) <= .03, 'Cooked finger surface mismatch')
    expected = {(p['part'], p['label']): p for p in recipe}
    need(len(expected) == len(recipe) and {p['part'] for p in recipe} == BODIES,
         'Duplicate/incomplete sealed geometry-probe recipe')
    actual = report['cooked_geometry_probes']
    need(len(actual) == len(recipe) and {(p['part'], p['label']) for p in actual} == set(expected),
         'Missing/replayed part geometry probes')
    for p in actual:
        e = expected[(p['part'], p['label'])]
        need(all(p[k] == v for k, v in e.items()), 'Changed authored probe ray/expected hit')
        hit = p['actual']
        need(bool(hit) == e['hit'], 'Cooked part hit/miss mismatch')
        if hit:
            need(hit['collider'].startswith('/World/Assembly_'+p['part']+'/C_')
                 and abs(number(hit['distance_m'])*1000-e['expected_distance_mm']) <= .03,
                 'Cooked part geometry distance mismatch')
    return {'component': 'source_fidelity_and_new_geometry_probes',
            'finger_and_part_surface_gates_pass': True, 'whole_audit_pass': False}


def audit_drives_and_materials(report):
    materials = report['installed_physics_materials']
    expected = {'/World/Physics/ObjectMaterial': [.9, .8, .05],
                '/World/Physics/TapeMaterial': [1.5, 1.3, 0.],
                '/World/Physics/SoftTipMaterial': [1.5, 1.3, 0.]}
    for path, values in expected.items():
        raw = materials[path]
        need(all(abs(number(raw[key])-value) < 1e-6 for key, value in
                 zip(['static_friction', 'dynamic_friction', 'restitution'], values)),
             'Original physics material changed: '+path)
    drives = report['installed_jaw_drives']
    prefix = '/World/ER4iA/J6_link/Gripper/'
    need(set(drives) == {prefix+'jaw_l', prefix+'jaw_r'}, 'Missing/extra jaw drive')
    for name, limits in [('jaw_l', [-.006, 0.]), ('jaw_r', [0., .006])]:
        raw = drives[prefix+name]
        need(raw['axis'] == 'Y'
             and abs(number(raw['lower_limit'])-limits[0]) < 1e-9
             and abs(number(raw['upper_limit'])-limits[1]) < 1e-9
             and raw['stiffness'] == 40000. and raw['damping'] == 3000.
             and raw['max_force'] == 180., 'Original jaw stroke/drive changed')
    return {'component': 'original_installed_jaw_drives_and_materials',
            'source_physical_parameters_match': True, 'whole_audit_pass': False}


def audit_authored_and_settled_fixture(report, protocol, design):
    need(set(protocol['authored_spawn']) == BODIES and set(report['authored_spawn']) == BODIES,
         'Missing authored fixture bodies')
    selected = protocol['selected_part']
    # Preserve the actual approved known fixture; source target is not replaced
    # with settled simulator truth. The spawn gap is setup, not a target offset.
    positions = {'base': [.58, .32, .8952], 'carrier': [.30, .36, .8952],
                 'cover': [.62, -.35, .8952], 'nest': [.10, .63, .8949]}
    positions[selected] = [.36, -.12, .8952]
    positions['pcb'] = [*positions['carrier'][:2], positions['carrier'][2]+.004]
    settled = {}
    for name, expected in protocol['authored_spawn'].items():
        need(near(expected['position_world_m'], positions[name], 1e-12)
             and expected['orientation_wxyz'] == [1., 0, 0, 0], 'Changed isolated authored fixture')
        raw = report['authored_spawn'][name]
        need(raw['position_world_m'] == expected['position_world_m']
             and raw['orientation_wxyz'] == expected['orientation_wxyz'], 'Authored fixture declaration mismatch')
        m = matrix(raw['actor_world_matrix'])
        need(near(m[3, :3], positions[name], 1e-12) and near(m[:3, :3], np.eye(3), 1e-12),
             'Actual authored USD spawn mismatch')
        pose_record = report['checkpoints'][0]['poses'][name]
        pos, rot = pose(pose_record)
        xy = float(np.linalg.norm(pos[:2]-np.asarray(positions[name])[:2])*1000)
        settled[name] = {'position_world_m': pos.tolist(), 'orientation_wxyz': pose_record['orientation_wxyz'],
                         'settled_xy_shift_from_authored_mm': xy}
    return {'component': 'authored_and_measured_settled_fixture', 'settled': settled,
            'known_fixture_not_perception_guided': True, 'whole_audit_pass': False}


def audit_report(report, source, expected_manifest_sha, *, expected_selected_part):
    """Compose source/data/trace/contact/mass/surface checks, media still separate.

    Called only with an independently reviewed external manifest hash. This
    function never imports executable code from the source tree.
    """
    import json
    root = Path(source)
    manifest = seal_source(root, expected_manifest_sha)
    part = expected_selected_part
    need(part in ['base', 'carrier', 'cover'], 'Unknown actual expected case')
    paths = {'design': root/'design.json', 'parts': root/'parts.json',
             'runtime': root/'option_a_pickup.py', 'protocol': root/('protocol-'+part+'.json')}
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    need(digest(paths['design']) == CAD_DESIGN_SHA and digest(paths['parts']) == CAD_PARTS_SHA,
         'Not reviewed CAD09')
    need(report['selected_part'] == part and report['task'] == 'option_a_known_fixture_binary_pickup'
         and report['source_manifest_sha256'] == expected_manifest_sha
         and report['trial_script_sha256'] == digest(paths['runtime'])
         and report['protocol_sha256'] == digest(paths['protocol'])
         and report['design_sha256'] == CAD_DESIGN_SHA and report['mesh_sha256'] == CAD_PARTS_SHA,
         'Actual report/source/case binding mismatch')
    for p in paths.values():
        need(str(p.relative_to(root)) in manifest, 'Required source not sealed')
    design = json.loads(paths['design'].read_text())
    parts = json.loads(paths['parts'].read_text())
    protocol = json.loads(paths['protocol'].read_text())
    need(report['protocol'] == protocol, 'Reported protocol differs from external source')
    contract = normalized_contract(protocol)
    recomposed = goals_for(design, contract['cases'][part])
    need(all(near(p['target_robot_m'], goal, 1e-12) for p, goal in zip(protocol['phases'], recomposed)),
         'Sealed protocol targets not recomposed from approved CAD/fixture')
    need(report['vision_guided'] is False and report['camera_freshness_qualified'] is False
         and report['physical_hardware_test'] is False and report['full_assembly_accepted'] is False
         and report['print_release'] is False, 'Premature qualification claim')
    sdk = {'cell.py': 'cfde65b149d4c87add24793734d65d968b79ea70563c169e2f46b458b32a4260',
           'scene_config.py': '6d5d0c7df867986e78ff02b2888236dca6c00cc3bc6b7ecf60f9a49d75facc1b',
           'gripper_geometry.py': FINGER_SHA,
           'fk.py': 'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed',
           'ik.py': '6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'}
    need(report['starter_hashes'] == sdk, 'Supplied SDK/gripper changed')
    components, errors = {}, {}
    calls = {
        'fixture': lambda: audit_authored_and_settled_fixture(report, protocol, design),
        'traces': lambda: audit_traces(report, contract, design, expected_selected_part=part),
        'contacts': lambda: audit_contacts(report, contract, parts, expected_selected_part=part),
        'mass': lambda: audit_mass_and_body_modes(report, design, parts),
        'probes': lambda: audit_cooking_probes(report, json.loads((root/'geometry-probes.json').read_text())),
        'drives_and_materials': lambda: audit_drives_and_materials(report)}
    for key, callback in calls.items():
        try:
            components[key] = callback()
        except Exception as exc:
            errors[key] = {'error_type': type(exc).__name__, 'error': str(exc)}
    traces, contacts = components.get('traces', {}), components.get('contacts', {})
    contact_stop = bool(contacts.get('observed_contact_violations'))
    process_stop = bool(traces.get('observed_trace_violations'))
    motion = (not errors and report['status'] == COMPLETE and traces['pickup_hold_trace_complete']
              and traces['measured_trace_gates_pass'] and contacts['original_contact_gates_pass'])
    classification = ('PICKUP_HOLD_VERIFIED_MEDIA_PENDING' if motion else
                      'OBSERVED_ORIGINAL_CONTACT_GATE_STOP' if contact_stop else
                      'OBSERVED_MEASURED_PROCESS_GATE_STOP' if process_stop else
                      'AUDIT_INCOMPLETE_OR_REJECTED' if errors else 'OBSERVED_STOP_REQUIRES_CAUSE_REVIEW')
    return {'classification': classification, 'source_identity_valid': True,
            'source_and_mechanical_evidence_valid': not errors, 'selected_part': part,
            'pickup_hold_verified_before_media': bool(motion), 'runtime_status': report['status'],
            'runtime_error': report.get('error'), **components, 'component_audit_errors': errors,
            'observed_contact_or_process_stop_verified': contact_stop or process_stop,
            'actual_video_identity_and_visual_review_required': True,
            'notification_ready': False, 'full_assembly_accepted': False,
            'reliability_physical_print_or_static_qualification': False}


def verify_media(report_path, report):
    """Run only where the actual media already exists, e.g. server CPU review.

    Never downloads or transfers a video. Identity/decoding are not visual review.
    """
    root = Path(report_path).resolve().parent
    images = []
    for shot in report['checkpoints']:
        need(set(shot['images']) == {'overview', 'table', 'side', 'wrist'}, 'Missing actual camera views')
        for camera, rec in shot['images'].items():
            expected_file = shot['phase']+'-'+camera+'.png'
            need(rec['path'] == expected_file and rec['native_pixel_age_certified'] is False,
                 'Image path or unearned freshness claim')
            path = root/expected_file
            need(path.resolve().is_relative_to(root) and not path.is_symlink()
                 and hashlib.sha256(path.read_bytes()).hexdigest() == rec['sha256'], 'Actual image identity mismatch')
            verify_png(path, (1280, 960) if camera == 'overview' else (640, 360))
            images.append({'camera': camera, 'path': str(path), 'sha256': rec['sha256']})
    v = report['video']
    need(v['download_to_laptop'] is False and v['fps'] == 30 and v['resolution'] == [1280, 960], 'Video scope changed')
    path = Path(v['path'])
    need(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root)
         and path.stat().st_size == v['bytes'] and hashlib.sha256(path.read_bytes()).hexdigest() == v['sha256'],
         'Actual video identity mismatch')
    frames, seconds, size, fps = video_decode_info(path)
    need(frames > 0 and frames == v['frames'] and abs(seconds-frames/30) < .05
         and abs(seconds-number(v['duration_s'])) < .05
         and size == (1280, 960) and abs(fps-30) < 1e-6, 'Actual video frame count/duration/format mismatch')
    frame_metrics = audit_video_frame_ledger(report)
    return {'actual_media_identity_verified': True, 'video': {'path': str(path), 'sha256': v['sha256'],
            'frames': frames, 'duration_s': seconds}, 'images': images, 'frame_ledger': frame_metrics,
            'video_visually_reviewed': False, 'pixel_age_or_physical_calibration_certified': False}


def audit_video_frame_ledger(report):
    v = report['video']
    ledger, rows = v['frame_ledger'], report['full_motion_observations']
    need(len(ledger) == v['frames'] and ledger, 'Video frame ledger omitted/truncated')
    previous, frozen, freeze_identity = None, 0, None
    ordinary = []
    for i, frame in enumerate(ledger, 1):
        need(integer(frame['frame_index']) == i and type(frame['frozen_tail']) is bool, 'Duplicate/gapped video frame identity')
        step, time = integer(frame['step']), number(frame['sim_time'])
        if frame['frozen_tail']:
            identity = (step, time, frame['phase'])
            if freeze_identity is None:
                freeze_identity = identity
            need(identity == freeze_identity, 'Frozen tail advanced state/time')
            frozen += 1
            continue
        need(frozen == 0 and 1 <= step <= len(rows) and step % 2 == 0, 'Ordinary video frame outside actual rendered rows')
        raw = rows[step-1]
        need(raw['step'] == step and abs(number(raw['sim_time'])-time) <= 2e-6
             and raw['phase'] == frame['phase'], 'Video frame state not raw physics row')
        need(previous is None or step == previous+2, 'Missing/repeated ordinary video frame')
        previous = step
        ordinary.append(frame)
    need(frozen == v.get('frozen_tail_frames', 0), 'Frozen-tail count mismatch')
    if report['status'] == COMPLETE:
        need(frozen == 0 and ordinary, 'Completed run missing actual ordinary video')
        required = {p['name'] for p in report['protocol']['phases']} | {report['protocol']['hold_phase']}
        need(required.issubset({f['phase'] for f in ordinary}), 'Completed video omits required pickup phases')
        need(ordinary[0]['step'] <= report['checkpoints'][0]['step']
             and ordinary[-1]['step'] >= len(rows)-1, 'Completed video loses start/end')
        h = report['pickup_hold']
        hold = [f for f in ordinary if f['phase'] == report['protocol']['hold_phase']]
        need(hold[0]['step'] <= h['start_step']+2 and hold[-1]['step'] >= h['end_step']-1,
             'Video omits fixed hold interval')
    return {'frame_count': len(ledger), 'ordinary_frames': len(ordinary), 'frozen_tail_frames': frozen,
            'actual_frame_step_phase_binding_verified': True, 'pixel_exposure_age_certified': False}


def verify_png(path, expected_size):
    """Validate actual noninterlaced cv2 PNG chunks, CRCs and pixel scanlines."""
    import struct
    import zlib
    data = Path(path).read_bytes()
    need(data[:8] == b'\x89PNG\r\n\x1a\n', 'Not a native PNG')
    pos, header, compressed, ended = 8, None, [], False
    while pos < len(data):
        need(pos+12 <= len(data), 'Truncated PNG chunk')
        length = struct.unpack('>I', data[pos:pos+4])[0]
        tag = data[pos+4:pos+8]
        payload = data[pos+8:pos+8+length]
        need(pos+12+length <= len(data), 'Truncated PNG data')
        crc = struct.unpack('>I', data[pos+8+length:pos+12+length])[0]
        need(zlib.crc32(tag+payload) & 0xffffffff == crc, 'PNG chunk CRC mismatch')
        if header is None:
            need(tag == b'IHDR' and length == 13, 'Missing first PNG header')
            header = struct.unpack('>IIBBBBB', payload)
            width, height, depth, color, compression, filtering, interlace = header
            need((width, height) == expected_size and depth == 8 and color in (2, 6)
                 and compression == filtering == interlace == 0, 'Camera PNG format/resolution changed')
        elif tag == b'IHDR':
            need(False, 'Duplicate PNG header')
        if tag == b'IDAT':
            compressed.append(payload)
        if tag == b'IEND':
            need(length == 0 and pos+12 == len(data), 'Bad PNG terminator/trailing data')
            ended = True
        pos += length+12
    need(ended and compressed and header is not None, 'Incomplete PNG stream')
    width, height, depth, color, *_ = header
    stride = width*(3 if color == 2 else 4)+1
    inflater = zlib.decompressobj()
    # Fixed reviewed resolution bounds memory; reject extra decompressed pixels.
    pixels = inflater.decompress(b''.join(compressed), height*stride+1)
    need(inflater.eof and not inflater.unused_data and not inflater.unconsumed_tail
         and len(pixels) == height*stride and all(pixels[i] <= 4 for i in range(0, len(pixels), stride)),
         'Invalid/truncated PNG pixel scanlines')


def video_decode_info(path):
    try:
        import imageio_ffmpeg
    except ImportError:
        import json
        import shutil
        import subprocess
        executable = shutil.which('ffprobe')
        need(executable, 'No local video decoder available')
        proc = subprocess.run([executable, '-v', 'error', '-select_streams', 'v:0', '-count_frames',
                               '-show_entries', 'stream=nb_read_frames,width,height,avg_frame_rate,duration',
                               '-of', 'json', str(path)], check=True, text=True, capture_output=True)
        streams = json.loads(proc.stdout)['streams']
        need(len(streams) == 1, 'No unique decoded video stream')
        stream = streams[0]
        numerator, denominator = stream['avg_frame_rate'].split('/')
        return (int(stream['nb_read_frames']), float(stream['duration']),
                (int(stream['width']), int(stream['height'])), float(numerator)/float(denominator))
    frames, seconds = imageio_ffmpeg.count_frames_and_secs(str(path))
    reader = imageio_ffmpeg.read_frames(str(path))
    try:
        meta = next(reader)
    finally:
        reader.close()
    return frames, seconds, tuple(meta['size']), float(meta['fps'])


def main(argv=None):
    import argparse
    import json
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--manifest-sha256', required=True)
    ap.add_argument('--part', choices=['base', 'carrier', 'cover'], required=True)
    ap.add_argument('--output', help='Defaults to independent-audit.json beside the native report')
    args = ap.parse_args(argv)
    path = Path(args.report)
    result = {'classification': 'AUDIT_INCOMPLETE_OR_REJECTED', 'notification_ready': False,
              'pickup_hold_verified': False, 'full_assembly_accepted': False,
              'physical_or_print_qualified': False}
    report = None
    try:
        report = json.loads(path.read_text())
        result.update(audit_report(report, args.source, args.manifest_sha256, expected_selected_part=args.part))
    except Exception as exc:
        result.update(error_type=type(exc).__name__, error=str(exc))
    if report is not None:
        result.update(actual_runtime_status=report.get('status'), actual_runtime_error=report.get('error'),
                      actual_selected_part=report.get('selected_part'),
                      raw_report_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        try:
            result['actual_media'] = verify_media(path, report)
        except Exception as exc:
            result['media_error'] = str(exc)
        verified = bool(result.get('source_identity_valid')
                        and (result.get('source_and_mechanical_evidence_valid')
                             or result.get('observed_contact_or_process_stop_verified'))
                        and result.get('actual_media', {}).get('actual_media_identity_verified'))
        result['pickup_hold_verified'] = bool(verified and result.get('pickup_hold_verified_before_media'))
        if result['pickup_hold_verified']:
            result['classification'] = 'OBSERVED_ISOLATED_PICKUP_HOLD_PASS_VISUAL_REVIEW_PENDING'
        result['notification_ready_after_actual_visual_review'] = verified
    result['notification_ready'] = False  # A human/model must still inspect actual video.
    output = Path(args.output) if args.output else path.parent/'independent-audit.json'
    need(output.resolve() != path.resolve() and not output.resolve().is_relative_to(Path(args.source).resolve()),
         'Audit must not overwrite native report or sealed source')
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['pickup_hold_verified'] else (2 if result.get('source_and_mechanical_evidence_valid') else 1)


if __name__ == '__main__':
    raise SystemExit(main())
