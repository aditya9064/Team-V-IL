"""Read-only simulator diagnostics; these records never certify insertion/release."""
import json
import math


def _vector(values, count):
    values = [float(v) for v in values]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError('Missing or nonfinite measured pose')
    return values


def _quat(values):
    q = _vector(values, 4)
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-8:
        raise ValueError('Zero measured quaternion')
    return [v/norm for v in q]


def _rotate(q, v):
    w, x, y, z = q
    uv = [y*v[2]-z*v[1], z*v[0]-x*v[2], x*v[1]-y*v[0]]
    uuv = [y*uv[2]-z*uv[1], z*uv[0]-x*uv[2], x*uv[1]-y*uv[0]]
    return [v[i]+2*(w*uv[i]+uuv[i]) for i in range(3)]


def peg_metrics(held_position, held_quaternion, fixed_position, fixed_quaternion):
    """Compute peg-root alignment in the socket frame from measured WXYZ poses.

    For the pinned Factory peg task, seated peg/socket roots coincide. These
    metrics alone establish neither collision fit nor released support.
    """
    held = _vector(held_position, 3)
    fixed = _vector(fixed_position, 3)
    held_q, fixed_q = _quat(held_quaternion), _quat(fixed_quaternion)
    inverse_fixed = [fixed_q[0]]+[-v for v in fixed_q[1:]]
    relative = _rotate(inverse_fixed, [a-b for a,b in zip(held, fixed)])
    axis = _rotate(inverse_fixed, _rotate(held_q, [0., 0., 1.]))
    return {'radial_error_m': math.hypot(relative[0], relative[1]),
            'depth_error_m': relative[2],
            'tilt_rad': math.acos(max(-1., min(1., axis[2]))),
            'held_root_in_socket_frame_m': relative}


def _first(tensor):
    row = tensor[0]
    if hasattr(row, 'detach'):
        row = row.detach().cpu()
    return row.tolist() if hasattr(row, 'tolist') else list(row)


def measured_sample(env, *, step, elapsed_s, terminated=False, truncated=False):
    """Read cached measured state only; no action, reset, IK, or geometry writes.

    Isaac Lab auto-resets before returning termination flags. A flagged sample
    is explicitly labelled post-reset, never described as the failing pose.
    """
    if not isinstance(step, int) or step < 0 or not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError('Invalid diagnostic sample time')
    held, held_q = _first(env.held_pos), _first(env.held_quat)
    fixed, fixed_q = _first(env.fixed_pos), _first(env.fixed_quat)
    row = {'kind': 'measured_state', 'step': step, 'elapsed_since_reset_s': elapsed_s,
           'state_phase': 'AUTO_RESET_POST_STEP_STATE' if terminated or truncated else 'CONTINUOUS_EPISODE',
           'terminated': bool(terminated), 'truncated': bool(truncated),
           'held_position_env_m': held, 'held_quaternion_wxyz': held_q,
           'fixed_position_env_m': fixed, 'fixed_quaternion_wxyz': fixed_q,
           'tcp_position_env_m': _first(env.fingertip_midpoint_pos),
           'tcp_quaternion_wxyz': _first(env.fingertip_midpoint_quat),
           'measured_joint_positions': _first(env.joint_pos),
           'commanded_joint_positions': _first(env.ctrl_target_joint_pos),
           'contact_audit': env.contact_audit.report(),
           'release_requested': bool(env.release_requested),
           'release_verified': False,
           'scope': 'simulator diagnostics only; no insertion/release acceptance'}
    if hasattr(env,'_robot'):
        row['jaw_body_poses_world']={}
        for name in ('JawL','JawR'):
            index=env._robot.body_names.index(name)
            row['jaw_body_poses_world'][name]={
                'position_m':_first(env._robot.data.body_pos_w[:,index]),
                'quaternion_wxyz':_first(env._robot.data.body_quat_w[:,index])}
    row.update(peg_metrics(held, held_q, fixed, fixed_q))
    row['tool_contact_audit']=env.tool_contact_audit.report() if hasattr(env,'tool_contact_audit') else None
    row['last_contact_guard_failure']=getattr(env,'last_contact_guard_failure',None)
    # Reject NaN anywhere, including raw joint state/contact records.
    json.dumps(row, allow_nan=False)
    return row


class DiagnosticTrace:
    """Flush each record so a failed simulator run retains preceding evidence."""
    def __init__(self, path, provenance):
        self.path = path
        self.file = path.open('x')
        self.samples = 0
        self.write({'kind': 'provenance', **provenance,
                    'scope': 'measured simulator trace, not assembly evidence'})

    def write(self, row):
        self.file.write(json.dumps(row, allow_nan=False)+'\n')
        self.file.flush()
        if row.get('kind') == 'measured_state':
            self.samples += 1

    def close(self):
        self.file.close()
