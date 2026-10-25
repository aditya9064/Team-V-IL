"""Passive evidence only; no simulator imports, state changes or acceptance rule.

The injected interface exposes only get_attached_stage() and is_sleeping().
Invalid evidence stays unknown and cannot alter movement or satisfy force gates.
"""
import math

BODY_NAMES = frozenset(('base', 'carrier', 'pcb', 'cover'))


def positive_integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


class PassiveSleepState:
    def __init__(self, interface, expected_stage_id, encoded_body_paths, body_paths):
        if not positive_integer(expected_stage_id):
            raise ValueError('Independent positive USD stage identity required')
        if set(encoded_body_paths) != BODY_NAMES or set(body_paths) != BODY_NAMES:
            raise ValueError('All four product rigid bodies required')
        if not all(positive_integer(x) for x in encoded_body_paths.values()):
            raise ValueError('Invalid encoded body identity')
        if len(set(encoded_body_paths.values())) != 4:
            raise ValueError('Aliased body identities')
        if body_paths != {name: '/World/Assembly_' + name for name in BODY_NAMES}:
            raise ValueError('Canonical product root identity mismatch')
        if len(set(body_paths.values())) != 4:
            raise ValueError('Aliased body paths')
        self.interface = interface
        self.stage_id = expected_stage_id
        self.ids = dict(encoded_body_paths)
        self.paths = dict(body_paths)
        self.rows = []
        self.errors = []
        self.last = None

    def capture(self, step, sim_time, phase):
        row = {'step': step, 'sim_time': sim_time, 'phase': phase,
               'expected_stage_id': self.stage_id, 'body_paths': dict(self.paths),
               'encoded_body_paths': dict(self.ids), 'valid': False,
               'sleeping': None, 'error': None}
        try:
            if not positive_integer(step) or isinstance(sim_time, bool) or not isinstance(sim_time, (int, float)) or not math.isfinite(sim_time) or sim_time < 0:
                raise ValueError('Invalid existing-step identity')
            if not isinstance(phase, str) or not phase:
                raise ValueError('Missing phase')
            if self.last is not None:
                if step != self.last['step'] + 1 or abs(sim_time - self.last['sim_time'] - 1 / 60) > 2e-6:
                    raise ValueError('Nonconsecutive or duplicate existing physics step')
            before = self.interface.get_attached_stage()
            if type(before) is not int or before != self.stage_id:
                raise ValueError('Attached stage differs from independent USD stage')
            values = {name: self.interface.is_sleeping(self.stage_id, self.ids[name])
                      for name in sorted(BODY_NAMES)}
            after = self.interface.get_attached_stage()
            if type(after) is not int or after != self.stage_id:
                raise ValueError('Attached stage changed during observation')
            if not all(type(value) is bool for value in values.values()):
                raise ValueError('SDK query did not return actual boolean values')
            row.update(valid=True, sleeping=values)
        except Exception as exc:
            row['error'] = type(exc).__name__ + ': ' + str(exc)
            self.errors.append({'step': step, 'error': row['error']})
        # Never stop, wait, retry, command, or mutate the engine from telemetry.
        self.rows.append(row)
        if positive_integer(step) and isinstance(sim_time, (int, float)) and not isinstance(sim_time, bool) and math.isfinite(sim_time):
            self.last = {'step': step, 'sim_time': sim_time}
        return row

    def evidence(self):
        return {'schema': 'PASSIVE_SLEEP_STATE_EVIDENCE_ONLY',
                'body_paths': dict(self.paths), 'expected_stage_id': self.stage_id,
                'records': self.rows, 'errors': self.errors,
                'all_samples_valid': bool(self.rows) and not self.errors,
                'used_for_motion_or_targets': False, 'force_gate_satisfied_by_sleep': False,
                'acceptance_policy_changed': False, 'extra_physics_steps': 0,
                'sleep_or_wake_state_modified': False}

class UnavailableSleepState:
    """Preserve unavailable telemetry and scope without changing robot execution."""
    def __init__(self, reason):
        self.reason = str(reason)
        self.rows = []

    def capture(self, step, sim_time, phase):
        row = {'step': step, 'sim_time': sim_time, 'phase': phase,
               'valid': False, 'sleeping': None, 'error': self.reason}
        self.rows.append(row)
        return row

    def evidence(self):
        return {'schema': 'PASSIVE_SLEEP_STATE_EVIDENCE_ONLY',
                'setup_error': self.reason, 'records': self.rows,
                'all_samples_valid': False, 'used_for_motion_or_targets': False,
                'force_gate_satisfied_by_sleep': False,
                'acceptance_policy_changed': False, 'extra_physics_steps': 0,
                'sleep_or_wake_state_modified': False}
