"""Prospective source-schema adapter; never reads poses or drives a simulator.

Pinned camera.py fff64d7bb7c6e8bc86c77df300227ba1c70932efac9285c36d43eabe8261d2a8
stores the fabric reference-time mapping in rendering_frame, while rendering_time
is the SimulationManager mapping to simulation time. The failed raw payload was
not saved: this is source-schema-backed, not a captured-payload replay.
"""
import math
from collections.abc import Mapping
from numbers import Integral, Real

MAX_FRAME_AGE_S = 2 / 60 + 1e-6
REFERENCE_KEYS = ('referenceTimeNumerator', 'referenceTimeDenominator')
SDK_CAMERA_SOURCE_SHA256 = 'fff64d7bb7c6e8bc86c77df300227ba1c70932efac9285c36d43eabe8261d2a8'


def _typed_diagnostic(value, depth=0):
    """JSON-safe value/type evidence; never coerces malformed data into validity."""
    result = {'type': type(value).__module__ + '.' + type(value).__qualname__}
    if value is None or isinstance(value, (str, bool)):
        result['value'] = value
    elif isinstance(value, Integral):
        result['value'] = int(value)
    elif isinstance(value, Real):
        converted = float(value)
        if math.isfinite(converted):
            result['value'] = converted
        else:
            result['nonfinite_repr'] = repr(converted)
    elif isinstance(value, Mapping) and depth < 4:
        result['items'] = [{'key': _typed_diagnostic(k, depth+1),
                            'value': _typed_diagnostic(v, depth+1)}
                           for k, v in list(value.items())[:32]]
        result['item_count'] = len(value)
        result['truncated'] = len(value) > 32
    else:
        result['unsupported_repr'] = repr(value)[:512]
    return result


def raw_camera_metadata_diagnostic(frame):
    """Only native metadata fields, never RGB arrays or guessed world-step IDs."""
    fields = {}
    for name in ('rendering_frame', 'rendering_time'):
        fields[name] = ({'available': True, 'raw': _typed_diagnostic(frame[name])}
                        if isinstance(frame, Mapping) and name in frame
                        else {'available': False})
    return {'capture_before_conversion': True,
            'SDK_camera_source_sha256': SDK_CAMERA_SOURCE_SHA256,
            'frame_container_type': type(frame).__module__ + '.' + type(frame).__qualname__,
            'native_metadata': fields}


def record_then_parse_camera_metadata(frame, record_diagnostic):
    """Caller must persist the diagnostic before conversion or robot dispatch.

    A failed recorder is fatal. A malformed timestamp is preserved and then
    rejected; this function never creates a replacement timestamp.
    """
    if not callable(record_diagnostic):
        raise ValueError('A diagnostic recorder is required before metadata parsing')
    record_diagnostic(raw_camera_metadata_diagnostic(frame))
    return camera_frame_metadata(frame)


def _real(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(label + ' must be a finite real scalar')
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError(label + ' must be finite and nonnegative')
    return value


def _integer(value, label, positive=False):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(label + ' must be an integer; coercion is forbidden')
    value = int(value)
    if value < (1 if positive else 0):
        raise ValueError(label + ' outside valid range')
    return value


def _reference(value):
    if not isinstance(value, Mapping) or any(k not in value for k in REFERENCE_KEYS):
        raise ValueError('Camera reference-time mapping is incomplete or unsupported')
    numerator = _integer(value[REFERENCE_KEYS[0]], REFERENCE_KEYS[0])
    denominator = _integer(value[REFERENCE_KEYS[1]], REFERENCE_KEYS[1], positive=True)
    return dict(zip(REFERENCE_KEYS, (numerator, denominator)))


def camera_frame_metadata(frame):
    """Retain native rational identity; do not invent an integer frame counter."""
    if not isinstance(frame, Mapping) or 'rendering_frame' not in frame or 'rendering_time' not in frame:
        raise ValueError('Missing native camera frame metadata')
    return {
        'rendering_reference_time': _reference(frame['rendering_frame']),
        'rendering_time': _real(frame['rendering_time'], 'rendering_time'),
        'camera_identity_kind': 'SDK_fabric_reference_time_rational',
        'reference_identity_is_not_integer_frame_counter': True,
    }


def validate_camera_pair(observations):
    """Freshness and repeat identity only; says nothing about pose accuracy."""
    if len(observations) != 2:
        raise ValueError('Exactly two camera observations required')
    checked = []
    for o in observations:
        if o.get('camera_identity_kind') != 'SDK_fabric_reference_time_rational':
            raise ValueError('Unsupported camera identity contract')
        ref = _reference(o.get('rendering_reference_time'))
        step = _integer(o.get('step'), 'step', positive=True)
        sim = _real(o.get('sim_time'), 'sim_time')
        render = _real(o.get('rendering_time'), 'rendering_time')
        age = sim - render
        if not 0 <= age <= MAX_FRAME_AGE_S:
            raise ValueError('Stale or future camera frame')
        checked.append((ref, step, sim, render, age))
    a, b = checked
    an, ad = (a[0][k] for k in REFERENCE_KEYS)
    bn, bd = (b[0][k] for k in REFERENCE_KEYS)
    # Exact cross multiplication preserves arbitrarily large reference ticks and
    # rejects equivalent fractions even if raw dictionaries differ.
    if not (bn * ad > an * bd and b[1] > a[1] and b[2] > a[2] and b[3] > a[3]):
        raise ValueError('Repeated, reversed, or nonfresh camera identity')
    return {'native_camera_identity_verified': True,
            'identity_kind': 'SDK_fabric_reference_time_rational',
            'maximum_frame_age_s': max(a[4], b[4]),
            'freshness_limit_s': MAX_FRAME_AGE_S,
            'absolute_image_accuracy_certified': False}
