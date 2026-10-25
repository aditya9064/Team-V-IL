"""Prospective observation-only collector. Never starts a simulator or moves it.

The caller must supply an initialized camera and read-only clock. This is not a
freshness bypass or proof of actual pixel acquisition time. At most six explicit
render-only refreshes; no adaptive waiting or automatic retry.
"""
import hashlib
import math
from collections.abc import Mapping

from camera_reference_stamp import raw_camera_metadata_diagnostic

MAX_RENDER_REFRESHES = 6


def image_fingerprint(image):
    if image is None:
        return {'available': False}
    shape = tuple(image.shape)
    if len(shape) != 3 or min(shape) <= 0 or shape[2] != 4 or str(image.dtype) != 'uint8':
        raise ValueError('Expected nonempty uint8 RGBA annotator data')
    value = image.tobytes(order='C')
    if len(value) != math.prod(shape):
        raise ValueError('RGBA shape/byte count mismatch')
    return {'available': True, 'shape': list(shape), 'dtype': str(image.dtype),
            'bytes': len(value), 'sha256': hashlib.sha256(value).hexdigest()}


def clock_record(read_clock):
    value = read_clock()
    if (set(value) != {'step', 'sim_time'} or type(value['step']) is not int
            or value['step'] < 0 or type(value['sim_time']) not in (int, float)
            or not math.isfinite(value['sim_time']) or value['sim_time'] < 0):
        raise ValueError('Invalid native physics clock readback')
    return dict(value)


def record_cached(record, key, frame):
    # Stage native metadata before any conversion that can fail. The outer
    # transaction persists this partial evidence, then stops on the same error.
    cached = {'metadata': raw_camera_metadata_diagnostic(frame)}
    record[key] = cached
    try:
        if not isinstance(frame, Mapping):
            raise ValueError('Cached camera frame is not a mapping')
        cached['rgba'] = image_fingerprint(frame.get('rgb'))
    except Exception as error:
        cached['rgba_error'] = repr(error)
        raise


def transaction(camera, read_clock, persist, index):
    """Log cached bundle → direct annotator → cached bundle, without rendering.

    Fingerprints compare data paths only. Equal hashes in a static scene do not
    prove equal exposure time. Metadata is retained in its original typed form.
    """
    record = {'kind': 'capture_transaction', 'index': index,
              'observation_only': True, 'pixel_age_certified': False}
    try:
        record['clock_before'] = clock_record(read_clock)
        record_cached(record, 'cached_before', camera.get_current_frame(clone=True))
        record['direct_rgba'] = image_fingerprint(camera.get_rgba())
        record_cached(record, 'cached_after', camera.get_current_frame(clone=True))
        record['clock_after'] = clock_record(read_clock)
        if record['clock_before'] != record['clock_after']:
            raise ValueError('Physics changed inside read-only capture transaction')
        record['complete'] = True
    except Exception as error:
        record['complete'] = False; record['error'] = repr(error)
        persist(record)
        raise
    persist(record)
    return record


def fixed_render_probe(camera, read_clock, render_only, persist, refreshes=MAX_RENDER_REFRESHES):
    if type(refreshes) is not int or not 0 <= refreshes <= MAX_RENDER_REFRESHES:
        raise ValueError('Render refresh cap must be an integer from zero to six')
    transaction(camera, read_clock, persist, 0)
    for index in range(1, refreshes + 1):
        record = {'kind': 'render_only_refresh', 'index': index}
        try:
            record['clock_before'] = clock_record(read_clock)
            render_only()
            record['clock_after'] = clock_record(read_clock)
            if record['clock_before'] != record['clock_after']:
                raise ValueError('Render-only refresh advanced physics')
            record['complete'] = True
        except Exception as error:
            record['complete'] = False; record['error'] = repr(error)
            persist(record)
            raise
        persist(record)
        transaction(camera, read_clock, persist, index)


def fixed_multi_camera_probe(cameras, read_clock, render_only, persist,
                             refreshes=MAX_RENDER_REFRESHES):
    """Three provided views, at most six GLOBAL refreshes (not six per camera)."""
    if tuple(cameras) != ('table', 'side', 'wrist'):
        raise ValueError('Exactly the provided table, side and wrist views required')
    if type(refreshes) is not int or not 0 <= refreshes <= MAX_RENDER_REFRESHES:
        raise ValueError('Render refresh cap must be an integer from zero to six')
    for index in range(refreshes + 1):
        if index:
            record = {'kind': 'render_only_refresh', 'index': index}
            try:
                record['clock_before'] = clock_record(read_clock)
                render_only()
                record['clock_after'] = clock_record(read_clock)
                if record['clock_before'] != record['clock_after']:
                    raise ValueError('Render-only refresh advanced physics')
                record['complete'] = True
            except Exception as error:
                record['complete'] = False; record['error'] = repr(error)
                persist(record)
                raise
            persist(record)
        for name, camera in cameras.items():
            def record_camera(record, name=name):
                persist(dict(record, camera=name))
            transaction(camera, read_clock, record_camera, index)
