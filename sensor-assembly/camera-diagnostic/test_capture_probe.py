import copy
import unittest
from capture_probe import fixed_render_probe, transaction


class Image:
    shape = (2, 2, 4)
    dtype = 'uint8'
    def __init__(self, value=0): self.value = value
    def tobytes(self, order): return bytes([self.value]) * 16


class Camera:
    def __init__(self):
        self.tick = 129; self.direct_value = 0; self.change_during_direct = False
    def get_current_frame(self, clone=False):
        assert clone is True
        return {'rendering_frame': {'referenceTimeNumerator': self.tick,
                                    'referenceTimeDenominator': 30},
                'rendering_time': 2.3333334550261497, 'rgb': Image(0)}
    def get_rgba(self):
        if self.change_during_direct: self.tick += 1
        return Image(self.direct_value)


class Tests(unittest.TestCase):
    def setUp(self):
        self.camera = Camera(); self.clock = {'step': 144, 'sim_time': 2.433333460241556}
        self.records = []
    def read_clock(self): return dict(self.clock)
    def persist(self, value): self.records.append(copy.deepcopy(value))

    def test_fixed_cap_and_no_adaptive_retry_for_stale_static_frame(self):
        calls = []
        fixed_render_probe(self.camera, self.read_clock, lambda: calls.append('render'), self.persist)
        self.assertEqual(calls, ['render'] * 6)
        self.assertEqual(len(self.records), 13)
        frames = [x for x in self.records if x['kind'] == 'capture_transaction']
        self.assertEqual(len(frames), 7)
        self.assertTrue(all(x['pixel_age_certified'] is False for x in frames))
        self.assertTrue(all(x['clock_before'] == self.clock for x in self.records))
        # Stale timestamps were retained, never changed into "current" time.
        for x in frames:
            raw = x['cached_before']['metadata']['native_metadata']['rendering_time']['raw']
            self.assertEqual(raw['value'], 2.3333334550261497)

    def test_async_metadata_change_and_distinct_pixels_are_preserved(self):
        self.camera.change_during_direct = True; self.camera.direct_value = 42
        value = transaction(self.camera, self.read_clock, self.persist, 0)
        self.assertNotEqual(value['cached_before']['metadata'], value['cached_after']['metadata'])
        self.assertNotEqual(value['cached_before']['rgba']['sha256'], value['direct_rgba']['sha256'])
        self.assertFalse(value['pixel_age_certified'])

    def test_unexpected_render_physics_step_is_recorded_then_stops(self):
        def bad_render(): self.clock['step'] += 1
        with self.assertRaisesRegex(ValueError, 'advanced physics'):
            fixed_render_probe(self.camera, self.read_clock, bad_render, self.persist)
        self.assertEqual(len(self.records), 2)
        self.assertFalse(self.records[-1]['complete'])
        self.assertEqual(self.records[-1]['clock_after']['step'], 145)

    def test_getter_failure_preserves_partial_transaction_without_retry(self):
        def bad_getter(): raise RuntimeError('annotator failed')
        self.camera.get_rgba = bad_getter
        with self.assertRaisesRegex(RuntimeError, 'annotator failed'):
            fixed_render_probe(self.camera, self.read_clock, lambda: None, self.persist)
        self.assertEqual(len(self.records), 1)
        self.assertIn('cached_before', self.records[0])
        self.assertFalse(self.records[0]['complete'])

    def test_invalid_metadata_is_typed_evidence_not_fabricated(self):
        self.camera.tick = None
        value = transaction(self.camera, self.read_clock, self.persist, 0)
        self.assertIn('builtins.NoneType', str(value['cached_before']['metadata']))
        self.assertFalse(value['pixel_age_certified'])

    def test_invalid_cap_rejects_before_camera_or_render_calls(self):
        for cap in (7, -1, True, 1.5):
            with self.assertRaises(ValueError):
                fixed_render_probe(None, None, None, self.persist, cap)
        self.assertEqual(self.records, [])

    def test_malformed_cached_pixels_preserve_native_metadata_before_failure(self):
        original = self.camera.get_current_frame
        def bad_frame(clone):
            value = original(clone); value['rgb'].shape = (2, 2, 3)
            return value
        self.camera.get_current_frame = bad_frame
        with self.assertRaisesRegex(ValueError, 'Expected nonempty uint8 RGBA'):
            fixed_render_probe(self.camera, self.read_clock, lambda: None, self.persist)
        self.assertEqual(len(self.records), 1)
        partial = self.records[0]['cached_before']
        self.assertIn('rendering_frame', partial['metadata']['native_metadata'])
        self.assertIn('Expected nonempty', partial['rgba_error'])
        self.assertFalse(self.records[0]['complete'])

    def test_nonmapping_cached_frame_preserves_raw_type_without_get_coercion(self):
        self.camera.get_current_frame = lambda clone: 123
        with self.assertRaisesRegex(ValueError, 'not a mapping'):
            fixed_render_probe(self.camera, self.read_clock, lambda: None, self.persist)
        self.assertEqual(len(self.records), 1)
        partial = self.records[0]['cached_before']
        self.assertEqual(partial['metadata']['frame_container_type'], 'builtins.int')
        self.assertFalse(partial['metadata']['native_metadata']['rendering_time']['available'])


if __name__ == '__main__': unittest.main()
