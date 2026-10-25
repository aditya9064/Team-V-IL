import copy
import json
import math
import sys
import unittest

from json_token_equivalence import assert_equivalent, signature, tokens


class SerializationEquivalence(unittest.TestCase):
    def encodings(self, value, **kwargs):
        before = json.dumps(value, indent=2, **kwargs)
        after = json.dumps(value, separators=(',', ':'), **kwargs)
        assert_equivalent(before, after)
        return before, after

    def test_nested_report_shape(self):
        self.encodings({'poses': [{'p': [1e-5, -2.3, 0.0], 'R': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]}], 'errors': [], 'pass': False, 'state': None})

    def test_empty_collections(self):
        self.encodings({'list': [], 'object': {}, 'nested': [[], {}]})

    def test_string_whitespace_preserved(self):
        value = {'a key  with spaces': ' lead\tinner  space\r\ntrail '}
        a, b = self.encodings(value)
        self.assertEqual(json.loads(a), value)
        self.assertEqual(json.loads(b), value)

    def test_quotes_and_escaped_backslashes(self):
        self.encodings({'text': 'quoted "x"; backslash \\; literal \\n; escaped quote \\\" end'})

    def test_unicode_default_ascii(self):
        self.encodings({'日本語': 'é — 🧪\u2028\u00a0', 'literal_escape': '\\u1234'})

    def test_unicode_unescaped_same_configuration(self):
        self.encodings({'value': 'é — 🧪\u2028\u00a0'}, ensure_ascii=False)

    def test_unpaired_surrogate_default_ascii(self):
        self.encodings({'text': '\ud800'})

    def test_floating_point_extremes_and_nested_values(self):
        self.encodings({'values': [[-0.0, 0.0, 5e-324], [sys.float_info.max, sys.float_info.min, 0.0508526053, 0.1488035438885546]]})

    def test_existing_nonfinite_encoder_behavior_is_unchanged(self):
        a, b = self.encodings({'values': [float('nan'), float('inf'), -float('inf')]})
        self.assertIn('NaN', a)
        self.assertIn('-Infinity', b)

    def test_scalar_roots(self):
        for v in [None, True, False, 0, -7, 3.5, ' a \\" ', []]:
            with self.subTest(value=repr(v)):
                self.encodings(v)

    def test_encoder_does_not_mutate_report(self):
        value = {'commands': [{'p': [1., 2., 3.], 'step': 7}], 'flags': [True, False]}
        old = copy.deepcopy(value)
        self.encodings(value)
        self.assertEqual(value, old)

    def test_deleted_field_is_not_equivalent(self):
        with self.assertRaises(ValueError):
            assert_equivalent('{"measured":1,"failure":false}', '{"measured":1}')

    def test_changed_numeric_value_is_not_equivalent(self):
        with self.assertRaises(ValueError):
            assert_equivalent('{"gap":0.0508526053}', '{"gap":0.05}')

    def test_whitespace_inside_strings_is_not_ignored(self):
        with self.assertRaises(ValueError):
            assert_equivalent('{"value":"a b"}', '{"value":"ab"}')
        with self.assertRaises(ValueError):
            assert_equivalent('{"a b":1}', '{"ab":1}')

    def test_adjacent_scalar_boundaries_remain_distinct(self):
        self.assertNotEqual(signature('1 2'), signature('12'))
        self.assertNotEqual(signature('true false'), signature('true'))

    def test_list_order_is_preserved(self):
        with self.assertRaises(ValueError):
            assert_equivalent('[1,2]', '[2,1]')

    def test_escape_spelling_not_normalized(self):
        with self.assertRaises(ValueError):
            assert_equivalent('"\\u0041"', '"A"')

    def test_negative_zero_is_not_normalized(self):
        with self.assertRaises(ValueError):
            assert_equivalent('-0.0', '0.0')

    def test_duplicate_key_tokens_are_not_collapsed(self):
        with self.assertRaises(ValueError):
            assert_equivalent('{"a":1,"a":2}', '{"a":2}')

    def test_unterminated_or_invalid_strings_rejected(self):
        for value in ['"unterminated', '"bad\\', '"bad\ncontrol"', '"bad\\q"']:
            with self.subTest(value=repr(value)):
                with self.assertRaises(ValueError):
                    list(tokens(value))

    def test_only_json_external_whitespace_is_ignored(self):
        assert_equivalent(' \r\n{ "a" \t: [ 1, 2 ] }\n', '{"a":[1,2]}')
        with self.assertRaises(ValueError):
            list(tokens('\u00a0{}'))


if __name__ == '__main__':
    unittest.main()
