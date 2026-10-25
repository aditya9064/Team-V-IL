import unittest
from passive_sleep_state import UnavailableSleepState
class Unavailable(unittest.TestCase):
 def test_constructor_failure_stays_unknown_without_engine(self):
  o=UnavailableSleepState('bad canonical identity')
  for i in range(1,5):self.assertFalse(o.capture(i,i/60,'motion')['valid'])
  e=o.evidence();self.assertEqual(len(e['records']),4);self.assertFalse(e['all_samples_valid']);self.assertFalse(e['acceptance_policy_changed']);self.assertTrue(all(x['sleeping'] is None for x in e['records']))
