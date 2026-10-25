import json,hashlib,unittest
from pathlib import Path
class CarrierGuides(unittest.TestCase):
 def setUp(self):
  self.r=Path(__file__).parent;self.p=json.loads((self.r/'carrier-guide-proof.json').read_text());self.g=json.loads((self.r/'trial-mesh.json').read_text())
 def test_all_other_parts_and_runtime_are_unchanged(self):
  for n,h in self.p['unchanged_parts_sha256'].items():self.assertEqual(hashlib.sha256(json.dumps(self.g['parts'][n],sort_keys=True).encode()).hexdigest(),h)
  for n,h in self.p['protected_runtime_sha256'].items():self.assertEqual(hashlib.sha256((self.r/n).read_bytes()).hexdigest(),h)
  self.assertTrue(self.p['limits_unchanged'] and self.p['payload_limits_unchanged'])
 def test_opening_clearance_and_misalignment_negative(self):
  self.assertEqual(self.p['slot_mm'],6.4);self.assertEqual(self.p['blade_mm'],6.);self.assertEqual(self.p['per_side_mm'],.2)
  self.assertTrue(all(v['interference_mm3']<1e-5 for v in self.p['opening_sweeps'][:3]));self.assertGreater(self.p['opening_sweeps'][-1]['interference_mm3'],1.)
 def test_original_closure_preload_and_penetration_gate(self):
  c=self.g['grip_commands']['carrier'];self.assertAlmostEqual(c['nominal_contact_per_jaw_mm'],2.0000152587890625);self.assertEqual(c['extra_travel_per_jaw_mm'],.05);self.assertEqual(c['penetration_stop_limit_mm'],.25);self.assertAlmostEqual(c['closed_grasp_command_per_jaw_m'],.0019500152587890625)
