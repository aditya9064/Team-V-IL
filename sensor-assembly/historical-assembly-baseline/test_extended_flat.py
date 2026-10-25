import json,hashlib,unittest
from pathlib import Path
class ExtendedFlat(unittest.TestCase):
 def setUp(self):
  self.r=Path(__file__).parent;self.p=json.loads((self.r/'extended-flat-proof.json').read_text());self.g=json.loads((self.r/'trial-mesh.json').read_text())
 def test_larger_flat_contact_without_extra_preload_or_nominal_interference(self):
  a,b=self.p['source_sections'];self.assertGreater(b['contact']['preload']['area_mm2'],2*a['contact']['preload']['area_mm2']);self.assertGreater(b['contact']['preload']['height_mm'],5.);self.assertEqual(b['contact']['nominal']['area_mm2'],0.);self.assertEqual(b['contact']['open']['area_mm2'],0.);self.assertTrue(self.p['no_extra_preload'] and self.p['grip_commands_unchanged'])
 def test_lid_clearance_and_cooked_top_probe(self):
  self.assertGreaterEqual(self.p['lid_roof_clearance_mm'],2.5);p=[v for v in self.g['geometry_probes'] if v['part']=='base' and v['label']=='flat_base_grip_top'][0];self.assertAlmostEqual(p['expected_distance_mm'],66.6);self.assertEqual(p['maximum_distance_error_mm'],.03)
 def test_other_parts_fingers_runtime_and_strict_gates_unchanged(self):
  for n,h in self.p['other_parts_sha256'].items():self.assertEqual(hashlib.sha256(json.dumps(self.g['parts'][n],sort_keys=True).encode()).hexdigest(),h)
  for n,h in self.p['runtime_sha256'].items():self.assertEqual(hashlib.sha256((self.r/n).read_bytes()).hexdigest(),h)
  self.assertTrue(self.p['limits_unchanged'] and self.p['gripper_geometry_unchanged']);self.assertEqual(self.g['limits']['maximum_intended_penetration_mm'],.25)
