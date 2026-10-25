import unittest,json
from pathlib import Path
from payload_checks import initial_supports,placement_supports,horizontal_seating_normals
from assembly_checks import role_pair,classify_contact,contact_violation
class SeatingSupportChecks(unittest.TestCase):
 def test_tabs_only_supported_after_seating_stage(self):
  pair=role_pair(('carrier','floor'),('base','floor'));self.assertNotIn(pair,initial_supports());self.assertIn(pair,placement_supports()['carrier'])
  self.assertNotIn(role_pair(('carrier','floor'),('base','wall')),placement_supports()['carrier'])
 def test_lateral_or_malformed_contact_remains_forbidden(self):
  self.assertTrue(horizontal_seating_normals([[0,0,1],[0,0,-1]]))
  for v in [[[0,1,0]],[[0,.03,.99955]],[],[[0,0,float('nan')]],[[0,0,2]]]:self.assertFalse(horizontal_seating_normals(v))
 def test_same_penetration_limit_for_correct_support_pair(self):
  self.assertTrue(contact_violation(True,-.000251,0,.25));self.assertFalse(contact_violation(True,-.000249,0,.25))
 def test_cad_seating_has_positive_exact_support_area(self):
  proof=json.loads(Path(__file__).with_name('seating-support-proof.json').read_text());self.assertEqual(len(proof),3)
  for item in proof:
   self.assertAlmostEqual(item['moving_surface_z_mm']+item['moving_bottom_z_mm'],item['fixed_surface_z_mm']+item['fixed_bottom_z_mm']);self.assertGreater(item['support_area_mm2'],0)
  self.assertAlmostEqual(proof[1]['support_area_mm2'],384.5)
if __name__=='__main__':unittest.main()
