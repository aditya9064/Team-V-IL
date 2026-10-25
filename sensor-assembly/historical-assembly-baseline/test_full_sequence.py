import json,unittest
from pathlib import Path
from payload_checks import initial_supports,placement_supports,horizontal_support_pairs
from assembly_checks import role_pair,contact_violation
class FullSequenceChecks(unittest.TestCase):
 def test_three_independent_product_steps(self):
  d=json.loads(Path(__file__).with_name('design.json').read_text());self.assertEqual(d['assembly']['sequence'],['base','carrier','cover']);self.assertFalse(d['current_fingers_changed']);self.assertTrue(d['pcb_intact'] and d['pcb_separate'])
 def test_base_not_preplaced_and_support_is_phase_gated(self):
  pair=role_pair(('base','floor'),('nest','floor'));self.assertNotIn(pair,initial_supports(False));self.assertIn(pair,placement_supports()['base'])
 def test_lid_only_seats_on_base_floor(self):
  pairs=placement_supports()['cover'];self.assertEqual(pairs,{role_pair(('cover','ear'),('base','floor'))});self.assertNotIn(role_pair(('cover','wall'),('carrier','bridge_leg')),pairs)
 def test_horizontal_seating_cannot_permit_wall_or_pcb_collision(self):
  self.assertNotIn(role_pair(('cover','wall'),('carrier','wall')),horizontal_support_pairs());self.assertNotIn(role_pair(('cover','ear'),('pcb','component')),horizontal_support_pairs());self.assertTrue(contact_violation(True,-.000251,0,.25))
 def test_lid_clears_intact_tray_handle(self):
  d=json.loads(Path(__file__).with_name('design.json').read_text());self.assertGreaterEqual(d['lid_internal_clearance_above_tray_handle_mm'],2.5)
if __name__=='__main__':unittest.main()
