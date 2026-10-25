import json,unittest
from pathlib import Path
from assembly_checks import classify_contact,contact_violation,role_pair
from payload_checks import initial_supports,placement_supports
class CompactFixtureChecks(unittest.TestCase):
 def test_only_specific_nest_support(self):
  state=dict(active='carrier',allow_grip=True,allow_active_table=False);a=['/World/Assembly_nest','/World/Assembly_base'];c=['/World/Assembly_nest/C_000_floor','/World/Assembly_base/C_000_floor']
  self.assertTrue(classify_contact(a,c,[('nest','floor'),('base','floor')],state,initial_supports(),'/World/Robot')[0])
  self.assertFalse(classify_contact(a,c,[('nest','wall'),('base','wall')],state,initial_supports(),'/World/Robot')[0])
 def test_fixture_does_not_allow_finger_contact(self):
  self.assertFalse(classify_contact(['/World/Assembly_nest','/World/Robot/JawL'],['nestfloor','jawpart'],[('nest','floor'),None],dict(active='carrier',allow_grip=True,allow_active_table=False),initial_supports(),'/World/Robot')[0])
 def test_destination_support_phase_gated(self):
  pair=role_pair(('base','floor'),('carrier','floor'));self.assertNotIn(pair,initial_supports());self.assertIn(pair,placement_supports()['carrier'])
 def test_retainer_fixture_starts_board_on_floor(self):
  g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text());self.assertEqual(g['payload_initial_gap_mm'],0.0)
 def test_retaining_rim_only_contacts_substrate(self):
  supports=initial_supports();self.assertIn(role_pair(('carrier','pcb_keeper'),('pcb','substrate')),supports);self.assertNotIn(role_pair(('carrier','pcb_keeper'),('pcb','component')),supports)
 def test_same_strict_limits_and_loose_pcb(self):
  g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text());d=json.loads(Path(__file__).with_name('design.json').read_text())
  self.assertEqual(g['limits']['maximum_intended_penetration_mm'],.25);self.assertTrue(contact_violation(True,-.000251,0,.25));self.assertTrue(d['pcb_intact'] and d['pcb_separate']);self.assertEqual(g['limits']['camera_speed_rad_s'],.02)
if __name__=='__main__':unittest.main()
