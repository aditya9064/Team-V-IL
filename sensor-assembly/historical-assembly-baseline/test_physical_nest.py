import unittest,json,hashlib,ast
from pathlib import Path
from fixture_contacts import guide_contact
from assembly_checks import contact_violation
class PhysicalNest(unittest.TestCase):
 def setUp(self):
  self.p=Path(__file__).parent;self.old=json.loads((self.p/'flat-lid-mesh.json').read_text());self.g=json.loads((self.p/'nest-mesh.json').read_text());self.d=json.loads((self.p/'nest-design.json').read_text());self.proof=json.loads((self.p/'nest-proof.json').read_text())
 def test_only_nest_geometry_changed(self):
  for n in ['base','carrier','pcb','cover']:self.assertEqual(self.g['parts'][n],self.old['parts'][n],n)
  for k in ['limits','payload_limits','grip_commands','layout_m','finger_profile_sha256','grip_collision_union_audit','payload_initial_gap_mm']:self.assertEqual(self.g[k],self.old[k],k)
 def test_no_locks_or_attachments_and_no_clamp_claim(self):
  self.assertFalse(self.d['current_fingers_changed']);self.assertTrue(self.d['pcb_intact'] and self.d['pcb_separate']);n=self.d['nest_fixture'];self.assertFalse(n['clamps'] or n['base_attachment']);self.assertEqual(n['support_top_mm'],3)
  a=(self.p/'gentle_full_assembly_trial.py').read_text();b=(self.p/'nest_full_assembly_trial.py').read_text()
  for before,after in reversed(json.loads((self.p/'nest-runtime-patches.json').read_text())):
   self.assertEqual(b.count(after),1);b=b.replace(after,before)
  self.assertEqual(a,b)
 def test_original_clearance_mouth_and_nominal_sweeps(self):
  self.assertGreater(self.proof['entry_mouth_minimum_margin_mm'],.4);self.assertGreater(self.proof['finger_clearance_lower_bound_mm'],.4);self.assertTrue(self.proof['com_inside_support_triangle']);self.assertEqual(len(self.proof['nominal_vertical_insertion_sweeps']),40)
  for c in self.proof['nominal_vertical_insertion_sweeps']:self.assertLess(c['intersection_mm3'],.0001)
 def test_permitted_guide_face_is_phase_gated_and_pair_exact(self):
  c=self.proof['guide_pair_contracts'][0];paths=[f'/World/Assembly_nest/C_{c["index"]:03d}_fixture_guide','/World/Assembly_base/C_000_floor'];roles=[('nest','fixture_guide'),('base','floor')]
  self.assertTrue(guide_contact(paths,roles,{'fixture_entry_enabled':True},[[1,0,0]]));self.assertFalse(guide_contact(paths,roles,{'fixture_entry_enabled':False},[[1,0,0]]));self.assertFalse(guide_contact(paths,roles,{'fixture_entry_enabled':True},[[0,1,0]]));self.assertFalse(guide_contact(paths,roles,{'fixture_entry_enabled':True},[[0,0,1]]))
  self.assertFalse(guide_contact([paths[0],'/World/Assembly_base/C_018_grip_root'],[roles[0],('base','grip_root')],{'fixture_entry_enabled':True},[[1,0,0]]))
 def test_never_permit_finger_tray_lid_or_unmapped_contacts(self):
  c=self.proof['guide_pair_contracts'][0];a=f'/World/Assembly_nest/C_{c["index"]:03d}_fixture_guide'
  for name in ['carrier','cover','pcb']:
   self.assertFalse(guide_contact([a,f'/World/Assembly_{name}/C_000_floor'],[('nest','fixture_guide'),(name,'floor')],{'fixture_entry_enabled':True},[[1,0,0]]))
  self.assertFalse(guide_contact([a,'/World/Robot/JawL/C'],[('nest','fixture_guide'),None],{'fixture_entry_enabled':True},[[1,0,0]]))
 def test_numeric_contact_negative_controls(self):
  self.assertTrue(contact_violation(True,-.000251,0,.25));self.assertFalse(contact_violation(True,-.000249,0,.25));self.assertTrue(contact_violation(False,.0001,1e-7,.25))
 def test_geometry_hash_and_probes(self):
  self.assertEqual(self.g['design_sha256'],hashlib.sha256((self.p/'nest-design.json').read_bytes()).hexdigest());self.assertEqual(len(self.g['geometry_probes']),73)
  for p in self.g['geometry_probes']:self.assertEqual(p['maximum_distance_error_mm'],.03) if p.get('hit') else None
if __name__=='__main__':unittest.main()
