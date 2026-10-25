import unittest,json,hashlib
from pathlib import Path
class FlatLid(unittest.TestCase):
 def setUp(self):
  self.root=Path(__file__).parent;self.old=json.loads((self.root/'trial-mesh.json').read_text());self.new=json.loads((self.root/'flat-lid-mesh.json').read_text());self.d=json.loads((self.root/'flat-lid-design.json').read_text());self.p=json.loads((self.root/'flat-lid-proof.json').read_text())
 def test_only_cover_changed(self):
  for n in ['base','carrier','pcb','nest']:self.assertEqual(self.old['parts'][n],self.new['parts'][n],n)
  for k in ['limits','finger_profile_sha256','payload_initial_gap_mm']:self.assertEqual(self.old[k],self.new[k],k)
  for n in ['base','carrier']:self.assertEqual(self.old['grip_commands'][n],self.new['grip_commands'][n])
 def test_same_proven_geometry_derived_closure(self):
  a=self.new['grip_commands']['carrier'];b=self.new['grip_commands']['cover']
  for k in ['nominal_contact_per_jaw_mm','closed_grasp_command_per_jaw_m','extra_travel_per_jaw_mm','penetration_stop_limit_mm']:self.assertEqual(a[k],b[k],k)
  self.assertEqual(b['extra_travel_per_jaw_mm'],.05);self.assertEqual(b['penetration_stop_limit_mm'],.25)
 def test_unchanged_housing_and_density_natural_mass(self):
  self.assertEqual(self.p['nonhandle_cover_difference_mm3'],0);self.assertLess(self.p['collision_visual_difference_mm3'],.05);self.assertEqual(self.p['density_kg_m3'],1240);self.assertAlmostEqual(self.p['new_cover_mass_kg'],self.new['parts']['cover']['volume_mm3']*1e-9*1240)
  self.assertEqual(json.loads((self.root/'design.json').read_text())['assembly'],self.d['assembly'])
 def test_lid_grip_is_translated_carrier_grip(self):
  def verts(n):return sorted(tuple(round(v,5) for v in p) for c in self.new['parts'][n]['colliders'] if c['role'] in ['grip_stem','grip_support'] for p in c['vertices_mm'])
  self.assertEqual([(x,y,round(z+24,5)) for x,y,z in verts('carrier')],verts('cover'))
  self.assertAlmostEqual(self.d['cover']['tcp_above_bottom_mm']-self.d['carrier']['tcp_above_bottom_mm'],24.)
 def test_probe_and_hash_contract(self):
  self.assertEqual(self.new['design_sha256'],hashlib.sha256((self.root/'flat-lid-design.json').read_bytes()).hexdigest());self.assertEqual(len(self.new['geometry_probes']),70)
  probes=[p for p in self.new['geometry_probes'] if p['part']=='cover' and p['label'].startswith('lid_')];self.assertEqual(len(probes),9)
  self.assertTrue(all(p['maximum_distance_error_mm']==.03 for p in probes))
