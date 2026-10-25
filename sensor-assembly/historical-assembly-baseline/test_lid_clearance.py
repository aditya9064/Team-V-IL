import ast,hashlib,json,unittest
from pathlib import Path
class LidClearance(unittest.TestCase):
 def setUp(self):
  self.r=Path(__file__).parent;self.old=json.loads((self.r/'trial-mesh.json').read_text());self.new=json.loads((self.r/'lid-clearance-mesh.json').read_text());self.d=json.loads((self.r/'lid-clearance-design.json').read_text());self.proof=json.loads((self.r/'lid-clearance-proof.json').read_text())
 def test_only_lid_part_data_changed_and_pcb_is_identical(self):
  for n in ['base','carrier','pcb','nest']:self.assertEqual(self.old['parts'][n],self.new['parts'][n],n)
  self.assertEqual(self.old['limits'],self.new['limits']);self.assertEqual(self.old['grip_commands'],self.new['grip_commands']);self.assertEqual(self.old['finger_profile_sha256'],self.new['finger_profile_sha256'])
 def test_clearance_margin_and_material_not_artificially_changed(self):
  p=self.proof;self.assertAlmostEqual((p['new_handle_span_mm']-p['unchanged_finger_thickness_mm'])/2,1.4);self.assertLess(p['original_side_clearance_mm']-.08,p['combined_contact_envelope_mm']);self.assertGreater(p['new_side_clearance_mm']-.75,p['combined_contact_envelope_mm']);self.assertEqual(p['nonhandle_cover_difference_mm3'],0);self.assertEqual(p['material_density_kg_m3'],1240);self.assertAlmostEqual(p['mass_after_kg'],self.new['parts']['cover']['volume_mm3']*1e-9*1240)
 def test_declared_cad_hash_and_all_new_probe_coverage(self):
  self.assertEqual(self.new['design_sha256'],hashlib.sha256((self.r/'lid-clearance-design.json').read_bytes()).hexdigest());self.assertEqual(len(self.new['geometry_probes']),66)
  probes=[v for v in self.new['geometry_probes'] if v['label'].startswith('axial_finger_clearance')];self.assertEqual(len(probes),2)
  for v in probes:self.assertEqual(v['expected_distance_mm'],1.4);self.assertEqual(v['maximum_distance_error_mm'],.03)
 def test_isolated_scope_exact_source_reversal_and_checks(self):
  proof=json.loads((self.r/'isolated-lid-provenance.json').read_text());s=(self.r/'isolated_lid_trial.py').read_text()
  for a,b in reversed(proof['replacements']):self.assertEqual(s.count(b),1,b[:60]);s=s.replace(b,a)
  self.assertEqual(s,(self.r/'gentle_full_assembly_trial.py').read_text());self.assertEqual(hashlib.sha256(s.encode()).hexdigest(),proof['parent_sha256'])
 def test_protected_contact_camera_payload_motion_and_release_functions_identical(self):
  p=self.r;sources=[(p/n).read_text() for n in ['gentle_full_assembly_trial.py','isolated_lid_trial.py']];sources[0]=sources[0].replace('SIMULATION | base > loaded tray > lid | current fingers','SIMULATION | isolated lid trial | current fingers');trees=[ast.parse(s) for s in sources]
  for name in ['move','on_contacts','fail_contacts','advance','command','feedback','save','poses','score_placed','measure_held']:
   f=[next(n for n in ast.walk(t) if isinstance(n,ast.FunctionDef) and n.name==name) for t in trees];self.assertEqual(ast.dump(f[0]),ast.dump(f[1]),name)
 def test_housing_mating_height_and_lid_grip_radius_tcp_unchanged(self):
  old=json.loads((self.r/'design.json').read_text());self.assertEqual(old['assembly'],self.d['assembly']);self.assertEqual(old['cover']['tcp_above_bottom_mm'],self.d['cover']['tcp_above_bottom_mm']);self.assertEqual(old['grip']['bar_radius_mm'],self.d['grip']['bar_radius_mm']);self.assertEqual(old['grip']['flange_radius_mm'],self.d['grip']['flange_radius_mm'])
