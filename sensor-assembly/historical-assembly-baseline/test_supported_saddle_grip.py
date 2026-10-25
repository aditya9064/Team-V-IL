"""CPU geometry/guard audits; no claim of simulator or physical success."""
import ast,copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
import cpu_saddle_audit as A
import check_supported_saddle_grip as C
import test_paired_lid_grip as F
HERE=Path(__file__).parent
NEW=json.loads((HERE/'lid-saddle-mesh.json').read_text());DESIGN=json.loads((HERE/'lid-saddle-design.json').read_text());REF=json.loads((HERE/'lid-saddle-mass-reference.json').read_text())

class GeometryTests(unittest.TestCase):
 def test_full_declared_geometry_and_natural_properties(self):self.assertTrue(A.audit(HERE)['candidate_CAD_and_natural_mass_and79_probes_validated'])
 def test_density_mass_no_hidden_old_mass(self):
  V,_,_=A.integrals(NEW['parts']['cover']['visual']);self.assertAlmostEqual(V*1e-9*1240,REF['cover']['mass_kg'],places=12);self.assertNotEqual(REF['cover']['mass_kg'],json.loads((HERE/'verified-seating-mass-reference.json').read_text())['cover']['mass_kg'])
 def test_retained_grip_neck_and_body(self):
  old=json.loads((HERE/'nest-mesh.json').read_text())['parts']['cover']['colliders'];new=NEW['parts']['cover']['colliders'];self.assertEqual(new[:53],old[:53]);self.assertEqual(new[53],old[54])
 def test_removed_guide_voids_are_explicit_probes(self):
  for p in NEW['geometry_probes']:
   if p['label'].startswith('lid_carrier_guide_inner_'):self.assertFalse(p['hit']);self.assertNotIn('expected_distance_mm',p)
 def test_unmodified_source_fingers(self):
  parent=json.loads((HERE/'v55-parent-sha256.json').read_text());self.assertEqual(A.sha(HERE/'finger_cooking.py'),parent['finger_cooking.py']);self.assertEqual(A.sha(HERE/'telemetry-finger-geometry.json'),parent['telemetry-finger-geometry.json'])
 def test_all_guards_control_functions_unchanged(self):
  def functions(text):return {x.name:ast.dump(x,include_attributes=False) for x in ast.walk(ast.parse(text)) if isinstance(x,ast.FunctionDef) and x.name!='main'}
  old=(HERE/'v55_isolated_lid_grip_trial.py').read_text();new=(HERE/'isolated_lid_grip_trial.py').read_text().replace("'SIMULATION | lid grip '+a.geometry_variant+' | current fingers'","'SIMULATION | lid post '+a.geometry_variant+' | current fingers'");self.assertEqual(functions(old),functions(new))
 def test_exact_reversible_runtime(self):
  text=(HERE/'isolated_lid_grip_trial.py').read_text()
  for p in reversed(json.loads((HERE/'v57-runtime-patches.json').read_text())):self.assertEqual(text.count(p['new']),1);text=text.replace(p['new'],p['old'])
  self.assertEqual(text,(HERE/'v55_isolated_lid_grip_trial.py').read_text())
 def test_bad_density_reference_rejected(self):
  ref=copy.deepcopy(REF['cover']);ref['mass_kg']*=1.01;V,_,_=A.integrals(NEW['parts']['cover']['visual']);self.assertGreater(abs(V*1e-9*1240-ref['mass_kg']),1e-8)

class CandidateGateTests(unittest.TestCase):
 def identity(self):
  r=F.identity_fixture();r['task']='isolated_lid_supported_saddle_diagnostic';r['grip_geometry_experiment'].update(variant='supported-saddle',extension_mm=0.);r.update(mesh_source_filename='lid-saddle-mesh.json',design_source_filename='lid-saddle-design.json',mass_reference_source_filename='lid-saddle-mass-reference.json',mesh_sha256=C.digest(HERE/'lid-saddle-mesh.json'),design_sha256=C.digest(HERE/'lid-saddle-design.json'));return r
 def physics(self):
  r=F.physics_fixture();r['installed_full_body_mass_properties']=copy.deepcopy(REF);r['separate_body_audit']['cover']['mass_kg']=NEW['parts']['cover']['volume_mm3']*1e-9*1240;r['physical_collider_counts']['cover']=58;r['cooked_geometry_probes']=[]
  for v in r['installed_full_body_mass_properties'].values():v.update(validated=True,physics_time_advanced_s=0.)
  for p in NEW['geometry_probes']:
   q={'part':p['part'],'label':p['label'],'expected_part_hit':p['hit'],'actual_part_hit':p['hit'],'distance_pass':True}
   if p['hit']:q.update(maximum_distance_error_mm=.03,expected_distance_mm=p['expected_distance_mm'],actual_distance_mm=p['expected_distance_mm'])
   r['cooked_geometry_probes'].append(q)
  return r
 def test_candidate_identity(self):C.check_identity(self.identity(),NEW,HERE,1,'supported-saddle')
 def test_candidate_mass_and_cooked_recipe(self):C.check_original_physics(self.physics(),NEW,DESIGN,True,HERE,'supported-saddle')
 def test_old_mass_cannot_stand_for_new(self):
  r=self.physics();r['installed_full_body_mass_properties']['cover']=F.physics_fixture()['installed_full_body_mass_properties']['cover']
  with self.assertRaises(C.AuditError):C.check_original_physics(r,NEW,DESIGN,True,HERE,'supported-saddle')
 def test_incorrect_CAD_binding_rejected(self):
  r=self.identity();r['mesh_sha256']=C.digest(HERE/'nest-mesh.json')
  with self.assertRaises(C.AuditError):C.check_identity(r,NEW,HERE,1,'supported-saddle')
 def test_incorrect_stillness_rejected(self):
  r=F.motion_fixture();r['checkpoints'][-1]['stillness_trace'][0]['maximum_raw_joint_speed_rad_s']=.021
  with self.assertRaises(C.AuditError):C.check_motion(r,DESIGN,True)
 def test_raw_contact_limit_stays_strict(self):
  r=self.physics();r['contact_summary']['synthetic grip']['minimum_separation_mm']=-.250001
  with self.assertRaises(C.AuditError):C.check_original_physics(r,NEW,DESIGN,True,HERE,'supported-saddle')
 def test_source_recipe_overridden_probe_rejected(self):
  r=self.physics();probe=next(p for p in r['cooked_geometry_probes'] if p['label']=='lid_load_shoulder_top');probe['expected_distance_mm']=probe['actual_distance_mm']=81.64
  with self.assertRaises(C.AuditError):C.check_original_physics(r,NEW,DESIGN,True,HERE,'supported-saddle')
 def test_false_empty_guide_hit_rejected(self):
  r=self.physics();probe=next(p for p in r['cooked_geometry_probes'] if p['label']=='lid_carrier_guide_inner_1');probe['expected_part_hit']=probe['actual_part_hit']=True
  with self.assertRaises(C.AuditError):C.check_original_physics(r,NEW,DESIGN,True,HERE,'supported-saddle')
 def test_PCB_unchanged_gate(self):
  r=self.physics();r['separate_body_audit']['pcb']['kinematic']=True
  with self.assertRaises(C.AuditError):C.check_original_physics(r,NEW,DESIGN,True,HERE,'supported-saddle')

if __name__=='__main__':unittest.main()
