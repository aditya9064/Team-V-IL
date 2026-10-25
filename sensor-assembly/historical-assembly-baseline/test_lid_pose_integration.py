import ast,hashlib,json,unittest
from pathlib import Path
from lid_diagnostic_contract import expected_phases,expected_checkpoints,validate_foot_budget
class LidIntegration(unittest.TestCase):
 def setUp(self):self.p=Path(__file__).parent;self.src=(self.p/'lid_pose_trial.py').read_text()
 def test_reversible_declared_runtime_patch(self):
  candidate=self.src
  for a,b in reversed(json.loads((self.p/'lid-pose-runtime-patches.json').read_text())):
   self.assertEqual(candidate.count(b),1);candidate=candidate.replace(b,a)
  self.assertEqual(candidate,(self.p/'gentle_lid_full_assembly_trial.py').read_text())
 def test_reversible_checker_keeps_parent_assertions(self):
  candidate=(self.p/'check_lid_pose.py').read_text()
  for a,b in reversed(json.loads((self.p/'lid-pose-checker-patches.json').read_text())):
   self.assertEqual(candidate.count(b),1);candidate=candidate.replace(b,a)
  self.assertEqual(candidate,(self.p/'check_gentle_lid_full.py').read_text())
 def test_geometry_physics_hardware_unchanged(self):
  c=json.loads((self.p/'lid-pose-integration-contract.json').read_text())
  for n,h in c['unchanged_assets'].items():self.assertEqual(hashlib.sha256((self.p/n).read_bytes()).hexdigest(),h,n)
 def test_exact_stage_counts_and_order(self):
  for stage,moves,shots in [('precontact',37,46),('seat',38,47),('release',41,51)]:
   self.assertEqual(len(expected_phases(stage)),moves)
   self.assertEqual(len(expected_checkpoints(stage)),shots)
   self.assertEqual(len(set(expected_phases(stage))),moves)
  self.assertEqual(expected_phases('seat')[-1],'3_cover_lower')
  self.assertNotIn('3_cover_release',expected_phases('seat'))
 def test_contact_and_core_acceptance_exact_parent(self):
  trees=[ast.parse((self.p/n).read_text()) for n in ['gentle_lid_full_assembly_trial.py','lid_pose_trial.py']]
  for name in ['on_contacts','fail_contacts','feedback','poses','measure_held','score_placed','check_inventory']:
   v=[next(x for x in ast.walk(t) if isinstance(x,ast.FunctionDef) and x.name==name) for t in trees]
   self.assertEqual(ast.dump(v[0]),ast.dump(v[1]),name)
 def test_prefix_control_branch_and_motion_generator_preserved(self):
  original=(self.p/'gentle_lid_full_assembly_trial.py').read_text()
  for start,end in [("        for number,name in enumerate(ORDER,1):","            if name=='base':\n                observed="),("            if name=='base':\n                observed=","            elif name=='cover':")]:
   a=original[original.index(start):original.index('            else:',original.index(start))] if 'observed=' in start else original[original.index(start):original.index(end)]
   self.assertIn(a,self.src)
  self.assertIn("samples=(lid_release_samples(duration,1/60) if label=='3_cover_release'",self.src)
  self.assertIn("3. if name=='cover' else 1.",self.src)
 def test_actual_issued_jaw_and_readonly_body_velocity_telemetry(self):
  self.assertIn("'commanded_jaw_per_side_m':float(issued_jaw)",self.src)
  self.assertIn("issued_jaw=float(j);issued_jaw_targets=",self.src)
  self.assertIn('get_linear_velocity()',self.src);self.assertIn('get_angular_velocity()',self.src)
  self.assertNotIn('set_linear_velocity',self.src);self.assertNotIn('set_angular_velocity',self.src)
 def test_fk_audit_result_contract_and_real_support_geometry(self):
  from foot_audit import observed_tcp_tracking_foot_errors,error_budget_at_feet
  mesh=json.loads((self.p/'nest-mesh.json').read_text());I=[[1,0,0],[0,1,0],[0,0,1]]
  pose={'position_world_m':[.3,.1,.9009],'rotation_world':I}
  base={'position_world_m':[.3,.1,.8979],'rotation_world':I}
  v=observed_tcp_tracking_foot_errors(mesh,pose,[.3,.1,.9382],I,[.3,.1,.93821],I,base)
  self.assertAlmostEqual(v['maximum_absolute_height_error_mm'],.01,places=7)
  b=error_budget_at_feet(mesh,[.01,.01,.01],.01,.01,.01,.01,.02)
  self.assertGreater(b['maximum_height_error_mm'],.05)
 def test_unknown_budget_blocks_and_explicit_evidence_required(self):
  for bad in [{},{'qualified':False}]:
   with self.assertRaises(ValueError):validate_foot_budget(bad)
  b=self.budget()
  self.assertEqual(validate_foot_budget(b),b)
  b['maximum_fk_position_error_mm']=.051
  with self.assertRaises(ValueError):validate_foot_budget(b)
 def budget(self):
  b={'kind':'PRECONTACT_ONLY_SIMULATION','qualified':True,'fit_foot_height_bounds_mm':[.05]*4,'calibration_height_error_mm':0.,'hold_drift_position_error_mm':.02,'maximum_fk_position_error_mm':.05,'support_z_error_mm':.35,'support_tilt_deg':1.,'support_xy_error_mm':.75,'fit_position_error_mm':.15,'fit_rotation_error_deg':.25}
  b['evidence']={k:'Unit-test fixture only, not actual qualification' for k in b if k not in ('kind','qualified')}
  return b
 def test_precontact_budget_cannot_authorize_final_seat_or_release(self):
  from lid_diagnostic_contract import precontact_budget
  from foot_audit import all_foot_gaps,qualify_prediction
  mesh=json.loads((self.p/'nest-mesh.json').read_text());b=self.budget();I=[[1,0,0],[0,1,0],[0,0,1]]
  budget=precontact_budget(mesh,b)
  self.assertAlmostEqual(budget['maximum_height_error_mm'],1.0659520888,places=6)
  for stage in ('seat','release'):
   with self.assertRaises(ValueError):validate_foot_budget(b,stage)
  base={'position_world_m':[0,0,0],'rotation_world':I}
  for gap,eligible in [(1.10,True),(.10,False)]:
   lid={'position_world_m':[0,0,(3+gap)*.001],'rotation_world':I}
   self.assertEqual(qualify_prediction(all_foot_gaps(mesh,lid,base),budget)['geometric_dispatch_eligible'],eligible)
 def test_full_measured_lid_orientation_retained_across_all_later_phases(self):
  import numpy as np
  from scipy.spatial.transform import Rotation
  from lid_diagnostic_contract import orientation_for_lid_phase
  R=Rotation.from_euler('xyz',[.2,-.3,.1],degrees=True).as_matrix()
  for phase in ['level_held','precontact','lower','release','side_exit','retreat']:
   selected=orientation_for_lid_phase('3_cover_'+phase,None,None,R)
   np.testing.assert_allclose(selected@np.array([0,0,1]),R@np.array([0,0,1]))
   np.testing.assert_allclose(selected@np.array([0,1,0]),R@np.array([0,1,0]))
   self.assertNotEqual(float((selected@np.array([0,1,0]))[2]),0.)
  self.assertIsNone(orientation_for_lid_phase('1_base_release',None,None,R))
  self.assertIsNone(orientation_for_lid_phase('2_carrier_lower',None,None,R))
  self.assertIn('new_orientation=orientation_for_lid_phase(label,new_orientation,new_spread,lid_orientation)',self.src)
  self.assertIn('Lid observed-FK conservative foot-clearance veto',self.src)
  self.assertIn('Lid grasp per-foot3D drift veto',self.src)
  self.assertIn("if label=='3_cover_leveled_inspection' else None)",self.src)
if __name__=='__main__':unittest.main()
