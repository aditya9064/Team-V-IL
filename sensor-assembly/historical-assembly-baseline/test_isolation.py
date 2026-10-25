import ast,hashlib,json,unittest,copy
from pathlib import Path
import numpy as np
from isolation_tools import fixture_offset,classify_terminal,comparable
R=Path(__file__).parent
class Isolation(unittest.TestCase):
 def test_complete_reverse_diff_recovers_exact_reference(self):
  p=json.loads((R/'isolation-provenance.json').read_text());s=(R/'base_seating_diagnostic.py').read_text()
  for a,b in reversed(p['patches']):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(R/'holder_assembly_trial.py').read_text());self.assertEqual(hashlib.sha256(s.encode()).hexdigest(),p['reference_runner_sha256'])
 def test_motion_grip_endpoint_stillness_functions_exactly_unchanged(self):
  def functions(p):return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(p.read_text())) if isinstance(n,ast.FunctionDef)}
  a,b=functions(R/'holder_assembly_trial.py'),functions(R/'base_seating_diagnostic.py')
  for name in ('move','command','feedback','save','fail_contacts','poses'):self.assertEqual(a[name],b[name],name)
 def test_collision_rules_cad_grippers_and_full_checker_pinned(self):
  p=json.loads((R/'isolation-provenance.json').read_text())
  for n,h in p['unchanged_files'].items():self.assertEqual(hashlib.sha256((R/n).read_bytes()).hexdigest(),h,n)
  g=json.loads((R/'trial-mesh.json').read_text());self.assertEqual(g['limits']['maximum_intended_penetration_mm'],.25);self.assertEqual(g['limits']['endpoint_mm'],2);self.assertEqual(g['limits']['camera_speed_rad_s'],.02)
 def test_fixture_intervention_fixed_and_unknown_rejected(self):
  np.testing.assert_array_equal(fixture_offset('present'),[0,0,0]);np.testing.assert_array_equal(fixture_offset('removed'),[1,0,0])
  with self.assertRaises(ValueError):fixture_offset('unknown')
 def test_pre_lower_failures_and_unfinished_runs_are_inconclusive(self):
  self.assertEqual(classify_terminal({'status':'FAILED'}),'INCONCLUSIVE_BEFORE_LOWERING');self.assertEqual(classify_terminal({'status':'RUNNING','diagnostic':{'lowering_started':True}}),'INCONCLUSIVE_NONTERMINAL')
 def test_completed_lowering_cannot_hide_a_collision(self):
  with self.assertRaises(ValueError):classify_terminal({'status':'DIAGNOSTIC_LOWERING_COMPLETED','diagnostic':{'lowering_started':True},'unexpected_contacts':[1]})
 def fixture(self):return {'diagnostic':{'table_top_z_m':.9},'checkpoints':[{'phase':'1_base_carry','tip_robot_m':[.3,.1,.15],'poses':{'base':{'position_world_m':[.284,.1,1.03],'tilt_deg':1.5,'yaw_deg':0}},'jaw_joints':{'jaw_l':-.004,'jaw_r':.004}}]}
 def test_identical_preintervention_poses_compare(self):self.assertTrue(comparable(self.fixture(),self.fixture())['comparable'])
 def test_excessive_held_tilt_or_jaw_difference_rejects_causal_comparison(self):
  a=self.fixture();b=copy.deepcopy(a);b['checkpoints'][0]['poses']['base']['tilt_deg']+=.3;self.assertFalse(comparable(a,b)['comparable'])
  b=copy.deepcopy(a);b['checkpoints'][0]['jaw_joints']['jaw_l']+=.00006;self.assertFalse(comparable(a,b)['comparable'])

class PlacementPrediction(unittest.TestCase):
 def test_known_half_mm_overshoot_without_changing_input(self):
  from isolation_tools import predict_free_held_placement
  shot={'tip_robot_m':[0,0,.0138],'poses':{'base':{'position_world_m':[0,0,.9],'orientation_wxyz':[1,0,0,0]}}};before=copy.deepcopy(shot)
  p=predict_free_held_placement(shot,[0,0,.0163],.9,.903,[[-1,-1,0],[1,1,0],[1,1,3]],13.2)
  self.assertAlmostEqual(p['predicted_base_origin_gap_to_support_mm'],-.5);self.assertAlmostEqual(p['captured_minus_cad_mm'],.6);self.assertEqual(shot,before)
 def test_invalid_pose_rejected(self):
  from isolation_tools import predict_free_held_placement
  shot={'tip_robot_m':[0,0,.0138],'poses':{'base':{'position_world_m':[0,0,float('nan')],'orientation_wxyz':[1,0,0,0]}}}
  with self.assertRaises(ValueError):predict_free_held_placement(shot,[0,0,.0163],.9,.903,[[0,0,0]],13.2)
