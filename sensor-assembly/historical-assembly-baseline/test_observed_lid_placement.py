import ast,copy,hashlib,json,types,unittest
from pathlib import Path
import numpy as np
from unittest.mock import Mock
import check_observed_lid_placement as C
from placement_policy import CONTRACT,PHASES,DURATIONS
from foot_audit import support_geometry
HERE=Path(__file__).parent
TEXT=(HERE/'original_lid_placement_trial.py').read_text();TREE=ast.parse(TEXT);MESH=json.loads((HERE/'nest-mesh.json').read_text())

def extract(name):return next(n for n in ast.walk(TREE) if isinstance(n,ast.FunctionDef) and n.name==name)
def samples():
 corners=np.array([v for f in support_geometry(MESH) for v in f['corners_mm']]);out=[]
 for i in range(120):
  p=[.00015*(i-95)/24 if i>=95 else 0.,0.,1.];tip=[0.,0.,1.]
  out.append({'part_rotation_world':np.eye(3).tolist(),'tool_rotation_world':np.eye(3).tolist(),'part_position_world_m':p,'tip_world_m':tip,'corners_tool_mm':(corners+[(p[0]-tip[0])*1000,0,0]).tolist()})
 return out

class OutcomePolicy(unittest.TestCase):
 def test_old_rocking_recorded_not_failed(self):
  r=C.historical_hold(samples(),MESH);self.assertGreater(r['fixed_mm'],.02);self.assertTrue(r['exceeded_old_limit']);self.assertFalse(r['terminates_this_diagnostic'])
 def test_no_motion_also_recorded(self):
  s=samples()
  for q in s:q['part_position_world_m']=[0,0,1];q['corners_tool_mm']=[v for f in support_geometry(MESH) for v in f['corners_mm']]
  self.assertFalse(C.historical_hold(s,MESH)['exceeded_old_limit'])
 def test_forged_sample_rejected(self):
  s=samples();s[119]['corners_tool_mm'][0][0]+=1
  with self.assertRaises(ValueError):C.historical_hold(s,MESH)
 def test_actual_alignment_miss_fails(self):
  p={'position_world_m':[.301,.1,.9009],'orientation_wxyz':[1,0,0,0]};self.assertFalse(C.score(p,[.30,.1,.9009])['pass'])
 def test_actual_pose_inside_original_envelope(self):
  p={'position_world_m':[.3001,.1,.901],'orientation_wxyz':[1,0,0,0]};self.assertTrue(C.score(p,[.30,.1,.9009])['pass'])
 def test_nonfinite_pose_rejected(self):
  with self.assertRaises(ValueError):C.score({'position_world_m':[0,0,float('nan')],'orientation_wxyz':[1,0,0,0]},[0,0,0])
 def test_qualify_false_does_not_terminate(self):
  report={'incoming_lid_stability':{'samples':[],'expected_arm_joints':[],'expected_jaw_joint_targets_m':{}},'lid_pose_foot_budget':{'hold_drift_position_error_mm':.02},'old_criterion_advisories':[]}
  scope={'report':report,'state':{'step':1},'incoming_corners':[],'persist':lambda:None,'qualify_incoming':lambda *a:{'pass':False,'maximum_pairwise_corner_diameter_mm':.15}}
  exec(compile(ast.Module(body=[extract('qualify_incoming_hold')],type_ignores=[]),'reviewed-helper','exec'),scope);scope['qualify_incoming_hold']('fixed_hold_result')
  self.assertFalse(report['old_criterion_advisories'][0]['terminates_this_diagnostic']);self.assertFalse(report['incoming_lid_stability']['fixed_hold_result']['pass'])
 def test_policy_is_not_full_assembly(self):self.assertFalse(CONTRACT['full_assembly_accepted']);self.assertEqual(CONTRACT['old_hold_policy'],'MEASURE_ONLY_NOT_TERMINATING')
 def test_actual_SE3_command_stop_not_jam(self):self.assertEqual(C.stop_category('Lid prospective command SE3 foot-residual veto before issue'),'TARGET_COMMAND_OR_EVIDENCE_INVALID_NOT_OBSERVED_PLACEMENT_FAILURE')
 def test_RGB_validity_stop_not_jam(self):self.assertEqual(C.stop_category('Lid RGB independent accuracy/0.05mm foot-height check failed'),'TARGET_COMMAND_OR_EVIDENCE_INVALID_NOT_OBSERVED_PLACEMENT_FAILURE')
 def test_tangential_impulse_not_bearing(self):self.assertEqual(C.bearing_impulse([0,0,1],[.1,0,0]),(0.,0.))
 def test_normal_impulse_is_bearing(self):self.assertEqual(C.bearing_impulse([0,0,-1],[0,0,.1]),(.1,.1))

class FrozenSource(unittest.TestCase):
 def test_original_guards_unchanged(self):
  old=ast.parse((HERE/'v57_original_parent.py').read_text());names=['feedback','poses','command','save','fail_contacts','score_placed','check_inventory','observed_command_fk','probe']
  for name in names:
   original=next(n for n in ast.walk(old) if isinstance(n,ast.FunctionDef) and n.name==name);self.assertEqual(ast.dump(extract(name)),ast.dump(original),name)
 def test_runtime_reverses_exactly(self):
  text=TEXT
  for p in reversed(json.loads((HERE/'v58-runtime-patches.json').read_text())):
   self.assertEqual(text.count(p['new']),p.get('count',1),p['new'][:80]);text=text.replace(p['new'],p['old'])
  self.assertEqual(text,(HERE/'v57_original_parent.py').read_text())
 def test_old_stop_messages_absent(self):
  for x in ['incoming_lid_grasp_unstable_before_measurement','Lid grasp per-foot3D drift veto','Lid observed-FK conservative foot-clearance veto','Lid conservative positive-clearance budget failed','Only lid grip is reachable in isolated diagnostic']:
   self.assertNotIn(x,TEXT)
 def test_single_release_reference(self):
  targets=[n for n in ast.walk(TREE) if isinstance(n,ast.Assign) and any(ast.unparse(t)=="state['released_reference']" for t in n.targets)];self.assertEqual(len(targets),1)
 def test_release_reference_kept_through_inspection(self):
  transition=TEXT.index("report['held_to_release_transition']");tail=TEXT[transition:TEXT.index('    except Exception as e:',transition)];self.assertNotIn("state['released_reference']=None",tail);self.assertIn("state['phase']='lid_final_inspection'",tail);self.assertIn('for _ in range(120)',tail)
 def test_no_full_assembly_acceptance(self):self.assertNotIn("report['full_assembly_accepted']=True",TEXT)
 def test_continuous_lower_and_hover(self):
  self.assertIn("label in ('3_cover_precontact','3_cover_lower')",TEXT);self.assertEqual(len(PHASES),14);self.assertEqual(len(PHASES),len(DURATIONS))
 def test_source_identity_intact(self):
  manifest=json.loads((HERE/'v55-parent-sha256.json').read_text())
  for n in ['nest-mesh.json','nest-design.json','finger_cooking.py','verified-seating-mass-reference.json','telemetry-finger-geometry.json','lid_pose.py']:
   self.assertEqual(hashlib.sha256((HERE/n).read_bytes()).hexdigest(),manifest[n])
 def test_raw_support_event_identity_retained(self):self.assertIn('record=serialize_header(',TEXT);self.assertIn("report['support_contact_records'].append(record)",TEXT)

if __name__=='__main__':unittest.main()
