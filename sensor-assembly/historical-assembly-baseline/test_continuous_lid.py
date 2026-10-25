import ast,copy,json,unittest
from pathlib import Path
import numpy as np
from continuous_lid_command import held_foot_points,foot_residual_mm,make_command_record,independent_fk
from check_continuous_lid_precontact import validate_continuous_commands
from replay_continuous_commands import replay
class ContinuousLid(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.p=Path(__file__).parent;cls.fixture=json.loads((cls.p/'offline-continuous-replay.json').read_text());cls.r=cls.fixture['report_fixture'];cls.g=json.loads((cls.p/'nest-mesh.json').read_text());cls.inputs=json.loads((cls.p/'v49-command-replay-inputs.json').read_text())
 def test_full240_actual_input_replay_and_guard(self):
  validate_continuous_commands(self.r,self.g);a=self.r['continuous_lid_commands'];self.assertEqual(len(a['records']),240);self.assertTrue(all(x['prospective_geometry_eligible'] for x in a['records']));self.assertLess(max(x['maximum_foot_residual_mm'] for x in a['records']),.002)
  prev=a['last_issued_q_rad']
  for x in a['records']:self.assertNotEqual(x['solved_q_rad'],prev);prev=x['solved_q_rad']
  p0=np.array(a['start_tip_robot_m']);p1=np.array(a['records'][0]['solved_tip_robot_m']);self.assertLess(np.linalg.norm(p1-p0)*1000,.007)
 def test_actual_float32_FK_regression_and_float64_distinction(self):
  i=self.inputs;R,p=independent_fk(i['final_RGB_actual_q_rad'],i['tool_offset_m'],'float32');np.testing.assert_allclose(R,i['actual_R_world'],atol=1e-10,rtol=0);np.testing.assert_allclose(p,i['actual_TCP_robot_m'],atol=1e-10,rtol=0)
  Rd,pd=independent_fk(i['final_RGB_actual_q_rad'],i['tool_offset_m'],'float64');self.assertGreater(np.max(abs(Rd-R)),1e-9);self.assertGreater(np.max(abs(pd-p)),1e-10)
 def test_actual_rotation_or_stale_translation_start_rejected(self):
  for key,value in [('start_tip_robot_m',self.inputs['remembered_goal_robot_m']),('start_rotation_world',self.inputs['actual_R_world'])]:
   bad=copy.deepcopy(self.r);bad['continuous_lid_commands'][key]=value
   with self.assertRaises(AssertionError,msg=key):validate_continuous_commands(bad,self.g)
 def test_default_IK_deadband_prospectively_rejected(self):
  r,g=replay('default_IK_deadband',count=6);a=r['continuous_lid_commands'];q=a['last_issued_q_rad'];self.assertTrue(all(x['solved_q_rad']==q for x in a['records'][:4]));self.assertFalse(a['records'][0]['prospective_geometry_eligible']);self.assertGreater(a['records'][0]['maximum_foot_residual_mm'],.002);self.assertNotEqual(a['records'][4]['solved_q_rad'],q)
 def test_full_SE3_rotation_and_permissive_IK_fallback_negatives(self):
  a=self.r['continuous_lid_commands'];points=a['estimated_foot_points_tool_m'];x=a['records'][0];R=np.array(x['desired_rotation_world']);tip=np.array(x['desired_tip_robot_m']);d=.001;D=np.array([[1,0,0],[0,np.cos(d),-np.sin(d)],[0,np.sin(d),np.cos(d)]])
  self.assertGreater(foot_residual_mm(points,tip,R,tip,R@D).max(),.002)
  rec=make_command_record(1,1,0,tip,R,x['solved_q_rad'],tip+[.004,0,0],R,x['jaw_joint_targets_m'],points,True);self.assertFalse(rec['prospective_geometry_eligible']);self.assertTrue(rec['ik_ok'])
  rec=make_command_record(1,1,0,tip,R,x['solved_q_rad'],tip,R,x['jaw_joint_targets_m'],points,False);self.assertFalse(rec['prospective_geometry_eligible'])
 def test_checker_tamper_residual_missing_timestamp_commands_negatives(self):
  for modify in [lambda a:a['records'].pop(),lambda a:a['records'][0].update(issued=False),lambda a:a['records'][0].update(maximum_foot_residual_mm=0),lambda a:a['records'][0]['solved_q_rad'].__setitem__(0,a['records'][0]['solved_q_rad'][0]+.001),lambda a:a['records'][0].update(sim_time_before_issue=0),lambda a:a['records'][0]['actual_before'].update(joint_dtype='float64'),lambda a:a['records'][0]['jaw_joint_targets_m'].update(jaw_l=0)]:
   bad=copy.deepcopy(self.r);modify(bad['continuous_lid_commands'])
   with self.assertRaises((AssertionError,ValueError,KeyError)):validate_continuous_commands(bad,self.g)
 def test_reversible_changes_and_original_guard_functions(self):
  for child,parent,patches in [('continuous_lid_precontact_trial.py','incoming_lid_precontact_trial.py','continuous-runtime-patches.json'),('check_continuous_lid_precontact.py','check_incoming_lid_precontact.py','continuous-checker-patches.json')]:
   s=(self.p/child).read_text()
   for a,b in reversed(json.loads((self.p/patches).read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
   self.assertEqual(s,(self.p/parent).read_text())
  trees=[ast.parse((self.p/n).read_text()) for n in ('incoming_lid_precontact_trial.py','continuous_lid_precontact_trial.py')]
  for name in ('advance','command','feedback','poses','measure_held','measure_lid','fixed_incoming_hold','qualify_incoming_hold','lid_clearance_guard','on_contacts','fail_contacts','score_placed','check_inventory'):
   nodes=[next(x for x in ast.walk(t) if isinstance(x,ast.FunctionDef) and x.name==name) for t in trees];self.assertEqual(ast.dump(nodes[0]),ast.dump(nodes[1]),name)
 def test_before_issue_veto_and_failure_logging_source_contract(self):
  s=(self.p/'continuous_lid_precontact_trial.py').read_text();loop=s[s.index("                if label=='3_cover_precontact':\n                    record="):s.index('            phase_duration=')]
  self.assertLess(loop.index("['records'].append(record)"),loop.index('proposed_q,ok=ik.solve'))
  self.assertLess(loop.index("if not record['prospective_geometry_eligible']"),loop.index('command(j)'))
  self.assertIn("finally:record['actual_after']=observed_command_fk()",loop)
  calls=[x for x in ast.walk(ast.parse(s)) if isinstance(x,ast.Call) and ast.unparse(x.func)=='ik.solve'];tight=[x for x in calls if any(k.arg=='tol' for k in x.keywords)];self.assertEqual(len(tight),1)
if __name__=='__main__':unittest.main()
