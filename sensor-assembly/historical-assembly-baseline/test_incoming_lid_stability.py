import ast,copy,json,unittest
from pathlib import Path
import numpy as np
from incoming_lid_stability import make_sample,qualify
from check_incoming_lid_precontact import validate_incoming_stability
from foot_audit import support_geometry

class IncomingStability(unittest.TestCase):
 def setUp(self):
  self.p=Path(__file__).parent;self.geometry=json.loads((self.p/'nest-mesh.json').read_text());self.corners=[v for f in support_geometry(self.geometry) for v in f['corners_mm']];self.q=[.1,.2,.3,.4,.5,.6];self.j={'jaw_l':-.00405,'jaw_r':.00405}
 def sample(self,i,offset=0.,rotation=None,coherent=False):
  T=np.eye(3) if rotation is None else rotation;tip=np.array([.3,.1,1.05]);p=np.array([.3,.1,1.01])+[offset,0,0]
  if coherent:tip=tip+[i*.0001,i*.0002,0];p=tip+T@np.array([0,0,-.04])
  return make_sample(101+i,10+(i+1)/60,p,T,tip,T,self.corners,self.q,self.j)
 def trace(self,n=144):return [self.sample(i) for i in range(n)]
 def score(self,x):return qualify(x,self.q,self.j,.02,self.corners)
 def test_fixed_predetermined_window_and_early_hold_motion(self):
  x=self.trace();x[50]=self.sample(50,.001);v=self.score(x);self.assertTrue(v['pass']);self.assertEqual(v['fixed_window_start_index'],95);self.assertEqual(v['evaluated_samples'],49);self.assertEqual(v['subsequent_samples'],24)
 def test_unstable_hold_and_rgb_return_to_quiet_never_clipped(self):
  for i in (100,123):
   x=self.trace();x[i]=self.sample(i,.00003);v=self.score(x);self.assertFalse(v['pass']);self.assertAlmostEqual(v['maximum_pairwise_corner_diameter_mm'],.03)
 def test_rotation_spike_constant_origin_detected(self):
  x=self.trace();angle=.001;R=np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]]);x[124]=self.sample(124,rotation=R);self.assertFalse(self.score(x)['pass'])
 def test_coherent_world_motion_unchanged_local_transform(self):
  xs=[]
  for i in range(144):
   a=.003*i;R=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]]);xs.append(self.sample(i,rotation=R,coherent=True))
  self.assertTrue(self.score(xs)['pass']);self.assertLess(self.score(xs)['maximum_pairwise_corner_diameter_mm'],1e-9)
 def test_invalid_missing_nonconsecutive_moving_commands_fail(self):
  good=self.trace();bad=[]
  x=copy.deepcopy(good);del x[121];bad.append(x)
  x=copy.deepcopy(good);x[1]['step']+=1;bad.append(x)
  x=copy.deepcopy(good);x[1]['sim_time']+=.001;bad.append(x)
  x=copy.deepcopy(good);x[10]['commanded_arm_joints'][0]+=.0001;bad.append(x)
  x=copy.deepcopy(good);x[10]['issued_jaw_joint_targets_m']['jaw_l']+=.0001;bad.append(x)
  x=copy.deepcopy(good);x[121]['corners_tool_mm'][0][0]=float('nan');bad.append(x)
  x=copy.deepcopy(good);x[121]['corners_tool_mm'].pop();bad.append(x)
  x=copy.deepcopy(good);x[121]['corners_tool_mm'][0][0]+=.001;bad.append(x)
  x=copy.deepcopy(good);del x[121]['part_rotation_world'];bad.append(x)
  bad.append(good[:119])
  for x in bad:
   with self.assertRaises((ValueError,KeyError,TypeError)):self.score(x)
 def fixture(self):
  samples=self.trace();a={'requested_duration_s':2.,'requested_commands':120,'issued_commands':120,'start_step':100,'end_step':220,'start_sim_time':10.,'end_sim_time':12.,'actual_duration_s':2.,'fixed_window_start_index':95,'fixed_window_samples':25,'adaptive_wait':False,'retry':False,'truth_used_for_targets':False,'reference_freeze_count':1,'reference_freeze_step':244,'reference_freeze_sim_time':12.4,'samples':samples,'expected_arm_joints':self.q,'expected_jaw_joint_targets_m':self.j,'fixed_hold_result':self.score(samples[:120]),'final_measurement_result':self.score(samples)}
  return {'incoming_lid_stability':a,'lid_pose_foot_budget':{'hold_drift_position_error_mm':.02},'checkpoints':[{'phase':phase,'step':step,'sim_time':time} for phase,step,time in [('3_cover_level_held',100,10.),('3_cover_leveled_inspection_a',232,12.2),('3_cover_leveled_inspection_b',244,12.4)]]}
 def test_checker_boundary_and_negatives(self):
  good=self.fixture();validate_incoming_stability(good,self.geometry)
  for key,value in [('issued_commands',119),('requested_duration_s',3.),('reference_freeze_count',2),('fixed_window_start_index',96),('adaptive_wait',True),('reference_freeze_step',243)]:
   bad=copy.deepcopy(good);bad['incoming_lid_stability'][key]=value
   with self.assertRaises(AssertionError,msg=key):validate_incoming_stability(bad,self.geometry)
 def test_reversible_runtime_checker_and_nested_controls(self):
  for child,parent,patchfile in [('incoming_lid_precontact_trial.py','slow_lid_precontact_trial.py','incoming-runtime-patches.json'),('check_incoming_lid_precontact.py','check_slow_lid_precontact.py','incoming-checker-patches.json')]:
   s=(self.p/child).read_text()
   for a,b in reversed(json.loads((self.p/patchfile).read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
   self.assertEqual(s,(self.p/parent).read_text())
  trees=[ast.parse((self.p/n).read_text()) for n in ('slow_lid_precontact_trial.py','incoming_lid_precontact_trial.py')]
  for name in ('move','command','feedback','poses','contact_pose_snapshot','measure_held','lid_clearance_guard','on_contacts','fail_contacts','score_placed','check_inventory'):
   nodes=[next(x for x in ast.walk(t) if isinstance(x,ast.FunctionDef) and x.name==name) for t in trees];self.assertEqual(ast.dump(nodes[0]),ast.dump(nodes[1]),name)
  child=(self.p/'incoming_lid_precontact_trial.py').read_text();self.assertIn('for _ in range(120):command(jaw);advance(1)',child);self.assertIn('Final lid grasp reference already frozen',child);self.assertIn("move(prefix+'_precontact',precontact,jaw,4.,new_orientation=target_R)",child)
  self.assertNotIn('SingleRigidPrim',child)
if __name__=='__main__':unittest.main()
