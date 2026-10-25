import ast,copy,hashlib,json,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from passive_velocity import PassiveVelocity
from check_passive_lid_precontact import validate_passive_velocity
class PassivePrecontact(unittest.TestCase):
 def setUp(self):self.p=Path(__file__).parent
 def test_runtime_reversible_and_frozen_parent(self):
  s=(self.p/'passive_lid_precontact_trial.py').read_text()
  for a,b in reversed(json.loads((self.p/'passive-runtime-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(self.p/'lid_pose_trial.py').read_text())
  self.assertEqual(hashlib.sha256((self.p/'lid_pose_trial.py').read_bytes()).hexdigest(),'ede337e5fc661cf6a8b06369086d9dc0c355a4b484aff3d7598176c3975b062e')
 def test_checker_reversible(self):
  s=(self.p/'check_passive_lid_precontact.py').read_text()
  for a,b in reversed(json.loads((self.p/'passive-checker-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(self.p/'check_lid_pose.py').read_text())
 def test_sealed_passive_helper_unchanged(self):
  self.assertEqual(hashlib.sha256((self.p/'passive_velocity.py').read_bytes()).hexdigest(),'86dd6b513d2c04a0798f32977d51b09b78689d30c1097966215dad38eaa0e23d')
 def test_no_rigid_registration_or_ab_instrumentation(self):
  s=(self.p/'passive_lid_precontact_trial.py').read_text()
  for name in ('SingleRigidPrim','velocity_readers','record_initialization','observer_initialization_audit','observer_mode','get_linear_velocity','get_angular_velocity'):self.assertNotIn(name,s)
  for n in ast.walk(ast.parse(s)):
   if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute):self.assertNotIn(n.func.attr,('create_simulation_view','create_rigid_body_view'))
 def test_motion_lid_estimator_orientation_and_all_gates_exact(self):
  trees=[ast.parse((self.p/n).read_text()) for n in ('lid_pose_trial.py','passive_lid_precontact_trial.py')]
  for name in ('advance','move','command','feedback','poses','on_contacts','fail_contacts','measure_held','measure_lid','lid_clearance_guard','score_placed','check_inventory'):
   v=[next(x for x in ast.walk(t) if isinstance(x,ast.FunctionDef) and x.name==name) for t in trees]
   self.assertEqual(ast.dump(v[0]),ast.dump(v[1]),name)
  s=(self.p/'passive_lid_precontact_trial.py').read_text();parent=(self.p/'lid_pose_trial.py').read_text()
  a='        for number,name in enumerate(ORDER,1):';b="        state['phase']='final_assembly_inspection'"
  self.assertEqual(s[s.index(a):s.index(b)],parent[parent.index(a):parent.index(b)])
 def matrix(self,pos,R):
  m=np.eye(4);m[:3,:3]=R.T;m[3,:3]=pos;return m.tolist()
 def test_available_null_and_repeated_time_contract(self):
  f=PassiveVelocity({'base':'/base'});I=np.eye(3)
  first=f.sample(0,{'/base':self.matrix([0,0,0],I)})['base'];validate_passive_velocity(first);self.assertFalse(first['available'])
  repeat=f.sample(0,{'/base':self.matrix([99,0,0],I)})['base'];validate_passive_velocity(repeat);self.assertEqual(repeat['reason'],'same_simulation_time')
  next_value=f.sample(1,{'/base':self.matrix([1,0,0],I)})['base'];validate_passive_velocity(next_value);np.testing.assert_allclose(next_value['linear_world_m_s'],[1,0,0])
  for malformed in [dict(first,linear_world_m_s=[0,0,0]),dict(repeat,interval_s=1),dict(next_value,interval_s=0),dict(next_value,angular_world_rad_s=[0,float('nan'),0]),dict(next_value,source='measured_COM')]:
   with self.assertRaises(AssertionError):validate_passive_velocity(malformed)
 def test_world_angular_rotation_noncommuting(self):
  f=PassiveVelocity({'base':'/base'});R=Rotation.from_euler('y',30,degrees=True).as_matrix();w=np.array([.3,.2,-.1]);dt=.02;Q=Rotation.from_rotvec(w*dt).as_matrix()@R
  f.sample(1,{'/base':self.matrix([0,0,0],R)})
  value=f.sample(1+dt,{'/base':self.matrix([0,0,0],Q)})['base'];validate_passive_velocity(value);np.testing.assert_allclose(value['angular_world_rad_s'],w,atol=1e-12)
 def test_velocity_fields_never_used_for_targets_or_physical_gates(self):
  tree=ast.parse((self.p/'passive_lid_precontact_trial.py').read_text())
  for name in ('advance','move','command','feedback','measure_held','measure_lid','lid_clearance_guard','score_placed','check_inventory'):
   node=next(x for x in ast.walk(tree) if isinstance(x,ast.FunctionDef) and x.name==name)
   self.assertNotIn('body_velocities',ast.unparse(node),name)
if __name__=='__main__':unittest.main()
