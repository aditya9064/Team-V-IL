"""Exercise actual SDK pose provenance and unchanged physical pose math."""
import ast, copy, math, unittest
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
def function(file,name):
 return next(n for n in ast.walk(ast.parse((H/file).read_text())) if isinstance(n,ast.FunctionDef) and n.name==name)
class Prim:
 def __init__(self,dtype):self.dtype=dtype;self.position=[.3,.1,.8949];self.quaternion=[1,0,0,0]
 def get_world_pose(self):return np.asarray(self.position,dtype=self.dtype),np.asarray(self.quaternion,dtype=self.dtype)
def execute(prims):
 s={'np':np,'math':math,'prims':prims,'report':{},'state':{'step':1}}
 exec(compile(ast.Module(body=[function('full_observed_assembly_trial.py','poses')],type_ignores=[]),'actual-poses','exec'),s)
 return s
class PoseReadback(unittest.TestCase):
 def test_float32_and_float64_are_recorded_before_math_conversion(self):
  for dtype in [np.float32,np.float64]:
   s=execute({name:Prim(dtype) for name in ['base','carrier','pcb','cover','nest']});p=s['poses']()
   for name in p:
    self.assertEqual(s['report']['pose_readback_dtypes'][name],{'position':np.dtype(dtype).name,'orientation':np.dtype(dtype).name});self.assertEqual(p[name]['position_world_m'],np.asarray([.3,.1,.8949],dtype=dtype).astype(float).tolist());self.assertEqual(s['report']['pose_readback_dtype_capture_counts'][name],1)
   self.assertEqual(s['report']['pose_readback_dtype_errors'],[])
 def test_changed_dtype_is_sticky_without_changing_motion_or_declared_identity(self):
  prim=Prim(np.float32);s=execute({'base':prim});s['poses']();prim.dtype=np.float64;s['state']['step']=2;p=s['poses']();s['poses']()
  self.assertEqual(p['base']['position_world_m'],prim.position);self.assertEqual(s['report']['pose_readback_dtypes']['base']['position'],'float32');self.assertEqual(len(s['report']['pose_readback_dtype_errors']),1);self.assertEqual(s['report']['pose_readback_dtype_errors'][0]['step'],2);self.assertEqual(s['report']['pose_readback_dtype_capture_counts']['base'],3)
 def test_invalid_pose_still_stops_original_physical_feedback_path(self):
  p=Prim(np.float64);s=execute({'base':p})
  for attr,value in [('position',[float('nan'),0,0]),('quaternion',[0,0,0,0])]:
   p.position=[.3,.1,.8949];p.quaternion=[1,0,0,0];setattr(p,attr,value)
   with self.assertRaisesRegex(RuntimeError,'Invalid object pose'):s['poses']()
 def test_pose_math_exact_parent_after_removing_passive_readback_block(self):
  new=copy.deepcopy(function('full_observed_assembly_trial.py','poses'));old=function('lid-parent-v59.py','poses');loop=new.body[1]
  first=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='readback_dtype' for t in n.targets))
  end=next(i for i,n in enumerate(loop.body) if i>first and isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='pos' for t in n.targets))
  # Keep the original float conversion and all physical validation unchanged.
  if end<=first:raise AssertionError('Unexpected conversion source')
  del loop.body[first:end]
  self.assertEqual(ast.dump(new),ast.dump(old))
 def test_motor_commands_feedback_and_hard_contact_gate_exact_parent(self):
  for name in ['command','feedback','move','measure_held','measure_lid','fail_contacts','contact_pose_snapshot','check_inventory','score_placed']:
   with self.subTest(function=name):self.assertEqual(ast.dump(function('full_observed_assembly_trial.py',name)),ast.dump(function('lid-parent-v59.py',name)))
if __name__=='__main__':unittest.main()
