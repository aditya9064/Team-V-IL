"""Execute the actual runtime's passive factory with typed read-only SDK fakes."""
import ast,types,unittest
from pathlib import Path
from passive_sleep_state import PassiveSleepState,UnavailableSleepState
H=Path(__file__).resolve().parent
class Attribute:
 def __init__(self,v):self.v=v
 def Get(self):return self.v
class Prim:
 def __init__(self,path,flags):self.path=path;self.flags=flags
 def IsValid(self):return self.flags.get('valid',True)
 def HasAPI(self,schema):return self.flags.get('rigid',True)
 def GetPath(self):return self.path
class Rigid:
 def __init__(self,p):self.p=p
 def GetRigidBodyEnabledAttr(self):return Attribute(self.p.flags.get('enabled',True))
 def GetKinematicEnabledAttr(self):return Attribute(self.p.flags.get('kinematic',False))
 def GetDisableGravityAttr(self):return Attribute(self.p.flags.get('gravity_disabled',False))
class QueryAPI:
 def get_attached_stage(self):return 7
 def is_sleeping(self,stage,body):return True
 def __getattr__(self,name):raise AssertionError('Forbidden engine access '+name)
class ForbiddenContext:
 def __getattr__(self,name):raise AssertionError('World context is not the observation interface: '+name)

def factory(fault=None):
 roots={n:'/World/Assembly_'+n for n in ['base','carrier','pcb','cover']};ids={p:i+101 for i,p in enumerate(roots.values())};flags={}
 if fault=='root_swap':roots['base'],roots['cover']=roots['cover'],roots['base']
 if fault in ['kinematic','gravity_disabled']:flags[fault]=True
 if fault in ['valid','rigid']:flags[fault]=False
 if fault=='enabled':flags['enabled']=False
 if fault=='flag_nonbool':flags['enabled']=1
 if fault=='flag_missing':flags['enabled']=None
 stage=types.SimpleNamespace(GetPrimAtPath=lambda path:Prim(path,flags))
 stageid=0 if fault=='stage_missing' else -1 if fault=='stage_negative' else True if fault=='stage_boolean' else 7
 cache=types.SimpleNamespace(GetId=lambda s:types.SimpleNamespace(ToLongInt=lambda:stageid))
 api=QueryAPI();getter_calls=[]
 def get_api():getter_calls.append('readonly_interface');return api
 scope={'roots':roots,'stage':stage,'report':{},'physics':ForbiddenContext(),'omni':types.SimpleNamespace(physx=types.SimpleNamespace(get_physx_simulation_interface=get_api)),'UsdUtils':types.SimpleNamespace(StageCache=types.SimpleNamespace(Get=lambda:cache)),'UsdPhysics':types.SimpleNamespace(RigidBodyAPI=Rigid),'PhysxSchema':types.SimpleNamespace(PhysxRigidBodyAPI=Rigid),'PhysicsSchemaTools':types.SimpleNamespace(sdfPathToInt=lambda p:ids[p],intToSdfPath=lambda i:'/World/WRONG' if fault=='decode_mismatch' else next(k for k,v in ids.items() if v==i)),'PassiveSleepState':PassiveSleepState,'UnavailableSleepState':UnavailableSleepState}
 tree=ast.parse((H/'full_observed_assembly_trial.py').read_text())
 node=next(n for n in ast.walk(tree) if isinstance(n,ast.Try) and isinstance(n.body[0],ast.Assign) and any(isinstance(t,ast.Name) and t.id=='sleep_api' for t in n.body[0].targets))
 exec(compile(ast.Module(body=[node],type_ignores=[]),'actual-passive-factory','exec'),scope)
 return scope,getter_calls
class RuntimeBinding(unittest.TestCase):
 def test_exact_interface_factory_and_canonical_binding(self):
  s,c=factory();self.assertEqual(c,['readonly_interface']);self.assertIsInstance(s['sleep_observer'],PassiveSleepState);self.assertTrue(s['sleep_observer'].capture(1,1/60,'initial')['valid']);self.assertEqual(s['report']['passive_sleep_binding']['body_paths'],s['roots'])
 def test_invalid_binding_never_changes_or_stops_motion(self):
  for fault in ['root_swap','kinematic','gravity_disabled','valid','rigid','enabled','flag_nonbool','flag_missing','stage_missing','stage_negative','stage_boolean','decode_mismatch']:
   with self.subTest(fault=fault):
    s,c=factory(fault);self.assertEqual(c,['readonly_interface']);self.assertIsInstance(s['sleep_observer'],UnavailableSleepState);r=s['sleep_observer'].capture(1,1/60,'moving');self.assertFalse(r['valid']);self.assertIsNone(r['sleeping']);self.assertFalse(s['sleep_observer'].evidence()['force_gate_satisfied_by_sleep'])
 def test_original_physics_step_render_and_reset_call_count_unchanged(self):
  def count(name):
   t=ast.parse((H/name).read_text());return {method:sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name) and n.func.value.id=='world' and n.func.attr==method for n in ast.walk(t)) for method in ['step','render','reset']}
  self.assertEqual(count('full_observed_assembly_trial.py'),count('lid-parent-v59.py'))
if __name__=='__main__':unittest.main()
