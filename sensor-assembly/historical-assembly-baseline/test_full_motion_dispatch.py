"""Execute actual dispatch AST against identical typed observations/feedback.

Golden parents are frozen V53 base/tray and V59 cover. This checks command and
failure propagation equivalence, not simulated or physical execution success.
"""
import ast,copy,json,types,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from held_pose_bounded import placement_from_held
from lid_pose import placement_from_held as place_lid_from_held
from release_plan import square_release_plan
from payload_schedule import profile_bounds
from lid_release_schedule import lid_release_bounds
H=Path(__file__).resolve().parent
G=json.loads((H/'nest-mesh.json').read_text());D=json.loads((H/'nest-design.json').read_text())
C=types.SimpleNamespace(TABLE_TOP_Z=.9,SURFACE_DZ=-.0051,JAW_OPEN=.006,TOOL_AXIS=(1,0,0),FINGER_SPREAD_AXIS=(0,1,0))

def loop_node(path):
 t=ast.parse(path.read_text())
 return next(n for n in ast.walk(t) if isinstance(n,ast.For) and ((isinstance(n.target,ast.Tuple) and [x.id for x in n.target.elts]==['number','name']) or (isinstance(n.target,ast.Name) and n.target.id=='name' and ast.unparse(n.iter)=='ORDER')))

def run(path,part,fault=None,parts=None):
 calls=[];layout=G['layout_m'];surface=C.TABLE_TOP_Z+C.SURFACE_DZ;state={'step':50,'held_reference':None,'released_reference':None};report={'part_results':[],'release_transitions':{}}
 initial={n:{'position_world_m':[*layout['pickup_body_centres'][n],surface+.0003],'orientation_wxyz':[1,0,0,0],'tilt_deg':0.,'yaw_deg':0.} for n in G['parts']}
 initial['pcb']['position_world_m'][2]+=D['carrier']['floor_thickness']*.001+G['payload_initial_gap_mm']*.001
 R=Rotation.from_euler('xyz',[.2,-.3,.4],degrees=True).as_matrix()
 obs={'position_world_m':[.303,.097,1.04],'rotation_world':Rotation.from_euler('xyz',[.7,-.5,.3],degrees=True).as_matrix().tolist()}
 s={'np':np,'C':C,'design':D,'geometry':G,'layout':layout,'surface':surface,'state':state,'report':report,'goal':np.array([.29,-.1,.2]),'jaw':.006,'q':np.zeros(6),'commands':G['grip_commands'],'limits':G['limits'],'initial_poses':initial,'ORDER':tuple(parts) if parts is not None else (part,),'placed':['nest']+([] if parts is not None else ['base'] if part=='carrier' else ['base','carrier','pcb'] if part=='cover' else []),'arm':types.SimpleNamespace(get_joint_positions=lambda:np.zeros(6,dtype=np.float32)),'ai':np.arange(6),'tool':(.293,0.,0.),'fk':types.SimpleNamespace(tool_pose=lambda q,tool:(R.copy(),np.array([.3,.1,.15]))),'placement_from_held':placement_from_held,'place_lid_from_held':place_lid_from_held,'square_release_plan':square_release_plan,'profile_bounds':profile_bounds,'lid_release_bounds':lid_release_bounds,'lid_orientation':None,'persist':lambda:None,'json':json,'enumerate':lambda values,start=1:list(enumerate(values,start)) if parts is not None else [({'base':1,'carrier':2,'cover':3}[part],part)]}
 def snapshot(label):
  poses=copy.deepcopy(initial)
  for n in G['parts']:
   poses[n]['position_world_m']=[*layout['assembly_body_centre'],surface+D['assembly']['part_bottom_z'][n]*.001]
  if label.endswith('_lift'):
   active=label.split('_')[1]
   poses[active]['position_world_m']=(np.array(initial[active]['position_world_m'])+[0,0,.03 if fault=='short_lift' else .04]).tolist()
   if active=='carrier':poses['pcb']['position_world_m']=(np.array(initial['pcb']['position_world_m'])+[0,0,.02 if fault=='PCB_not_lifted' else .04]).tolist()
  if fault=='release_drag' and label.endswith('_retreat'):poses[label.split('_')[1]]['position_world_m'][0]+=.001
  return {'phase':label,'step':state['step'],'sim_time':state['step']/60,'poses':poses,'tip_robot_m':s['goal'].tolist(),'jaw_joints':{'jaw_l':.001 if fault=='jaws_not_open' and label.endswith('_retreat') else 0.,'jaw_r':0.}}
 def move(label,target,jaw,duration,**kw):
  calls.append(('move',label,np.asarray(target).tolist(),float(jaw),duration,{k:np.asarray(v).tolist() if isinstance(v,np.ndarray) else v for k,v in kw.items()}))
  if fault==label:raise RuntimeError('injected physical or endpoint gate')
  s['goal']=np.asarray(target).copy();s['jaw']=float(jaw);state['step']+=1;s['world'].current_time=state['step']/60
  return snapshot(label)
 def measure(label):
  calls.append(('measure',label))
  if fault in ['RGB_failure','RGB_disagreement']:raise RuntimeError(fault)
  return copy.deepcopy(obs)
 def score(name,value):return {'pass':not (fault=='actual_placement' and name==part) and not (fault=='PCB_placement' and name=='pcb')}
 s['world']=types.SimpleNamespace(current_time=50/60)
 s['issued_jaw_targets']={'jaw_l':-.004,'jaw_r':.004};s['issued_jaw']=.004
 s.update(move=move,measure_held=measure,measure_lid=measure,score_placed=score,check_inventory=lambda shot:calls.append(('inventory',shot['phase'])),fixed_incoming_hold=lambda:calls.append(('fixed_hold',)),save=lambda label:(calls.append(('save',label)) or snapshot(label)),qualify_incoming_hold=lambda label:calls.append(('qualify_hold',label)),lid_clearance_guard=lambda *args:calls.append(('clearance',args[-1])))
 error=None
 try:
  node=loop_node(path);assigned=sorted({x.id for x in ast.walk(node) if isinstance(x,ast.Name) and isinstance(x.ctx,ast.Store)})
  fn=ast.FunctionDef(name='_actual_dispatch',args=ast.arguments(posonlyargs=[],args=[],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[ast.Global(names=assigned),node],decorator_list=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(path),'exec'),s);s['_actual_dispatch']()
 except Exception as exc:error=(type(exc).__name__,str(exc))
 return {'calls':calls,'error':error,'results':report['part_results'],'placed':s['placed'],'held_reference':state['held_reference'],'released_reference':state['released_reference'],'release_transitions':report.get('release_transitions',{})}

class DispatchEquivalence(unittest.TestCase):
 def compare(self,part,fault=None):
  parent=H/('lid-parent-v59.py' if part=='cover' else 'full-prefix-parent-v53.py')
  old=run(parent,part,fault);new=run(H/'full_observed_assembly_trial.py',part,fault)
  self.assertEqual({k:v for k,v in new.items() if k!='release_transitions'},{k:v for k,v in old.items() if k!='release_transitions'});return new
 def test_three_parts_execute_one_loop_in_order_with_fixed_release_references(self):
  r=run(H/'full_observed_assembly_trial.py','base',parts=['base','carrier','cover']);self.assertIsNone(r['error']);self.assertEqual([x['part'] for x in r['results']],['base','carrier','cover']);self.assertEqual(r['placed'],['nest','base','carrier','pcb','cover']);self.assertEqual(len([c for c in r['calls'] if c[0]=='move']),41);self.assertEqual(set(r['release_transitions']),{'base','carrier','cover'});self.assertTrue(all(x['placement_verified'] and x['release_verified'] for x in r['results']));self.assertIsNone(r['held_reference']);self.assertIsNotNone(r['released_reference'])
 def test_sequence_stops_before_lid_if_tray_fails(self):
  r=run(H/'full_observed_assembly_trial.py','base','2_carrier_close',parts=['base','carrier','cover']);self.assertIsNotNone(r['error']);self.assertEqual([x['part'] for x in r['results']],['base']);self.assertFalse(any(c[0]=='move' and c[1].startswith('3_cover') for c in r['calls']))
 def test_base_nonzero_handle_and_measured_rotation(self):
  r=self.compare('base');self.assertIsNone(r['error']);self.assertTrue(r['results'][0]['placement_verified']);self.assertEqual(len([c for c in r['calls'] if c[0]=='move']),15)
  self.assertNotEqual([c for c in r['calls'] if c[0]=='move' and c[1]=='1_base_lower'][0][2][:2],G['layout_m']['assembly_body_centre'])
 def test_loaded_tray_with_separate_PCB(self):
  r=self.compare('carrier');self.assertIsNone(r['error']);self.assertTrue(r['results'][0]['separate_pcb_placement']['pass']);self.assertEqual(len([c for c in r['calls'] if c[0]=='move']),12)
 def test_cover_whole_V59_sequence(self):
  r=self.compare('cover');self.assertIsNone(r['error']);self.assertTrue(r['results'][0]['release_verified']);self.assertEqual(len([c for c in r['calls'] if c[0]=='move']),14)
 def test_short_lift_stops_without_placement(self):
  for p in ['base','carrier','cover']:
   with self.subTest(part=p):r=self.compare(p,'short_lift');self.assertIsNotNone(r['error']);self.assertFalse(r['results'][0]['placement_verified'])
 def test_PCB_not_lifted_stops_without_placement(self):
  r=self.compare('carrier','PCB_not_lifted');self.assertIsNotNone(r['error']);self.assertFalse(r['results'][0]['placement_verified'])
 def test_RGB_errors_propagate(self):
  for p in ['base','cover']:
   for error in ['RGB_failure','RGB_disagreement']:
    with self.subTest(part=p,error=error):self.assertIsNotNone(self.compare(p,error)['error'])
 def test_gate_failures_do_not_get_caught_or_credited(self):
  for p in ['base','carrier','cover']:
   for phase in ['close','lift','carry','lower','release','side_exit','retreat']:
    with self.subTest(part=p,phase=phase):self.assertIsNotNone(self.compare(p,f'{ {"base":1,"carrier":2,"cover":3}[p]}_{p}_{phase}')['error'])
 def test_actual_placement_failure(self):
  for p in ['base','carrier','cover']:
   with self.subTest(part=p):self.assertIsNotNone(self.compare(p,'actual_placement')['error'])
 def test_release_drag_and_not_open_fail(self):
  for p in ['base','carrier','cover']:
   for fault in ['release_drag','jaws_not_open']:
    with self.subTest(part=p,fault=fault):self.assertIsNotNone(self.compare(p,fault)['error'])
 def test_bad_separate_PCB_final_pose_fails(self):self.assertIsNotNone(self.compare('carrier','PCB_placement')['error'])
if __name__=='__main__':unittest.main()
