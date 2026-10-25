import ast,copy,hashlib,json,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from enum import IntEnum
from payload_contact_trace import PayloadContactTrace,selected_pair
from check_payload_contact_trace import validate_trace,verify_source,validate_cooked
from cooked_payload_evidence import serialize_convex
PCB='/World/Assembly_pcb';TRAY='/World/Assembly_carrier';JAW='/World/ER4iA/JawL'
PATHS={1:PCB,2:TRAY,3:PCB+'/C_000_substrate',4:TRAY+'/C_010_floor',5:JAW,6:JAW+'/Collision/Part02'}
class Event(IntEnum):FOUND=1;PERSISTS=2;LOST=3
def point():return NS(position=[.4,-.16,.936],normal=[0,0,-1],impulse=[-.001,.002,-.003],separation=-.0002519)
def header(reverse=False,lost=False):return NS(actor0=2 if reverse else 1,actor1=1 if reverse else 2,collider0=4 if reverse else 3,collider1=3 if reverse else 4,type=Event.LOST if lost else Event.PERSISTS,stage_id=9,contact_data_offset=-1 if lost else 1,num_contact_data=0 if lost else 1)
def snap(step,time):return {'step':step,'sim_time':time,'phase':'2_carrier_orient','actor_world_matrices':{a:[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[.4,-.16,.936,1.]] for a in (PCB,TRAY)}}
def capture(t,h,data,time):
 token=t.begin_callback(t.current['state_step'],'2_carrier_orient',time);t.observe_header(token,h,data,PATHS.__getitem__,{});t.end_callback(token)
def report():
 t=PayloadContactTrace(PCB,[TRAY]);p=snap(10,1.);t.begin_step(p,0,'2_carrier_orient',True)
 capture(t,header(),[None,point()],1.);capture(t,header(True),[None,point()],1.);capture(t,header(lost=True),[],1.);t.end_step(10,1+1/60)
 p2=snap(11,1+1/60);t.note_existing_snapshot(p2);t.begin_step(p2,1,'2_carrier_orient',False);t.end_step(11,1+2/60)
 post=snap(11,1+2/60);t.note_existing_snapshot(post);t.finalize('FAILED','original raw contact gate')
 return {'status':'FAILED','error':'original raw contact gate','diagnostic_stage':'precontact','full_assembly_accepted':False,'physical_hardware_test':False,'print_release':False,'payload_contact_trace_provenance':{'control_or_truth_steering':False,'extra_physics_queries_or_registration':False,'runtime_logging_IO_per_step':False},'payload_contact_trace':t.value,'payload_contact_pose_trace':[p,p2],'contact_timing_diagnostics':{'post_step':post}}
class PayloadTests(unittest.TestCase):
 def test_order_raw_signs_zero_lost_absence_and_failed_outcome(self):
  r=report();v=validate_trace(r);self.assertTrue(v['telemetry_integrity_valid']);self.assertFalse(v['mechanical_success_checked_here']);self.assertEqual(v['original_outcome'],'FAILED');self.assertEqual(v['selected_headers'],3);self.assertEqual(v['selected_points'],2)
  hs=r['payload_contact_trace']['rows'][0]['headers'];self.assertEqual(hs[1]['selection']['pcb_index'],1)
  for h in hs[:2]:self.assertEqual(h['points'][0]['normal_world_raw'],[0,0,-1]);self.assertEqual(h['points'][0]['impulse_Ns_raw'],[-.001,.002,-.003])
  self.assertEqual(hs[2]['points'],[]);self.assertEqual(r['payload_contact_trace']['rows'][1]['reported_record_state'],'NO_REPORTED_RECORD')
 def test_unrelated_and_similar_prefix_rejected(self):
  self.assertIsNone(selected_pair([PCB,TRAY+'Extra'],[PATHS[3],TRAY+'Extra/floor'],PCB,[TRAY]));self.assertIsNone(selected_pair([PCB,JAW],[PATHS[3],PATHS[6]],PCB,[TRAY]))
 def test_corrupt_or_missing_trace_rejected(self):
  changes=[lambda x:x['payload_contact_trace']['rows'].pop(),lambda x:x['payload_contact_trace']['rows'][0].update(pcb_carrier_header_count=0),lambda x:x['payload_contact_trace']['rows'][0]['pre_step_snapshot_link'].update(index=1),lambda x:x['payload_contact_trace']['rows'][0]['headers'][0]['points'][0]['normal_world_raw'].__setitem__(0,float('nan')),lambda x:x['payload_contact_trace'].update(original_runtime_status='DIAGNOSTIC_PASS'),lambda x:x['payload_contact_pose_trace'][0].update(phase='2_carrier_lift')]
  for change in changes:
   bad=copy.deepcopy(report());change(bad)
   with self.assertRaises((AssertionError,ValueError,IndexError)):validate_trace(bad)
 def test_errors_isolated_no_data_read_for_lost(self):
  class NoData:
   def __getitem__(self,index):raise AssertionError('zero point read')
  t=PayloadContactTrace(PCB,[TRAY]);t.begin_step(snap(10,1.),0,'2_carrier_orient',True);capture(t,header(lost=True),NoData(),1.);bad=point();bad.separation=float('nan');capture(t,header(),[None,bad],1.);capture(t,header(),[None,point()],1.);t.end_step(10,1+1/60);t.finalize('FAILED','original')
  self.assertFalse(t.value['complete']);self.assertEqual(len(t.value['errors']),1);self.assertEqual(t.value['selected_header_count'],2);self.assertEqual(t.value['original_runtime_error'],'original')
 def test_cooked_serializer_topology_and_invalid_data(self):
  raw=NS(vertices=[NS(x=x,y=y,z=z) for x,y,z in [(0,0,0),(1,0,0),(0,1,0),(0,0,1)]],indices=[0,1,2,0,3,1,0,2,3,1,3,2],polygons=[NS(index_base=i,num_vertices=3) for i in (0,3,6,9)])
  self.assertEqual(len(serialize_convex(raw)['vertices_mesh_local']),4)
  for mutate in (lambda x:x.indices.__setitem__(0,9),lambda x:setattr(x.polygons[0],'index_base',-1),lambda x:setattr(x.vertices[0],'z',float('nan'))):
   bad=copy.deepcopy(raw);mutate(bad)
   with self.assertRaises(ValueError):serialize_convex(bad)
 def test_cooked_completeness_cannot_promote_motion(self):
  identity=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]
  item={'actor':PCB,'callback_count':1,'result_valid':True,'convexes':[{'vertices_mesh_local':[[0,0,0],[1,0,0],[0,1,0],[0,0,1]],'indices':[0,1,2],'polygons':[{'index_base':0,'num_vertices':3}]}],**{k:identity for k in ('mesh_to_world_matrix','actor_to_world_matrix','mesh_to_actor_matrix')}}
  evidence={'schema':'POST_TERMINAL_PAYLOAD_COOKING_REPRESENTATION_V1','original_status':'FAILED','complete':True,'errors':[],'physics_or_target_use':False,'may_obtain_or_recompute_cached_cooking_data':True,'kind':'cooking-service convex representation, NOT direct runtime PxShape readback','sim_time_before_s':1.,'sim_time_after_s':1.,'expected_paths':[PATHS[3]],'colliders':{PATHS[3]:item}}
  self.assertEqual(validate_cooked(evidence,'FAILED'),1)
  for change in (lambda x:x.update(sim_time_after_s=1.1),lambda x:x.update(complete=False),lambda x:x['colliders'][PATHS[3]].update(callback_count=0),lambda x:x['colliders'][PATHS[3]]['convexes'][0]['indices'].__setitem__(0,9)):
   bad=copy.deepcopy(evidence);change(bad)
   with self.assertRaises(AssertionError):validate_cooked(bad,'FAILED')
 def test_exact_runtime_reversal_and_physics_control_function_identity(self):
  p=Path(__file__).parent;verify_source(p)
  old=ast.parse((p/'contact_trace_lid_precontact_trial.py').read_text());new=ast.parse((p/'payload_contact_lid_precontact_trial.py').read_text())
  funcs=lambda tree:{n.name:ast.dump(n) for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
  a,b=funcs(old),funcs(new)
  for name in a:
   if name not in ('main','contact_pose_snapshot','on_contacts','advance'):self.assertEqual(a[name],b[name],name)
  def mutations(tree):
   return [ast.dump(n) for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and (n.func.attr in ('Apply','Set','Define','subscribe_contact_report_events','step','reset','render','apply_action','set_joint_positions') or isinstance(n.func.value,ast.Name) and n.func.value.id in ('ik','fk'))]
  self.assertEqual(mutations(old),mutations(new))
  reader=ast.parse((p/'cooked_payload_evidence.py').read_text());self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in ('step','reset','render','update','Set','Apply','force_load_physics_from_usd') for n in ast.walk(reader)))
if __name__=='__main__':unittest.main()
