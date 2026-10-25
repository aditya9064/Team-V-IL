"""Synthetic checker negatives, not simulation evidence or a placement result."""
import copy,json,sys,unittest,os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
SRC=Path(os.environ.get('FULL_MOTION_REFERENCE_SOURCE',str(HERE if (HERE/'nest-mesh.json').is_file() else Path('/private/tmp/team-v-compact-lid-placement-v59-20261023/snapshot'))));sys.path.insert(0,str(SRC))
FIXTURE=HERE/'v59-component-regression-fixture.json'
import check_full_observed_assembly as C
from payload_checks import payload_score
from foot_audit import all_foot_gaps
G=json.loads((SRC/'nest-mesh.json').read_text());D=json.loads((SRC/'nest-design.json').read_text())
def p(v,q=None):return {'position_world_m':list(v),'orientation_wxyz':q or [1,0,0,0]}
def fixture():
 targets={n:[.3,.1,.8949+D['assembly']['part_bottom_z'][n]*.001] for n in C.BODIES}
 initial={n:p(targets[n]) for n in C.BODIES}
 for n in ['base','carrier','pcb','cover']:initial[n]['position_world_m'][2]-=.04
 poses=copy.deepcopy(initial);shots=[{'phase':'initial','step':0,'poses':copy.deepcopy(poses)}];rows=[];results=[]
 for number,name in enumerate(C.ORDER,1):
  for phase in ['lift','lower','retreat']:
   step=len(rows)+1;poses[name]=p(targets[name])
   if name=='carrier':poses['pcb']=p(targets['pcb'])
   tip=(np.array(poses[name]['position_world_m'])+[0,0,.2]-[0,0,.9]).tolist();shot={'phase':f'{number}_{name}_{phase}','step':step,'tip_robot_m':tip,'poses':copy.deepcopy(poses),'jaw_joints':{'left':0.,'right':0.}}
   shots.append(shot);rows.append({'step':step,'sim_time':step/60,'phase':shot['phase'],'poses':copy.deepcopy(poses),'tip_world_m':(np.array(tip)+[0,0,.9]).tolist()})
  results.append({'part':name,'pickup':{'lift_mm':40.,'xy_slip_mm':0.,'tilt_deg':0.,'yaw_deg':0.,'pass':True},'placement_verified':True,'release_verified':True,'separate_pcb_pickup_lift_mm':40.})
 return rows,shots,results
class ProcessTests(unittest.TestCase):
 def audit(self,f):return C.check_process(*f,G,D,payload_score)
 def test_native_three_part_recomputation(self):self.assertEqual(set(self.audit(fixture())),set(C.ORDER))
 def test_prefix_cannot_be_omitted(self):
  rows,shots,res=fixture()
  with self.assertRaises(C.EvidenceError):self.audit((rows,shots,res[1:]))
 def test_pcb_not_carried(self):
  rows,shots,res=fixture();shot=next(s for s in shots if s['phase']=='2_carrier_lift');shot['poses']['pcb']['position_world_m'][2]-=.04
  with self.assertRaises(C.ProcessFailure):self.audit((rows,shots,res))
 def test_continuous_loose_PCB_shift(self):
  f=fixture();f[0][3]['poses']['pcb']['position_world_m'][0]+=.000751
  with self.assertRaises(C.ProcessFailure):self.audit(f)
 def test_held_base_slip(self):
  f=fixture();f[0][1]['tip_world_m'][0]+=.002001
  with self.assertRaises(C.ProcessFailure):self.audit(f)
 def test_previously_placed_base_moves_during_tray(self):
  f=fixture();f[0][3]['poses']['base']['position_world_m'][0]+=.000751
  with self.assertRaises(C.ProcessFailure):self.audit(f)
 def test_release_excursion_through_retreat(self):
  f=fixture();f[0][-1]['poses']['cover']['position_world_m'][0]+=.000251
  with self.assertRaises(C.ProcessFailure):self.audit(f)
 def test_jaws_not_open_after_carrier(self):
  f=fixture();next(x for x in f[1] if x['phase']=='2_carrier_retreat')['jaw_joints']['left']=.000251
  with self.assertRaises(C.ProcessFailure):self.audit(f)
 def test_forged_pickup_summary(self):
  f=fixture();f[2][0]['pickup']['lift_mm']=45.
  with self.assertRaises(C.EvidenceError):self.audit(f)
 def test_third_part_not_released(self):
  f=fixture();f[2][2]['release_verified']=False
  with self.assertRaises(C.EvidenceError):self.audit(f)
class SupportTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.packet=json.loads(FIXTURE.read_text());cls.geom=json.loads((SRC/'telemetry-finger-geometry.json').read_text())
 def fixture(self):
  d=copy.deepcopy(self.packet);r=d['report'];r['checkpoints'][-1]['phase']='final_assembly_inspection';return r,r['placement_observations'],{v['step']:v for v in d['actual_pre_step_frames']},d['last_actual_snapshot']
 def run_support(self,f):return C.final_support(*f,G,D,self.geom,all_foot_gaps)
 def contacts(self,f,n=(0,0,1),j=(0,0,.001),indices=(4,5,6,7),halves=(0,1)):
  r=f[0];s=r['final_inspection_start_step'];r['support_contact_records']=[]
  for half in halves:
   for i in indices:r['support_contact_records'].append({'num_contact_data':1,'points':[{'separation_mm':0.,'normal_world_raw':list(n),'impulse_Ns_raw':list(j)}],'event_type_code':2,'event_type_repr':'CONTACT_PERSIST','callback_sequence':1+half,'header_ordinal':i-4,'state_step':s+1+half*60,'colliders':[f'/World/Assembly_cover/C_{i:03d}_ear',f'/World/Assembly_base/C_{i+4:03d}_floor']})
 def test_actual_V59_gap_stays_inconclusive(self):self.assertFalse(self.run_support(self.fixture())['strict_support_complete'])
 def test_synthetic_all_four_both_halves(self):
  f=self.fixture();self.contacts(f);self.assertTrue(self.run_support(f)['strict_support_complete'])
 def test_sleep_never_substitutes(self):
  f=self.fixture();f[0]['passive_sleep_state']={'all_samples_valid':True,'records':[{'sleeping':True}]};self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_pure_tangential_force_does_not_count(self):
  f=self.fixture();self.contacts(f,j=(.001,0,0));self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_exact_threshold_does_not_count(self):
  f=self.fixture();self.contacts(f,j=(0,0,1e-8));self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_missing_second_half(self):
  f=self.fixture();self.contacts(f,halves=(0,));self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_only_three_feet(self):
  f=self.fixture();self.contacts(f,indices=(4,5,6));self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_wrong_collider_pair(self):
  f=self.fixture();self.contacts(f);f[0]['support_contact_records'][-1]['colliders'][1]='/World/Assembly_base/C_010_floor';self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_lateral_contact_normal(self):
  f=self.fixture();self.contacts(f,n=(1,0,0),j=(.001,0,0));self.assertFalse(self.run_support(f)['strict_support_complete'])
 def test_bad_unit_normal(self):
  f=self.fixture();self.contacts(f,n=(0,0,2))
  with self.assertRaises(C.EvidenceError):self.run_support(f)
 def test_depth_violation_preserved(self):
  f=self.fixture();self.contacts(f);f[0]['support_contact_records'][0]['points'][0]['separation_mm']=-.251
  with self.assertRaises(C.ProcessFailure):self.run_support(f)
 def test_final_hold_truncation(self):
  f=self.fixture();del f[1][-25:]
  with self.assertRaises(C.EvidenceError):self.run_support(f)
 def test_release_reference_reset(self):
  f=self.fixture();f[0]['held_to_release_transition']['step']+=1
  with self.assertRaises(C.EvidenceError):self.run_support(f)
class SleepTests(unittest.TestCase):
 def fixture(self):
  rows=[{'step':i,'sim_time':i/60,'phase':'final_assembly_inspection'} for i in range(1,4)];records=[dict(x,valid=True,sleeping=dict.fromkeys(['base','carrier','pcb','cover'],True),body_paths={n:'/World/Assembly_'+n for n in ['base','carrier','pcb','cover']},error=None) for x in rows]
  binding={'stage_id':7,'read_only_queries':True,'body_paths':{n:'/World/Assembly_'+n for n in ['base','carrier','pcb','cover']},'encoded_body_paths':dict(zip(['base','carrier','pcb','cover'],range(1,5))),'body_flags':{n:{'rigid_body_enabled':True,'kinematic':False,'disable_gravity':False} for n in ['base','carrier','pcb','cover']}}
  for a in records:a.update(expected_stage_id=7,encoded_body_paths=binding['encoded_body_paths'])
  return {'passive_sleep_binding':binding,'passive_sleep_state':{'records':records,'all_samples_valid':True,'errors':[],'used_for_motion_or_targets':False,'force_gate_satisfied_by_sleep':False,'acceptance_policy_changed':False,'extra_physics_steps':0,'sleep_or_wake_state_modified':False}},rows
 def test_matching_scope_valid(self):r,rows=self.fixture();self.assertTrue(C.check_sleep_evidence(r,rows)['all_samples_valid'])
 def test_missing_prefix_sleep_rows(self):
  r,rows=self.fixture();r['passive_sleep_state']['records'].pop(0)
  with self.assertRaises(C.EvidenceError):C.check_sleep_evidence(r,rows)
 def test_false_valid_flag(self):
  r,rows=self.fixture();r['passive_sleep_state']['records'][1]['sleeping']['cover']=1
  with self.assertRaises(C.EvidenceError):C.check_sleep_evidence(r,rows)
 def test_unknown_stays_unknown_not_process_failure(self):
  r,rows=self.fixture();s=r['passive_sleep_state'];s['records'][1].update(valid=False,sleeping=None,error='query failed');s['errors']=[{'step':2,'error':'query failed'}];s['all_samples_valid']=False;self.assertFalse(C.check_sleep_evidence(r,rows)['all_samples_valid'])
 def test_unavailable_setup_remains_unknown(self):
  r,rows=self.fixture();s=r['passive_sleep_state'];s['setup_error']='missing ABI';s['all_samples_valid']=False;s.pop('errors');r.pop('passive_sleep_binding')
  for a in s['records']:a.update(valid=False,sleeping=None,error='missing ABI');a.pop('body_paths');a.pop('encoded_body_paths');a.pop('expected_stage_id')
  self.assertFalse(C.check_sleep_evidence(r,rows)['all_samples_valid'])
 def test_no_sleep_force_gate_substitution(self):
  r,rows=self.fixture();r['passive_sleep_state']['force_gate_satisfied_by_sleep']=True
  with self.assertRaises(C.EvidenceError):C.check_sleep_evidence(r,rows)
 def test_wrong_root_binding(self):
  r,rows=self.fixture();r['passive_sleep_state']['records'][0]['body_paths']['base']='/World/Assembly_cover'
  with self.assertRaises(C.EvidenceError):C.check_sleep_evidence(r,rows)
class HoldTests(unittest.TestCase):
 def fixture(self):
  r=json.loads(FIXTURE.read_text())['report'];r['full_motion_observations']=r['placement_observations'];return r
 def test_actual_V59_excess_hold_is_advisory(self):
  from check_observed_lid_placement import historical_hold
  values=C.check_fixed_holds(self.fixture(),G,historical_hold);self.assertTrue(all(x['exceeded_old_limit'] for x in values))
 def test_changed_hold_command_rejected(self):
  from check_observed_lid_placement import historical_hold
  r=self.fixture();r['incoming_lid_holds'][0]['samples'][0]['commanded_arm_joints'][0]+=.01
  with self.assertRaises(C.EvidenceError):C.check_fixed_holds(r,G,historical_hold)
 def test_adaptive_hold_rejected(self):
  from check_observed_lid_placement import historical_hold
  r=self.fixture();r['incoming_lid_holds'][0]['adaptive_wait']=True
  with self.assertRaises(C.EvidenceError):C.check_fixed_holds(r,G,historical_hold)
 def test_lost_hold_sample_rejected(self):
  from check_observed_lid_placement import historical_hold
  r=self.fixture();r['incoming_lid_holds'][0]['samples'].pop(30)
  with self.assertRaises(C.EvidenceError):C.check_fixed_holds(r,G,historical_hold)
class DtypeTests(unittest.TestCase):
 def test_static_nest_float32_readback_roundtrip(self):
  raw=np.array([.3,.1,.8949]);observed=np.array([.30000001192092896,.10000000149011612,.8949000239372253]);self.assertGreater(abs(raw[2]-observed[2]),1e-8);self.assertTrue(np.allclose(C.sdk_translation(raw,{'position':'float32','orientation':'float32'}),observed,atol=1e-8,rtol=0))
 def test_declared_float64_does_not_round_away_mismatch(self):
  raw=np.array([.3,.1,.8949]);observed=raw.astype('float32').astype(float);self.assertFalse(np.allclose(C.sdk_translation(raw,{'position':'float64','orientation':'float64'}),observed,atol=1e-8,rtol=0))
 def test_unknown_or_missing_dtype_rejected(self):
  for value in [{},{'position':'float16','orientation':'float32'},{'position':'float32'},{'position':'float32','orientation':'unknown'}]:
   with self.assertRaises(C.EvidenceError):C.sdk_translation([.3,.1,.8949],value)
 def test_real_mismatch_still_exceeds_original_tolerance(self):
  raw=np.array([.3,.1,.8949]);expected=C.sdk_translation(raw,{'position':'float32','orientation':'float32'});wrong=expected.copy();wrong[2]+=2e-8;self.assertFalse(np.allclose(expected,wrong,atol=1e-8,rtol=0))
class IdentityTests(unittest.TestCase):
 def test_old_lid_scope_rejected_before_projection(self):
  r=json.loads(FIXTURE.read_text())['report']
  with self.assertRaisesRegex(C.EvidenceError,'native full'):C.check_identity(r,G,{},SRC,'x','x','x')
 def test_preassembled_actual_initial_scene_rejected(self):
  r=json.loads(FIXTURE.read_text())['report']
  with self.assertRaises(C.EvidenceError):C.check_startup(r,G)
 def test_exact_phase_durations_and_count(self):self.assertEqual(len(C.PHASES),41);self.assertEqual(len(C.DURATIONS),41);self.assertEqual(C.DURATIONS[C.PHASES.index('2_carrier_carry')],4.);self.assertEqual(C.DURATIONS[C.PHASES.index('3_cover_release')],3.)
class LidCommandTests(unittest.TestCase):
 def fixture(self):
  r=json.loads(FIXTURE.read_text())['report'];r['full_motion_observations']=r['placement_observations'];return r
 def check(self,r):
  from full_motion_command_integrity import check_lid_commands
  return check_lid_commands(r,G,D)
 def test_actual_all360_archived_commands_replay(self):self.assertEqual(self.check(self.fixture())['lid_continuous_commands_verified'],360)
 def test_genuine_RGB_gate_not_waived(self):
  r=self.fixture();r['lid_pose_observations'][0]['views']['inspection_x']['edge_p95_px']=2.001
  with self.assertRaises(C.EvidenceError):self.check(r)
 def test_camera_disagreement_still_fails(self):
  r=self.fixture();r['lid_pose_observations'][1]['position_world_m'][0]+=.0002
  with self.assertRaises(C.ProcessFailure):self.check(r)
 def test_changed_desired_path_rejected(self):
  r=self.fixture();r['placement_command_traces']['3_cover_lower']['records'][0]['desired_tip_robot_m'][0]+=.00001
  with self.assertRaises(C.EvidenceError):self.check(r)
 def test_wrong_held_FK_reference_rejected(self):
  r=self.fixture();r['placement_command_traces']['3_cover_lower']['held_actual_FK_tip_world_m'][0]+=.001
  with self.assertRaises(C.EvidenceError):self.check(r)
 def test_changed_issued_command_rejected(self):
  r=self.fixture();r['placement_command_traces']['3_cover_lower']['records'][0]['solved_q_rad'][0]+=.001
  with self.assertRaises(C.EvidenceError):self.check(r)
 def test_missing_lid_command_rejected(self):
  r=self.fixture();r['placement_command_traces']['3_cover_lower']['records'].pop()
  with self.assertRaises(C.EvidenceError):self.check(r)
 def test_command_residual_limit_cannot_be_relaxed(self):
  r=self.fixture();r['placement_command_traces']['3_cover_lower']['records'][0]['residual_limit_mm']=.003
  with self.assertRaises(C.EvidenceError):self.check(r)
 def test_changed_final_goal_rejected(self):
  r=self.fixture();r['placement_command_traces']['3_cover_lower']['unchanged_target_tip_robot_m'][0]+=.001
  with self.assertRaises(C.EvidenceError):self.check(r)
if __name__=='__main__':unittest.main()
