"""Native full-motion offline audit draft. Never edits reports or runs a simulator.

No lid-only metadata projection: every full-run sample and all three parts are
checked. Motion completion and the unchanged strict force certificate are
separate. Sleep is recorded evidence only, never a substitute for force.
"""
import argparse,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ORDER=['base','carrier','cover'];BODIES=['base','carrier','pcb','cover','nest']
TASK='original_full_observed_assembly_diagnostic';CASE='full-observed-assembly';STAGE='observed-full-assembly'
BASE_SUFFIX=['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','level_held','lower','relax_clamp','square_fingers','release','side_exit','retreat']
CARRIER_SUFFIX=['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','lower','release','side_exit','retreat']
COVER_SUFFIX=['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','level_held','precontact','lower','release','side_exit','retreat']
PHASES=['1_base_'+x for x in BASE_SUFFIX]+['2_carrier_'+x for x in CARRIER_SUFFIX]+['3_cover_'+x for x in COVER_SUFFIX]
DURATIONS=[1,2,2,1.5,3,1.5,1.5,2,2,2,2,2,1,1.5,2]+[1,2,2,1.5,3,1.5,1.5,4,2,1,1.5,2]+[1,2,2,1.5,3,1.5,1.5,2,2,4,2,3,1.5,2]
LIMITS={'endpoint_mm':2.,'endpoint_samples':6,'camera_speed_rad_s':.02,'camera_samples':6,'pickup_lift_mm':40.,'minimum_lift_mm':35.,'maximum_pickup_xy_slip_mm':2.,'maximum_pickup_tilt_deg':5.,'maximum_pickup_yaw_deg':3.,'placement_xy_mm':.75,'placement_z_mm':.35,'placement_tilt_deg':1.,'placement_yaw_deg':1.,'release_follow_mm':.25,'maximum_intended_penetration_mm':.25}
MESH_SHA='728f96ba4be877e8d768ab57a5505a5c2c6be08e750791014274241d3bb9fcd5';DESIGN_SHA='3808dbe0fd8bbda6f6990e46b796cde1d38055c4fdf84450cccd39a858acc62a'
class EvidenceError(ValueError):pass
class ProcessFailure(ValueError):pass

def need(v,msg):
 if not v:raise EvidenceError(msg)
def physical(v,msg):
 if not v:raise ProcessFailure(msg)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def arr(v,shape=None):
 a=np.asarray(v,float);need(np.isfinite(a).all() and (shape is None or a.shape==shape),'Invalid finite array');return a
def scalar(v):need(type(v) in (int,float) and math.isfinite(v),'Invalid scalar');return float(v)
def pose(v):
 p=arr(v['position_world_m'],(3,));q=arr(v['orientation_wxyz'],(4,));need(abs(np.linalg.norm(q)-1)<=1e-3,'Invalid quaternion');return p,Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
def pose_angles(v):
 p,R=pose(v);return p,math.degrees(math.acos(float(np.clip(R[2,2],-1,1)))),abs(math.degrees(math.atan2(R[1,0],R[0,0])))
def placed_metrics(v,target):
 p,t,y=pose_angles(v);delta=p-arr(target,(3,));return {'xy_mm':float(np.linalg.norm(delta[:2])*1000),'z_mm':float(abs(delta[2])*1000),'tilt_deg':t,'yaw_deg':y}
def placed_ok(v,target):return all(x<=limit for x,limit in zip(placed_metrics(v,target).values(),[.75,.35,1.,1.]))
def rigid_matrix(v):
 M=arr(v,(4,4));need(np.allclose(M[:,3],[0,0,0,1],atol=1e-8,rtol=0),'Invalid USD matrix');R=M[:3,:3].T;need(np.allclose(R.T@R,np.eye(3),atol=1e-5,rtol=0) and abs(np.linalg.det(R)-1)<1e-5,'Nonrigid matrix');return M

def check_source(source,runtime_file,runtime_sha,manifest_sha,policy_file):
 source=Path(source);need(sha(source/'snapshot-sha256.json')==manifest_sha,'Manifest not independently reviewed');manifest=json.loads((source/'snapshot-sha256.json').read_text())
 for name,h in manifest.items():
  p=(source/name).resolve();need(p.is_relative_to(source.resolve()) and p.is_file() and sha(p)==h,'Source manifest mismatch: '+name)
 need(runtime_file in manifest and sha(source/runtime_file)==runtime_sha,'Unreviewed runtime')
 need(sha(source/'nest-mesh.json')==MESH_SHA and sha(source/'nest-design.json')==DESIGN_SHA,'Original CAD changed')
 need(policy_file in manifest,'Unsealed full-motion policy');policy=json.loads((source/policy_file).read_text())
 need(policy['task']==TASK and policy['case']==CASE,'Wrong source policy scope')
 need(policy['part_order']==ORDER and policy['only_preplaced_fixture']==['nest'] and policy['phases']==PHASES and policy['durations_s']==DURATIONS,'Policy fixture/order/phase timing changed')
 return json.loads((source/'nest-mesh.json').read_text()),json.loads((source/'nest-design.json').read_text()),policy

def check_identity(r,g,policy,source,runtime_file,runtime_sha,policy_file):
 need(r['task']==TASK and r['case']==CASE and r['diagnostic_stage']==STAGE,'Not native full-motion evidence')
 need(r['full_motion_policy']==policy and r['full_motion_policy_sha256']==sha(Path(source)/policy_file),'Policy identity mismatch')
 need(r['trial_script_sha256']==runtime_sha and r['mesh_sha256']==MESH_SHA and r['design_sha256']==DESIGN_SHA,'Report source identity mismatch')
 need(r['part_order']==ORDER and r['physical_parts']==BODIES,'Wrong full part order/set')
 need(r['full_assembly_accepted'] is False and not r['physical_hardware_test'] and not r['print_release'],'Premature qualification')
 need(r['status'] in ('FAILED','MOTION_COMPLETE'),'Nonterminal runtime')
 need(r['limits']==g['limits']==LIMITS,'Original physical thresholds changed')
 need(r['payload_limits']==g['payload_limits']=={'xy_mm':.75,'z_mm':.35,'relative_tilt_deg':1.,'relative_yaw_deg':1.},'PCB limits changed')
 need(r['physics']=={'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':1},'Original physics changed')
 init=r['initial_preparation'];need(init['product_parts_preassembled_together'] is False and init['pcb_preloaded_in_carrier'] is True and init['only_preplaced_fixture']==['nest'] and init['robot_placement_order']==ORDER,'Preassembled parts or wrong fixture credit')
 need(r['preplaced_fixture_parts']==['nest'],'Product fixture falsely credited')
 need(not r.get('historical_foot_budget_qualified_for_this_run',False) and not r.get('predictive_clearance_certified',False),'Historical prediction certification falsely claimed')
 telemetry=json.loads((Path(source)/'telemetry-finger-geometry.json').read_text());sdk=dict(telemetry['SDK_source_sha256']);sdk.update({'fk.py':'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed','ik.py':'6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'});need(r['starter_hashes']==sdk,'Robot SDK changed')
 k=r['kinematic_binding'];need(k['arm_joint_names']==['J'+str(i) for i in range(1,7)] and k['arm_joint_indices']==list(range(6)) and k['joint_dtype']=='float32' and k['tool_offset_m']==[.293,0,0] and k['table_top_z_m']==.9,'FK binding changed')

def check_startup(r,g):
 initial=r['checkpoints'][0];need(initial['phase']=='initial','Missing actual initial checkpoint')
 # Bind declared separate known pickup locations to actual initial evidence.
 for name in ['base','carrier','pcb','cover']:
  p,_=pose(initial['poses'][name]);target=arr(g['layout_m']['pickup_body_centres'][name],(2,));need(np.linalg.norm(p[:2]-target)*1000<=.75,'Actual initial product is not at known pickup: '+name)
 need(r['initial_preparation'].get('preplaced_dynamic_parts',[])==[],'Dynamic fixture credit present')

def check_motion(r,complete):
 phases=r['motion_sampling']['phases'];names=[x['phase'] for x in phases];need(names==PHASES[:len(names)] and len(names)<=len(PHASES),'Full motion phase order invalid')
 if complete:need(names==PHASES,'Full sequence truncated')
 need(r['motion_sampling']['command_hz']==60 and r['motion_sampling']['physics_steps_per_command']==1,'Control sampling changed')
 for row,duration in zip(phases,DURATIONS):
  need(row['requested_duration_s']==duration and row['commands']==round(duration*60) and abs(scalar(row['actual_duration_s'])-duration)<=2e-6 and abs(scalar(row['maximum_command_interval_s'])-1/60)<=2e-6,'Original phase timing changed')
  tr=r['endpoint_traces'][row['phase']];need(len(tr)==6,'Missing endpoint evidence')
  for x in tr:physical(scalar(x['tip_error_mm'])<=2 and scalar(x['tool_axis_error_deg'])<=1 and scalar(x['spread_axis_error_deg'])<=1,'Actual endpoint gate failed')
 for shot in r['checkpoints']:
  tr=shot['stillness_trace'];need(len(tr)==6,'Missing camera stillness')
  physical(all(0<=scalar(x['maximum_raw_joint_speed_rad_s'])<.02 for x in tr),'Actual camera stillness failed')
 # Mandatory native checkpoint order; additional named RGB/hold checkpoints are explicit.
 expected=['initial']
 for name in PHASES:
  expected.append(name)
  if name=='1_base_carry':expected+=['1_base_held_inspection_a','1_base_held_inspection_b']
  if name=='1_base_level_held':expected+=['1_base_leveled_inspection_a','1_base_leveled_inspection_b']
  if name=='3_cover_lift':expected+=['3_cover_incoming_hold']
  if name=='3_cover_carry':expected+=['3_cover_held_inspection_a','3_cover_held_inspection_b']
  if name=='3_cover_level_held':expected+=['3_cover_leveled_inspection_a','3_cover_leveled_inspection_b']
 expected+=['final_assembly_inspection']
 actual=[x['phase'] for x in r['checkpoints']];need(actual==expected[:len(actual)] and len(actual)<=len(expected),'Unexpected full checkpoint sequence')
 if complete:need(actual==expected,'Missing full checkpoints')

def sdk_translation(raw_position,dtype_record):
 need(isinstance(dtype_record,dict) and set(dtype_record)=={'position','orientation'} and all(v in ('float32','float64') for v in dtype_record.values()),'Unknown/unpinned SDK pose readback dtype')
 # Normalize the raw USD value to the exact declared SDK output precision.
 # Physical scores still use the actual SDK evidence without changed tolerances.
 return arr(raw_position,(3,)).astype(np.dtype(dtype_record['position'])).astype(float)

def validate_ledger(r,independent_fk):
 rows=r['full_motion_observations'];need(rows,'No native full motion ledger');need(rows[0]['step']==1,'Full prefix omitted from ledger')
 need(len(rows)==r['step_timing_audit']['samples'],'Missing existing post-step observations')
 pre=r['diagnostic_trace'];by={x['step']:x for x in pre};need(len(by)==len(pre),'Duplicate raw pre-step frames')
 last=r.get('full_motion_last_snapshot') or r['lid_contact_trace']['last_existing_snapshot'];need(last is not None,'No terminal raw snapshot')
 k=r['kinematic_binding'];previous=None;dtypes=r['pose_readback_dtypes'];need(set(dtypes)==set(BODIES),'Missing per-body SDK dtype provenance');need(r['pose_readback_dtype_errors']==[],'SDK pose dtype changed during run');counts=r['pose_readback_dtype_capture_counts'];need(set(counts)==set(BODIES) and all(type(v) is int and v>=len(rows) for v in counts.values()),'Incomplete SDK dtype capture scope')
 for name in BODIES:sdk_translation([0.,0.,0.],dtypes[name])
 for row in rows:
  need(type(row['step']) is int and row['step']>0 and scalar(row['sim_time'])>=0,'Invalid full ledger step/time')
  if previous:need(row['step']==previous['step']+1 and abs(row['sim_time']-previous['sim_time']-1/60)<=2e-6,'Missing full physics step')
  previous=row;arr(row['actual_qdot_rad_s']);q=arr(row['actual_q_rad']);R,t=independent_fk(q[k['arm_joint_indices']],k['tool_offset_m'],'float32')
  need(np.allclose(R,arr(row['tool_rotation_world'],(3,3)),atol=1e-7,rtol=0) and np.linalg.norm(np.asarray(t)+[0,0,.9]-arr(row['tip_world_m'],(3,)))*1000<1e-5,'Actual full FK mismatch')
  post=by.get(row['step']+1,last);need(abs(post['sim_time']-row['sim_time'])<2e-6,'Raw full pose time mismatch')
  need(np.allclose(arr(post['joint_positions']),q,atol=0,rtol=0),'Actual joint ledger not raw snapshot')
  for name in BODIES:
   p,A=pose(row['poses'][name]);M=rigid_matrix(post['actor_world_matrices']['/World/Assembly_'+name]);need(np.allclose(sdk_translation(M[3,:3],dtypes[name]),p,atol=1e-8,rtol=0) and np.allclose(M[:3,:3].T,A,atol=1e-7,rtol=0),'Actual full pose ledger mismatch: '+name)
 cover=[x for x in rows if x['phase'].startswith('3_cover_') or x['phase']=='final_assembly_inspection'];need(r['placement_observations']==cover,'Cover ledger disagrees with full ledger')
 return rows,by,last

def check_process(rows,shots,results,g,design,payload_score,complete=True):
 # Recompute continuous PCB, held/release and inventory from all parts, not summary flags.
 targets={n:[*g['layout_m']['assembly_body_centre'],.8949+design['assembly']['part_bottom_z'][n]*.001] for n in BODIES}
 shotmap={x['phase']:x for x in shots};need(len(shotmap)==len(shots),'Duplicate checkpoints');need(shots[0]['phase']=='initial','Missing initial')
 if complete:need([x['part'] for x in results]==ORDER,'Wrong three-part results')
 initial=shots[0];metrics={};placed_at={'nest':0}
 for number,name in enumerate(ORDER,1):
  prefix=f'{number}_{name}';lift=shotmap.get(prefix+'_lift');lower=shotmap.get(prefix+'_lower');retreat=shotmap.get(prefix+'_retreat')
  if lift is None:continue
  p0=arr(initial['poses'][name]['position_world_m'],(3,));p,t,y=pose_angles(lift['poses'][name]);delta=(p-p0)*1000
  physical(delta[2]>=35 and np.linalg.norm(delta[:2])<=2 and t<=5 and y<=3,'Actual pickup failed: '+name)
  result=next((x for x in results if x['part']==name),None);need(result is not None,'Missing actual part result')
  expected={'lift_mm':float(delta[2]),'xy_slip_mm':float(np.linalg.norm(delta[:2])),'tilt_deg':t,'yaw_deg':y}
  need(result['pickup']['pass'] is True and all(abs(scalar(result['pickup'][k])-v)<1e-6 for k,v in expected.items()),'Pickup summary differs from actual: '+name)
  held_ref=p-arr(lift['tip_robot_m'],(3,))-[0,0,.9];held=[]
  for row in rows:
   if lift['step']<row['step']<=(lower['step'] if lower else rows[-1]['step']):
    hp,ht,hy=pose_angles(row['poses'][name]);slip=float(np.linalg.norm(hp-arr(row['tip_world_m'],(3,))-held_ref)*1000);held.append(slip);physical(slip<=2 and ht<=5 and hy<=3,'Continuous held part failed: '+name)
  metrics[name]={'pickup':expected,'maximum_held_slip_mm':max(held,default=0.)}
  if name=='carrier':
   pcb_lift=float((arr(lift['poses']['pcb']['position_world_m'])-arr(initial['poses']['pcb']['position_world_m']))[2]*1000);physical(pcb_lift>=35,'PCB not lifted with tray');need(abs(scalar(result['separate_pcb_pickup_lift_mm'])-pcb_lift)<1e-6,'PCB pickup summary mismatch')
  if lower is None:continue
  ref=arr(lower['poses'][name]['position_world_m']);laststep=rows[-1]['step'] if name=='cover' else retreat['step'] if retreat else rows[-1]['step'];release=[v for v in rows if lower['step']<v['step']<=laststep];exc=max((float(np.linalg.norm(arr(v['poses'][name]['position_world_m'])-ref)*1000) for v in release),default=0.)
  physical(exc<=.25,'Continuous release excursion failed: '+name);metrics[name]['maximum_release_excursion_mm']=exc
  if retreat:
   physical(placed_ok(retreat['poses'][name],targets[name]),'Placement failed: '+name);physical(len(retreat['jaw_joints'])==2 and all(abs(scalar(v))<=.00025 for v in retreat['jaw_joints'].values()),'Jaws not open: '+name)
   need(result['placement_verified'] is True and result['release_verified'] is True,'Completed part not verified');placed_at[name]=retreat['step']
   if name=='carrier':physical(placed_ok(retreat['poses']['pcb'],targets['pcb']),'Separate PCB final placement failed');placed_at['pcb']=retreat['step']
 for row in rows:
  physical(payload_score(row['poses']['carrier'],row['poses']['pcb'],design['carrier']['floor_thickness']*.001)['pass'],'Continuous loose PCB failed')
  for name,step in placed_at.items():
   if row['step']>=step:physical(placed_ok(row['poses'][name],targets[name]),'Previously placed inventory moved: '+name)
 return metrics

def final_support(r,rows,by,last,g,design,geometry,all_foot_gaps):
 start=r['final_inspection_start_step'];final=[x for x in rows if start<x['step']<=start+120];need(len(final)==120 and [x['step'] for x in final]==list(range(start+1,start+121)),'Final120 truncated')
 end=r['checkpoints'][-1];need(end['phase']=='final_assembly_inspection' and end['step']>=start+120,'Missing full final camera');physical(all(abs(scalar(v))<=.00025 for v in end['jaw_joints'].values()) and len(end['jaw_joints'])==2,'Final jaws not open')
 transition=r['held_to_release_transition'];lower=next(x for x in r['checkpoints'] if x['phase']=='3_cover_lower');need(transition['step']==lower['step'] and transition['reference_world_m']==lower['poses']['cover']['position_world_m'],'Cover release reference reset')
 minclear=math.inf;finalgaps=None
 for row in final:
  p,R=pose(row['poses']['cover']);bp,BR=pose(row['poses']['base']);gap=all_foot_gaps(g,{'position_world_m':p.tolist(),'rotation_world':R.tolist()},{'position_world_m':bp.tolist(),'rotation_world':BR.tolist()});physical(gap['all_feet_over_pads'],'Actual foot misses pad');finalgaps=gap
  post=by.get(row['step']+1,last);lidtop=(arr(g['parts']['cover']['visual']['vertices_mm'])*.001@R.T+p)[:,2].max()
  for key,item in geometry['colliders'].items():
   jaw='/World/ER4iA/'+('JawL' if '/JawL/' in key else 'JawR');M=rigid_matrix(post['actor_world_matrices'][jaw]);v=arr(item['authored_cooking_vertices_in_jaw_m']);world=np.c_[v,np.ones(len(v))]@M;minclear=min(minclear,float(world[:,2].min()-lidtop)*1000)
 physical(minclear>0,'Source fingers not geometrically clear')
 bearing=[set(),set()];normal=[0.,0.];vertical=[0.,0.];positive=0
 support_ids=set()
 for c in r['support_contact_records']:
  key=(c['callback_sequence'],c['header_ordinal']);need(key not in support_ids,'Duplicate raw support header');support_ids.add(key)
  if 'actors' in c:need(set(c['actors'])=={'/World/Assembly_cover','/World/Assembly_base'},'Wrong raw support actor pair')
  need(c['num_contact_data']==len(c['points']) and type(c['event_type_code']) is int and isinstance(c['event_type_repr'],str),'Malformed support header');need(c['callback_sequence']<=r['support_callback_count'] and c['header_ordinal']>=0,'Invalid support identity')
  for pt in c['points']:physical(scalar(pt['separation_mm'])>=-.25,'Raw support penetration failed')
  if not start<c['state_step']<=start+120:continue
  for i in range(4,8):
   if set(c['colliders'])!={f'/World/Assembly_cover/C_{i:03d}_ear',f'/World/Assembly_base/C_{i+4:03d}_floor'}:continue
   for pt in c['points']:
    n=arr(pt['normal_world_raw'],(3,));j=arr(pt['impulse_Ns_raw'],(3,));need(abs(np.linalg.norm(n)-1)<1e-3,'Invalid support normal');nj=abs(float(n@j));half=int(c['state_step']>start+60);normal[half]+=nj;vertical[half]+=abs(float(j[2]))
    if abs(n[2])>=math.cos(math.radians(1)) and nj>1e-8:bearing[half].add(i);positive+=1
 root_speed=max(float(np.linalg.norm(arr(b['poses']['cover']['position_world_m'])-arr(a['poses']['cover']['position_world_m']))*60000) for a,b in zip(final,final[1:]));angular_speed=max(float(Rotation.from_matrix(pose(a['poses']['cover'])[1].T@pose(b['poses']['cover'])[1]).magnitude()*60*180/math.pi) for a,b in zip(final,final[1:]))
 return {'strict_support_complete':all(x=={4,5,6,7} for x in bearing),'final_support_halves':[sorted(x) for x in bearing],'normal_impulses_Ns_by_half':normal,'absolute_vertical_impulses_Ns_by_half':vertical,'positive_bearing_points':positive,'finger_clearance_min_mm':minclear,'final_actual_foot_geometry':finalgaps,'final_peak_CAD_root_speed_mm_s':root_speed,'final_peak_rotation_speed_deg_s':angular_speed,'static_equilibrium_certified':False}

def check_fixed_holds(r,g,historical_hold):
 holds=r['incoming_lid_holds'];need(len(holds)==2,'Missing two fixed lid holds');by={v['step']:v for v in r['full_motion_observations']};results=[]
 starts=[next(x['step'] for x in r['checkpoints'] if x['phase']==name) for name in ['3_cover_lift','3_cover_level_held']]
 for h,start in zip(holds,starts):
  need(h['requested_commands']==h['issued_commands']==120 and h['requested_duration_s']==2. and abs(h['actual_duration_s']-2.)<=2e-6 and h['fixed_window_start_index']==95 and h['fixed_window_samples']==25 and h['adaptive_wait'] is False and h['retry'] is False and h['truth_used_for_targets'] is False,'Fixed hold policy changed')
  need(h['start_step']==start and h['end_step']==start+120 and len(h['samples'])>=120,'Fixed hold boundaries changed')
  for index,v in enumerate(h['samples'],1):
   need(v['step']==start+index and abs(v['sim_time']-h['start_sim_time']-index/60)<=2e-6,'Hold sample gap');row=by[v['step']];p,R=pose(row['poses']['cover'])
   need(np.allclose(v['part_position_world_m'],p,atol=1e-8,rtol=0) and np.allclose(v['part_rotation_world'],R,atol=1e-7,rtol=0) and np.allclose(v['tip_world_m'],row['tip_world_m'],atol=1e-8,rtol=0) and np.allclose(v['tool_rotation_world'],row['tool_rotation_world'],atol=1e-7,rtol=0),'Hold is not actual sampled pose/FK')
   need(v['commanded_arm_joints']==h['expected_arm_joints']==row['issued_arm_q_rad'] and v['issued_jaw_joint_targets_m']==h['expected_jaw_joint_targets_m']==row['issued_jaw_targets_m'],'Hold target changed')
  results.append(historical_hold(h['samples'],g))
 return results

def check_sleep_evidence(r,rows):
 s=r['passive_sleep_state'];records=s['records'];need(s['used_for_motion_or_targets'] is False and s['force_gate_satisfied_by_sleep'] is False and s['acceptance_policy_changed'] is False and s['extra_physics_steps']==0 and s['sleep_or_wake_state_modified'] is False,'Observer changed experiment')
 need(len(records)==len(rows),'Sleep coverage incomplete');setup_error=s.get('setup_error');valid=True
 if setup_error:need(isinstance(setup_error,str) and s['all_samples_valid'] is False,'Invalid unavailable observer status')
 else:
  binding=r['passive_sleep_binding'];need(type(binding['stage_id']) is int and binding['stage_id']>0 and binding['read_only_queries'] is True,'Invalid sleep stage binding')
  need(binding['body_paths']=={n:'/World/Assembly_'+n for n in BODIES if n!='nest'},'Observer wrong body paths')
  ids=binding['encoded_body_paths'];need(set(ids)==set(BODIES)-{'nest'} and len(set(ids.values()))==4 and all(type(x) is int and x>0 for x in ids.values()),'Invalid observer body IDs')
  need(binding['body_flags']=={n:{'rigid_body_enabled':True,'kinematic':False,'disable_gravity':False} for n in ids},'Observer body mode invalid')
 for a,b in zip(records,rows):
  need(a['step']==b['step'] and abs(scalar(a['sim_time'])-b['sim_time'])<=2e-6 and a['phase']==b['phase'],'Sleep scope/time mismatch')
  if not setup_error:need(a['body_paths']==binding['body_paths'] and a['encoded_body_paths']==ids and a['expected_stage_id']==binding['stage_id'],'Sleep identity changed')
  if a['valid']:
   need(not setup_error and a['error'] is None and set(a['sleeping'])==set(BODIES)-{'nest'} and all(type(v) is bool for v in a['sleeping'].values()),'Invalid sleep values')
  else:need(a['sleeping'] is None and bool(a['error']),'Unknown sleep coerced');valid=False
 need(s['all_samples_valid']==(valid and bool(records) and not s.get('errors',[]) and not setup_error),'Forged observer validity')
 return {'all_samples_valid':s['all_samples_valid'],'setup_error':setup_error,'force_certificate_unchanged':True,'sleep_does_not_satisfy_support':True}

def verify_native_video(r):
 import imageio_ffmpeg
 v=r['video'];need(v['download_to_laptop'] is False,'Video remote policy changed');p=Path(v['path']);need(p.is_file() and p.stat().st_size==v['bytes'] and sha(p)==v['sha256'],'Actual video identity mismatch');frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(p));need(frames==v['frames'] and abs(seconds-frames/30)<.05 and abs(seconds-v['duration_s'])<.05,'Actual video timing mismatch')
 return {'identity_verified':True,'sha256':sha(p),'frames':frames,'seconds':seconds,'path':str(p),'actual_video_visually_reviewed':False}

def audit(r,source,runtime_file,runtime_sha,manifest_sha,policy_file='full_motion_policy.json',media=True,evidence_directory=None):
 source=Path(source);sys.path.insert(0,str(source))
 import check_supported_saddle_grip as generic
 from continuous_lid_command import independent_fk
 from foot_audit import all_foot_gaps
 from payload_checks import payload_score
 from check_observed_lid_placement import historical_hold
 g,design,policy=check_source(source,runtime_file,runtime_sha,manifest_sha,policy_file);check_identity(r,g,policy,source,runtime_file,runtime_sha,policy_file)
 complete=r['status']=='MOTION_COMPLETE';check_startup(r,g);generic.check_original_physics(r,g,design,complete,source,'original');check_motion(r,complete)
 rows,by,last=validate_ledger(r,independent_fk);metrics=check_process(rows,r['checkpoints'],r['part_results'],g,design,payload_score,complete)
 # Native raw trace validation lives alongside this checker; no scope mutation.
 from full_motion_trace_integrity import validate_full_traces
 traces=validate_full_traces(r,source,runtime_file)
 sleep=check_sleep_evidence(r,rows)
 out={'evidence_integrity_valid':True,'task':TASK,'runtime_status':r['status'],'runtime_error':r.get('error'),'full_motion_completed':complete,'full_assembly_accepted':False,'strict_support_accepted':False,'physical_hardware_qualified':False,'print_qualified':False,'static_equilibrium_certified':False,'per_part_actual_metrics':metrics,'raw_traces':traces,'sleep_evidence':sleep}
 if not complete:
  need(bool(r.get('error')),'Failed run missing stop cause');out['classification']='OBSERVED_PROCESS_STOP_REQUIRES_CAUSE_REVIEW';return out
 need(set(r['release_transitions'])==set(ORDER),'Missing three fixed release references')
 for number,name in enumerate(ORDER,1):
  lower=next(x for x in r['checkpoints'] if x['phase']==f'{number}_{name}_lower');t=r['release_transitions'][name];need(t['step']==lower['step'] and t['sim_time']==lower['sim_time'] and t['reference_world_m']==lower['poses'][name]['position_world_m'] and t['reference_not_used_for_targets'] is True,'Per-part release reference reset: '+name)
 out['historical_hold_measurements']=check_fixed_holds(r,g,historical_hold)
 from full_motion_command_integrity import check_full_commands
 out['commands']=check_full_commands(r,g,design,source,evidence_directory or Path(r['video']['path']).parent)
 geometry=json.loads((source/'telemetry-finger-geometry.json').read_text());support=final_support(r,rows,by,last,g,design,geometry,all_foot_gaps);out.update(support)
 out['strict_support_accepted']=support['strict_support_complete'];out['classification']='FULL_OBSERVED_ASSEMBLY_PASS' if support['strict_support_complete'] else 'FULL_MOTION_COMPLETE_INCONCLUSIVE_SUPPORT_EVIDENCE'
 out['full_assembly_accepted']=support['strict_support_complete']
 out['video']=verify_native_video(r) if media else {'identity_verified':False,'reason':'Explicit CPU draft/review mode; no qualification notification until media verified'}
 if not media:out['full_assembly_accepted']=False;out['classification']='DRAFT_CPU_REVIEW_NO_MEDIA_QUALIFICATION'
 return out

def main():
 p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('--source',required=True);p.add_argument('--runtime-file',required=True);p.add_argument('--runtime-sha256',required=True);p.add_argument('--manifest-sha256',required=True);p.add_argument('--policy-file',default='full_motion_policy.json');a=p.parse_args();path=Path(a.report)
 try:
  r=json.loads(path.read_text());out=audit(r,a.source,a.runtime_file,a.runtime_sha256,a.manifest_sha256,a.policy_file,evidence_directory=path.parent);code=0
 except ProcessFailure as e:out={'evidence_integrity_valid':False,'classification':'PHYSICAL_LIMIT_IN_RECORDED_EVIDENCE_AUDIT_INCOMPLETE','error':str(e),'full_assembly_accepted':False};code=2
 except Exception as e:out={'evidence_integrity_valid':False,'classification':'INCONCLUSIVE_EVIDENCE_OR_MODEL_INVALID','error_type':type(e).__name__,'error':str(e),'full_assembly_accepted':False};code=2
 out['actual_runtime_status']=locals().get('r',{}).get('status');out['actual_runtime_error']=locals().get('r',{}).get('error');out['raw_report_sha256']=sha(path);print(json.dumps(out,allow_nan=False));path.with_name('independent-full-observed-audit.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');return code
if __name__=='__main__':raise SystemExit(main())
