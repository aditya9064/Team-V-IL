"""Native full-task contact telemetry validation; no report rewriting/slicing."""
import hashlib,json,math
from pathlib import Path
from check_full_observed_assembly import need,arr,rigid_matrix,TASK,STAGE

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def finite_tree(v):
 if isinstance(v,float):need(math.isfinite(v),'Nonfinite trace scalar')
 elif isinstance(v,dict):
  for x in v.values():finite_tree(x)
 elif isinstance(v,list):
  for x in v:finite_tree(x)

def one(r,source,prefix,collection,helper,root_key,other_key,first_phase,countkey,pointkey):
 mod=__import__(helper);p=r[prefix+'_contact_trace_provenance'];t=r[prefix+'_contact_trace'];frames=r[collection]
 need(all(p[k] is False for k in ['control_or_truth_steering','extra_physics_queries_or_registration','runtime_logging_IO_per_step']),'Trace observer altered experiment')
 need(p['helper_sha256']==sha(source/(helper+'.py')),'Raw helper identity mismatch')
 need(t['schema']==mod.SCHEMA and t['absence_semantics']==mod.ABSENCE and t['physics_or_target_use'] is False,'Trace schema/absence changed')
 need(t['complete'] is True and not t['errors'] and t['original_runtime_status']==r['status'] and t['original_runtime_error']==r.get('error'),'Incomplete raw contact trace');finite_tree(t)
 rows=t['rows']
 if not rows:
  need(not t['capture_scope_reached'] and r['status']=='FAILED','Required contact scope absent');return {'capture_scope_reached':False,'telemetry_integrity_valid':True}
 need(t['capture_scope_reached'] is True,'Raw trace flag mismatch');first=next((x for x in frames if x['phase']==first_phase),None);need(first and rows[0]['state_step']==first['step'] and abs(rows[0]['pre_step_time_s']-first['sim_time'])<=2e-6,'Trace scope begins at wrong phase')
 by={x['step']:x for x in frames};need(len(by)==len(frames),'Duplicate raw frames');last=t['last_existing_snapshot'];need(last and abs(last['sim_time']-rows[-1]['post_step_time_s'])<=2e-6,'Wrong final raw snapshot')
 need(t['first_step']==rows[0]['state_step'] and t['last_step']==rows[-1]['state_step'] and t['first_pre_step_time_s']==rows[0]['pre_step_time_s'] and t['last_post_step_time_s']==rows[-1]['post_step_time_s'],'Raw trace bounds mismatch')
 g=json.loads((source/'nest-mesh.json').read_text());roles={f'/World/Assembly_{name}/C_{i:03d}_{c["role"]}':[name,c['role']] for name,value in g['parts'].items() for i,c in enumerate(value['colliders'])};callbacks=[];headers=points=total=zero=0;previous=None
 for index,row in enumerate(rows):
  need(row['closed'] is True and abs(row['post_step_time_s']-row['pre_step_time_s']-1/60)<=2e-6,'Unclosed or wrong-timestep contact row')
  if previous:need(row['state_step']==previous['state_step']+1 and abs(row['pre_step_time_s']-previous['post_step_time_s'])<=2e-6,'Raw trace dropped step')
  previous=row;link=row['pre_step_snapshot_link'];need(link['collection']==collection and 0<=link['index']<len(frames),'Raw snapshot link wrong');pre=frames[link['index']];need(pre['step']==link['step']==row['state_step'] and pre['sim_time']==link['sim_time']==row['pre_step_time_s'],'Raw snapshot link differs');post=by.get(row['state_step']+1,last);need(abs(post['sim_time']-row['post_step_time_s'])<=2e-6,'Raw poststep frame mismatch')
  for actor in [t[root_key],*t[other_key]]:
   for snap in [pre,post]:rigid_matrix(snap['actor_world_matrices'][actor])
  need(row['callback_count']==len(row['callbacks']) and row[countkey]==len(row['headers']) and row[pointkey]==sum(len(h['points']) for h in row['headers']),'Raw counts mismatch');need(row['reported_record_state']==('REPORTED_RECORDS' if row['headers'] else 'NO_REPORTED_RECORD'),'Absence evidence forged')
  need(row['header_count_total']==sum(c['header_count_total'] for c in row['callbacks']),'Raw callback totals mismatch');headers+=row[countkey];points+=row[pointkey];total+=row['header_count_total'];zero+=int(not row['headers'])
  for cb in row['callbacks']:
   need(cb['closed'] is True and cb['row_index']==index and cb['state_step']==row['state_step'] and row['pre_step_time_s']-2e-6<=cb['callback_world_time_s']<=row['post_step_time_s']+2e-6,'Raw callback identity/time mismatch')
   selected=[h for h in row['headers'] if h['callback_sequence']==cb['sequence']];need(len(selected)==cb[countkey] and sum(h['num_contact_data'] for h in selected)==cb[pointkey],'Selected callback totals mismatch');ordinals=[h['header_ordinal'] for h in selected];need(ordinals==sorted(set(ordinals)) and all(0<=x<cb['header_count_total'] for x in ordinals),'Raw ordinals invalid')
   for h in selected:
    need(h['state_step']==row['state_step'] and h['phase']==cb['phase'] and h['callback_world_time_s']==cb['callback_world_time_s'],'Raw header identity mismatch');need(h['selection']==mod.selected_pair(h['actors'],h['colliders'],t[root_key],t[other_key]) and h['selection'] is not None,'Wrong selected body pair')
    need(all(type(x) is str and x.startswith('/') for x in h['actors']+h['colliders']),'Invalid raw paths');need(all(type(x) is str and str(int(x))==x for x in h['actor_ids_decimal']+h['collider_ids_decimal']),'Invalid raw encoded paths');need(type(h['event_type_code']) is int and isinstance(h['event_type_repr'],str),'Raw event type invalid');need(h['event_type_name']['available'] is (h['event_type_name']['value'] is not None),'Raw event availability invalid')
    need(h['num_contact_data']==len(h['points']) and [pt['raw_data_index'] for pt in h['points']]==list(range(h['contact_data_offset'],h['contact_data_offset']+h['num_contact_data'])),'Contact point ranges invalid');need(h['roles']==[roles.get(x) for x in h['colliders']],'Raw collider roles mismatch')
    for pt in h['points']:
     for key in ['position_world_m','normal_world_raw','impulse_Ns_raw']:arr(pt[key],(3,))
     need(pt['separation_mm']==pt['separation_m']*1000,'Raw separation units mismatch')
     # These selected pairs are intended contacts with unchanged .250mm depth.
     from check_full_observed_assembly import physical
     physical(pt['separation_mm']>=-.25,'Actual selected raw contact depth failed')
     for optional in pt['optional_fields'].values():need(optional['available'] is (optional['value'] is not None),'Optional raw availability invalid')
   callbacks.append(cb)
 need([x['sequence'] for x in callbacks]==list(range(1,len(callbacks)+1)),'Contact callback sequence omitted');need(t['callback_count']==len(callbacks) and t['header_count_total']==total and t['selected_header_count']==headers and t['selected_point_count']==points,'Raw terminal totals mismatch')
 return {'telemetry_integrity_valid':True,'capture_scope_reached':True,'steps':len(rows),'callbacks':len(callbacks),'selected_headers':headers,'selected_points':points,'zero_matching_header_steps':zero,'absence_semantics':mod.ABSENCE,'mechanical_success_checked_here':False}

def validate_full_traces(r,source,runtime_file):
 source=Path(source);need(r['task']==TASK and r['diagnostic_stage']==STAGE and r['full_assembly_accepted'] is False,'Not native unqualified full report');need(r['trial_script_sha256']==sha(source/runtime_file),'Trace runtime identity mismatch')
 lid=one(r,source,'lid','diagnostic_trace','lid_contact_trace','cover_root','jaw_roots','3_cover_close','cover_jaw_header_count','cover_jaw_point_count')
 pcb=one(r,source,'payload','payload_contact_pose_trace','payload_contact_trace','pcb_root','carrier_roots','2_carrier_orient','pcb_carrier_header_count','pcb_carrier_point_count')
 if r['status']=='MOTION_COMPLETE':need(lid['capture_scope_reached'] and pcb['capture_scope_reached'],'Complete full run lacks raw scope')
 from check_payload_contact_trace import validate_cooked
 validate_cooked(r['payload_cooked_geometry'],r['status'])
 return {'cover_jaws':lid,'PCB_carrier':pcb,'native_full_scope':True}
