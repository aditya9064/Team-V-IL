"""Independent actual-outcome audit. Never calls a placement a full assembly."""
import argparse,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from placement_policy import CONTRACT,PHASES,DURATIONS
from foot_audit import all_foot_gaps,support_geometry
from payload_checks import payload_score
import check_supported_saddle_grip as OLD

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def need(x,message):
 if not x:raise ValueError(message)
def array(x,shape=None):
 a=np.asarray(x,float);need(np.isfinite(a).all() and (shape is None or a.shape==shape),'Invalid finite geometry/feedback');return a
def stop_category(error):
 if any(s in error for s in ['RGB','IK','endpoint','Camera','Non-finite','geometry','mass','prospective command','SE3','residual','stillness']):return 'TARGET_COMMAND_OR_EVIDENCE_INVALID_NOT_OBSERVED_PLACEMENT_FAILURE'
 if 'Unintended contact' in error:return 'OBSERVED_CONTACT_LIMIT'
 if any(s in error for s in ['slipped','tilted','Dragged','dragged','alignment','placement','release','PCB','moved']):return 'OBSERVED_PROCESS_LIMIT'
 return 'PROCESS_STOP_CAUSE_REQUIRES_REVIEW'
def bearing_impulse(normal,impulse):
 n=array(normal,(3,));j=array(impulse,(3,));need(abs(np.linalg.norm(n)-1)<1e-3,'Invalid raw contact normal')
 return abs(float(np.dot(n,j))),abs(float(j[2]))
def pose(p):return {'position_world_m':array(p['position_world_m'],(3,)).tolist(),'rotation_world':Rotation.from_quat(array(p['orientation_wxyz'],(4,))[[1,2,3,0]]).as_matrix().tolist()}
def score(p,target):
 R=np.asarray(pose(p)['rotation_world']);d=array(p['position_world_m'],(3,))-array(target,(3,))
 out={'xy_mm':float(np.linalg.norm(d[:2])*1000),'z_mm':float(abs(d[2])*1000),'tilt_deg':math.degrees(math.acos(float(np.clip(R[2,2],-1,1)))),'yaw_deg':abs(math.degrees(math.atan2(R[1,0],R[0,0])))}
 out['pass']=out['xy_mm']<=.75 and out['z_mm']<=.35 and out['tilt_deg']<=1 and out['yaw_deg']<=1;return out
def historical_hold(samples,mesh):
 need(len(samples)>=120,'Missing fixed hold samples');points=[]
 corners=array([v for foot in support_geometry(mesh) for v in foot['corners_mm']])*.001
 for s in samples:
  B=array(s['part_rotation_world'],(3,3));T=array(s['tool_rotation_world'],(3,3));p=array(s['part_position_world_m'],(3,));t=array(s['tip_world_m'],(3,));world=np.einsum('ij,kj->ik',corners,B)+p;local=np.einsum('ij,jk->ik',world-t,T)*1000
  need(np.allclose(local,array(s['corners_tool_mm'],(16,3)),atol=1e-7,rtol=0),'Incorrect historical held corners');points.append(local)
 fixed=np.asarray(points[95:120]);allpoints=np.asarray(points[95:])
 def diameter(v):return max(float(np.linalg.norm(v[j+1:]-v[j],axis=2).max()) for j in range(len(v)-1))
 return {'fixed_mm':diameter(fixed),'through_later_samples_mm':diameter(allpoints),'old_limit_mm':.020,'exceeded_old_limit':diameter(allpoints)>.020,'terminates_this_diagnostic':False}
def source_check(source,expected_runtime):
 source=Path(source);manifest=json.loads((source/'snapshot-sha256.json').read_text())
 for name,h in manifest.items():need(sha(source/name)==h,'Source changed: '+name)
 need(sha(source/'original_lid_placement_trial.py')==expected_runtime,'Unreviewed runtime')
 parent=json.loads((source/'v55-parent-sha256.json').read_text())
 for name in ['nest-mesh.json','nest-design.json','finger_cooking.py','telemetry-finger-geometry.json','verified-seating-mass-reference.json','lid_pose.py']:
  need(sha(source/name)==parent[name],'Original source changed: '+name)
 return json.loads((source/'nest-mesh.json').read_text()),json.loads((source/'nest-design.json').read_text())
def audit(r,source,expected_runtime,media=True):
 source=Path(source);g,d=source_check(source,expected_runtime)
 need(r['task']=='original_lid_observed_placement_diagnostic' and r['diagnostic_stage']=='observed-lid-placement','Wrong diagnostic')
 need(r['placement_policy']==CONTRACT and r['placement_policy_sha256']==sha(source/'placement_policy.py'),'Wrong policy')
 need(r['full_assembly_accepted'] is False and not r['physical_hardware_test'] and not r['print_release'],'Invalid qualification claim')
 need(r['trial_script_sha256']==expected_runtime and r['mesh_sha256']==sha(source/'nest-mesh.json') and r['design_sha256']==sha(source/'nest-design.json'),'Identity mismatch')
 need(r['grip_geometry_experiment']['variant']=='original' and r['part_order']==['cover'],'Wrong physical intervention')
 need(r['initial_preparation']['preplaced_dynamic_parts']==['base','carrier','pcb'] and r['initial_preparation']['fixtures_credited_as_robot_placed'] is False,'Fixture credit invalid')
 need(r['historical_foot_budget_qualified_for_this_run'] is False and r['predictive_clearance_certified'] is False,'Old certification falsely claimed')
 need(r['limits']==g['limits'] and r['payload_limits']==g['payload_limits'],'Physical limits changed')
 need(r['physics']=={'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':1},'Original physics changed')
 telemetry=json.loads((source/'telemetry-finger-geometry.json').read_text());expected=dict(telemetry['SDK_source_sha256'])
 expected.update({'fk.py':'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed','ik.py':'6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'})
 need(r['starter_hashes']==expected,'Robot source identity changed')
 need(r['kinematic_binding']['arm_joint_indices']==[0,1,2,3,4,5] and r['kinematic_binding']['joint_dtype']=='float32','Articulation source indices changed')
 complete=r['status']=='DIAGNOSTIC_PASS'
 OLD.check_original_physics(r,g,d,complete,source,'original')
 phases=r['motion_sampling']['phases'];need([x['phase'] for x in phases]==PHASES[:len(phases)],'Motion phase order changed')
 for row,duration in zip(phases,DURATIONS):
  need(row['requested_duration_s']==duration and row['commands']==round(duration*60) and abs(row['actual_duration_s']-duration)<2e-6 and abs(row['maximum_command_interval_s']-1/60)<2e-6,'Motion timing changed')
  tr=r['endpoint_traces'][row['phase']];need(len(tr)==6 and all(v['tip_error_mm']<=2 and v['tool_axis_error_deg']<=1 and v['spread_axis_error_deg']<=1 for v in tr),'Actual endpoint failed')
 for shot in r['checkpoints']:
  tr=shot['stillness_trace'];need(len(tr)==6 and all(0<=v['maximum_raw_joint_speed_rad_s']<.02 for v in tr),'Actual camera stillness failed')
  need(payload_score(shot['poses']['carrier'],shot['poses']['pcb'],d['carrier']['floor_thickness']*.001)['pass'],'Actual PCB failed')
 from placement_trace_integrity import validate_trace
 trace=validate_trace(r,source)
 holds=[]
 for h in r['incoming_lid_holds']:
  need(h['requested_commands']==h['issued_commands']==120 and h['requested_duration_s']==2 and abs(h['actual_duration_s']-2)<2e-6 and h['fixed_window_start_index']==95 and h['fixed_window_samples']==25 and not h['adaptive_wait'] and not h['truth_used_for_targets'],'Fixed hold policy changed')
  holds.append(historical_hold(h['samples'],g))
 rows=r['placement_observations'];need(rows,'No actual outcome ledger')
 for a,b in zip(rows,rows[1:]):need(b['step']==a['step']+1 and abs(b['sim_time']-a['sim_time']-1/60)<2e-6,'Dropped outcome samples')
 from continuous_lid_command import independent_fk
 kin=r['kinematic_binding'];need(kin['tool_offset_m']==[.293,0,0] and kin['table_top_z_m']==.9,'FK source binding changed')
 by={v['step']:v for v in r['diagnostic_trace']};last=r['lid_contact_trace']['last_existing_snapshot']
 for row in rows:
  array(row['actual_qdot_rad_s']);q=array(row['actual_q_rad']);R,t=independent_fk(q[kin['arm_joint_indices']],kin['tool_offset_m'],'float32')
  need(np.allclose(R,array(row['tool_rotation_world'],(3,3)),atol=1e-7,rtol=0) and np.linalg.norm(np.asarray(t)+[0,0,.9]-row['tip_world_m'])*1000<1e-5,'Actual FK ledger mismatch')
  post=by.get(row['step']+1,last);need(abs(post['sim_time']-row['sim_time'])<2e-6,'Wrong actual post-step sample')
  for name in ['base','carrier','pcb','cover']:
   M=array(post['actor_world_matrices']['/World/Assembly_'+name],(4,4));p=pose(row['poses'][name]);need(np.allclose(M[3,:3],p['position_world_m'],atol=1e-8,rtol=0) and np.allclose(M[:3,:3].T,p['rotation_world'],atol=1e-7,rtol=0),'Actual pose ledger mismatch')
  need(payload_score(row['poses']['carrier'],row['poses']['pcb'],d['carrier']['floor_thickness']*.001)['pass'],'Continuous actual loosePCB score failed')
 video=OLD.verify_video(r) if media else {'identity_verified':False}
 out={'evidence_integrity_valid':True,'classification':'OBSERVED_PROCESS_STOP' if not complete else 'OUTCOME_AUDIT_PENDING','runtime_error':r.get('error'),'historical_hold_measurements':holds,'full_assembly_accepted':False,'isolated_lid_placement_accepted':False,'trace':trace,'video':video,'raw_report_sha256':None,'source_manifest_sha256':sha(source/'snapshot-sha256.json')}
 if not complete:
  out['stop_category']=stop_category(r.get('error',''));return out
 need([x['phase'] for x in phases]==PHASES and len(r['part_results'])==1 and len(holds)==2,'Incomplete placement sequence')
 initial=next(v for v in r['checkpoints'] if v['phase']=='initial');lift=next(v for v in r['checkpoints'] if v['phase']=='3_cover_lift')
 delta=array(lift['poses']['cover']['position_world_m'])-array(initial['poses']['cover']['position_world_m']);held=pose(lift['poses']['cover']);R=array(held['rotation_world'])
 need(delta[2]*1000>=35 and np.linalg.norm(delta[:2])*1000<=2 and math.degrees(math.acos(float(np.clip(R[2,2],-1,1))))<=5 and abs(math.degrees(math.atan2(R[1,0],R[0,0])))<=3,'Actual pickup failed')
 need(set(r['placement_command_traces'])=={'3_cover_precontact','3_cover_lower'},'Final continuous command trace missing')
 for name,duration in [('3_cover_precontact',4.),('3_cover_lower',2.)]:
  c=r['placement_command_traces'][name];need(c['duration_s']==duration and c['commands']==len(c['records'])==round(duration*60),'Missing continuous commands')
  from continuous_lid_command import foot_residual_mm
  for q in c['records']:
   solved_R,solved_tip=independent_fk(q['solved_q_rad'],kin['tool_offset_m'],'float64')
   need(np.allclose(solved_R,q['solved_rotation_world'],atol=1e-9,rtol=0) and np.linalg.norm(np.asarray(solved_tip)-q['solved_tip_robot_m'])*1000<1e-6,'Solved command FK mismatch')
   residual=foot_residual_mm(c['estimated_foot_points_tool_m'],q['desired_tip_robot_m'],q['desired_rotation_world'],q['solved_tip_robot_m'],q['solved_rotation_world'])
   need(q['issued'] and q['physics_advanced'] and q['prospective_geometry_eligible'] and q['residual_limit_mm']==.002 and q['ik_position_tol_m']==1e-6 and float(residual.max())<=.002 and np.allclose(residual,q['per_corner_residual_mm'],atol=1e-8,rtol=0),'Continuous command invalid')
 transition=r['held_to_release_transition'];lower=next(s for s in r['checkpoints'] if s['phase']=='3_cover_lower')
 need(transition['step']==lower['step'] and transition['reference_world_m']==lower['poses']['cover']['position_world_m'],'Release reference reset')
 held_ref=array(lift['poses']['cover']['position_world_m'])-array(lift['tip_robot_m'])-[0,0,.9]
 held_slip=0.
 for row in rows:
  if not lift['step']<row['step']<=transition['step']:continue
  relative=array(row['poses']['cover']['position_world_m'])-array(row['tip_world_m']);held_slip=max(held_slip,float(np.linalg.norm(relative-held_ref)*1000))
  R=array(pose(row['poses']['cover'])['rotation_world']);need(held_slip<=2 and math.degrees(math.acos(float(np.clip(R[2,2],-1,1))))<=5 and abs(math.degrees(math.atan2(R[1,0],R[0,0])))<=3,'Actual held slip/tilt/yaw failed')
 reference=array(transition['reference_world_m'],(3,));release_rows=[v for v in rows if v['step']>transition['step']]
 excursion=max(float(np.linalg.norm(array(v['poses']['cover']['position_world_m'])-reference)*1000) for v in release_rows);need(excursion<=.25,'Actual release excursion failed')
 start=r['final_inspection_start_step'];final=[v for v in rows if start<v['step']<=start+120];need(len(final)==120,'Final inspection truncated')
 end=r['checkpoints'][-1];need(end['phase']=='lid_final_inspection' and len(end['jaw_joints'])==2 and all(abs(v)<=.00025 for v in end['jaw_joints'].values()),'Actual final jaws not open')
 raw=json.loads((source/'telemetry-finger-geometry.json').read_text());source_points=raw['colliders'];minclear=float('inf');foot_gaps=[]
 for row in final:
  for name in ['base','carrier','pcb','cover','nest']:
   target=[*g['layout_m']['assembly_body_centre'],.8949+d['assembly']['part_bottom_z'][name]*.001];need(score(row['poses'][name],target)['pass'],'Actual final pose failed: '+name)
  gaps=all_foot_gaps(g,pose(row['poses']['cover']),pose(row['poses']['base']));need(gaps['all_feet_over_pads'],'Actual foot misses assigned pad');foot_gaps.append(gaps)
  post=by.get(row['step']+1,last);world_lid=np.einsum('ij,jk->ik',array(g['parts']['cover']['visual']['vertices_mm'])*.001,np.asarray(pose(row['poses']['cover'])['rotation_world']).T)+row['poses']['cover']['position_world_m'];lidtop=float(world_lid[:,2].max())
  for key,item in source_points.items():
   jaw='/World/ER4iA/'+('JawL' if '/JawL/' in key else 'JawR');M=array(post['actor_world_matrices'][jaw]);v=array(item['authored_cooking_vertices_in_jaw_m']);world=np.einsum('ij,jk->ik',np.c_[v,np.ones(len(v))],M);minclear=min(minclear,float(world[:,2].min()-lidtop)*1000)
 need(minclear>0,'Actual fingers not geometrically above and clear of lid')
 bearing=[set(),set()];positive=0;normal_impulses=[0.,0.];vertical_impulses=[0.,0.]
 for contact in r['support_contact_records']:
  need(contact['num_contact_data']==len(contact['points']) and isinstance(contact['event_type_code'],int) and isinstance(contact['event_type_repr'],str),'Raw support header incomplete')
  step=contact['state_step']
  need(contact['callback_sequence']<=r['support_callback_count'] and contact['header_ordinal']>=0,'Support callback identity invalid')
  if not start<step<=start+120:continue
  colliders=contact['colliders'];cover=next((p for p in colliders if p.startswith('/World/Assembly_cover/')),None);base=next((p for p in colliders if p.startswith('/World/Assembly_base/')),None)
  for i in range(4,8):
   if cover!=f'/World/Assembly_cover/C_{i:03d}_ear' or base!=f'/World/Assembly_base/C_{i+4:03d}_floor':continue
   for point in contact['points']:
    normal=array(point['normal_world_raw'],(3,));imp=array(point['impulse_Ns_raw'],(3,));need(point['separation_mm']>=-.25,'Invalid support raw geometry');normal_j,vertical_j=bearing_impulse(normal,imp);half=int(step>start+60)
    normal_impulses[half]+=normal_j;vertical_impulses[half]+=vertical_j
    if abs(normal[2])>=math.cos(math.radians(1)) and normal_j>1e-8:bearing[half].add(i);positive+=1
 support_complete=all(v=={4,5,6,7} for v in bearing)
 peak_linear=max(float(np.linalg.norm(array(b['poses']['cover']['position_world_m'])-array(a['poses']['cover']['position_world_m']))*60000) for a,b in zip(final,final[1:]))
 peak_angular=max(float(Rotation.from_matrix(array(pose(a['poses']['cover'])['rotation_world']).T@array(pose(b['poses']['cover'])['rotation_world'])).magnitude()*60*180/math.pi) for a,b in zip(final,final[1:]))
 out.update(classification='ISOLATED_LID_PLACEMENT_PASS' if support_complete else 'INCONCLUSIVE_SUPPORT_EVIDENCE',isolated_lid_placement_accepted=support_complete,actual_held_maximum_slip_mm=held_slip,actual_release_maximum_3D_excursion_mm=excursion,actual_finger_clearance_min_mm=minclear,final_support_halves=[sorted(v) for v in bearing],observed_normal_impulse_Ns_by_half=normal_impulses,observed_absolute_vertical_impulse_Ns_by_half=vertical_impulses,final_peak_CAD_root_speed_mm_s=peak_linear,final_peak_rotation_speed_deg_s=peak_angular,static_equilibrium_certified=False,positive_final_bearing_points=positive,final_actual_foot_geometry=foot_gaps[-1],actual_final_inspection_samples=120)
 return out
def main():
 ap=argparse.ArgumentParser();ap.add_argument('report');ap.add_argument('--source',required=True);ap.add_argument('--runtime-sha256',required=True);a=ap.parse_args();p=Path(a.report)
 try:
  r=json.loads(p.read_text());out=audit(r,a.source,a.runtime_sha256);okay=True
 except Exception as exc:out={'evidence_integrity_valid':False,'classification':'INCONCLUSIVE_EVIDENCE_OR_MODEL_INVALID','error_type':type(exc).__name__,'error':str(exc),'actual_runtime_error':locals().get('r',{}).get('error'),'full_assembly_accepted':False,'isolated_lid_placement_accepted':False};okay=False
 out['raw_report_sha256']=sha(p) if p.exists() else None;p.with_name('independent-placement-audit.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps(out,allow_nan=False));raise SystemExit(0 if okay else 2)
if __name__=='__main__':main()
