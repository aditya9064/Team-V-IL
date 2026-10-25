"""Offline independent V54 audit. Never runs simulator or changes source/report.

Default CLI succeeds only for an isolated diagnostic pass. --audit-only may
succeed for valid negative-control evidence, without calling it a motion pass.
A reviewed runtime hash is required; source paths alone are not approval.
"""
import argparse,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

class AuditError(ValueError):pass
def need(value,message):
 if not value:raise AuditError(message)
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def finite(value):
 need(isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value),'Nonfinite/non-numeric scalar')
 return float(value)
def array(value,shape):
 a=np.asarray(value,float);need(a.shape==shape and np.isfinite(a).all(),'Invalid finite array '+str(shape));return a
def rigid(value):
 a=array(value,(3,3));need(np.allclose(a.T@a,np.eye(3),atol=1e-5,rtol=0) and abs(np.linalg.det(a)-1)<1e-5,'Improper rotation');return a
def row_matrix(value):
 m=array(value,(4,4));need(np.allclose(m[:,3],[0,0,0,1],atol=1e-8,rtol=0),'Invalid USD rigid matrix');rigid(m[:3,:3].T);return m
def pose(value):
 p=array(value['position_world_m'],(3,));q=array(value['orientation_wxyz'],(4,));need(abs(np.linalg.norm(q)-1)<=1e-3,'Invalid quaternion');return p,Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
def payload_metrics(carrier,pcb,floor=.002):
 a,A=pose(carrier);b,B=pose(pcb);x=A.T@(b-a);R=A.T@B
 return [float(np.linalg.norm(x[:2])*1000),float(abs(x[2]-floor)*1000),math.degrees(math.acos(float(np.clip(R[2,2],-1,1)))),abs(math.degrees(math.atan2(R[1,0],R[0,0])))]
def payload_ok(carrier,pcb,floor=.002):
 return all(v<=lim for v,lim in zip(payload_metrics(carrier,pcb,floor),[.75,.35,1.,1.]))
def matrix_pose(m):
 m=row_matrix(m);q=Rotation.from_matrix(m[:3,:3].T).as_quat();return {'position_world_m':m[3,:3].tolist(),'orientation_wxyz':q[[3,0,1,2]].tolist()}
def diameter(points):
 p=np.asarray(points,float);need(p.ndim==3 and p.shape[1:]==(16,3) and len(p)>1 and np.isfinite(p).all(),'Missing same-corner points')
 # Different computation from runtime broadcast: walk pairs, never mix corners.
 per=np.zeros(16)
 for i in range(len(p)-1):per=np.maximum(per,np.linalg.norm(p[i+1:]-p[i],axis=2).max(axis=0))
 return float(max(per)),per.tolist()

PHASES=['3_cover_orient','3_cover_above','3_cover_approach','3_cover_side_entry','3_cover_close','3_cover_lift']
DURATIONS=[1.,2.,2.,1.5,3.,1.5]
CHECKPOINTS=['initial']+PHASES+['3_cover_isolated_hold']
FORBIDDEN=['lid_pose_observations','lid_measured_placement_plan','lid_clearance_guards','lid_pre_release_geometry_hold','lid_grasp_corner_drift_veto','lid_release_profile','release_motion_audits','continuous_lid_commands']
BODY_NAMES=['base','carrier','pcb','cover']

def verify_source(source,expected_runtime_sha256):
 source=Path(source);runtime=source/'isolated_lid_grip_trial.py'
 need(isinstance(expected_runtime_sha256,str) and len(expected_runtime_sha256)==64,'Reviewed runtime SHA256 required')
 need(digest(runtime)==expected_runtime_sha256,'Unreviewed or changed runtime identity')
 parent=json.loads((source/'snapshot-sha256.json').read_text())
 for name,sha in parent.items():need(digest(source/name)==sha,'Inherited source mismatch: '+name)
 text=runtime.read_text();patches=json.loads((source/'isolated-grip-reversible-patches.json').read_text())
 for p in reversed(patches):need(text.count(p['new'])==1,'Nonunique reverse patch: '+p['label']);text=text.replace(p['new'],p['old'])
 need(text==(source/'payload_contact_lid_precontact_trial.py').read_text(),'Child does not reverse exactly to parent')
 return json.loads((source/'nest-mesh.json').read_text()),json.loads((source/'nest-design.json').read_text())

def check_identity(r,g,source,velocity):
 source=Path(source)
 need(r['trial_script_sha256']==digest(source/'isolated_lid_grip_trial.py'),'Report runtime hash mismatch')
 for key,name in [('mesh_sha256','nest-mesh.json'),('design_sha256','nest-design.json'),('incoming_lid_stability_helper_sha256','incoming_lid_stability.py'),('foot_audit_sha256','foot_audit.py'),('finger_cooking_script_sha256','finger_cooking.py')]:need(r[key]==digest(source/name),'Report source mismatch: '+name)
 need(r['limits']==g['limits'] and r['limits']['maximum_intended_penetration_mm']==.25,'Original limits changed')
 need(r['lid_pose_foot_budget']['hold_drift_position_error_mm']==.02,'Original readiness budget changed')
 need(r['payload_limits']==g['payload_limits']=={'xy_mm':.75,'z_mm':.35,'relative_tilt_deg':1.,'relative_yaw_deg':1.},'Payload limits changed')
 need(r['layout_m']==g['layout_m'],'Fixture layout changed')
 need(r['physics']=={'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':velocity},'Unexpected physics configuration')
 need(r['velocity_iteration_experiment']['requested']==velocity,'Case parameter mismatch')
 need(r['status'] in ['FAILED','DIAGNOSTIC_PASS'],'Nonterminal or unknown outcome')
 need(r['task']=='isolated_lid_grip_convergence_diagnostic' and r['case']=='isolated-lid-grip','Wrong task/case')
 need(r['diagnostic_stage']=='isolated-grip','Wrong isolated scope')
 need(r['full_assembly_accepted'] is False and r.get('isolated_lid_accepted',False) is False,'Assembly or lid placement falsely accepted')
 need(r['vision_guided'] is False and r['physical_hardware_test'] is False and r['print_release'] is False,'Unsupported scope/physical claim')
 need(all(r.get(k) is False for k in ['precontact_accepted','seating_accepted','release_accepted']),'Unexercised operation accepted or unspecified')
 geometry_binding=json.loads((source/'telemetry-finger-geometry.json').read_text())
 expected_starter=dict(geometry_binding['SDK_source_sha256'])
 expected_starter.update({'fk.py':'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed','ik.py':'6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'})
 need(r['starter_hashes']==expected_starter,'Original starter source identities changed')
 binding=r['kinematic_binding']
 need(binding['arm_joint_names']==['J'+str(i) for i in range(1,7)] and binding['arm_joint_indices']==[0,1,2,3,4,5],'Measured articulation joint indices not explicitly bound')
 need(binding['tool_offset_m']==[.293,0.,0.] and binding['table_top_z_m']==.9 and binding['joint_dtype']=='float32','Hardcoded independent FK assumptions not bound')
 need(r['part_order']==['cover'] and r['physical_parts']==BODY_NAMES[:3]+['cover','nest'],'Wrong motion or physical part set')
 need(not any(r.get(k) for k in FORBIDDEN),'Forbidden transport/vision/precontact/release outcome')
 need(r['expected_motion_phases']==PHASES and r['expected_checkpoint_phases']==CHECKPOINTS,'Wrong phase contract')
 if r['status']=='FAILED':need(bool(r.get('error')) and not r.get('diagnostic_stage_accepted'),'Failed run accepted')
 else:need(r['diagnostic_stage_accepted']=='isolated-grip' and not r.get('error'),'Wrong accepted diagnostic')


def check_original_physics(r,g,design,require_complete,source=None):
 need(r['payload_initial_gap_mm']==g['payload_initial_gap_mm']==0.,'Payload gap changed')
 need(not r['part_attachment_joints'],'Product attachments present')
 need(set(r['separate_body_audit'])==set(BODY_NAMES+['nest']),'Missing separate bodies')
 for name in BODY_NAMES+['nest']:
  b=r['separate_body_audit'][name]
  need(b['root']=='/World/Assembly_'+name and b['dynamic']==(name!='nest') and b['fixture_static']==(name=='nest') and not b['kinematic'] and not b['gravity_disabled'],'Invalid body mode: '+name)
  need(abs(b['mass_kg']-g['parts'][name]['volume_mm3']*1e-9*1240)<=1e-12,'Body mass changed')
  need(r['physical_collider_counts'][name]==len(g['parts'][name]['colliders']),'Collider count changed')
 need(set(r['installed_full_body_mass_properties'])==set(BODY_NAMES),'Mass readback missing')
 for val in r['installed_full_body_mass_properties'].values():need(val['validated'] and val['physics_time_advanced_s']==0.,'Invalid full body mass readback')
 if source is not None:
  reference=json.loads((Path(source)/'verified-seating-mass-reference.json').read_text())
  for name,val in r['installed_full_body_mass_properties'].items():
   expected=reference[name];need(abs(finite(val['mass_kg'])-expected['mass_kg'])<=1e-8 and np.linalg.norm(array(val['centre_of_mass_local_m'],(3,))-expected['centre_of_mass_local_m'])<=1e-7,'Mass or COM changed')
   def inertia(v):
    q=array(v['principal_axes_wxyz'],(4,));R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();return R@np.diag(array(v['diagonal_inertia_kg_m2'],(3,)))@R.T
   need(np.allclose(inertia(val),inertia(expected),rtol=1e-5,atol=1e-12),'Body inertia changed')
 for cmd in r['grip_commands'].values():
  need(cmd['extra_travel_per_jaw_mm']==.05 and cmd['penetration_stop_limit_mm']==.25,'Preload changed')
  need(abs(cmd['closed_grasp_command_per_jaw_m']-(cmd['nominal_contact_per_jaw_mm']-.05)*.001)<1e-12,'Clamp identity changed')
 need(r['grip_commands']==g['grip_commands'],'Per-part grip command changed')
 need(r['contact_monitor_setup']['api_and_subscription_before_reset'] and r['contact_monitor_setup']['runtime_checks_active_during_reset'],'Contact guards not active during reset')
 need(r['contact_headers_seen']>0,'No contact callbacks observed')
 if require_complete:
  need(not r['unexpected_contacts'] and not r['contact_monitor_errors'] and not r.get('payload_failure') and not r.get('transient_inventory_failure'),'Original physical gate failed')
 for v in r['contact_summary'].values():
  sep=finite(v['minimum_separation_mm']);imp=finite(v['maximum_impulse_Ns'])
  if require_complete:need(not ((not v['allowed_pair'] and (sep<-.001 or imp>1e-8)) or (v['allowed_pair'] and sep<-.25)),'Contact summary violates original gate')
 c=r['finger_cooking'];need(c['mode']=='source_fidelity_cooking' and c['collider_count']==138 and c['source_profile_dimensions_unchanged'] and c['min_thickness_m']==0 and c['hull_vertex_limit']==255 and finite(c['maximum_world_source_vertex_change_m'])<=1e-7,'Finger source fidelity failed')
 if require_complete:
  ray=r['finger_surface_audit'];need(ray['source_surface_match'] and len(ray['probes'])==12 and ray['maximum_allowed_absolute_error_mm']==.03,'Finger surface audit failed')
  for v in ray['probes']:need(v['same_jaw_hit'] and abs(finite(v['inward_error_mm']))<=.03,'Cooked finger surface error')
  need(r['cooked_geometry_probe_pass'],'Cooked part geometry failed')
  expected={(p['part'],p['label']) for p in g['geometry_probes']};actual=r['cooked_geometry_probes'];need(len(actual)==len(expected) and {(p['part'],p['label']) for p in actual}==expected,'Missing part probes')
  for p in actual:
   need(p['actual_part_hit']==p['expected_part_hit'] and p.get('distance_pass',True),'Part probe failed')
   if p['expected_part_hit']:need(p['maximum_distance_error_mm']==.03 and abs(finite(p['actual_distance_mm'])-p['expected_distance_mm'])<=.03,'Part probe distance failed')
 t=r['step_timing_audit'];need(t['samples']>0 and finite(t['maximum_step_error_s'])<=2e-6 and finite(t['maximum_render_advance_s'])<=1e-9,'Physics step/render timing failed')
 a=r['payload_motion_audit'];need(a['samples']==t['samples'],'Missing per-step PCB guard')
 if require_complete:need(all(finite(a[k])<=lim for k,lim in [('maximum_xy_mm',.75),('maximum_z_mm',.35),('maximum_relative_tilt_deg',1),('maximum_relative_yaw_deg',1)]),'Per-step payload guard failed')


def check_motion(r,design,complete):
 phases=r['motion_sampling']['phases'];actual=[x['phase'] for x in phases]
 need(actual==PHASES[:len(actual)] and len(actual)<=6,'Unexpected/missing motion order')
 if complete:need(actual==PHASES,'Missing pickup phase')
 need(r['motion_sampling']['command_hz']==60 and r['motion_sampling']['physics_steps_per_command']==1,'Command rate changed')
 for row,duration in zip(phases,DURATIONS):
  need(row['requested_duration_s']==duration and row['commands']==round(duration*60),'Original pickup duration changed')
  need(abs(finite(row['actual_duration_s'])-duration)<=2e-6 and abs(finite(row['maximum_command_interval_s'])-1/60)<=2e-6,'Motion timing changed')
  trace=r['endpoint_traces'][row['phase']];need(len(trace)==6,'Endpoint gate truncated')
  for s in trace:need(finite(s['tip_error_mm'])<=2 and finite(s['tool_axis_error_deg'])<=1 and finite(s['spread_axis_error_deg'])<=1,'Endpoint gate failed')
 shots=r['checkpoints'];names=[x['phase'] for x in shots]
 need(names==CHECKPOINTS[:len(names)] and len(names)<=8,'Unexpected checkpoint sequence')
 if complete:need(names==CHECKPOINTS[:7] or names==CHECKPOINTS,'Missing checkpoint')
 for shot in shots:
  trace=shot['stillness_trace'];need(len(trace)==6,'Camera stillness gate missing')
  for item in trace:need(finite(item['maximum_raw_joint_speed_rad_s'])<.02,'Camera not still')
  for a,b in zip(trace,trace[1:]):need(b['step']>a['step'] and abs(b['sim_time']-a['sim_time']-(b['step']-a['step'])/60)<2e-6,'Camera timing inconsistent')
  need(payload_ok(shot['poses']['carrier'],shot['poses']['pcb'],design['carrier']['floor_thickness']*.001),'PCB checkpoint pose failed')
 if complete:
  results=r['part_results'];need(len(results)==1 and results[0]['part']=='cover','Wrong pickup results')
  result=results[0];need(not result['placement_verified'] and not result['release_verified'],'Placement/release falsely accepted')
  p0,_=pose(shots[0]['poses']['cover']);pn,R=pose(next(s for s in shots if s['phase']=='3_cover_lift')['poses']['cover']);d=(pn-p0)*1000
  calculated={'lift_mm':float(d[2]),'xy_slip_mm':float(np.linalg.norm(d[:2])),'tilt_deg':math.degrees(math.acos(float(np.clip(R[2,2],-1,1)))),'yaw_deg':abs(math.degrees(math.atan2(R[1,0],R[0,0])))}
  for k,v in calculated.items():need(abs(v-finite(result['pickup'][k]))<1e-6,'Forged pickup summary: '+k)
  need(result['pickup']['pass'] and calculated['lift_mm']>=35 and calculated['xy_slip_mm']<=2 and calculated['tilt_deg']<=5 and calculated['yaw_deg']<=3,'Pickup physical gate failed')
  h=r['held_motion_audits']['cover'];need(h['samples']>0 and finite(h['maximum_relative_slip_mm'])<=2 and finite(h['maximum_tilt_deg'])<=5 and finite(h['maximum_yaw_deg'])<=3,'Held body motion guard failed')
 return names


def check_incoming(r,g,source):
 a=r['incoming_lid_stability'];samples=a['samples']
 need(a['requested_duration_s']==2. and a['requested_commands']==a['issued_commands']==120,'Fixed hold not completed')
 need(a['end_step']-a['start_step']==120 and abs(a['end_sim_time']-a['start_sim_time']-2.)<=2e-6 and abs(a['actual_duration_s']-2.)<=2e-6,'Fixed hold timing wrong')
 need(a['fixed_window_start_index']==95 and a['fixed_window_samples']==25,'Rolling or changed hold window')
 need(a['adaptive_wait'] is False and a['retry'] is False and a['truth_used_for_targets'] is False,'Adaptive or truth-steered hold')
 need(a.get('reference_freeze_count',0)==0 and 'reference_freeze_step' not in a,'Grasp reference reset/freeze in isolated test')
 need(len(samples)>=120 and samples[0]['step']==a['start_step']+1 and samples[119]['step']==a['end_step'],'Incomplete hold samples')
 from foot_audit import support_geometry
 from continuous_lid_command import independent_fk
 corners=np.array([v for foot in support_geometry(g) for v in foot['corners_mm']]);by={x['step']:x for x in r['diagnostic_trace']}
 need(len(by)==len(r['diagnostic_trace']),'Duplicate diagnostic pose steps')
 latest=r['lid_contact_trace']['last_existing_snapshot'];points=[];jp={j:[] for j in ['JawL','JawR']}
 for i,s in enumerate(samples):
  need(s['step']==a['start_step']+i+1 and abs(s['sim_time']-a['start_sim_time']-(i+1)/60)<=2e-6,'Missing consecutive hold sample')
  need(s['commanded_arm_joints']==a['expected_arm_joints'] and s['issued_jaw_joint_targets_m']==a['expected_jaw_joint_targets_m'],'Hold command changed')
  B=rigid(s['part_rotation_world']);p=array(s['part_position_world_m'],(3,));T=rigid(s['tool_rotation_world']);tip=array(s['tip_world_m'],(3,))
  world=corners*.001@B.T+p;computed=(world-tip)@T*1000
  need(np.allclose(computed,array(s['corners_tool_mm'],(16,3)),atol=1e-7,rtol=0),'Corrupt reported corner reconstruction')
  post=by.get(s['step']+1,latest);need(abs(post['sim_time']-s['sim_time'])<=2e-6,'Unmatched post-step pose')
  cm=row_matrix(post['actor_world_matrices']['/World/Assembly_cover'])
  need(np.allclose(cm[3,:3],p,atol=1e-8,rtol=0) and np.allclose(cm[:3,:3].T,B,atol=1e-7,rtol=0),'Reported part sample not actual post-step body')
  F,fp=independent_fk(post['joint_positions'][:6],[.293,0,0],'float32')
  need(np.allclose(T,F,atol=1e-7,rtol=0) and np.linalg.norm(tip-np.asarray(fp)-[0,0,.9])*1000<1e-5,'Tool sample differs from independently reconstructed measured FK')
  for jaw in jp:
   jm=row_matrix(post['actor_world_matrices']['/World/ER4iA/'+jaw]);jp[jaw].append((np.c_[world,np.ones(16)]@np.linalg.inv(jm))[:,:3]*1000)
  for body in BODY_NAMES:need('/World/Assembly_'+body in post['actor_world_matrices'],'Dynamic body pose missing: '+body)
  need(payload_ok(matrix_pose(post['actor_world_matrices']['/World/Assembly_carrier']),matrix_pose(post['actor_world_matrices']['/World/Assembly_pcb'])),'Recorded PCB pose failed')
  points.append(computed)
 need(abs(samples[119]['sim_time']-a['end_sim_time'])<=2e-6,'Hold end not exact')
 fixed,per=diameter(points[95:120]);allvalue,allper=diameter(points[95:]);stats={'fixed_maximum_mm':fixed,'through_terminal_maximum_mm':allvalue,'sample_count':len(samples),'actual_jaw_diameters_mm':{j:diameter(v[95:])[0] for j,v in jp.items()}}
 for key,value,pc in [('fixed_hold_result',fixed,per),('post_camera_hold_result',allvalue,allper)]:
  if key in a:
   result=a[key];need(result['budget_mm']==.02 and abs(result['maximum_pairwise_corner_diameter_mm']-value)<1e-7 and np.allclose(result['per_corner_diameter_mm'],pc,atol=1e-7,rtol=0),'Incorrect readiness result')
   need(result['pass']==(value<=.02) and result['fixed_window_start_index']==95 and result['fixed_window_samples']==25,'Forged readiness outcome')
 need('fixed_hold_result' in a,'Fixed hold score missing')
 return stats


def verify_video(r):
 import imageio_ffmpeg
 v=r['video'];need(v['download_to_laptop'] is False,'Video remote policy changed')
 path=Path(v.get('path') or v.get('failed_recording_path',''));need(path.is_file() and path.stat().st_size>0,'Video missing')
 sha=digest(path);frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(path));need(frames==v['frames'] and abs(seconds-frames/30)<.05,'Video count/duration mismatch')
 if r['status']=='DIAGNOSTIC_PASS':need(path.stat().st_size==v['bytes'] and sha==v['sha256'] and abs(seconds-v['duration_s'])<.05,'Video identity mismatch')
 else:need(v['frozen_failure_tail_frames']==60 and frames>=60,'Failure tail missing')
 return {'identity_verified':True,'path':str(path),'sha256':sha,'bytes':path.stat().st_size,'frames':frames,'seconds':seconds,'actual_video_visually_reviewed':False}


def validate_report(r,source,expected_runtime_sha256,velocity,check_media=True):
 need(velocity in [1,8,16],'Unexpected prospective case')
 source=Path(source);sys.path.insert(0,str(source))
 g,design=verify_source(source,expected_runtime_sha256)
 check_identity(r,g,source,velocity)
 from isolated_trace_integrity import validate_trace
 trace=validate_trace(r,source)
 readiness_error=r.get('error')=='incoming_lid_grasp_unstable_before_measurement'
 complete=r['status']=='DIAGNOSTIC_PASS' or readiness_error
 # An unrelated gate failure remains NONDIAGNOSTIC even if readiness exists.
 if complete:
  check_original_physics(r,g,design,True,source);names=check_motion(r,design,True)
  stats=check_incoming(r,g,source)
  need(trace['capture_scope_reached'] and trace['selected_headers']>0,'No selected contact evidence')
  # Independently enforce original .250mm raw depth, with no force threshold.
  for row in r['lid_contact_trace']['rows']:
   for h in row['headers']:
    for pt in h['points']:need(finite(pt['separation_mm'])>=-.25,'Raw intended contact depth failed')
  if r['status']=='DIAGNOSTIC_PASS':
   need(names==CHECKPOINTS and stats['through_terminal_maximum_mm']<=.02,'Claimed pass fails complete fixed-plus-later readiness')
   need('post_camera_hold_result' in r['incoming_lid_stability'],'Terminal camera readiness missing')
   classification='ISOLATED_DIAGNOSTIC_PASS'
  else:
   need(stats['through_terminal_maximum_mm']>.02,'Readiness error without reproduced readiness failure')
   corroborated=all(v>.02 for v in stats['actual_jaw_diameters_mm'].values())
   classification='NEGATIVE_CONTROL_REPRODUCED' if velocity==1 and corroborated else 'NONDIAGNOSTIC'
 else:
  stats=None;classification='NONDIAGNOSTIC'
 payload_trace=r.get('payload_contact_trace',{})
 need(payload_trace.get('complete') is True and not payload_trace.get('errors'),'Payload recorder reports errors')
 need(payload_trace.get('capture_scope_reached') is False and not payload_trace.get('rows'),'Unexpected payload raw capture scope; separately review it')
 video=verify_video(r) if check_media else {'identity_verified':False,'actual_video_visually_reviewed':False}
 return {'audit_integrity_valid':True,'classification':classification,'runtime_sha256':expected_runtime_sha256,'velocity_iterations':velocity,'original_status':r['status'],'original_error':r.get('error'),'isolated_diagnostic_pass':classification=='ISOLATED_DIAGNOSTIC_PASS','full_assembly_accepted':False,'precontact_seating_release_accepted':False,'trace':trace,'payload_raw_acquisition':'UNEXERCISED; per-step original PCB motion/contact gates remain applicable','readiness':stats,'video':video,'limitations':['Fresh isolated after-lift state omits V53 prefix/transport/RGB leveling and contact cache history.','Only articulation iterations varied; independent product body iteration settings are not a global intervention.','Valid acquisition or expected failing control is not a mechanical pass.','Video identity/count checks do not constitute viewing the actual recording.']}


def classify_comparison(audits):
 need(len(audits)==3 and {a['velocity_iterations'] for a in audits}=={1,8,16},'Need all three prospective cases')
 by={a['velocity_iterations']:a for a in audits}
 if by[1]['classification']!='NEGATIVE_CONTROL_REPRODUCED':return 'NONDIAGNOSTIC_CONTROL_DID_NOT_REPRODUCE'
 if all(by[n]['classification']=='ISOLATED_DIAGNOSTIC_PASS' for n in [8,16]):return 'LOCAL_ITERATION_SENSITIVITY_SUPPORTED_FULL_SEQUENCE_REQUIRED'
 return 'NO_CONVERGED_REPAIR_DEMONSTRATED'


def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('report');ap.add_argument('--source',required=True);ap.add_argument('--expected-runtime-sha256',required=True);ap.add_argument('--velocity-iterations',type=int,choices=[1,8,16],required=True);ap.add_argument('--audit-only',action='store_true');ap.add_argument('--audit-out');ap.add_argument('--skip-media',action='store_true',help='CPU/compact evidence only; marks media unverified')
 a=ap.parse_args();dest=Path(a.audit_out) if a.audit_out else Path(a.report).with_name('independent-isolated-audit.json')
 try:
  r=json.loads(Path(a.report).read_text());result=validate_report(r,a.source,a.expected_runtime_sha256,a.velocity_iterations,not a.skip_media);okay=a.audit_only or result['isolated_diagnostic_pass']
 except Exception as exc:
  result={'audit_integrity_valid':False,'classification':'NONDIAGNOSTIC','error_type':type(exc).__name__,'error':str(exc),'full_assembly_accepted':False};okay=False
 dest.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,allow_nan=False));return 0 if okay else 1
if __name__=='__main__':raise SystemExit(main())
