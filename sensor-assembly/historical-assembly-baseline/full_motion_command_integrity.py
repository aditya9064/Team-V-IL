"""Independent command/RGB checks for native full report, without metadata projection."""
import json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from check_full_observed_assembly import need,physical,arr,pose,sha

def check_base_commands(r,g,design,source,evidence_directory):
 from continuous_lid_command import independent_fk,foot_residual_mm,held_foot_points
 from check_multistart_lid_precontact import validate_multistart_observation
 from held_pose_bounded import placement_from_held
 # Every base frame's25-seed image-only selection is replayed from saved images.
 obs=r['held_pose_observations'];need([v['label'] for v in obs]==['1_base_held_inspection_a','1_base_held_inspection_b','1_base_leveled_inspection_a','1_base_leveled_inspection_b'],'Missing base RGB correction prefix')
 for v in obs:validate_multistart_observation(v,evidence_directory,g['parts']['base']['visual']['vertices_mm'],design)
 rep=r['held_pose_repeatability'];need([v['label'] for v in rep]==['1_base_held_inspection','1_base_leveled_inspection'],'Base two-frame scores missing')
 shots={v['phase']:v for v in r['checkpoints']}
 for i,label in enumerate(['1_base_held_inspection','1_base_leveled_inspection']):
  a,b=obs[2*i:2*i+2];delta=float(np.linalg.norm(arr(a['position_world_m'])-arr(b['position_world_m']))*1000);angle=float(Rotation.from_matrix(arr(a['rotation_world']).T@arr(b['rotation_world'])).magnitude()*180/math.pi);need(abs(rep[i]['position_difference_mm']-delta)<1e-7 and abs(rep[i]['orientation_difference_deg']-angle)<1e-7,'Base repeatability summary mismatch');physical(delta<=.1 and angle<=.15,'Base RGB frame disagreement')
  actual=shots[label+'_b']['poses']['base'];p,R=pose(actual);pe=float(np.linalg.norm(p-arr(b['position_world_m']))*1000);ae=float(Rotation.from_matrix(arr(b['rotation_world']).T@R).magnitude()*180/math.pi);physical(pe<=.15 and ae<=.25,'Base original independent accuracy failed')
 finalobs=obs[-1];shot=shots[finalobs['label']];actualR,tip=independent_fk(np.array(next(v for v in r['full_motion_observations'] if v['step']==shot['step'])['actual_q_rad'])[:6],[.293,0,0],'float32');planned=placement_from_held(finalobs['position_world_m'],finalobs['rotation_world'],np.asarray(tip)+[0,0,.9],actualR,[*g['layout_m']['assembly_body_centre'],.8979],g['parts']['base']['visual']['vertices_mm'])
 for key in ['tip_world_m','tool_rotation_world']:need(np.allclose(planned[key],r['measured_placement_plan'][key],atol=1e-9,rtol=0),'Base placement plan not RGB/FK-derived')
 return {'base_multistart_RGB_frames_verified':4,'base_plan_recomputed_from_RGB_FK':True,'prefix_command_source_equivalence_requires_separate_review':True}

def check_lid_commands(r,g,design):
 from continuous_lid_command import independent_fk,foot_residual_mm,held_foot_points
 shots={v['phase']:v for v in r['checkpoints']}
 lidobs=r['lid_pose_observations'];need([x['label'] for x in lidobs]==['3_cover_held_inspection_a','3_cover_held_inspection_b','3_cover_leveled_inspection_a','3_cover_leveled_inspection_b'],'Missing lid RGB observations')
 for o in lidobs:
  p=arr(o['position_world_m'],(3,));R=arr(o['rotation_world'],(3,3));offset=arr(o['offset_mm_and_xyz_deg'],(6,));need(np.allclose(R.T@R,np.eye(3),atol=1e-5,rtol=0) and abs(np.linalg.det(R)-1)<1e-5,'Invalid lid estimated rotation');need(o['optimizer']['success'] is True and all(x['success'] is True for x in o['optimizer']['rounds']),'Invalid lid RGB optimization')
  need(np.all(np.abs(offset[:3])<1.95) and np.all(np.abs(offset[3:])<2.95),'Lid RGB fit reaches bounds')
  need(set(o['views'])=={'inspection_x','inspection_y'},'Lid camera evidence missing')
  for v in o['views'].values():need(v['edge_p95_px']<=2 and v['edge_coverage_within_2px']>=.95 and v['reverse_edge_p95_px']<=2 and v['reverse_edge_coverage_within_2px']>=.95,'Lid genuine RGB validity failed')
 for i,label in enumerate(['3_cover_held_inspection','3_cover_leveled_inspection']):
  a,b=lidobs[2*i:2*i+2];delta=float(np.linalg.norm(arr(a['position_world_m'])-arr(b['position_world_m']))*1000);angle=float(Rotation.from_matrix(arr(a['rotation_world']).T@arr(b['rotation_world'])).magnitude()*180/math.pi);physical(delta<=.1 and angle<=.15,'Lid RGB frame disagreement')
 from lid_pose import placement_from_held as place_lid
 final_lid=lidobs[-1];shot=shots[final_lid['label']];actualR,tip=independent_fk(np.array(next(v for v in r['full_motion_observations'] if v['step']==shot['step'])['actual_q_rad'])[:6],[.293,0,0],'float32');lidplan=place_lid(final_lid['position_world_m'],final_lid['rotation_world'],np.asarray(tip)+[0,0,.9],actualR,[*g['layout_m']['assembly_body_centre'],.9009],g['parts']['cover']['visual']['vertices_mm'],gap_mm=.10)
 for key in ['tip_world_m','tool_rotation_world']:need(np.allclose(lidplan[key],r['lid_measured_placement_plan'][key],atol=1e-9,rtol=0),'Lid target is not recorded RGB/FK recipe')
 full_by={x['step']:x for x in r['full_motion_observations']}
 traces=r['placement_command_traces'];need(set(traces)=={'3_cover_precontact','3_cover_lower'},'Lid continuous commands missing')
 for name,duration in [('3_cover_precontact',4.),('3_cover_lower',2.)]:
  c=traces[name];need(c['duration_s']==duration and c['commands']==len(c['records'])==round(duration*60),'Lid command count/duration invalid')
  need(np.allclose(c['held_actual_FK_tip_world_m'],np.asarray(tip)+[0,0,.9],atol=1e-10,rtol=0) and np.allclose(c['held_actual_FK_rotation_world'],actualR,atol=1e-7,rtol=0),'Lid held FK source not final camera sample')
  need(c['held_RGB_observation']==final_lid and c['truth_used_for_command_targets_or_residual'] is False and c['foot_geometry_mesh_sha256']==r['mesh_sha256'],'Lid command RGB source mismatch')
  startR,starttip=independent_fk(c['last_issued_q_rad'],[.293,0,0],'float64');need(np.allclose(startR,c['start_rotation_world'],atol=1e-9,rtol=0) and np.allclose(starttip,c['start_tip_robot_m'],atol=1e-10,rtol=0),'Lid command start not last issued FK')
  goal=np.array(lidplan['tip_world_m'])-[0,0,.9]+([0,0,.001] if name=='3_cover_precontact' else [0,0,0]);goalR=np.array(lidplan['tool_rotation_world']);need(np.allclose(goal,c['unchanged_target_tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(goalR,c['unchanged_target_rotation_world'],atol=1e-9,rtol=0),'Lid final command target changed');rotation_path=Slerp([0,1],Rotation.from_matrix(np.array([startR,goalR])))
  points=held_foot_points(final_lid,c['held_actual_FK_tip_world_m'],c['held_actual_FK_rotation_world'],[v for f in __import__('foot_audit').support_geometry(g) for v in f['corners_mm']]);need(np.allclose(points,c['estimated_foot_points_tool_m'],atol=1e-10,rtol=0),'Wrong held command foot geometry')
  previous=None
  for index,row in enumerate(c['records'],1):
   need(row['index']==index and (previous is None or row['next_step']==previous['next_step']+1),'Lid command gaps');previous=row
   u=index/c['commands'];fraction=u*u*(3-2*u);target=np.asarray(starttip)+(goal-np.asarray(starttip))*fraction;need(np.allclose(target,row['desired_tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(rotation_path(fraction).as_matrix(),row['desired_rotation_world'],atol=1e-9,rtol=0),'Lid continuous desired path changed')
   need(row['seed_q_rad']==(c['last_issued_q_rad'] if index==1 else c['records'][index-2]['solved_q_rad']),'Lid IK seed continuity changed');actual=full_by[row['next_step']];need(actual['issued_arm_q_rad']==row['solved_q_rad'] and actual['issued_jaw_targets_m']==row['jaw_joint_targets_m'],'Actual issued command not ledger bound')
   for boundary,step in [('actual_before',row['next_step']-1),('actual_after',row['next_step'])]:
    v=row[boundary];actual=full_by[step];need(v['step']==step and abs(v['sim_time']-actual['sim_time'])<=2e-6 and v['q_rad']==actual['actual_q_rad'][:6],'Lid command feedback not raw full ledger')
   R,t=independent_fk(row['solved_q_rad'],[.293,0,0],'float64');need(np.allclose(R,row['solved_rotation_world'],atol=1e-9,rtol=0) and np.linalg.norm(np.asarray(t)-row['solved_tip_robot_m'])*1000<1e-6,'Solved lid FK mismatch')
   residual=foot_residual_mm(c['estimated_foot_points_tool_m'],row['desired_tip_robot_m'],row['desired_rotation_world'],row['solved_tip_robot_m'],row['solved_rotation_world']);need(row['issued'] and row['physics_advanced'] and row['prospective_geometry_eligible'] and row['ik_ok'] and row['residual_limit_mm']==.002 and row['ik_position_tol_m']==1e-6 and float(residual.max())<=.002 and np.allclose(residual,row['per_corner_residual_mm'],atol=1e-8,rtol=0),'Invalid actual lid command')
 return {'lid_continuous_commands_verified':360,'lid_RGB_target_and_command_FK_verified':True}

def check_full_commands(r,g,design,source,evidence_directory):
 return dict(check_base_commands(r,g,design,source,evidence_directory),**check_lid_commands(r,g,design))
