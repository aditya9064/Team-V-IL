"""Portable pure-IK replay from actualV49 inputs. No simulator/jobs/robot API."""
from pathlib import Path
import json,hashlib,importlib.util,sys
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from continuous_lid_command import held_foot_points,make_command_record
from foot_audit import support_geometry
P=Path(__file__).parent

def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def replay(mode='continuous',count=240):
 inputs=json.loads((P/'v49-command-replay-inputs.json').read_text());g=json.loads((P/'nest-mesh.json').read_text())
 for name,file in [('fk.py','original_fk.py'),('ik.py','original_ik.py')]:assert hashlib.sha256((P/'fixtures'/file).read_bytes()).hexdigest()==inputs['source_pins'][name]['sha256']
 fk=module('_replay_fk',P/'fixtures/original_fk.py');old=sys.modules.get('fk');sys.modules['fk']=fk
 try:ik=module('_replay_ik',P/'fixtures/original_ik.py')
 finally:
  if old is None:sys.modules.pop('fk',None)
  else:sys.modules['fk']=old
 ik.set_tool_axis(inputs['tool_axis_local']);ik.set_spread_axis(inputs['finger_spread_axis_local'])
 q=np.array(inputs['last_issued_q_rad']);startR,start=fk.tool_pose(q,inputs['tool_offset_m']);actualR,actual=fk.tool_pose(np.array(inputs['final_RGB_actual_q_rad'],dtype=np.float32),inputs['tool_offset_m']);end=np.array(inputs['unchanged_final_target_robot_m']);endR=np.array(inputs['unchanged_final_R_world']);tol=1e-6
 if mode=='stale_position':start=np.array(inputs['remembered_goal_robot_m'])
 if mode=='actual_R_start':startR=actualR
 if mode=='default_IK_deadband':tol=1e-4
 path=Slerp([0,1],Rotation.from_matrix([startR,endR]));corners=[v for f in support_geometry(g) for v in f['corners_mm']];points=held_foot_points(inputs['final_RGB_observation'],actual+[0,0,.9],actualR,corners)
 a={'phase':'3_cover_precontact','start_step':inputs['reference_freeze_step'],'start_sim_time':inputs['reference_freeze_sim_time'],'duration_s':4.,'commands':240,'local_ik_position_tol_m':tol,'foot_command_residual_limit_mm':.002,'last_issued_q_rad':inputs['last_issued_q_rad'],'start_tip_robot_m':start.tolist(),'start_rotation_world':startR.tolist(),'actual_start_q_rad':inputs['final_RGB_actual_q_rad'],'actual_start_joint_dtype':'float32','actual_start_tip_robot_m':actual.tolist(),'actual_start_rotation_world':actualR.tolist(),'tool_offset_m':inputs['tool_offset_m'],'unchanged_target_tip_robot_m':end.tolist(),'unchanged_target_rotation_world':endR.tolist(),'held_RGB_observation':inputs['final_RGB_observation'],'held_actual_FK_tip_world_m':(actual+[0,0,.9]).tolist(),'held_actual_FK_rotation_world':actualR.tolist(),'estimated_foot_points_tool_m':points.tolist(),'foot_geometry_mesh_sha256':inputs['mesh_sha256'],'truth_used_for_command_targets_or_residual':False,'records':[]}
 previous_q=q.copy();traces=[]
 for i in range(1,count+1):
  u=i/240;s=u*u*(3-2*u);target=start+(end-start)*s;desiredR=path(s).as_matrix();q,ok=ik.solve(q,target,desiredR@inputs['tool_axis_local'],np.array(inputs['joint_limits_rad']),iters=500,tol=tol,tool_offset=inputs['tool_offset_m'],spread_dir=desiredR@inputs['finger_spread_axis_local']);solvedR,solved=fk.tool_pose(q,inputs['tool_offset_m']);record=make_command_record(i,a['start_step']+i,a['start_sim_time']+(i-1)/60,target,desiredR,q,solved,solvedR,inputs['expected_jaw_joint_targets_m'],points,ok);record['issued']=True;record['status']='ISSUED';record['seed_q_rad']=previous_q.tolist();record['physics_advanced']=True;record['actual_before']={'step':a['start_step']+i-1,'sim_time':record['sim_time_before_issue'],'joint_dtype':'float32','q_rad':inputs['final_RGB_actual_q_rad'],'tip_robot_m':actual.tolist(),'rotation_world':actualR.tolist()};record['actual_after']=dict(record['actual_before'],step=a['start_step']+i,sim_time=record['sim_time_before_issue']+1/60);traces.append({'step':record['next_step'],'sim_time':record['sim_time_before_issue'],'commanded_arm_joints':q.tolist(),'issued_jaw_joint_targets_m':inputs['expected_jaw_joint_targets_m'],'joint_positions':inputs['final_RGB_actual_q_rad']});a['records'].append(record);previous_q=q.copy()
 r={'fixture_scope':'PureIK replay; actualV49 initialfloat32feedback repeated synthetically forcheckerunit tests, no simulation acceptance','diagnostic_trace':traces,'continuous_lid_commands':a,'lid_pose_foot_budget':{'hold_drift_position_error_mm':.02},'starter_hashes':{'fk.py':inputs['source_pins']['fk.py']['sha256'],'ik.py':inputs['source_pins']['ik.py']['sha256']},'mesh_sha256':inputs['mesh_sha256'],'slow_precontact_profile':{'commanded_start_robot_m':start.tolist()},'incoming_lid_stability':{'expected_arm_joints':inputs['last_issued_q_rad'],'expected_jaw_joint_targets_m':inputs['expected_jaw_joint_targets_m'],'reference_freeze_step':inputs['reference_freeze_step'],'reference_freeze_sim_time':inputs['reference_freeze_sim_time']},'lid_pose_observations':[inputs['final_RGB_observation']],'lid_measured_placement_plan':inputs['lid_measured_placement_plan']}
 return r,g
if __name__=='__main__':
 r,g=replay();a=r['continuous_lid_commands'];summary={'commands':len(a['records']),'max_foot_residual_mm':max(x['maximum_foot_residual_mm'] for x in a['records']),'all_prospective_checks_pass':all(x['prospective_geometry_eligible'] for x in a['records']),'start_q_rad':a['last_issued_q_rad'],'first_command':a['records'][0],'last_command':a['records'][-1]};(P/'offline-continuous-replay.json').write_text(json.dumps({'summary':summary,'report_fixture':r},indent=2)+'\n');print(json.dumps(summary,indent=2))
