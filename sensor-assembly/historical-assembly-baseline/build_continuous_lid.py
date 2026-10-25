"""Build V50 command-continuous child; no simulation/job operations."""
import json
from pathlib import Path
p=Path(__file__).parent;s=(p/'incoming_lid_precontact_trial.py').read_text();patches=[]
def patch(a,b):
 global s
 assert s.count(a)==1,(a[:110],s.count(a));s=s.replace(a,b);patches.append([a,b])
patch('from passive_velocity import PassiveVelocity','from passive_velocity import PassiveVelocity\nfrom continuous_lid_command import held_foot_points,make_command_record,IK_POSITION_TOL_M,FOOT_COMMAND_RESIDUAL_MM')
patch("    report['diagnostic_trace']=[]","    report['diagnostic_trace']=[]\n    report['continuous_lid_command_helper_sha256']=digest(Path(__file__).with_name('continuous_lid_command.py'))")
patch("        def move(label,newgoal,newjaw,duration,allow_table=False,new_support=False,new_spread=None,new_orientation=None):", """        def observed_command_fk():
            actual_q=arm.get_joint_positions()[ai].copy();actual_R,actual_tip=fk.tool_pose(actual_q,tool)
            return {'step':state['step'],'sim_time':float(world.current_time),'joint_dtype':str(actual_q.dtype),'q_rad':actual_q.tolist(),'tip_robot_m':np.asarray(actual_tip).tolist(),'rotation_world':actual_R.tolist()}
        def move(label,newgoal,newjaw,duration,allow_table=False,new_support=False,new_spread=None,new_orientation=None):""")
patch("            phase_start=float(world.current_time);command_times=[]", """            phase_start=float(world.current_time);command_times=[]
            if label=='3_cover_precontact':
                command_start_q=q.copy();command_start_R,command_start_tip=fk.tool_pose(command_start_q,tool)
                actual_q=arm.get_joint_positions()[ai].copy();actual_start_R,actual_start_tip=fk.tool_pose(actual_q,tool)
                start=np.asarray(command_start_tip).copy()
                reference=state['lid_fk_reference']
                command_foot_points=held_foot_points(reference['estimate'],reference['tip_world_m'],reference['tool_rotation'],incoming_corners)
                report['continuous_lid_commands']={'phase':label,'start_step':state['step'],'start_sim_time':phase_start,'duration_s':4.,'commands':240,'local_ik_position_tol_m':IK_POSITION_TOL_M,'foot_command_residual_limit_mm':FOOT_COMMAND_RESIDUAL_MM,'start_source':'FK of last issued arm command q; never object truth','last_issued_q_rad':command_start_q.tolist(),'start_tip_robot_m':start.tolist(),'start_rotation_world':command_start_R.tolist(),'actual_start_q_rad':actual_q.tolist(),'actual_start_joint_dtype':str(actual_q.dtype),'actual_start_tip_robot_m':np.asarray(actual_start_tip).tolist(),'actual_start_rotation_world':actual_start_R.tolist(),'tool_offset_m':list(tool),'unchanged_target_tip_robot_m':np.asarray(newgoal).tolist(),'unchanged_target_rotation_world':np.asarray(new_orientation).tolist(),'held_RGB_observation':reference['estimate'],'held_actual_FK_tip_world_m':reference['tip_world_m'],'held_actual_FK_rotation_world':reference['tool_rotation'],'estimated_foot_points_tool_m':command_foot_points.tolist(),'foot_geometry_mesh_sha256':report['mesh_sha256'],'truth_used_for_command_targets_or_residual':False,'records':[]}
                persist()""")
patch("                start_R,_=fk.tool_pose(arm.get_joint_positions()[ai],tool)", "                start_R,_=fk.tool_pose(command_start_q if label=='3_cover_precontact' else arm.get_joint_positions()[ai],tool)")
patch("                q,ok=ik.solve(q,target,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)\n                if not ok:raise RuntimeError('IK failed at '+label)", """                if label=='3_cover_precontact':
                    record={'index':len(report['continuous_lid_commands']['records'])+1,'next_step':state['step']+1,'sim_time_before_issue':float(world.current_time),'seed_q_rad':q.tolist(),'desired_tip_robot_m':target.tolist(),'desired_rotation_world':desired_R.tolist(),'solved_q_rad':None,'solved_tip_robot_m':None,'solved_rotation_world':None,'jaw_joint_targets_m':dict(cell.jaw_targets(j)),'status':'SOLVING','issued':False,'actual_before':observed_command_fk()}
                    record['actual_after']=dict(record['actual_before']);record['physics_advanced']=False
                    report['continuous_lid_commands']['records'].append(record)
                    proposed_q,ok=ik.solve(q,target,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500,tol=IK_POSITION_TOL_M)
                    solved_R,solved_tip=fk.tool_pose(proposed_q,tool)
                    record.update(make_command_record(record['index'],state['step']+1,float(world.current_time),target,desired_R,proposed_q,solved_tip,solved_R,cell.jaw_targets(j),command_foot_points,ok));record['status']='SOLVED'
                    if not record['prospective_geometry_eligible']:
                        record['status']='REJECTED_BEFORE_ISSUE';persist();raise RuntimeError('Lid prospective command SE3 foot-residual veto before issue')
                    q=proposed_q
                else:q,ok=ik.solve(q,target,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
                if not ok:raise RuntimeError('IK failed at '+label)""")
patch("                command_times.append(float(world.current_time));command(j);advance(1)", """                command_times.append(float(world.current_time));command(j)
                if label=='3_cover_precontact':record['issued']=True;record['status']='ISSUED'
                if label=='3_cover_precontact':
                    try:advance(1)
                    finally:record['actual_after']=observed_command_fk();record['physics_advanced']=record['actual_after']['sim_time']>record['actual_before']['sim_time']
                else:advance(1)""")
patch("                path_length=float(np.linalg.norm(precontact-goal))", "                command_start_tip=fk.tool_pose(q,tool)[1]\n                path_length=float(np.linalg.norm(precontact-command_start_tip))")
patch("'commanded_start_robot_m':goal.tolist()", "'commanded_start_robot_m':np.asarray(command_start_tip).tolist()")
(p/'continuous_lid_precontact_trial.py').write_text(s);(p/'continuous-runtime-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
s=(p/'check_incoming_lid_precontact.py').read_text();patches=[]
patch("[('incoming_lid_precontact_trial.py','trial_script_sha256')", "[('continuous_lid_precontact_trial.py','trial_script_sha256')")
patch('def main():', '''def validate_continuous_commands(r,g):
    import numpy as np
    from scipy.spatial.transform import Rotation,Slerp
    from continuous_lid_command import independent_fk,held_foot_points,make_command_record,ORIGINAL_FK_SHA256,ORIGINAL_IK_SHA256
    from foot_audit import support_geometry
    a=r['continuous_lid_commands'];assert a['phase']=='3_cover_precontact' and a['duration_s']==4. and a['commands']==240
    assert a['local_ik_position_tol_m']==1e-6 and a['foot_command_residual_limit_mm']==.002==r['lid_pose_foot_budget']['hold_drift_position_error_mm']/10
    assert r['starter_hashes']['fk.py']==ORIGINAL_FK_SHA256 and r['starter_hashes']['ik.py']==ORIGINAL_IK_SHA256
    assert a['truth_used_for_command_targets_or_residual'] is False and a['foot_geometry_mesh_sha256']==r['mesh_sha256']
    R0,p0=independent_fk(a['last_issued_q_rad'],a['tool_offset_m']);Ra,pa=independent_fk(a['actual_start_q_rad'],a['tool_offset_m'],a['actual_start_joint_dtype'])
    assert np.allclose(p0,a['start_tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(R0,a['start_rotation_world'],atol=1e-10,rtol=0)
    assert np.allclose(pa,a['actual_start_tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(Ra,a['actual_start_rotation_world'],atol=1e-10,rtol=0)
    assert np.allclose(r['slow_precontact_profile']['commanded_start_robot_m'],p0,atol=1e-10,rtol=0)
    assert a['last_issued_q_rad']==r['incoming_lid_stability']['expected_arm_joints']
    assert a['start_step']==r['incoming_lid_stability']['reference_freeze_step'] and a['start_sim_time']==r['incoming_lid_stability']['reference_freeze_sim_time']
    observation=r['lid_pose_observations'][-1];assert a['held_RGB_observation']==observation and observation['label']=='3_cover_leveled_inspection_b'
    assert np.allclose(a['held_actual_FK_tip_world_m'],pa+[0,0,.9],atol=1e-10,rtol=0) and np.allclose(a['held_actual_FK_rotation_world'],Ra,atol=1e-10,rtol=0)
    corners=[v for foot in support_geometry(g) for v in foot['corners_mm']];points=held_foot_points(observation,a['held_actual_FK_tip_world_m'],a['held_actual_FK_rotation_world'],corners)
    assert np.allclose(points,a['estimated_foot_points_tool_m'],atol=1e-10,rtol=0)
    end=np.array(r['lid_measured_placement_plan']['tip_world_m'])+[0,0,.001-.9];endR=np.array(r['lid_measured_placement_plan']['tool_rotation_world'])
    assert np.allclose(a['unchanged_target_tip_robot_m'],end,atol=1e-10,rtol=0) and np.allclose(a['unchanged_target_rotation_world'],endR,atol=1e-10,rtol=0)
    path=Slerp([0,1],Rotation.from_matrix([R0,endR]));assert len(a['records'])==240
    traces={x['step']:x for x in r['diagnostic_trace']}
    for i,x in enumerate(a['records'],1):
        u=i/240;s=u*u*(3-2*u);target=p0+(end-p0)*s;desiredR=path(s).as_matrix();solvedR,solvedtip=independent_fk(x['solved_q_rad'],a['tool_offset_m'])
        assert x['index']==i and x['next_step']==a['start_step']+i and abs(x['sim_time_before_issue']-(a['start_sim_time']+(i-1)/60))<=2e-6
        assert np.allclose(target,x['desired_tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(desiredR,x['desired_rotation_world'],atol=1e-10,rtol=0)
        assert np.allclose(solvedtip,x['solved_tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(solvedR,x['solved_rotation_world'],atol=1e-10,rtol=0)
        for boundary,expected_step,expected_time in [('actual_before',x['next_step']-1,x['sim_time_before_issue']),('actual_after',x['next_step'],x['sim_time_before_issue']+1/60)]:
            feedback=x[boundary];feedbackR,feedbackp=independent_fk(feedback['q_rad'],a['tool_offset_m'],feedback['joint_dtype'])
            assert feedback['step']==expected_step and abs(feedback['sim_time']-expected_time)<=2e-6
            assert np.allclose(feedbackp,feedback['tip_robot_m'],atol=1e-10,rtol=0) and np.allclose(feedbackR,feedback['rotation_world'],atol=1e-10,rtol=0)
        trace=traces[x['next_step']];assert trace['commanded_arm_joints']==x['solved_q_rad'] and trace['issued_jaw_joint_targets_m']==x['jaw_joint_targets_m'] and trace['joint_positions'][:6]==x['actual_before']['q_rad'] and trace['sim_time']==x['sim_time_before_issue']
        assert x['seed_q_rad']==(a['last_issued_q_rad'] if i==1 else a['records'][i-2]['solved_q_rad'])
        expected=make_command_record(i,x['next_step'],x['sim_time_before_issue'],target,desiredR,x['solved_q_rad'],solvedtip,solvedR,x['jaw_joint_targets_m'],points,x['ik_ok'])
        assert x['jaw_joint_targets_m']==r['incoming_lid_stability']['expected_jaw_joint_targets_m']
        assert x['issued'] is True and x['ik_ok'] is True and x['prospective_geometry_eligible'] is True
        for key in ('per_corner_residual_mm','maximum_foot_residual_mm'):assert np.allclose(x[key],expected[key],atol=1e-8,rtol=0)
        assert x['residual_limit_mm']==.002 and x['ik_position_tol_m']==1e-6 and expected['prospective_geometry_eligible']


def main():''')
patch("    validate_incoming_stability(r,g)", "    assert r['continuous_lid_command_helper_sha256']==digest(source/'continuous_lid_command.py')\n    validate_continuous_commands(r,g)\n    validate_incoming_stability(r,g)")
(p/'check_continuous_lid_precontact.py').write_text(s);(p/'continuous-checker-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
