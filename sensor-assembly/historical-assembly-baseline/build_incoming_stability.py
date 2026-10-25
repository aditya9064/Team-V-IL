"""Derive fixed-hold readiness-veto candidate; no simulation/job calls."""
import json
from pathlib import Path
p=Path(__file__).parent;s=(p/'slow_lid_precontact_trial.py').read_text();patches=[]
def patch(a,b):
 global s
 assert s.count(a)==1,(a[:100],s.count(a));s=s.replace(a,b);patches.append([a,b])
patch('from passive_velocity import PassiveVelocity','from passive_velocity import PassiveVelocity\nfrom incoming_lid_stability import make_sample as incoming_sample,qualify as qualify_incoming')
patch("    report['diagnostic_trace']=[]","    report['diagnostic_trace']=[]\n    report['incoming_lid_stability_helper_sha256']=digest(Path(__file__).with_name('incoming_lid_stability.py'))")
patch("                if state.get('lid_clearance_ready'):","""                if state.get('incoming_lid_capture'):
                    live_R,live_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                    actual_cover=poses()['cover']
                    sample=incoming_sample(state['step'],float(world.current_time),actual_cover['position_world_m'],mass_rotation(actual_cover['orientation_wxyz']),(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),live_R.tolist(),incoming_corners,q.tolist(),issued_jaw_targets)
                    report['incoming_lid_stability']['samples'].append(sample)
                if state.get('lid_clearance_ready'):""")
patch("        def measure_lid(label):", """        incoming_corners=[v for foot in __import__('foot_audit').support_geometry(geometry) for v in foot['corners_mm']]
        def qualify_incoming_hold(boundary):
            audit=report['incoming_lid_stability']
            result=qualify_incoming(audit['samples'],audit['expected_arm_joints'],audit['expected_jaw_joint_targets_m'],report['lid_pose_foot_budget']['hold_drift_position_error_mm'],incoming_corners)
            audit[boundary]=result;persist()
            if not result['pass']:raise RuntimeError('incoming_lid_grasp_unstable_before_measurement')
        def fixed_incoming_hold():
            if state.get('lid_drift_reference') is not None:raise RuntimeError('Incoming hold cannot reset an existing grasp reference')
            audit={'scope':'Ideal-simulation incoming readiness veto only; no physical sensor claim','source':'actual CAD foot corners transformed through independent simulation pose and measured FK; truth only veto, never targets or hold duration','expected_arm_joints':q.tolist(),'expected_jaw_joint_targets_m':dict(issued_jaw_targets),'start_step':state['step'],'start_sim_time':float(world.current_time),'requested_duration_s':2.,'requested_commands':120,'fixed_window_start_index':95,'fixed_window_samples':25,'adaptive_wait':False,'retry':False,'truth_used_for_targets':False,'samples':[]}
            report['incoming_lid_stability']=audit;state['incoming_lid_capture']=True
            for _ in range(120):command(jaw);advance(1)
            audit['end_step']=state['step'];audit['end_sim_time']=float(world.current_time);audit['actual_duration_s']=audit['end_sim_time']-audit['start_sim_time'];audit['issued_commands']=120
            if abs(audit['actual_duration_s']-2.)>2e-6:raise RuntimeError('Incoming fixed hold timing mismatch')
            qualify_incoming_hold('fixed_hold_result')
        def measure_lid(label):""")
patch("            state['lid_drift_reference']=({'actual_pose':actual", """            if label=='3_cover_leveled_inspection':
                if report['incoming_lid_stability'].get('reference_freeze_count',0)!=0:raise RuntimeError('Final lid grasp reference already frozen')
                qualify_incoming_hold('final_measurement_result')
                state['incoming_lid_capture']=False
                report['incoming_lid_stability']['reference_freeze_step']=state['step']
                report['incoming_lid_stability']['reference_freeze_sim_time']=float(world.current_time)
                report['incoming_lid_stability']['reference_freeze_count']=1
            state['lid_drift_reference']=({'actual_pose':actual""")
patch("                observed=measure_lid('3_cover_leveled_inspection')", "                fixed_incoming_hold()\n                observed=measure_lid('3_cover_leveled_inspection')")
(p/'incoming_lid_precontact_trial.py').write_text(s);(p/'incoming-runtime-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
s=(p/'check_slow_lid_precontact.py').read_text();patches=[]
patch("[('slow_lid_precontact_trial.py','trial_script_sha256')", "[('incoming_lid_precontact_trial.py','trial_script_sha256')")
patch('def main():', '''def validate_incoming_stability(r,g):
    from incoming_lid_stability import qualify
    from foot_audit import support_geometry
    a=r['incoming_lid_stability'];corners=[v for foot in support_geometry(g) for v in foot['corners_mm']]
    assert a['requested_duration_s']==2. and a['requested_commands']==a['issued_commands']==120
    assert a['end_step']-a['start_step']==120 and abs(a['end_sim_time']-a['start_sim_time']-2.)<=2e-6 and abs(a['actual_duration_s']-2.)<=2e-6
    assert a['fixed_window_start_index']==95 and a['fixed_window_samples']==25
    assert a['adaptive_wait'] is False and a['retry'] is False and a['truth_used_for_targets'] is False
    samples=a['samples'];assert len(samples)>120 and samples[0]['step']==a['start_step']+1 and samples[119]['step']==a['end_step']
    assert abs(samples[0]['sim_time']-a['start_sim_time']-1/60)<=2e-6 and samples[119]['sim_time']==a['end_sim_time']
    assert a['reference_freeze_count']==1 and a['reference_freeze_step']==samples[-1]['step'] and a['reference_freeze_sim_time']==samples[-1]['sim_time']
    shots={x['phase']:x for x in r['checkpoints']};level=shots['3_cover_level_held'];photoA=shots['3_cover_leveled_inspection_a'];photoB=shots['3_cover_leveled_inspection_b']
    assert a['start_step']==level['step'] and a['start_sim_time']==level['sim_time'] and a['end_step']<photoA['step']<photoB['step']==a['reference_freeze_step']
    assert photoB['sim_time']==a['reference_freeze_sim_time']
    for shot in (photoA,photoB):
        assert len([x for x in samples if x['step']==shot['step'] and x['sim_time']==shot['sim_time']])==1
    budget=r['lid_pose_foot_budget']['hold_drift_position_error_mm'];assert budget==.02
    for key,trace in [('fixed_hold_result',samples[:120]),('final_measurement_result',samples)]:
        expected=qualify(trace,a['expected_arm_joints'],a['expected_jaw_joint_targets_m'],budget,corners)
        assert a[key]==expected and expected['pass'] and expected['fixed_window_start_index']==95


def main():''')
patch("    validate_slow_precontact(r)", "    assert r['incoming_lid_stability_helper_sha256']==digest(source/'incoming_lid_stability.py')\n    validate_incoming_stability(r,g)\n    validate_slow_precontact(r)")
(p/'check_incoming_lid_precontact.py').write_text(s);(p/'incoming-checker-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
