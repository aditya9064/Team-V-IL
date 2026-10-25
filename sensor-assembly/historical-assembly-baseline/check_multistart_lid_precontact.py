"""Independent success checker. Failure/Slurm zero never imply a pass."""
import argparse,hashlib,json,math
from pathlib import Path
from assembly_checks import contact_violation,grasp_command_m
from payload_checks import payload_score
from lid_diagnostic_contract import expected_phases,expected_checkpoints,validate_foot_budget,precontact_budget


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def validate_passive_velocity(value):
    assert value['available'] is True or value['available'] is False
    assert value['source']=='finite_difference_existing_USD_pose_snapshots'
    assert value['linear_reference']=='body origin, not center of mass'
    assert value['angular_reference']=='world log(Rnow Rprev.T)/dt'
    if value['available']:
        assert isinstance(value['interval_s'],(int,float)) and not isinstance(value['interval_s'],bool) and math.isfinite(value['interval_s']) and value['interval_s']>0
        for name in ('linear_world_m_s','angular_world_rad_s'):
            vector=value[name];assert isinstance(vector,list) and len(vector)==3
            assert all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in vector)
    else:
        assert value['linear_world_m_s'] is None and value['angular_world_rad_s'] is None
        assert (value.get('reason')=='first_sample' and value['interval_s'] is None) or (value.get('reason')=='same_simulation_time' and value['interval_s']==0)


ORIGINAL_PHASE_DURATIONS={'1_base_orient':1.,'1_base_above':2.,'1_base_approach':2.,'1_base_side_entry':1.5,'1_base_close':3.,'1_base_lift':1.5,'1_base_raise_for_transfer':1.5,'1_base_carry':2.,'1_base_level_held':2.,'1_base_lower':2.,'1_base_relax_clamp':2.,'1_base_square_fingers':2.,'1_base_release':1.,'1_base_side_exit':1.5,'1_base_retreat':2.,'2_carrier_orient':1.,'2_carrier_above':2.,'2_carrier_approach':2.,'2_carrier_side_entry':1.5,'2_carrier_close':3.,'2_carrier_lift':1.5,'2_carrier_raise_for_transfer':1.5,'2_carrier_carry':4.,'2_carrier_lower':2.,'2_carrier_release':1.,'2_carrier_side_exit':1.5,'2_carrier_retreat':2.,'3_cover_orient':1.,'3_cover_above':2.,'3_cover_approach':2.,'3_cover_side_entry':1.5,'3_cover_close':3.,'3_cover_lift':1.5,'3_cover_raise_for_transfer':1.5,'3_cover_carry':2.,'3_cover_level_held':2.,'3_cover_precontact':2.}

def validate_slow_precontact(r):
    rows=r['motion_sampling']['phases'];assert [x['phase'] for x in rows]==list(ORIGINAL_PHASE_DURATIONS)
    for x in rows:
        duration=4. if x['phase']=='3_cover_precontact' else ORIGINAL_PHASE_DURATIONS[x['phase']]
        assert x['requested_duration_s']==duration and x['commands']==round(duration*60)
    v=r['slow_precontact_profile'];assert v['phase']=='3_cover_precontact' and v['law']=='s(u)=3u^2-2u^3; original motion_samples'
    a=v['commanded_start_robot_m'];b=v['commanded_end_robot_m'];assert len(a)==len(b)==3
    assert all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in a+b)
    distance=math.sqrt(sum((y-x)**2 for x,y in zip(a,b)));assert distance>0
    expected={'path_length_m':distance,'before_duration_s':2.,'duration_s':4.,'duration_ratio':2.,'before_peak_speed_m_s':1.5*distance/2.,'peak_speed_m_s':1.5*distance/4.,'speed_ratio':.5,'before_peak_acceleration_m_s2':6*distance/4.,'peak_acceleration_m_s2':6*distance/16.,'acceleration_ratio':.25,'before_interior_jerk_m_s3':12*distance/8.,'interior_jerk_m_s3':12*distance/64.,'interior_jerk_ratio':.125}
    for k,value in expected.items():assert isinstance(v[k],(int,float)) and not isinstance(v[k],bool) and math.isfinite(v[k]) and math.isclose(v[k],value,rel_tol=1e-10,abs_tol=1e-12),k
    for k in ('endpoint_acceleration_discontinuity_retained','orientation_endpoints_unchanged','grasp_reference_unchanged','thresholds_unchanged','bounds_are_commanded_analytic_not_measured'):assert v[k] is True


def validate_incoming_stability(r,g):
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


def validate_continuous_commands(r,g):
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


def validate_multistart_observation(o,evidence,vertices,design):
    import cv2,numpy as np
    from scipy.spatial.transform import Rotation
    import held_pose_bounded as original
    bank=[[0.]*6]
    for magnitude in (.5,1.):
        for axis in range(6):
            for sign in (-1,1):
                seed=[0.]*6;seed[axis]=sign*magnitude;bank.append(seed)
    a=o['multistart'];assert a['protocol_sha256']=='b22cb0055a6f462454a0933086f3c33513de573cb5db7f918cf7755e4fbf85c1'
    assert a['fixed_seed_parameters_mm_xyz_deg']==bank and a['candidate_count']==len(a['candidates'])==25
    assert a['original_zero_seed_gate_required'] is True and a['original_zero_seed_gate_pass'] is True
    assert a['truth_inputs_or_selection'] is False and a['early_stop'] is False and a['adaptive_seed_bank'] is False
    options={'bounds':[[-2,-2,-2,-3,-3,-3],[2,2,2,3,3,3]],'diff_step':.01,'x_scale':[1,1,1,1,1,1],'max_nfev':400,'ftol':1e-8,'xtol':1e-8,'gtol':1e-8}
    assert json.loads(json.dumps(a['solver_options']))==options
    models=o['camera_models'];images={k:cv2.cvtColor(cv2.imread(str(Path(evidence)/(o['label']+'-'+k+'.png'))),cv2.COLOR_BGR2RGB) for k in models}
    assert o['camera_model_sha256']=={k:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest() for k,v in models.items()}
    assert o['CAD_vertices_mm_sha256']==hashlib.sha256(np.ascontiguousarray(vertices,dtype=np.float64).tobytes()).hexdigest()
    assert o['image_RGB_sha256']=={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in images.items()}
    nominal=np.array(o['tip_world_m'])-np.array([*np.array(design['base']['handle_centre_xy'])*.001,design['base']['tcp_above_bottom_mm']*.001])
    assert np.allclose(nominal,o['nominal_position_m'],atol=1e-12,rtol=0)
    captured={}
    class Captured(Exception):pass
    def capture(fun,x0,**kwargs):captured.update(fun=fun,kwargs=kwargs);raise Captured
    previous=original.least_squares;original.least_squares=capture
    try:
        try:original.estimate(images,models,vertices,nominal)
        except Captured:pass
    finally:original.least_squares=previous
    assert json.loads(json.dumps(captured['kwargs']))==options
    fun=captured['fun'];closure=dict(zip(fun.__code__.co_freevars,[cell.cell_contents for cell in fun.__closure__]));views=closure['views'];eligible=[]
    for i,c in enumerate(a['candidates']):
        assert c['seed_index']==i and c['seed_parameters_mm_xyz_deg']==bank[i]
        if c['offset_mm_and_xyz_deg'] is None:
            assert c['image_eligible'] is False and c['cost'] is None and c['optimizer'] is None and len(c['rejection_reasons'])==1 and c['rejection_reasons'][0].startswith('invalid_solver_or_output:');continue
        x=np.array(c['offset_mm_and_xyz_deg']);assert x.shape==(6,) and np.isfinite(x).all();errors=fun(x);assert errors.shape==(280,) and np.isfinite(errors).all()
        opt=c['optimizer'];assert math.isfinite(c['cost']) and c['cost']>=0 and math.isclose(c['cost'],float(errors@errors/2),rel_tol=1e-8,abs_tol=1e-8) and opt['cost']==c['cost'] and opt['max_nfev']==400 and math.isfinite(opt['optimality']) and opt['optimality']>=0
        assert isinstance(opt['success'],bool) and isinstance(opt['status'],int) and not isinstance(opt['status'],bool) and isinstance(opt['message'],str) and isinstance(opt['nfev'],int) and not isinstance(opt['nfev'],bool) and isinstance(opt['njev'],int) and not isinstance(opt['njev'],bool) and 0<opt['nfev']<=400 and 0<opt['njev']<=400
        assert np.allclose(c['position_world_m'],nominal+x[:3]*.001,atol=1e-12,rtol=0) and np.allclose(c['rotation_world'],Rotation.from_euler('xyz',x[3:],degrees=True).as_matrix(),atol=1e-12,rtol=0)
        reasons=[]
        if not opt['success']:reasons.append('solver_not_successful')
        for j,(name,model,v,d,contour) in enumerate(views):
            e=errors[j*140:(j+1)*140];score=c['views'][name]
            expected={'edge_rms_px':float(np.sqrt(np.mean(e**2))),'edge_p95_px':float(np.percentile(e,95)),'edge_coverage_within_2px':float(np.mean(e<=2)),'observed_contour_points':len(contour)}
            for key,value in expected.items():assert math.isclose(score[key],value,rel_tol=1e-8,abs_tol=1e-8),key
            if expected['edge_p95_px']>2:reasons.append(name+':p95_exceeds_2px')
            if expected['edge_coverage_within_2px']<.95:reasons.append(name+':coverage_below_.95')
        if np.any(abs(x[:3])>=1.95):reasons.append('translation_near_original_bound')
        if np.any(abs(x[3:])>=2.95):reasons.append('rotation_near_original_bound')
        assert c['rejection_reasons']==reasons and c['image_eligible'] is (not reasons)
        assert math.isclose(c['rms_px'],float(np.sqrt(np.mean(errors**2))),rel_tol=1e-8,abs_tol=1e-8)
        if not reasons:eligible.append(c)
    assert a['candidates'][0]['image_eligible'] is True
    assert eligible;selected=min(eligible,key=lambda c:(c['cost'],c['seed_index']));assert a['selected_seed_index_IMAGE_ONLY']==selected['seed_index']
    for key in ('position_world_m','rotation_world','offset_mm_and_xyz_deg','views','rms_px','optimizer'):assert o[key]==selected[key]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('report');a=ap.parse_args();r=json.loads(Path(a.report).read_text());source=Path(__file__).parent;g=json.loads((source/'nest-mesh.json').read_text());l=r['limits']
    stage=r['diagnostic_stage'];assert stage=='precontact','Final seating/release remains BLOCKED'
    assert r['status']==('SIMULATION_PASS' if stage=='release' else 'DIAGNOSTIC_PASS'),r.get('error')
    assert r['diagnostic_stage_accepted']==stage
    assert r['expected_motion_phases']==expected_phases(stage)
    assert r['expected_checkpoint_phases']==expected_checkpoints(stage)
    assert r['held_pose_estimator_sha256']==digest(source/'held_pose_multistart.py')
    assert r['base_multistart_original_estimator_sha256']==digest(source/'held_pose_bounded.py')
    design=json.loads((source/'nest-design.json').read_text())
    for observation in r['held_pose_observations']:validate_multistart_observation(observation,Path(a.report).parent,g['parts']['base']['visual']['vertices_mm'],design)
    assert r['physical_nest']=={'contract_sha256':digest(source/'nest-proof.json'),'contact_checker_sha256':digest(source/'fixture_contacts.py'),'base_dynamic_throughout':True,'product_attachments':False,'no_clamp_or_preload':True}
    from fixture_contacts import guide_contact
    for c in r.get('nest_guide_contacts',[]):
        assert c['allowed'] and guide_contact(c['colliders'],[('nest','fixture_guide') if '/Assembly_nest/' in x else ('base','floor' if '_floor' in x else 'wall') for x in c['colliders']],{'fixture_entry_enabled':True},c['normals'])
        assert not contact_violation(True,c['minimum_separation_mm']*.001,c['maximum_impulse_Ns'],.25)
    assert r['initial_preparation']=={'product_parts_preassembled_together':False,'pcb_preloaded_in_carrier':True,'only_preplaced_fixture':(['base','nest'] if r['case']=='loaded-carrier' else ['nest']),'robot_placement_order':['base','carrier','cover']}
    sampling=r['motion_sampling'];assert sampling['command_hz']==60 and sampling['physics_steps_per_command']==1 and not sampling['path_and_phase_durations_unchanged'] and not sampling['original_phase_durations_retained'] and sampling['only_duration_change']=={'2_carrier_carry':{'before_s':2.,'after_s':4.},'3_cover_release':{'before_s':1.,'after_s':3.},'3_cover_precontact':{'before_s':2.,'after_s':4.}}
    assert [x['phase'] for x in sampling['phases']]==expected_phases(stage)
    assert r['incoming_lid_stability_helper_sha256']==digest(source/'incoming_lid_stability.py')
    assert r['continuous_lid_command_helper_sha256']==digest(source/'continuous_lid_command.py')
    validate_continuous_commands(r,g)
    validate_incoming_stability(r,g)
    validate_slow_precontact(r)
    transfer=next(p for p in sampling['phases'] if p['phase']=='2_carrier_carry');assert transfer['requested_duration_s']==4. and transfer['commands']==240
    from lid_release_schedule import lid_release_bounds
    if stage=='release':
        assert r['lid_release_profile']==lid_release_bounds(abs(r['grip_commands']['cover']['closed_grasp_command_per_jaw_m']))
        release_phase=next(p for p in sampling['phases'] if p['phase']=='3_cover_release');assert release_phase['requested_duration_s']==3. and release_phase['commands']==180
    else:assert 'lid_release_profile' not in r
    assert r['payload_transfer_schedule']['endpoint_acceleration_m_s2']==0. and r['payload_transfer_schedule']['acceleration_ratio']<.241
    for phase in sampling['phases']:
        assert phase['commands']==round(phase['requested_duration_s']*60)
        assert abs(phase['actual_duration_s']-phase['requested_duration_s'])<=2e-6
        assert abs(phase['maximum_command_interval_s']-1/60)<=2e-6
    assert r['payload_initial_gap_mm']==g['payload_initial_gap_mm']==0.0
    setup=r['contact_monitor_setup'];assert setup['api_and_subscription_before_reset'] and setup['runtime_checks_active_during_reset']
    seating=r['horizontal_seating_contacts'];assert seating['events']>0 and seating['only_horizontal'] and seating['cad_support_proof_sha256']==digest(source/'seating-support-proof.json')
    assert r['case'] in ['loaded-carrier','full-assembly'];loaded=r['case']=='loaded-carrier'
    order=['base','carrier','cover'];parts=['base','carrier','pcb','nest'] if loaded else ['base','carrier','pcb','cover','nest']
    assert r['part_order']==order and r['physical_parts']==parts and [v['part'] for v in r['part_results']]==order
    assert r['preplaced_fixture_parts']==(['base','nest'] if loaded else ['nest'])
    assert r['task']=='full_assembly_with_verified_base_held_pose_fix' and r['full_assembly_accepted'] is (stage=='release')
    assert r['vision_guided'] is True and r['physical_hardware_test'] is False and r['print_release'] is False
    for filename,key in [('multistart_lid_precontact_trial.py','trial_script_sha256'),('nest-design.json','design_sha256'),('nest-mesh.json','mesh_sha256'),('finger_cooking.py','finger_cooking_script_sha256')]:assert digest(source/filename)==r[key],filename
    assert len(r['native_primitive_audit'])==8 and all(v['full_volume_retained'] and v['geometry_identity_error_m']<=1e-7 for v in r['native_primitive_audit'])
    assert r['native_box_contact_migration']['parts_and_gripper_unchanged'] and r['native_box_contact_migration']['source_seating_gpu']==286456
    assert set(r['installed_full_body_mass_properties'])=={'base','carrier','pcb','cover'}
    for value in r['installed_full_body_mass_properties'].values():assert value['validated'] and value['physics_time_advanced_s']==0.
    assert r['physics']=={'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':1}
    timing=r['step_timing_audit'];assert timing['samples']>0 and timing['maximum_step_error_s']<=2e-6 and timing['maximum_render_advance_s']<=1e-9
    assert set(r['separate_body_audit'])==set(parts) and not r['part_attachment_joints']
    assert len({v['root'] for v in r['separate_body_audit'].values()})==len(parts)
    for name,v in r['separate_body_audit'].items():
        assert v['root']=='/World/Assembly_'+name and v['dynamic']==(name!='nest') and v['fixture_static']==(name=='nest') and not v['kinematic'] and not v['gravity_disabled']
        assert math.isclose(v['mass_kg'],g['parts'][name]['volume_mm3']*1e-9*1240,abs_tol=1e-12)
        assert r['physical_collider_counts'][name]==len(g['parts'][name]['colliders'])
    assert set(r['grip_commands'])=={'base','carrier','cover'}
    for cmd in r['grip_commands'].values():
        assert cmd['extra_travel_per_jaw_mm']==.05 and cmd['penetration_stop_limit_mm']==.25
        assert math.isclose(grasp_command_m(cmd['nominal_contact_per_jaw_mm'],.05,.25),cmd['closed_grasp_command_per_jaw_m'],abs_tol=1e-12)
    for name,u in r['grip_collision_union_audit'].items():assert u['cad_visual_unchanged'] and u['design_unchanged'] and u['after_colliders']==u['before_colliders'] and u['symmetric_difference_mm3']<=1e-4
    for v in r['collision_geometry_audit'].values():assert v['symmetric_difference_mm3']<=.05
    assert r['contact_headers_seen']>0 and r.get('payload_contact_events',0)>0 and not r['unexpected_contacts'] and not r['contact_monitor_errors']
    assert not r.get('transient_inventory_failure') and not r.get('payload_failure')
    c=r['finger_cooking'];assert c['mode']=='source_fidelity_cooking' and c['collider_count']==138 and c['source_profile_dimensions_unchanged']
    assert c['min_thickness_m']==0 and c['hull_vertex_limit']==255 and math.isfinite(c['maximum_world_source_vertex_change_m']) and c['maximum_world_source_vertex_change_m']<=1e-7
    ray=r['finger_surface_audit'];assert ray['source_surface_match'] and len(ray['probes'])==12 and ray['maximum_allowed_absolute_error_mm']==.03
    assert {(p['jaw'],p['u_m']) for p in ray['probes']}=={(j,u) for j in ['L','R'] for u in [.0445,.050,.056,.061,.068,.0733]}
    assert all(p['same_jaw_hit'] and p['inward_error_mm'] is not None and math.isfinite(p['inward_error_mm']) and abs(p['inward_error_mm'])<=.03 for p in ray['probes'])
    expected=[p for p in g['geometry_probes'] if p['part'] in parts];actual=r['cooked_geometry_probes']
    assert r['cooked_geometry_probe_pass'] and len(actual)==len(expected)
    assert {(p['part'],p['label']) for p in actual}=={(p['part'],p['label']) for p in expected}
    for p in actual:
        assert p['actual_part_hit']==p['expected_part_hit'] and p.get('distance_pass',True)
        if p['expected_part_hit']:assert p['maximum_distance_error_mm']==.03 and math.isfinite(p['actual_distance_mm']) and abs(p['actual_distance_mm']-p['expected_distance_mm'])<=.03
    assert l==g['limits'] and l['endpoint_mm']==2 and l['camera_speed_rad_s']==.02 and l['camera_samples']==6 and l['maximum_intended_penetration_mm']==.25
    assert r['payload_limits']=={'xy_mm':.75,'z_mm':.35,'relative_tilt_deg':1.,'relative_yaw_deg':1.}
    p=r['payload_motion_audit'];assert p['samples']==timing['samples'] and p['maximum_xy_mm']<=.75 and p['maximum_z_mm']<=.35 and p['maximum_relative_tilt_deg']<=1 and p['maximum_relative_yaw_deg']<=1
    for part in r['part_results']:
        assert part['pickup']['pass']
        pending_lid=part['part']=='cover' and stage!='release'
        assert (not part['placement_verified'] and not part['release_verified']) if pending_lid else (part['placement_verified'] and part['release_verified'])
        v=part['pickup'];assert v['lift_mm']>=35 and v['xy_slip_mm']<=2 and v['tilt_deg']<=5 and v['yaw_deg']<=3
        if not pending_lid:
            v=part['placement'];assert v['xy_mm']<=.75 and v['z_mm']<=.35 and v['tilt_deg']<=1 and v['yaw_deg']<=1
            assert part['release_follow_mm']<=.25
        h=r['held_motion_audits'][part['part']];assert h['samples']>0 and h['maximum_relative_slip_mm']<=2 and h['maximum_tilt_deg']<=5 and h['maximum_yaw_deg']<=3
        if not pending_lid:
            d=r['release_motion_audits'][part['part']];assert d['samples']>0 and d['maximum_drift_mm']<=.25
        if part['part']=='carrier':assert part['separate_pcb_pickup_lift_mm']>=35 and part['separate_pcb_placement']['pass']
    for shot in r['checkpoints']:
        t=shot['stillness_trace'];assert len(t)==6
        assert all(math.isfinite(v['maximum_raw_joint_speed_rad_s']) and v['maximum_raw_joint_speed_rad_s']<.02 for v in t)
        assert all(t[i+1]['step']>t[i]['step'] and t[i+1]['sim_time']>t[i]['sim_time'] and abs((t[i+1]['sim_time']-t[i]['sim_time'])-(t[i+1]['step']-t[i]['step'])/60)<2e-6 for i in range(5))
        assert payload_score(shot['poses']['carrier'],shot['poses']['pcb'])['pass']
    assert [v['phase'] for v in r['checkpoints']]==expected_checkpoints(stage)
    assert set(r['endpoint_traces'])==set(expected_phases(stage))
    assert r['designed_placement_seating_allowance_mm']=={'carrier':.10}
    assert r['lid_measured_placement_plan']['clearance_mm']==.10
    assert r['measured_placement_plan']['clearance_mm']==.10
    assert r['staged_release_plan']['relaxed_per_jaw_m']==.0023 and r['staged_release_plan']['recipe_based']
    assert [v['phase'] for v in r['motion_sampling']['phases']][10:15]==['1_base_relax_clamp','1_base_square_fingers','1_base_release','1_base_side_exit','1_base_retreat']
    assert len(r['held_pose_observations'])==4 and len(r['held_pose_repeatability'])==2 and len(r['held_pose_independent_scores'])==2
    for v in r['held_pose_repeatability']:assert v['pass'] and v['position_difference_mm']<=.1 and v['orientation_difference_deg']<=.15
    for v in r['held_pose_independent_scores']:assert v['position_error_mm']<=.15 and v['orientation_error_deg']<=.25
    for v in r['held_pose_observations']:
        assert v['optimizer']['success'] and v['optimizer']['status']>0 and 0<v['optimizer']['nfev']<=v['optimizer']['max_nfev']==400
        assert set(v['views'])=={'inspection_x','inspection_y'}
        for q in v['views'].values():assert q['edge_p95_px']<=2 and q['edge_coverage_within_2px']>=.95
    for traces in r['endpoint_traces'].values():
        assert len(traces)==6
        assert all(math.isfinite(t['tip_error_mm']) and t['tip_error_mm']<=2 and math.isfinite(t['tool_axis_error_deg']) and t['tool_axis_error_deg']<=1 and math.isfinite(t['spread_axis_error_deg']) and t['spread_axis_error_deg']<=1 for t in traces)
    assert set(r['final_part_scores'])==(set(parts) if stage=='release' else set(parts)-{'cover'}) and all(p['pass'] for p in r['final_part_scores'].values())
    assert r['lid_pose_estimator_sha256']==digest(source/'lid_pose.py')
    assert r['diagnostic_contract_sha256']==digest(source/'lid_diagnostic_contract.py')
    assert r['foot_audit_sha256']==digest(source/'foot_audit.py')
    b=validate_foot_budget(r['lid_pose_foot_budget'])
    assert len(r['lid_pose_observations'])==4 and len(r['lid_pose_repeatability'])==2 and len(r['lid_pose_independent_scores'])==2
    for v in r['lid_pose_observations']:
        assert v['optimizer']['success'] and v['optimizer']['status']>0 and len(v['optimizer']['rounds'])==3
        assert set(v['views'])=={'inspection_x','inspection_y'}
        for view in v['views'].values():assert view['edge_p95_px']<=2 and view['edge_coverage_within_2px']>=.95 and view['reverse_edge_p95_px']<=2 and view['reverse_edge_coverage_within_2px']>=.95
    for v in r['lid_pose_repeatability']:assert v['pass'] and v['position_difference_mm']<=.1 and v['orientation_difference_deg']<=.15
    for v in r['lid_pose_independent_scores']:assert v['position_error_mm']<=.15 and v['orientation_error_deg']<=.25 and v['maximum_foot_height_error_mm']<=.05
    from foot_audit import all_foot_gaps,error_budget_at_feet,qualify_prediction
    guards=r['lid_clearance_guards'];assert [v['label'] for v in guards]==(['precontact_dispatch','precontact_actual_fk_hold'] if stage=='precontact' else ['precontact_dispatch','precontact_actual_fk_hold','final_dispatch','pre_release_geometry_hold'])
    assert max(b['fit_foot_height_bounds_mm'])<=.05
    design=json.loads((source/'nest-design.json').read_text())
    for v in guards:
        assert v['base_recipe']['position_world_m'][:2]==r['layout_m']['assembly_body_centre']
        assert abs(v['base_recipe']['position_world_m'][2]-(r['checkpoints'][0]['poses']['nest']['position_world_m'][2]+design['assembly']['part_bottom_z']['base']*.001))<1e-6
        assert v['base_recipe']['rotation_world']==[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
        assert v['gaps']==all_foot_gaps(g,v['prediction'],v['base_recipe'])
        assert math.isfinite(v['fit_axis_extra_mm']) and v['fit_axis_extra_mm']>=0
        assert v['budget']==precontact_budget(g,b,fit_extra_mm=v['fit_axis_extra_mm'])
        assert v['qualification']==qualify_prediction(v['gaps'],v['budget']) and v['qualification']['geometric_dispatch_eligible']
        assert len(v['gaps']['feet'])==4 and all(len(f['corner_gaps_mm'])==4 for f in v['gaps']['feet'])
    assert r['lid_precontact_fk_tracking']['maximum_position_error_mm']<=b['maximum_fk_position_error_mm']
    drift=r['lid_grasp_corner_drift_veto'];assert drift['samples']>0 and drift['maximum_position_error_mm']<=b['hold_drift_position_error_mm'] and drift['truth_used_for_targets'] is False
    clearance=r['lid_observed_fk_clearance_guard'];assert clearance['samples']>0 and clearance['minimum_clearance_lower_bound_mm']>0 and clearance['minimum_pad_edge_lower_bound_mm']>0
    if stage!='precontact':
        hold=r['lid_pre_release_geometry_hold'];assert hold['pass'] and hold['physically_seated_claim'] is False and hold['placement_score']['pass'] and hold['clearance_intent_mm']==.10 and hold['no_forced_insertion']
        assert r['lid_actual_fk_foot_height_error_mm']<=b['maximum_fk_foot_height_error_mm']
    assert r['body_velocity_telemetry']=={'source':'finite_difference_existing_USD_pose_snapshots','helper_sha256':digest(source/'passive_velocity.py'),'linear_reference':'body origin, not center of mass','angular_reference':'world log(Rnow Rprev.T)/dt','instantaneous_velocity_claim':False,'used_for_targets_or_motion_gates':False,'physical_scoring_input':False}
    for shot in r['checkpoints']:
        assert set(shot['body_velocities'])=={'base','carrier','pcb','cover'}
        for body in shot['body_velocities'].values():validate_passive_velocity(body)
    assert r['diagnostic_trace']
    for v in r['diagnostic_trace']:
        assert math.isfinite(v['commanded_jaw_per_side_m']) and set(v['issued_jaw_joint_targets_m'])=={'jaw_l','jaw_r'}
        assert set(v['body_velocities'])=={'base','carrier','pcb','cover'}
        for body in v['body_velocities'].values():validate_passive_velocity(body)
    if stage=='release':
        supports=[v for k,v in r['contact_summary'].items() if 'cover' in k and 'ear' in k and 'base' in k and 'floor' in k and 'part_support' in k]
        assert supports and all(v['allowed_pair'] for v in supports)
        final_supports=[v for k,v in r['contact_summary'].items() if k.startswith('final_assembly_inspection|') and 'cover' in k and 'ear' in k and 'base' in k and 'floor' in k and 'part_support' in k]
        assert final_supports and any(v['maximum_impulse_Ns']>1e-8 for v in final_supports)
    for v in r['contact_summary'].values():assert not contact_violation(v['allowed_pair'],v['minimum_separation_mm']*.001,v['maximum_impulse_Ns'],.25)
    video=r['video'];assert video['download_to_laptop'] is False;path=Path(video['path']);assert path.is_file() and path.stat().st_size==video['bytes'] and digest(path)==video['sha256']
    import imageio_ffmpeg
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(path));assert frames==video['frames'] and abs(seconds-video['duration_s'])<.05
    print('TEAM_V_LID_POSE_STAGE_REPORT_AND_VIDEO_VERIFIED',json.dumps({'case':r['case'],'placements':sum(bool(v['placement_verified']) for v in r['part_results']),'diagnostic_stage':stage,'full_assembly_accepted':r['full_assembly_accepted'],'pcb_separate_dynamic':True,'video_frames':frames,'video_seconds':seconds,'video_stays_remote':True}),flush=True)


if __name__=='__main__':main()
