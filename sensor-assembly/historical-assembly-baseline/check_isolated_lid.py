"""Independent success checker. Failure/Slurm zero never imply a pass."""
import argparse,hashlib,json,math
from pathlib import Path
from assembly_checks import contact_violation,grasp_command_m
from payload_checks import payload_score


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('report');a=ap.parse_args();r=json.loads(Path(a.report).read_text());source=Path(__file__).parent;g=json.loads((source/'lid-clearance-mesh.json').read_text());l=r['limits']
    assert r['status']=='LID_ONLY_PASS' and r['isolated_lid_accepted'],r.get('error')
    assert r['initial_preparation']=={'product_parts_preassembled_together':False,'pcb_preloaded_in_carrier':True,'only_preplaced_fixture':['nest','base','carrier','pcb'],'robot_placement_order':['cover'],'prepositioned_product_parts_are_dynamic':True}
    sampling=r['motion_sampling'];assert sampling['command_hz']==60 and sampling['physics_steps_per_command']==1 and not sampling['path_and_phase_durations_unchanged'] and sampling['original_phase_durations_retained']
    assert len(sampling['phases'])==12
    for phase in sampling['phases']:
        assert phase['commands']==round(phase['requested_duration_s']*60)
        assert abs(phase['actual_duration_s']-phase['requested_duration_s'])<=2e-6
        assert abs(phase['maximum_command_interval_s']-1/60)<=2e-6
    assert r['payload_initial_gap_mm']==g['payload_initial_gap_mm']==0.0
    setup=r['contact_monitor_setup'];assert setup['api_and_subscription_before_reset'] and setup['runtime_checks_active_during_reset']
    seating=r['horizontal_seating_contacts'];assert seating['events']>0 and seating['only_horizontal'] and seating['cad_support_proof_sha256']==digest(source/'seating-support-proof.json')
    assert r['case'] in ['loaded-carrier','full-assembly'];loaded=r['case']=='loaded-carrier'
    order=['cover'];parts=['base','carrier','pcb','nest'] if loaded else ['base','carrier','pcb','cover','nest']
    assert r['part_order']==order and r['physical_parts']==parts and [v['part'] for v in r['part_results']]==order
    assert r['preplaced_fixture_parts']==['nest','base','carrier','pcb']
    assert r['task']=='isolated_lid_clearance_pick_place_release' and r['full_assembly_accepted'] is False
    assert r['vision_guided'] is False and r['physical_hardware_test'] is False and r['print_release'] is False
    for filename,key in [('isolated_lid_trial.py','trial_script_sha256'),('lid-clearance-design.json','design_sha256'),('lid-clearance-mesh.json','mesh_sha256'),('finger_cooking.py','finger_cooking_script_sha256')]:assert digest(source/filename)==r[key],filename
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
        assert part['pickup']['pass'] and part['placement_verified'] and part['release_verified']
        v=part['pickup'];assert v['lift_mm']>=35 and v['xy_slip_mm']<=2 and v['tilt_deg']<=5 and v['yaw_deg']<=3
        v=part['placement'];assert v['xy_mm']<=.75 and v['z_mm']<=.35 and v['tilt_deg']<=1 and v['yaw_deg']<=1
        assert part['release_follow_mm']<=.25
        h=r['held_motion_audits'][part['part']];assert h['samples']>0 and h['maximum_relative_slip_mm']<=2 and h['maximum_tilt_deg']<=5 and h['maximum_yaw_deg']<=3
        d=r['release_motion_audits'][part['part']];assert d['samples']>0 and d['maximum_drift_mm']<=.25
        if part['part']=='carrier':assert part['separate_pcb_pickup_lift_mm']>=35 and part['separate_pcb_placement']['pass']
    for shot in r['checkpoints']:
        t=shot['stillness_trace'];assert len(t)==6
        assert all(math.isfinite(v['maximum_raw_joint_speed_rad_s']) and v['maximum_raw_joint_speed_rad_s']<.02 for v in t)
        assert all(t[i+1]['step']>t[i]['step'] and t[i+1]['sim_time']>t[i]['sim_time'] and abs((t[i+1]['sim_time']-t[i]['sim_time'])-(t[i+1]['step']-t[i]['step'])/60)<2e-6 for i in range(5))
        assert payload_score(shot['poses']['carrier'],shot['poses']['pcb'])['pass']
    assert len(r['checkpoints'])==14 and len(r['endpoint_traces'])==12
    assert r['designed_placement_seating_allowance_mm']=={'cover':0.}
    for traces in r['endpoint_traces'].values():
        assert len(traces)==6
        assert all(math.isfinite(t['tip_error_mm']) and t['tip_error_mm']<=2 and math.isfinite(t['tool_axis_error_deg']) and t['tool_axis_error_deg']<=1 and math.isfinite(t['spread_axis_error_deg']) and t['spread_axis_error_deg']<=1 for t in traces)
    assert set(r['final_part_scores'])==set(parts) and all(p['pass'] for p in r['final_part_scores'].values())
    for v in r['contact_summary'].values():assert not contact_violation(v['allowed_pair'],v['minimum_separation_mm']*.001,v['maximum_impulse_Ns'],.25)
    video=r['video'];assert video['download_to_laptop'] is False;path=Path(video['path']);assert path.is_file() and path.stat().st_size==video['bytes'] and digest(path)==video['sha256']
    import imageio_ffmpeg
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(path));assert frames==video['frames'] and abs(seconds-video['duration_s'])<.05
    print('TEAM_V_ISOLATED_LID_REPORT_AND_VIDEO_VERIFIED_NOT_FULL_ASSEMBLY',json.dumps({'case':r['case'],'placements':len(order),'pcb_separate_dynamic':True,'video_frames':frames,'video_seconds':seconds,'video_stays_remote':True}),flush=True)


if __name__=='__main__':main()
