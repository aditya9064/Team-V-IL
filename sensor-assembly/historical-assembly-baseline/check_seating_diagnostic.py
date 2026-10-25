"""Validates evidence and strict stopping, never labels an assembly successful."""
import argparse,json,hashlib,math
from pathlib import Path
from assembly_checks import contact_violation
from isolation_tools import classify_terminal,fixture_offset

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def check(path):
    path=Path(path);r=json.loads(path.read_text());s=Path(__file__).parent;g=json.loads((s/'trial-mesh.json').read_text());p=json.loads((s/'isolation-provenance.json').read_text())
    for n,h in p['unchanged_files'].items():assert digest(s/n)==h,n
    assert r['task']=='isolated_base_seating_diagnostic' and r['diagnostic']['not_assembly_acceptance'] and r['part_order']==['base']
    assert r['limits']==g['limits'] and r['payload_limits']==g['payload_limits'] and r['grip_commands']==g['grip_commands']
    for n,k in [('base_seating_diagnostic.py','trial_script_sha256'),('design.json','design_sha256'),('trial-mesh.json','mesh_sha256'),('finger_cooking.py','finger_cooking_script_sha256')]:assert digest(s/n)==r[k],n
    assert r['physics']=={'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':1}
    assert r['contact_monitor_setup']['api_and_subscription_before_reset'] and not r['contact_monitor_errors']
    assert r['finger_surface_audit']['source_surface_match'] and len(r['finger_surface_audit']['probes'])==12
    assert r['finger_cooking']['maximum_world_source_vertex_change_m']<=1e-7
    assert r['cooked_geometry_probe_pass'] and len(r['cooked_geometry_probes'])==64
    assert all(v['actual_part_hit']==v['expected_part_hit'] and v.get('distance_pass',True) for v in r['cooked_geometry_probes'])
    assert not r['part_attachment_joints'] and not r['print_release'] and not r['vision_guided'] and not r['physical_hardware_test']
    for n,v in r['separate_body_audit'].items():assert v['dynamic']==(n!='nest') and not v['kinematic'] and not v['gravity_disabled']
    outcome=classify_terminal(r);assert not outcome.startswith('INCONCLUSIVE'),outcome
    d=r['diagnostic'];assert d['fixture_intervention_applied'] and d['robot_targets_unchanged'] and d['lowering_started']
    assert d['lower_duration_s']==2 and math.isclose(d['lower_jaw_command_m'],g['grip_commands']['base']['closed_grasp_command_per_jaw_m'],abs_tol=1e-12)
    assert all(abs(a+b-c)<1e-7 for a,b,c in zip(d['fixture_initial_expected_m'],fixture_offset(d['condition']),d['fixture_expected_after_m']))
    assert all(abs(a-b)<1e-7 for a,b in zip(d['fixture_expected_after_m'],d['fixture_readback_after_m']))
    assert d['fixture_intervention_step']==next(v['step'] for v in r['checkpoints'] if v['phase']=='1_base_carry')
    probes=d['fixture_query_after_first_physics_step'];assert len(probes)==2 and all(v['actual_nest_hit']==v['expected_nest_hit'] and (not v['expected_nest_hit'] or abs(v['distance_mm']-3.)<=.03) for v in probes)
    for v in r['checkpoints']:
        trace=v['stillness_trace'];assert len(trace)==6 and all(math.isfinite(t['maximum_raw_joint_speed_rad_s']) and t['maximum_raw_joint_speed_rad_s']<.02 for t in trace)
    for trace in r['endpoint_traces'].values():assert len(trace)==6 and all(v['tip_error_mm']<=2 and v['tool_axis_error_deg']<=1 and v['spread_axis_error_deg']<=1 for v in trace)
    assert not r.get('transient_inventory_failure') and not r.get('payload_failure')
    assert r['payload_motion_audit']['maximum_xy_mm']<=.75 and r['payload_motion_audit']['maximum_z_mm']<=.35 and r['payload_motion_audit']['maximum_relative_tilt_deg']<=1 and r['payload_motion_audit']['maximum_relative_yaw_deg']<=1
    assert r['motion_sampling']['path_and_phase_durations_unchanged'] and r['motion_sampling']['command_hz']==60
    for ph in r['motion_sampling']['phases']:assert ph['commands']==round(ph['requested_duration_s']*60) and abs(ph['actual_duration_s']-ph['requested_duration_s'])<2e-6 and abs(ph['maximum_command_interval_s']-1/60)<2e-6
    assert len(r['part_results'])==1 and r['part_results'][0]['pickup']['pass']
    assert not r['part_results'][0]['placement_verified'] and not r['part_results'][0]['release_verified']
    if outcome=='LOWERING_COMPLETED_WITHOUT_STRICT_STOP':assert not r['unexpected_contacts'] and d['lowering_completed']
    else:
        assert r['status']=='FAILED' and r['unexpected_contacts'] and all(v['phase']=='1_base_lower' for v in r['unexpected_contacts'])
        assert all(contact_violation(v['allowed_pair'],v['minimum_separation_mm']*.001,v['maximum_impulse_Ns'],.25) for v in r['unexpected_contacts'])
    t=r['diagnostic_lower_trace'];assert t and {x['boundary'] for x in t}=={'pre-step','post-step'}
    assert all(v['actor_world_matrices'] and math.isfinite(v['base_pose']['tilt_deg']) for v in t)
    assert r['diagnostic_contact_trace'] and r['diagnostic_collision_geometry']
    v=r['video'];video=Path(v.get('path') or v.get('failed_recording_path'));assert video.resolve().parent==path.resolve().parent and video.is_file()
    assert v['download_to_laptop'] is False
    import imageio_ffmpeg
    frames,secs=imageio_ffmpeg.count_frames_and_secs(str(video));assert frames==v['frames'] and abs(secs-frames/30)<.05
    return {'condition':d['condition'],'outcome':outcome,'raw_status':r['status'],'report_sha256':digest(path),'video_sha256':digest(video),'video_frames':frames,'source_geometry_unchanged':True,'physical_checks_unchanged':True,'assembly_accepted':False}
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('report');a=ap.parse_args();print('DIAGNOSTIC_EVIDENCE_VALID_NOT_ASSEMBLY_PASS',json.dumps(check(a.report)),flush=True)
