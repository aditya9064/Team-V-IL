"""Prospective observation-only setup audit. No simulator imports or report edits.

Reuses the frozen raw FK/pose, physical/preload/contact and PCB checks. A valid
setup/collection is never a pickup, freshness, assembly or hardware certificate.
The caller must supply an independently reviewed complete-source manifest hash.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

RUNTIME_SHA = '7b233a793141b508f814179f2fcf872349728e1ba0e346a3800a519d5849a7ad'
MESH_SHA = '728f96ba4be877e8d768ab57a5505a5c2c6be08e750791014274241d3bb9fcd5'
DESIGN_SHA = '3808dbe0fd8bbda6f6990e46b796cde1d38055c4fdf84450cccd39a858acc62a'


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(ok, message):
    if not ok: raise ValueError(message)


def verify_source(source, expected_manifest_sha):
    source = Path(source).resolve(); manifest_path = source/'snapshot-sha256.json'
    require(len(expected_manifest_sha)==64 and sha(manifest_path)==expected_manifest_sha,
            'Complete-source manifest is not independently bound')
    manifest = json.loads(manifest_path.read_text())
    files = {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()
             and '__pycache__' not in p.relative_to(source).parts
             and p != manifest_path}
    require(set(manifest)==files, 'Source files omitted or added beyond sealed manifest')
    for name, digest in manifest.items():
        path=source/name
        require(not path.is_symlink() and path.resolve().is_relative_to(source)
                and sha(path)==digest, 'Source content/path changed: '+name)
    for name, expected in [('camera_association_trial.py', RUNTIME_SHA),
                           ('nest-mesh.json', MESH_SHA), ('nest-design.json', DESIGN_SHA)]:
        require(manifest.get(name)==expected, 'Unreviewed runtime or original CAD')
    return manifest


def audit_fixture(r, g, d, source):
    import numpy as np
    from check_full_observed_assembly import rigid_matrix, pose
    from reliability_fixture import case_spec, spawn_transform
    fixture=r['initial_pose_pilot']
    require(fixture['schema']=='INITIAL_POSE_ONLY_PILOT_V61' and fixture['case']=='b'
            and fixture['declared_delta_mm_mm_deg']==case_spec('b'), 'Wrong observation fixture')
    require(fixture['helper_sha256']==sha(source/'reliability_fixture.py')
            and r['layout_m']==g['layout_m'], 'Fixture helper/layout changed')
    for flag in ('deterministic','applied_before_first_physics_step',
                 'nominal_pickup_recipe_unchanged','PCB_separate_dynamic'):
        require(fixture[flag] is True, 'Wrong fixture scope: '+flag)
    require(fixture['randomness_used'] is False and fixture['controller_reads_perturbations'] is False,
            'Fixture data fed to controller')
    require(set(fixture['authored'])=={'base','carrier','pcb','cover','nest'}, 'Missing authored body')
    for name, record in fixture['authored'].items():
        expected_z=.8949+(0 if name=='nest' else .0003)
        if name=='pcb': expected_z+=(d['carrier']['floor_thickness']+g['payload_initial_gap_mm'])*.001
        nominal=record['nominal_position_m']
        require(nominal[:2]==g['layout_m']['pickup_body_centres'][name]
                and abs(nominal[2]-expected_z)<=1e-12, 'Original spawn height/XY/gap changed')
        expected=spawn_transform(name,g['layout_m'],expected_z,'b')
        require(record['expected']==expected, 'Wrong authored transform')
        m=rigid_matrix(record['actual_USD_world_matrix'])
        p, rot=pose({'position_world_m':expected['position_m'],'orientation_wxyz':expected['orientation_wxyz']})
        require(np.allclose(m[3,:3],p,atol=1e-12,rtol=0)
                and np.allclose(m[:3,:3].T,rot,atol=1e-12,rtol=0), 'Authored world pose mismatch')


def audit_raw_setup(r, g, d):
    import math
    import numpy as np
    from check_full_observed_assembly import validate_ledger, check_process, arr
    from continuous_lid_command import independent_fk
    from payload_checks import payload_score
    k=r['kinematic_binding']
    require(k=={'arm_joint_names':['J'+str(i) for i in range(1,7)],
                'arm_joint_indices':list(range(6)),'tool_offset_m':[.293,0.,0.],
                'table_top_z_m':.9,'joint_dtype':'float32'}, 'Unreviewed FK binding')
    rows,_,_=validate_ledger(r,independent_fk)
    require(132<=len(rows)<=480 and (len(rows)-120)%2==0, 'Initial fixed settle/stillness scope changed')
    require(all(row['phase']=='initial_settling' for row in rows), 'Unapproved robot phase')
    require(r['part_results']==[] and r['part_order']==[] and r['motion_sampling']['phases']==[], 'Unexpected pickup')
    require(r['expected_motion_phases']==[] and r['expected_checkpoint_phases']==['initial'], 'Motion scope changed')
    shots=r['checkpoints']
    require(len(shots)==1 and shots[0]['phase']=='initial' and shots[0]['step']==len(rows), 'Initial checkpoint mismatch')
    first=rows[0]
    command_contract=Path(__file__).with_name('HOME-COMMAND-CONTRACT.json')
    require(sha(command_contract)=='93f999f41503ac6ea0e6ad19fe3abe738a38bdfcd718cf67f7cabd4cebd38de5',
            'Unpinned original home command contract')
    contract=json.loads(command_contract.read_text())
    qcmd=arr(first['issued_arm_q_rad'],(6,))
    joint_limits=np.deg2rad(contract['joint_limits_deg'])
    require(np.all(qcmd>=joint_limits[:,0]) and np.all(qcmd<=joint_limits[:,1]), 'Original home command outside joint limits')
    cmd_R,cmd_tip=independent_fk(qcmd,k['tool_offset_m'],'float64')
    goal=np.array([*contract['source_home_goal_xy_m'],
                   contract['surface_dz_m']+g['layout_m']['travel_tip_height_above_table']])
    desired_R=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
    error_angle=math.acos(float(np.clip((np.trace(desired_R@cmd_R.T)-1)/2,-1,1)))
    require(np.linalg.norm(np.asarray(cmd_tip)-goal)<contract['normal_home_IK_tolerance_m']
            and error_angle<contract['normal_home_IK_rotation_tolerance_rad'], 'Command does not match original home goal/orientation')
    require(first['issued_jaw_targets_m']==contract['joint_targets_for_jaw_open'], 'Command does not match original gripper-open keys/targets')
    for row in rows:
        require(row['issued_arm_q_rad']==first['issued_arm_q_rad']
                and row['issued_jaw_targets_m']==first['issued_jaw_targets_m'], 'Home/initial jaw command changed')
    # Original complete per-row loose-PCB/inventory scoring; no pickup prefix is credited.
    require(check_process(rows,shots,[],g,d,payload_score,False)=={}, 'Unexpected process metrics')
    shot=shots[0]; require(abs(shot['sim_time']-rows[-1]['sim_time'])<2e-6, 'Initial time not raw ledger')
    trace=shot['stillness_trace']; require(len(trace)==6, 'Missing six original stillness samples')
    require([v['step'] for v in trace]==list(range(len(rows)-10,len(rows)+1,2)), 'Stillness samples not consecutive rendered setup samples')
    for item in trace:
        raw=rows[item['step']-1]; speed=max(abs(x) for x in raw['actual_qdot_rad_s'][:6])
        require(math.isfinite(speed) and 0<=speed<.02
                and abs(speed-item['maximum_raw_joint_speed_rad_s'])<1e-7, 'Raw original camera stillness failed')
        require(abs(item['sim_time']-raw['sim_time'])<2e-6 and
                np.linalg.norm(arr(item['tip_robot_m'])-(arr(raw['tip_world_m'])-[0,0,.9]))<1e-8,
                'Stillness time/TCP not raw feedback')
    for name in ('base','carrier','pcb','cover','nest'):
        require(shot['poses'][name]==rows[-1]['poses'][name], 'Initial pose not terminal raw row')
    config=r['camera_probe'].get('configuration')
    if config is not None:
        require(config['runtime_state_step']==len(rows)
                and abs(config['native_clock_before']['sim_time']-rows[-1]['sim_time'])<2e-6,
                'Capture setup is not last raw physics row')
    return {'raw_rows':len(rows),'all_original_FK_actor_PCB_checks_passed':True,
            'six_raw_camera_stillness_samples_verified':True,'pickup_steps':0}


def audit(r, source, expected_manifest_sha):
    source=Path(source).resolve(); verify_source(source,expected_manifest_sha)
    # Source identities are verified before importing their pure offline audits.
    sys.path.insert(0,str(source))
    from check_full_observed_assembly import LIMITS
    from check_supported_saddle_grip import check_original_physics
    from check_camera_association import audit as association_audit
    g=json.loads((source/'nest-mesh.json').read_text());d=json.loads((source/'nest-design.json').read_text())
    require(r['task']=='camera_association_observation_only' and r['case']=='camera-association'
            and r['diagnostic_stage']=='camera-association', 'Wrong observation task')
    require(r['status'] in ('FAILED','OBSERVATION_DIAGNOSTIC_COMPLETE'), 'Nonterminal runtime')
    require(r['trial_script_sha256']==RUNTIME_SHA and r['mesh_sha256']==MESH_SHA
            and r['design_sha256']==DESIGN_SHA, 'Runtime/CAD identity mismatch')
    require(r['limits']==g['limits']==LIMITS and r['payload_limits']==g['payload_limits'], 'Physical limits changed')
    require(r['physics']=={'solver':'PGS','physics_dt':1/60,'render_dt':1/30,
                           'position_iterations':32,'velocity_iterations':1}, 'Original physics changed')
    expected_sdk=dict(json.loads((source/'telemetry-finger-geometry.json').read_text())['SDK_source_sha256'])
    expected_sdk.update({'fk.py':'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed',
                         'ik.py':'6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'})
    require(r['starter_hashes']==expected_sdk, 'Robot/source binding changed')
    require(r['physical_hardware_test'] is False and r['print_release'] is False
            and r['full_assembly_accepted'] is False and r['vision_guided'] is False, 'Unqualified positive claim')
    require(r['preplaced_fixture_parts']==['nest'] and r['initial_preparation']['robot_placement_order']==[]
            and not r['initial_preparation']['preplaced_dynamic_parts'], 'Wrong fixture/motion credit')
    # Enforce all original physical checks even when the camera collection failed.
    # A real physical stop makes this audit fail; the old failure remains untouched.
    check_original_physics(r,g,d,True,source,'original')
    audit_fixture(r,g,d,source); setup=audit_raw_setup(r,g,d)
    complete=r['status']=='OBSERVATION_DIAGNOSTIC_COMPLETE'
    association=None
    if complete:
        association=association_audit(r,source,json.loads((source/'provided-camera-source-packet.json').read_text()))
    else:
        require(bool(r.get('error')), 'Runtime failure missing original error')
    return {'setup_evidence_valid':True,'classification': 'OBSERVATION_COLLECTION_AND_SETUP_AUDITED' if complete else 'SETUP_AUDITED_RUNTIME_FAILED',
            'runtime_status':r['status'],'runtime_error':r.get('error'),'setup':setup,'association':association,
            'pixel_age_certified':False,'freshness_qualified':False,'pickup_qualified':False,
            'full_assembly_accepted':False,'physical_hardware_qualified':False,
            'historical_evidence_modified':False}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--manifest-sha256',required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    require(a.report.resolve()!=a.out.resolve(), 'Never overwrite original report')
    try:
        r=json.loads(a.report.read_text());out=audit(r,a.source,a.manifest_sha256);code=0
    except Exception as e:
        out={'setup_evidence_valid':False,'classification':'INCOMPLETE_OR_REJECTED_SETUP_EVIDENCE',
             'error':str(e),'error_type':type(e).__name__,'full_assembly_accepted':False,'pickup_qualified':False};code=1
    out['report_sha256']=sha(a.report);a.out.write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    print(json.dumps(out,allow_nan=False));raise SystemExit(code)
