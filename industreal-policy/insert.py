"""Allocated-only calibrated insertion baseline in NVIDIA Factory/IndustReal port.

Not a learned policy. One-time simulator fixture/grasp calibration is privileged
and explicitly recorded. No live part state drives targets after calibration.
"""
import argparse,hashlib,json,os,traceback,math
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binding',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--seed',type=int,default=20261022)
    args=p.parse_args()
    if not os.environ.get('SLURM_JOB_ID'):p.error('Use an allocated GPU session')
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['TEAM_V_FANUC_BINDING']=str(args.binding.resolve())
    os.environ['TEAM_V_INDUSTREAL_INSERTION_BASELINE']='1'
    from team_v_industreal.doctor import inspect
    preflight=inspect(args.binding);(args.out/'preflight.json').write_text(json.dumps(preflight,indent=2)+'\n')
    if preflight['source_errors'] or preflight['missing_dependencies'] or preflight['runtime_errors']:raise RuntimeError('Preflight failed')
    root=Path(__file__).parent
    report={'status':'STARTING','scope':'calibrated simulated peg insertion and released support; not trained policy',
        'control_mode':'TCP feedback plus frozen one-time privileged simulator fixture/grasp calibration',
        'seed':args.seed,'simulation_steps':0,'simulation_executed':False,'policy_trained':False,
        'full_assembly_accepted':False,'physical_hardware_tested':False,
        'binding_sha256':hashlib.sha256(args.binding.read_bytes()).hexdigest(),
        'implementation_manifest_sha256':hashlib.sha256((root/'implementation-manifest.json').read_bytes()).hexdigest()}
    app=None;env=None;trace=None;scoring=[]
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,device='cuda:0').app
        import gymnasium as gym
        import torch
        import team_v_industreal
        from isaaclab_tasks.utils import parse_env_cfg
        from team_v_industreal.insertion_calibration import calibrate,target_tcp,interpolate,release_state,gradual_release_opening
        from team_v_industreal.telemetry import DiagnosticTrace,measured_sample
        from team_v_industreal.evaluation import score,within_capture_envelope
        from launch import validate_smoke_transition
        team_v_industreal.register();cfg=parse_env_cfg(team_v_industreal.TASK_ID,device='cuda:0',num_envs=1)
        cfg.seed=args.seed;cfg.curriculum_enabled=False;cfg.episode_length_s=40.
        env=gym.make(team_v_industreal.TASK_ID,cfg=cfg);env.reset();e=env.unwrapped
        trace=DiagnosticTrace(args.out/'measured-state.jsonl',{'seed':args.seed,'binding_sha256':report['binding_sha256'],
             'implementation_manifest_sha256':report['implementation_manifest_sha256'],'control_mode':report['control_mode']})
        def sample(phase,terminated=False,truncated=False):
            row=measured_sample(e,step=report['simulation_steps'],elapsed_s=report['simulation_steps']*e.step_dt,terminated=terminated,truncated=truncated)
            row['insertion_phase']=phase
            joints=row['measured_joint_positions'];forces=row['contact_audit']['last_pair_normal_force_world_n']
            release=release_state(joints,forces,bool(e.release_requested))
            row['measured_jaw_opening_per_jaw_m']=release['opening_per_jaw_m']
            row['measured_grip_force_n']=release['grip_force_n']
            row['release_verified']=release['released']
            row['peg_linear_velocity_m_s']=e._held_asset.data.root_lin_vel_w[0].detach().cpu().tolist()
            row['peg_angular_velocity_rad_s']=e._held_asset.data.root_ang_vel_w[0].detach().cpu().tolist()
            trace.write(row)
            return row
        sample('CALIBRATED_START')
        def advance(phase,position,quaternion):
            e.set_calibrated_insertion_target(position,quaternion)
            obs,reward,terminated,truncated,_=env.step(torch.zeros((1,6),device=e.device))
            report['simulation_steps']+=1;report['simulation_executed']=True;report['active_phase']=phase
            term=bool(torch.as_tensor(terminated).any());trunc=bool(torch.as_tensor(truncated).any())
            row=sample(phase,term,trunc)
            validate_smoke_transition(obs,reward,terminated,truncated)
            if e.contact_audit.invalid or e.tool_contact_audit.invalid:raise RuntimeError('Insertion contact guard failed')
            return row
        def travel(phase,end,steps):
            start=(e.fingertip_midpoint_pos.clone(),e.fingertip_midpoint_quat.clone())
            for i in range(steps):
                fraction=(i+1)/steps
                if phase=='INSERT':fraction=fraction*fraction*(3.-2.*fraction)
                advance(phase,*interpolate(*start,*end,fraction))
        held_start=(e.held_pos-e.fingertip_midpoint_pos).clone()
        hold_target=(e.fingertip_midpoint_pos.clone(),e.fingertip_midpoint_quat.clone())
        hold_drift=0.
        for _ in range(30):
            advance('GRIP_REQUALIFY',*hold_target)
            hold_drift=max(hold_drift,float(torch.linalg.vector_norm((e.held_pos-e.fingertip_midpoint_pos)-held_start,dim=-1).max()))
        grip=e.contact_audit.report()
        if hold_drift>.001 or not grip['last_bilateral_grip'] or grip['bilateral_grip_fraction']<.8:
            raise RuntimeError('Higher shaft grasp failed the unchanged grip requalification')
        if grip['maximum_pair_normal_force_n'].get('FixedAsset',0.)>.001:raise RuntimeError('Socket supported peg during grip requalification')
        report['grip_requalification_maximum_relative_slip_m']=hold_drift
        calibration=calibrate(e.fingertip_midpoint_pos,e.fingertip_midpoint_quat,e.held_pos,e.held_quat,e.fixed_pos,e.fixed_quat)
        serial={k:v.detach().cpu().tolist() for k,v in calibration.items()}
        (args.out/'privileged-frozen-calibration.json').write_text(json.dumps({'assumption':'ideal one-time simulator calibration, not camera perception','values':serial},indent=2)+'\n')
        above=target_tcp(calibration,.030)
        seated=target_tcp(calibration,.00005)
        travel('ALIGN',above,30)
        for _ in range(10):advance('ALIGN_HOLD',*above)
        travel('INSERT',seated,100)
        for _ in range(10):pre_release=advance('SEATED_HOLD',*seated)
        if not within_capture_envelope(pre_release['radial_error_m'],pre_release['depth_error_m'],pre_release['tilt_rad']):
            raise RuntimeError('Pre-release cavity capture gate failed')
        report['pre_release_measured_state']=pre_release
        before_withdraw=e.fingertip_midpoint_pos.clone()
        e.begin_release(gradual=True)
        for i in range(60):
            e.set_release_opening_per_jaw(gradual_release_opening(e.binding['grasp_opening_per_jaw_m'],(i+1)/60))
            advance('OPEN_JAWS',*seated)
        withdrawn=(seated[0]+.040*calibration['fixture_axis'],seated[1])
        travel('WITHDRAW_TOOL',withdrawn,30)
        for _ in range(45):advance('RELEASE_SETTLE',*withdrawn)
        for _ in range(15):
            row=advance('RELEASED_VERIFY',*withdrawn)
            audit=row['contact_audit'];tool=row['tool_contact_audit']
            scoring.append(dict(time_s=row['elapsed_since_reset_s'],radial_error_m=row['radial_error_m'],depth_error_m=row['depth_error_m'],
                tilt_rad=row['tilt_rad'],max_contact_depth_m=max(audit['max_contact_depth_m'],tool['max_contact_depth_m']),
                released=row['release_verified'],contact_monitor_valid=audit['contact_monitor_valid'] and tool['contact_monitor_valid'],
                socket_contact_observed=audit['socket_contact_observed']))
            support_z=sum(audit['last_pair_normal_force_world_n'].get(name,[0.,0.,0.])[2] for name in ('FixedAsset','Table'))
            if not .8*e.cfg_task.held_asset_cfg.mass*9.81<=support_z<=1.2*e.cfg_task.held_asset_cfg.mass*9.81:
                raise RuntimeError('Released peg lacks measured gravity support')
            if math.sqrt(sum(v*v for v in row['peg_linear_velocity_m_s']))>.01:raise RuntimeError('Released peg is not stationary')
            if math.sqrt(sum(v*v for v in row['peg_angular_velocity_rad_s']))>.1:raise RuntimeError('Released peg is rotating')
        withdrawal=float(((e.fingertip_midpoint_pos-before_withdraw)*calibration['fixture_axis']).sum())
        report['measured_tool_withdrawal_m']=withdrawal
        if withdrawal<.014:raise RuntimeError('Tool failed to withdraw at least14mm after jaw opening')
        if withdrawal<.039:raise RuntimeError('Tool did not reach the planned full-clearance39mm withdrawal')
        result=score(scoring);report['independent_score']=result
        (args.out/'release-samples.json').write_text(json.dumps(scoring,indent=2)+'\n')
        if result['status']!='INSERTION_BENCHMARK_PASS':raise RuntimeError('Independent released-insertion scoring failed: '+repr(result['reasons']))
        report.update(status='CALIBRATED_INSERTION_RELEASE_BENCHMARK_PASS',physics_steps=report['simulation_steps']*cfg.decimation,
                      final_measured_state=row,release_verified=True)
    except Exception as exc:
        report.update(status='FAILED',error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        if env is not None:
            report['provenance']=env.unwrapped.benchmark_provenance();report['last_contact_guard_failure']=env.unwrapped.last_contact_guard_failure
        if trace is not None:report['measured_state_samples']=trace.samples;trace.close()
        (args.out/'insertion-report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        if env is not None:env.close()
        if app is not None:app.close()


if __name__=='__main__':main()
