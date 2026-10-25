"""Launch only inside an allocated simulator GPU session; never submits jobs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import runpy
import sys
import traceback


def validate_smoke_transition(observations,reward,terminated,truncated):
    """Reject failures even when Isaac Lab has already auto-reset the scene."""
    import torch
    for name,flags in [('termination',terminated),('timeout',truncated)]:
        flags=torch.as_tensor(flags)
        if flags.numel()==0 or bool(flags.any()):
            raise RuntimeError('Integration smoke encountered environment '+name+'; auto-reset is not a successful diagnostic')
    reward=torch.as_tensor(reward)
    if reward.numel()==0 or not bool(torch.isfinite(reward).all()):
        raise RuntimeError('Integration smoke returned missing/nonfinite reward')
    count=0
    def check(value,path):
        nonlocal count
        if isinstance(value,dict):
            for key,item in value.items():check(item,path+'/'+str(key))
        elif isinstance(value,(list,tuple)):
            for index,item in enumerate(value):check(item,path+'/'+str(index))
        elif isinstance(value,torch.Tensor) and value.numel()>0:
            count+=1
            if not bool(torch.isfinite(value).all()):
                raise RuntimeError('Nonfinite integration observation: '+path)
        else:
            raise RuntimeError('Missing or unsupported integration observation: '+path)
    check(observations,'observations')
    if not count:raise RuntimeError('Integration smoke returned no observation tensors')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['smoke','train'],required=True)
    parser.add_argument('--binding',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--isaaclab-root',type=Path)
    parser.add_argument('--steps',type=int,default=30)
    parser.add_argument('--iterations',type=int,default=1)
    parser.add_argument('--smoke-report',type=Path)
    parser.add_argument('--seed',type=int,default=20261022)
    parser.add_argument('--grip-lift-m',type=float,default=0.0,
                        help='Diagnostic TCP lift after30holdsteps; no part-truth action inputs')
    args=parser.parse_args()
    if not os.environ.get('SLURM_JOB_ID'):
        parser.error('Use an allocated GPU session; this launcher never runs on a login/management host')
    if not 1<=args.steps<=600 or not 1<=args.iterations<=10:
        parser.error('Initial smoke limits: 1..600 steps, 1..10 training iterations')
    if not 0<=args.grip_lift_m<=0.01 or (args.grip_lift_m and args.steps<75):
        parser.error('Grip lift must be0..10mm and requires at least75controlsteps')
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ['TEAM_V_FANUC_BINDING']=str(args.binding.resolve())
    from team_v_industreal.doctor import inspect
    preflight=inspect(args.binding)
    (args.out/'preflight.json').write_text(json.dumps(preflight,indent=2)+'\n')
    if preflight['source_errors'] or preflight['missing_dependencies'] or preflight['runtime_errors']:
        raise RuntimeError('Dependency/source/runtime-profile preflight failed; see preflight.json')
    import team_v_industreal
    team_v_industreal.register()
    if args.mode=='train':
        if not args.isaaclab_root: parser.error('--isaaclab-root is required for training')
        if not args.smoke_report: parser.error('--smoke-report is required before a training smoke')
        smoke=json.loads(args.smoke_report.read_text())
        binding_hash=hashlib.sha256(args.binding.read_bytes()).hexdigest()
        manifest_hash=hashlib.sha256((Path(__file__).parent/'upstream-manifest.json').read_bytes()).hexdigest()
        if smoke.get('status')!='INTEGRATION_SMOKE_EXECUTED_NOT_ASSEMBLY' or smoke.get('binding_sha256')!=binding_hash or smoke.get('upstream_manifest_sha256')!=manifest_hash:
            raise RuntimeError('Prior successful integration smoke must match this asset and source manifest')
        if smoke.get('implementation_manifest_sha256')!=hashlib.sha256((Path(__file__).parent/'implementation-manifest.json').read_bytes()).hexdigest():
            raise RuntimeError('Prior smoke does not match this port implementation')
        driver=args.isaaclab_root/'scripts/reinforcement_learning/rl_games/train.py'
        manifest=json.loads((Path(__file__).parent/'upstream-manifest.json').read_text())
        if hashlib.sha256(driver.read_bytes()).hexdigest()!=manifest['train_driver_sha256']:
            raise RuntimeError('Training driver differs from reviewed pinned source')
        sys.argv=[str(driver),'--task',team_v_industreal.TASK_ID,'--num_envs','1',
                  '--max_iterations',str(args.iterations),'--seed',str(args.seed),'--headless',
                  'agent.params.config.full_experiment_name='+args.out.name]
        runpy.run_path(str(driver),run_name='__main__')
        return
    report={'status':'STARTING','simulation_steps':0,'simulation_executed':False,
            'policy_trained':False,'physical_hardware_tested':False,'full_assembly_accepted':False,
            'scope':'preheld peg hold plus TCP-feedback lift' if args.grip_lift_m else 'zero-action preheld peg hold','seed':args.seed}
    report['binding_sha256']=hashlib.sha256(args.binding.read_bytes()).hexdigest()
    report['upstream_manifest_sha256']=hashlib.sha256((Path(__file__).parent/'upstream-manifest.json').read_bytes()).hexdigest()
    report['implementation_manifest_sha256']=hashlib.sha256((Path(__file__).parent/'implementation-manifest.json').read_bytes()).hexdigest()
    app=None;env=None;trace=None
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,device='cuda:0').app
        import gymnasium as gym
        import torch
        from isaaclab_tasks.utils import parse_env_cfg
        cfg=parse_env_cfg(team_v_industreal.TASK_ID,device='cuda:0',num_envs=1)
        cfg.seed=args.seed
        env=gym.make(team_v_industreal.TASK_ID,cfg=cfg)
        observations,_=env.reset()
        from team_v_industreal.telemetry import DiagnosticTrace,measured_sample
        trace=DiagnosticTrace(args.out/'measured-state.jsonl',{
            'seed':args.seed,'binding_sha256':report['binding_sha256'],
            'implementation_manifest_sha256':report['implementation_manifest_sha256']})
        trace.write(measured_sample(env.unwrapped,step=0,elapsed_s=0.0))
        first_relative=env.unwrapped.held_pos-env.unwrapped.fingertip_midpoint_pos
        first_tcp_z=float(env.unwrapped.fingertip_midpoint_pos[0,2])
        max_relative_change=0.0
        for index in range(args.steps):
            action=torch.zeros((1,6),device=env.unwrapped.device)
            if args.grip_lift_m and index>=30:
                # Read TCP feedback only. Do not servo from privileged peg pose.
                error=first_tcp_z+args.grip_lift_m-float(env.unwrapped.fingertip_midpoint_pos[0,2])
                action[0,2]=max(-1.,min(1.,error/float(env.unwrapped.pos_threshold[0,2])))
            step_observations,reward,terminated,truncated,_=env.step(action)
            report['simulation_steps']+=1
            report['simulation_executed']=True
            report['last_step_terminated']=bool(torch.as_tensor(terminated).any())
            report['last_step_truncated']=bool(torch.as_tensor(truncated).any())
            trace.write(measured_sample(env.unwrapped,step=report['simulation_steps'],
                elapsed_s=report['simulation_steps']*env.unwrapped.step_dt,
                terminated=report['last_step_terminated'],truncated=report['last_step_truncated']))
            validate_smoke_transition(step_observations,reward,terminated,truncated)
            if env.unwrapped.contact_audit.invalid:
                raise RuntimeError('Physical contact guard stopped the integration smoke')
            relative=env.unwrapped.held_pos-env.unwrapped.fingertip_midpoint_pos
            max_relative_change=max(max_relative_change,float(torch.linalg.vector_norm(relative-first_relative,dim=-1).max()))
        if not env.unwrapped.contact_audit.grip_contact_observed:
            raise RuntimeError('No reported peg/finger contact; grasp remains unverified')
        audit=env.unwrapped.contact_audit.report()
        if (not audit['tensor_contact_points'] or audit['force_crosscheck_frames']!=audit['tensor_frames']
                or audit['net_force_coverage_frames']!=audit['tensor_frames']):
            raise RuntimeError('Contact points require independent force-matrix and total-force corroboration')
        if audit['maximum_pair_normal_force_n'].get('FixedAsset',0.)>.001:
            raise RuntimeError('Socket supported or struck the peg during the independent grip qualification')
        if not audit['last_bilateral_grip'] or audit['bilateral_grip_fraction']<0.8:
            raise RuntimeError('Peg lacks sustained measured contact force from both original jaws')
        report['maximum_preheld_relative_translation_change_m']=max_relative_change
        report['physics_steps']=report['simulation_steps']*env.unwrapped.cfg.decimation
        if max_relative_change>0.001:
            raise RuntimeError('Preheld peg drifted by more than the declared 1 mm benchmark smoke limit')
        if args.grip_lift_m:
            report['requested_tcp_lift_m']=args.grip_lift_m
            report['measured_tcp_lift_m']=float(env.unwrapped.fingertip_midpoint_pos[0,2])-first_tcp_z
            if abs(report['measured_tcp_lift_m']-args.grip_lift_m)>0.001:
                raise RuntimeError('Measured TCP lift did not reach the requested diagnostic displacement')
        report.update(status='INTEGRATION_SMOKE_EXECUTED_NOT_ASSEMBLY',provenance=env.unwrapped.benchmark_provenance())
    except Exception as exc:
        report.update(status='FAILED',error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        if env is not None:
            report['last_contact_guard_failure']=env.unwrapped.last_contact_guard_failure
            report['provenance']=env.unwrapped.benchmark_provenance()
        if trace is not None:
            report['measured_state_trace']='measured-state.jsonl'
            report['measured_state_samples']=trace.samples
            trace.close()
        (args.out/'smoke-report.json').write_text(json.dumps(report,indent=2)+'\n')
        if env is not None: env.close()
        if app is not None: app.close()


if __name__=='__main__': main()
