"""Read-only dependency and source checks; never starts Isaac or a GPU job."""
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
from .runtime import inspect_runtime


def inspect(binding_path=None):
    root=Path(__file__).resolve().parents[1]
    manifest=json.loads((root/'upstream-manifest.json').read_text())
    failures=[]
    for name,expected in manifest['files'].items():
        p=root/name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
            failures.append('Upstream source mismatch: '+name)
    implementation=root/'implementation-manifest.json'
    if implementation.is_file():
        for name,expected in json.loads(implementation.read_text())['files'].items():
            p=root/name
            if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
                failures.append('Implementation source mismatch: '+name)
    else:
        failures.append('Missing implementation manifest')
    dependencies={}
    for name in ('torch','numpy','gymnasium','isaacsim','isaaclab','rl_games'):
        try: dependencies[name]=importlib.util.find_spec(name) is not None
        except (ImportError,ValueError): dependencies[name]=False
    binding=None
    if binding_path:
        try:
            binding=json.loads(Path(binding_path).read_text())
            robot=Path(binding['robot_usd'])
            if hashlib.sha256(robot.read_bytes()).hexdigest()!=binding['robot_usd_sha256']:
                failures.append('Exported robot USD checksum mismatch')
            if binding['arm_joints']!=['J'+str(i) for i in range(1,7)] or binding['jaw_joints']!=['jaw_l','jaw_r']:
                failures.append('Incorrect FANUC/SMC joint names')
        except (OSError,KeyError,ValueError) as exc: failures.append('Invalid binding: '+str(exc))
    runtime=inspect_runtime()
    return {'status':'SOURCE_CHECKS_PASS_RUNTIME_UNVERIFIED' if not failures else 'FAILED',
            'python':platform.python_version(),'dependencies':dependencies,
            'missing_dependencies':[k for k,v in dependencies.items() if not v],
            'robot_binding_supplied':binding is not None,'source_errors':failures,
            'reference_isaaclab_commit':manifest['isaaclab_commit'],
            'runtime_profile':runtime['profile'],
            'package_versions':runtime['package_versions'],
            'runtime_errors':runtime['runtime_errors'],
            'runtime_profile_matches':runtime['runtime_profile_matches'],
            'runtime_compatibility_verified':False,'simulation_executed':False,
            'policy_trained':False,'physical_hardware_tested':False,'full_assembly_accepted':False}


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--binding')
    args=p.parse_args()
    result=inspect(args.binding)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if not result['source_errors'] else 1)
