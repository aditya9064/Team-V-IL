"""Pilot generator: one Isaac application, a fresh world/stage per sample.

Runs with an Isaac-compatible interpreter. Preserves starter capture semantics;
this removes application restarts, not stage rebuild or 90-frame warmup costs.
"""
import argparse
import gc
import hashlib
import json
import math
import random
import sys
import time
from pathlib import Path


def prepare_capture(source):
    """Adapt only known lifecycle lines; reject changed starter code."""
    replacements = {
        'sim_app = SimulationApp({"headless": True})': 'sim_app = SHARED_APP',
        'sim_app.close()': '# application closed once by the batch controller',
        'cell.build(stage)': '''cell.build(stage)
for light_prim in stage.Traverse():
    intensity = light_prim.GetAttribute("inputs:intensity")
    if intensity and intensity.Get() is not None:
        intensity.Set(float(intensity.Get()) * LIGHT_SCALE)''',
    }
    for old, new in replacements.items():
        if source.count(old) != 1:
            raise ValueError('Unsupported starter capture source: ' + old)
        source = source.replace(old, new)
    compile(source, 'persistent starter capture', 'exec')
    return source


def validate_capture(folder, generation):
    """Check saved outputs; full-resolution visibility QA remains separate."""
    from PIL import Image
    folder = Path(folder)
    with Image.open(folder / 'world_camera.png') as image:
        image.load()
        if image.size != (3840, 2160) or image.mode != 'RGB':
            raise ValueError('Expected 3840x2160 RGB table capture')
        if all(lo == hi for lo, hi in image.getextrema()):
            raise ValueError('Constant image')
    camera = json.loads((folder / 'camera.json').read_text())
    truth = json.loads((folder / 'ground_truth.json').read_text())
    state = json.loads((folder / 'scene_state.json').read_text())
    saved_generation = json.loads((folder / 'generation.json').read_text())
    if saved_generation != generation:
        raise ValueError('Generation metadata changed')
    if camera['resolution'] != [3840, 2160]:
        raise ValueError('Camera resolution mismatch')
    matrix = camera['camera_to_world']
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError('Invalid camera matrix')
    if not all(math.isfinite(x) for row in matrix for x in row):
        raise ValueError('Non-finite camera matrix')
    cubes = [obj for obj in truth['objects'] if obj['category'] == 'cube']
    if len(cubes) != len(generation['cube_layout']) or len(cubes) != 6:
        raise ValueError('Cube inventory mismatch')
    names = [obj['name'] for obj in truth['objects']]
    if len(set(names)) != len(names) or set(names) != set(state['objects']):
        raise ValueError('Duplicate/missing/unexpected scene object')
    for obj, expected, yaw in zip(cubes, generation['cube_layout'], generation['cube_yaws_deg']):
        if obj['colour'] != expected[2]:
            raise ValueError('Cube colour mismatch')
        if abs(obj['yaw_deg'] - yaw) > 1e-8:
            raise ValueError('Cube yaw mismatch')
        actual = state['objects'][obj['name']]['position']
        if len(actual) != 3 or not all(math.isfinite(x) for x in actual):
            raise ValueError('Invalid actual position')
        for j in (0, 1):
            if abs(obj['xyz'][j] - expected[j]) > 1e-8 or abs(actual[j] - expected[j]) > 1e-6:
                raise ValueError('Layout, labels and captured stage disagree')
    return {'cube_count': len(cubes), 'object_count': len(names), 'resolution': camera['resolution']}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--starter', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--count', type=int, default=10)
    parser.add_argument('--start-seed', type=int, default=424201)
    args = parser.parse_args()
    if not 1 <= args.count <= 10:
        parser.error('Pilot permits 1–10 scenes; validate before scaling.')
    starter, out = args.starter.resolve(), args.out.resolve()
    if out.exists():
        parser.error('Use a new output directory; existing captures are preserved.')
    capture_path = starter / 'capture_scene.py'
    code = prepare_capture(capture_path.read_text())
    out.mkdir(parents=True)
    sys.path.insert(0, str(starter))
    import scene_config as C
    original_layout = list(C.SORT_CUBES)
    original_yaws = list(C.SORT_CUBE_YAWS_DEG)
    if C.SORT_TASK != 'cube_stack' or len(original_layout) != 6:
        parser.error('Pilot expects the unchanged six-cube starter configuration.')
    report = {'mode': 'one_application_fresh_stage_per_scene', 'requested_count': args.count,
              'samples': [], 'complete': False, 'startup_s': None,
              'limits': ['yellow positions remain fixed', 'no reachability or physical transfer validation',
                         'visibility QA required', 'stage rebuild and 90 render frames retained']}
    report['source_sha256'] = {name: hashlib.sha256((starter/name).read_bytes()).hexdigest()
        for name in ('capture_scene.py','scene_config.py','cell.py','render_settings.py')}
    total_start = time.monotonic()
    app = None
    original_argv = sys.argv[:]
    namespace = None
    error = None
    try:
        from isaacsim import SimulationApp
        startup = time.monotonic()
        app = SimulationApp({'headless': True})
        report['startup_s'] = time.monotonic() - startup
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import create_new_stage
        for index in range(args.count):
            seed = args.start_seed + index
            folder = out / f'seed-{seed}'
            folder.mkdir()
            rng = random.Random(seed)
            C.SORT_CUBES = list(original_layout)
            C.SORT_CUBES = C.hackathon_cubes(seed)
            C.SORT_CUBE_YAWS_DEG = [rng.uniform(-45, 45) for _ in C.SORT_CUBES]
            generation = {'seed': seed, 'cube_layout': C.SORT_CUBES,
                'cube_yaws_deg': C.SORT_CUBE_YAWS_DEG,
                'light_intensity_scale': rng.uniform(0.7, 1.3),
                'source': 'fanuc_er4ia_hackathon_starter_v05',
                'yellow_positions_fixed': True, 'reachability_verified': False,
                'real_camera_matched': False, 'capture_mode': report['mode']}
            # JSON round-trip makes tuples/lists identical for metadata checks.
            generation = json.loads(json.dumps(generation))
            (folder/'generation.json').write_text(json.dumps(generation, indent=2)+'\n')
            sample_start = time.monotonic()
            record = {'seed': seed, 'valid': False, 'elapsed_s': None, 'error': None}
            report['samples'].append(record)
            try:
                if namespace is not None:
                    namespace['world'].stop()
                    namespace['cam'].pause()
                    namespace.clear()
                    namespace = None
                    World.clear_instance()
                    gc.collect()
                create_new_stage()
                app.update()
                sys.argv = [str(capture_path), '--camera', 'table', '--out', str(folder)]
                namespace = {'__name__':'__main__', '__file__':str(capture_path),
                    'SHARED_APP':app, 'LIGHT_SCALE':generation['light_intensity_scale']}
                exec(compile(code, str(capture_path), 'exec'), namespace)
                record.update(validate_capture(folder, generation))
                record['valid'] = True
            except Exception as exception:
                record['error'] = f'{type(exception).__name__}: {exception}'
                raise
            finally:
                record['elapsed_s'] = time.monotonic() - sample_start
                (out/'batch_report.json').write_text(json.dumps(report, indent=2)+'\n')
                print('[persistent]', json.dumps(record), flush=True)
        report['complete'] = len(report['samples']) == args.count and all(s['valid'] for s in report['samples'])
    except Exception as exception:
        error = f'{type(exception).__name__}: {exception}'
        raise
    finally:
        report['error'] = error
        report['total_s_before_app_close'] = time.monotonic() - total_start
        report['valid_samples'] = sum(s['valid'] for s in report['samples'])
        (out/'batch_report.json').write_text(json.dumps(report, indent=2)+'\n')
        C.SORT_CUBES, C.SORT_CUBE_YAWS_DEG = original_layout, original_yaws
        sys.argv = original_argv
        if namespace is not None:
            namespace.clear()
        if app is not None:
            app.close()
    return 0 if report['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
