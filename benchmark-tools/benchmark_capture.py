"""Three-scene current-pipeline benchmark; no cloud creation or stopping."""
import argparse, hashlib, json, os, platform, signal, subprocess, sys, time
from pathlib import Path

def run_command(cmd, timeout):
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, start_new_session=True) as process:
        try:
            output, _ = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                output, _ = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                output, _ = process.communicate()
            return 124, output
        return process.returncode, output

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--python', required=True, help='Actual Isaac-compatible Python executable')
    parser.add_argument('--starter', type=Path, required=True)
    parser.add_argument('--capture', type=Path, default=Path(__file__).with_name('capture_dataset.py'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--host-label', required=True)
    parser.add_argument('--rate-per-hour', type=float, default=0.0)
    args = parser.parse_args()
    starter, capture, out = args.starter.resolve(), args.capture.resolve(), args.out.resolve()
    if out.exists():
        parser.error('Use a fresh output directory; existing results are preserved.')
    if not capture.is_file() or not (starter / 'capture_scene.py').is_file():
        parser.error('Capture script or extracted starter is missing.')
    if not Path(args.python).is_file():
        parser.error('--python must be a real executable path, not a placeholder.')
    out.mkdir(parents=True)
    start = time.monotonic()
    summary = {'host_label': args.host_label, 'platform': platform.platform(),
               'protocol': '3 sequential current-pipeline process launches; 4K table camera; seeds 424201-424203',
               'wall_time_limit_s': 900, 'samples': [],
               'rate_per_hour': args.rate_per_hour, 'gpu_probe': '',
               'budget_note': 'Script timing excludes provisioning/setup/idle/storage. This script DOES NOT stop the cloud instance.'}
    files = [capture, starter/'capture_scene.py', starter/'scene_config.py',
             starter/'cell.py', starter/'render_settings.py']
    summary['source_sha256'] = {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    try:
        _, summary['gpu_probe'] = run_command(
            ['nvidia-smi', '--query-gpu=name,driver_version,memory.total', '--format=csv'], 20)
    except FileNotFoundError:
        summary['gpu_probe'] = 'nvidia-smi unavailable'
    try:
        for seed in (424201, 424202, 424203):
            remaining = 900 - (time.monotonic() - start)
            if remaining <= 0:
                break
            folder = out / f'seed-{seed}'
            sample_start = time.monotonic()
            code, output = run_command([args.python, str(capture), '--starter', str(starter),
                '--out', str(folder), '--seed', str(seed)], remaining)
            (out / f'seed-{seed}.log').write_text(output)
            record = {'seed': seed, 'returncode': code, 'elapsed_s': time.monotonic()-sample_start,
                      'valid': False, 'error': None}
            try:
                if code != 0:
                    raise RuntimeError(f'Capture returned {code}')
                from PIL import Image
                with Image.open(folder/'world_camera.png') as image:
                    image.load()
                    if image.size != (3840, 2160) or image.mode != 'RGB':
                        raise ValueError(f'Unexpected image: {image.size} {image.mode}')
                    if all(lo == hi for lo, hi in image.getextrema()):
                        raise ValueError('Constant image')
                generation = json.loads((folder/'generation.json').read_text())
                truth = json.loads((folder/'ground_truth.json').read_text())
                camera = json.loads((folder/'camera.json').read_text())
                json.loads((folder/'scene_state.json').read_text())
                cubes = [o for o in truth['objects'] if o['category'] == 'cube']
                if generation['seed'] != seed or len(cubes) != 6 or camera['resolution'] != [3840,2160]:
                    raise ValueError('Seed/count/resolution mismatch')
                for obj, expected, yaw in zip(cubes, generation['cube_layout'], generation['cube_yaws_deg']):
                    if obj['colour'] != expected[2] or any(abs(obj['xyz'][j]-expected[j])>1e-8 for j in (0,1)) or abs(obj['yaw_deg']-yaw)>1e-8:
                        raise ValueError('Generation and labels disagree')
                record['valid'] = True
            except Exception as error:
                record['error'] = f'{type(error).__name__}: {error}'
            summary['samples'].append(record)
            print(json.dumps(record), flush=True)
            if not record['valid']:
                break
    finally:
        summary['total_elapsed_s'] = time.monotonic()-start
        valid = sum(s['valid'] for s in summary['samples'])
        summary['valid_samples'] = valid
        summary['valid_images_per_hour_including_script_startups'] = valid*3600/max(summary['total_elapsed_s'],1e-9)
        summary['estimated_script_compute_cost'] = args.rate_per_hour*summary['total_elapsed_s']/3600
        summary['complete'] = len(summary['samples'])==3 and valid==3
        (out/'benchmark.json').write_text(json.dumps(summary,indent=2)+'\n')
    return 0 if summary['complete'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
