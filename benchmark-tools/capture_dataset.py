"""Render one reproducible scene using the unchanged FANUC v05 starter."""
import argparse
import json
import random
import sys
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--starter', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--seed', type=int, required=True)
a = p.parse_args()
starter = a.starter.resolve()
out = a.out.resolve()
if out.exists():
    raise SystemExit('Choose a new output directory; existing samples are preserved.')
sys.path.insert(0, str(starter))
import scene_config as C
rng = random.Random(a.seed)
C.SORT_CUBES = C.hackathon_cubes(a.seed)
C.SORT_CUBE_YAWS_DEG = [rng.uniform(-45, 45) for _ in C.SORT_CUBES]
light_scale = rng.uniform(0.7, 1.3)
out.mkdir(parents=True)
(out / 'generation.json').write_text(json.dumps({
    'seed': a.seed, 'cube_layout': C.SORT_CUBES,
    'cube_yaws_deg': C.SORT_CUBE_YAWS_DEG,
    'light_intensity_scale': light_scale,
    'source': 'fanuc_er4ia_hackathon_starter_v05',
    'yellow_positions_fixed': True,
    'reachability_verified': False,
    'real_camera_matched': False
}, indent=2))
source = (starter / 'capture_scene.py').read_text()
needle = 'cell.build(stage)'
assert source.count(needle) == 1
lighting = '''
changed_lights = []
for light_prim in stage.Traverse():
    intensity = light_prim.GetAttribute('inputs:intensity')
    if intensity and intensity.Get() is not None:
        intensity.Set(float(intensity.Get()) * LIGHT_SCALE)
        changed_lights.append(str(light_prim.GetPath()))
'''
source = source.replace(needle, needle + '\n' + lighting)
sys.argv = [str(starter / 'capture_scene.py'), '--camera', 'table', '--out', str(out)]
exec(compile(source, str(starter / 'capture_scene.py'), 'exec'), {
    '__name__': '__main__', '__file__': str(starter / 'capture_scene.py'),
    'LIGHT_SCALE': light_scale
})
# Fail the job when Kit exits without saving all capture artifacts.
for name in ('world_camera.png', 'camera.json', 'ground_truth.json', 'scene_state.json'):
    if not (out / name).is_file():
        raise RuntimeError('Missing capture artifact: ' + name)
