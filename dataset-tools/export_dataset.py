"""Export projected cube boxes and world poses; no detector predictions used."""
import argparse
import hashlib
import itertools
import json
import math
import shutil
from pathlib import Path

CLASSES = {'yellow': 0, 'blue': 1, 'red': 2}

def rotate(v, q):
    # Isaac get_world_pose returns scalar-first quaternion (w,x,y,z).
    w, x, y, z = q
    norm = math.sqrt(sum(t*t for t in q))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError('Invalid saved orientation')
    w, x, y, z = [t / norm for t in q]
    a, b, c = v
    return [(1-2*y*y-2*z*z)*a+(2*x*y-2*z*w)*b+(2*x*z+2*y*w)*c,
            (2*x*y+2*z*w)*a+(1-2*x*x-2*z*z)*b+(2*y*z-2*x*w)*c,
            (2*x*z-2*y*w)*a+(2*y*z+2*x*w)*b+(1-2*x*x-2*y*y)*c]

def project(point, camera):
    matrix = camera['camera_to_world']
    delta = [point[i]-matrix[3][i] for i in range(3)]
    local = [sum(matrix[i][j]*delta[j] for j in range(3)) for i in range(3)]
    if local[2] >= -1e-9:
        raise ValueError('Object behind camera')
    f = camera['focal_length_mm']
    tx = camera['horizontal_aperture_mm'] / (2*f)
    ty = camera['vertical_aperture_mm'] / (2*f)
    return [(local[0]/(-local[2])/tx+1)/2,
            (1-local[1]/(-local[2])/ty)/2]

def main():
    from PIL import Image
    p = argparse.ArgumentParser()
    p.add_argument('--samples', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--edge-m', type=float, required=True)
    a = p.parse_args()
    if not 0 < a.edge_m < 0.1:
        raise ValueError('Specify verified cube edge in meters')
    if a.out.exists():
        raise SystemExit('Output already exists; choose a fresh version')
    scenes = sorted(a.samples.glob('varied-*'))
    if not scenes:
        raise SystemExit('No varied-* scenes; render pilots first')
    records = []
    for folder in scenes:
        camera = json.loads((folder/'camera.json').read_text())
        gt = json.loads((folder/'ground_truth.json').read_text())
        state = json.loads((folder/'scene_state.json').read_text())
        generation = json.loads((folder/'generation.json').read_text())
        image_path = folder/'world_camera.png'
        with Image.open(image_path) as image:
            image.load()
            if list(image.size) != camera['resolution'] or image.mode != 'RGB':
                raise ValueError('Image/calibration mismatch: '+folder.name)
        group = str(generation['seed'])
        bucket = int(hashlib.sha256(group.encode()).hexdigest()[:8], 16) % 10
        split = 'test' if bucket == 0 else 'val' if bucket == 1 else 'train'
        rows, poses = [], []
        cubes = [o for o in gt['objects'] if o['category'] == 'cube']
        if len(cubes) != 6:
            raise ValueError('Expected six cubes: '+folder.name)
        for obj in cubes:
            pose = state['objects'][obj['name']]
            points = []
            for signs in itertools.product((-1,1), repeat=3):
                # Starter SortCube root is at the bottom-face centre, not centre.
                local = [signs[0]*a.edge_m/2, signs[1]*a.edge_m/2,
                         (signs[2]+1)*a.edge_m/2]
                offset = rotate(local, pose['orientation'])
                points.append(project([pose['position'][i]+offset[i] for i in range(3)], camera))
            xmin, ymin = [min(v[i] for v in points) for i in range(2)]
            xmax, ymax = [max(v[i] for v in points) for i in range(2)]
            if not all(math.isfinite(v) for v in (xmin,ymin,xmax,ymax)):
                raise ValueError('Non-finite box')
            if not (0 <= xmin < xmax <= 1 and 0 <= ymin < ymax <= 1):
                raise ValueError('Clipped cube: '+folder.name)
            rows.append(f"{CLASSES[obj['colour']]} {(xmin+xmax)/2:.9f} {(ymin+ymax)/2:.9f} {xmax-xmin:.9f} {ymax-ymin:.9f}")
            poses.append(dict(name=obj['name'], colour=obj['colour'],
                              world_position_m=pose['position'],
                              world_orientation_wxyz=pose['orientation'],
                              bbox_xyxy_normalized=[xmin,ymin,xmax,ymax]))
        records.append((folder, split, rows, poses, generation))
    # All source validation happens before creating an export.
    a.out.mkdir(parents=True)
    manifest = []
    for folder, split, rows, poses, generation in records:
        for kind in ('images','labels','poses'):
            (a.out/kind/split).mkdir(parents=True, exist_ok=True)
        shutil.copy2(folder/'world_camera.png', a.out/'images'/split/(folder.name+'.png'))
        (a.out/'labels'/split/(folder.name+'.txt')).write_text('\n'.join(rows)+'\n')
        (a.out/'poses'/split/(folder.name+'.json')).write_text(json.dumps(poses,indent=2))
        manifest.append(dict(scene=folder.name, split=split, generation=generation,
                             image_sha256=hashlib.sha256((folder/'world_camera.png').read_bytes()).hexdigest()))
    config = f'path: {a.out.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: yellow\n  1: blue\n  2: red\n'
    (a.out/'dataset.yaml').write_text(config)
    (a.out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    counts = {s:sum(r['split']==s for r in manifest) for s in ('train','val','test')}
    print('Exported scenes:', counts)
    print('Projected AMODAL boxes: visibility/occlusion still requires QA.')
    if min(counts.values()) == 0:
        print('NOT TRAINING READY: at least one split is empty; generate more scenes.')

if __name__ == '__main__':
    main()
