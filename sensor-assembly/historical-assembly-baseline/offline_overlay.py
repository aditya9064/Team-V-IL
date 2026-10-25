"""Replay archived RGB using only separated camera/TCP/CAD input files."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from lid_pose import estimate, render_mask, _mesh, _silhouette_points, project, _world


def load_inputs(evidence, mesh_path, prefix='3_cover_carry'):
    evidence = Path(evidence)
    models = json.loads((evidence/'camera-models.json').read_text())
    inputs = json.loads((evidence/'carry-input.json').read_text())
    nominal = np.asarray(inputs['tip_robot_m']) + [0, 0, inputs['table_top_z_m']-inputs['nominal_tcp_above_part_bottom_mm']*.001]
    images = {}
    for name in models:
        image = cv2.imread(str(evidence/(prefix+'-'+name+'.png')))
        if image is None:
            raise ValueError('Missing archived image '+name)
        images[name] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mesh = json.loads(Path(mesh_path).read_text())['parts']['cover']['visual']
    return images, models, mesh, nominal


def overlays(images, models, mesh, fit, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    p, R = np.asarray(fit['position_world_m']), np.asarray(fit['rotation_world'])
    for name, original in images.items():
        rgb = original.copy()
        # Display actual selected CAD measurement points, not AABB substitutes.
        points = _silhouette_points(_mesh(mesh), mesh, p, R, models[name])
        uv = project(_world(points, R, p), models[name])
        for u, v in uv:
            cv2.circle(rgb, (round(u), round(v)), 1, (255, 40, 20), -1)
        cv2.imwrite(str(output/(name+'-fit.png')), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence', required=True)
    parser.add_argument('--mesh', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--prefix', default='3_cover_carry')
    args = parser.parse_args()
    images, models, mesh, nominal = load_inputs(args.evidence, args.mesh, args.prefix)
    fit = estimate(images, models, mesh, nominal)
    fit['offline_input_contract'] = 'No report/part pose loaded; RGB + separated calibration/TCP/CAD only'
    fit['estimator_sha256'] = hashlib.sha256(Path(__file__).with_name('lid_pose.py').read_bytes()).hexdigest()
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    overlays(images, models, mesh, fit, output)
    (output/'fit.json').write_text(json.dumps(fit, indent=2)+'\n')
    print(json.dumps({'fit_accepted': True, 'offset_mm_and_xyz_deg': fit['offset_mm_and_xyz_deg'],
                      'clearance_qualified': False, 'output': str(output)}, indent=2))


if __name__ == '__main__':
    main()
