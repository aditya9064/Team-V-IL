"""Offline QA for saved pilot captures; never supplies robot commands."""
import argparse
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageDraw
from persistent_capture import validate_capture


def project(camera, point):
    matrix = camera['camera_to_world']
    relative = [point[i] - matrix[3][i] for i in range(3)]
    local = [sum(matrix[i][j] * relative[j] for j in range(3)) for i in range(3)]
    if local[2] >= 0:
        raise ValueError('Cube behind camera')
    width, height = camera['resolution']
    tan_h = camera['horizontal_aperture_mm'] / (2 * camera['focal_length_mm'])
    tan_v = camera['vertical_aperture_mm'] / (2 * camera['focal_length_mm'])
    return ((1 + local[0] / -local[2] / tan_h) * width / 2,
            (1 - local[1] / -local[2] / tan_v) * height / 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch', type=Path)
    args = parser.parse_args()
    root = args.batch
    report = json.loads((root / 'batch_report.json').read_text())
    assert report['complete'] and report['valid_samples'] == report['requested_count']
    rows = report['samples']
    sheet = Image.new('RGB', (960, len(rows) * 110), 'white')
    draw = ImageDraw.Draw(sheet)
    first_camera = None
    layouts, hashes = set(), set()
    for index, record in enumerate(rows):
        folder = root / f"seed-{record['seed']}"
        generation = json.loads((folder / 'generation.json').read_text())
        assert generation['seed'] == record['seed']
        validate_capture(folder, generation)
        camera = json.loads((folder / 'camera.json').read_text())
        if first_camera is None:
            first_camera = camera
        assert camera == first_camera, 'Camera changed during batch'
        layouts.add(json.dumps(generation['cube_layout']))
        hashes.add(hashlib.sha256((folder / 'world_camera.png').read_bytes()).hexdigest())
        truth = json.loads((folder / 'ground_truth.json').read_text())
        state = json.loads((folder / 'scene_state.json').read_text())
        cubes = [obj for obj in truth['objects'] if obj['category'] == 'cube']
        with Image.open(folder / 'world_camera.png') as image:
            image.load()
            for column, obj in enumerate(cubes):
                bottom = state['objects'][obj['name']]['position']
                u, v = project(camera, [bottom[0], bottom[1], bottom[2]+.005])
                assert 40 <= u < image.width - 40 and 40 <= v < image.height - 40
                patch = image.crop((round(u)-40, round(v)-40, round(u)+40, round(v)+40))
                x, y = column * 160, index * 110
                sheet.paste(patch, (x+40, y+25))
                draw.text((x+5, y+5), f"{record['seed']} {obj['colour']} {obj['name']}", fill='black')
    assert len(layouts) == len(rows), 'Repeated layout inside batch'
    assert len(hashes) == len(rows), 'Repeated image inside batch'
    sheet.save(root / 'cube-patches-qa.png')
    summary = {'samples_valid': len(rows), 'camera_identical': True,
               'distinct_layouts': len(layouts), 'distinct_images': len(hashes),
               'cube_patches': 6 * len(rows),
               'visibility_review': 'manual inspection of cube-patches-qa.png required'}
    (root / 'offline_qa.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
