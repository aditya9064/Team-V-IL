"""Synthetic label QA: bottom-origin cube projection versus rendered pixels.

Uses USD state only to generate/evaluate synthetic labels, never robot commands.
Rejects uncertain/occluded cubes; does not infer labels for arbitrary assets.
"""
import itertools
import json
import math
from pathlib import Path
import cv2
import numpy as np

CLASSES = {'yellow': 0, 'blue': 1, 'red': 2}


def rotate(v, q):
    q = np.asarray(q, float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('Invalid scalar-first quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                     [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                     [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]]) @ v


def project(camera, point):
    matrix = np.array(camera['camera_to_world'], float)
    if matrix.shape != (4,4) or not np.all(np.isfinite(matrix)):
        raise ValueError('Invalid camera transform')
    p = matrix[:3,:3] @ (np.array(point)-matrix[3,:3])
    if p[2] >= -1e-9:
        raise ValueError('Behind camera')
    w, h = camera['resolution']
    tx = camera['horizontal_aperture_mm']/(2*camera['focal_length_mm'])
    ty = camera['vertical_aperture_mm']/(2*camera['focal_length_mm'])
    return np.array([(1+p[0]/-p[2]/tx)*w/2, (1-p[1]/-p[2]/ty)*h/2])


def cube_hull(camera, pose, edge):
    points = [project(camera, np.array(pose['position']) + rotate(
        np.array([sx*edge/2, sy*edge/2, sz*edge]), pose['orientation']))
        for sx, sy, sz in itertools.product((-1,1), (-1,1), (0,1))]
    return cv2.convexHull(np.array(points,np.float32)).reshape(-1,2)


def raster_hull(hull, shape):
    # Sample at pixel centres; avoid inflating small objects by inclusive fill.
    yy, xx = np.indices(shape)
    samples = np.stack([xx+.5, yy+.5],-1)
    edges = np.roll(hull,-1,axis=0)-hull
    delta = samples[:,:,None,:]-hull
    cross = edges[:,0]*delta[:,:,:,1]-edges[:,1]*delta[:,:,:,0]
    return np.all(cross>=0,axis=-1) | np.all(cross<=0,axis=-1)


def colour_mask(image, colour):
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    if colour == 'yellow':
        hue = (h>=12)&(h<=42)
    elif colour == 'blue':
        hue = (h>=95)&(h<=135)
    elif colour == 'red':
        hue = (h<=12)|(h>=170)
    else:
        raise ValueError('Unknown colour')
    return (hue & (s>=45) & (v>=35)).astype(np.uint8)


def verify_labels(folder, generation, edge=.01, write_outputs=True):
    folder=Path(folder)
    image=cv2.imread(str(folder/'world_camera.png'))
    if image is None or image.shape != (2160,3840,3):
        raise ValueError('Missing/invalid 4K image')
    if not np.all(image.max(axis=(0,1))>image.min(axis=(0,1))):
        raise ValueError('Constant channel')
    cam=json.loads((folder/'camera.json').read_text())
    truth=json.loads((folder/'ground_truth.json').read_text())
    state=json.loads((folder/'scene_state.json').read_text())
    if cam['resolution'] != [3840,2160]:
        raise ValueError('Image/calibration mismatch')
    if 'table_camera_config' in generation:
        requested=generation['table_camera_config']
        actual_eye=np.array(cam['camera_to_world'])[3,:3]
        if not np.allclose(actual_eye,requested['eye'],atol=1e-5,rtol=0):
            raise ValueError('Saved camera eye differs from requested variation')
        if abs(cam['focal_length_mm']-requested['focal_length'])>1e-5:
            raise ValueError('Saved camera focal length differs from requested variation')
    cubes=[o for o in truth['objects'] if o['category']=='cube']
    expected=generation['cube_layout']
    if len(cubes) != len(expected) or len(cubes) != generation['cube_count']:
        raise ValueError('Wrong inventory')
    names=[o['name'] for o in truth['objects']]
    if len(set(names)) != len(names) or set(names) != set(state['objects']):
        raise ValueError('Wrong scene-state inventory')
    overlay=image.copy(); records=[]; rows=[]; selected=[]
    for i,(obj,entry) in enumerate(zip(cubes,expected)):
        pose=state['objects'][obj['name']]
        if obj['name'] != f'SortCube{i}' or obj['colour'] != entry[2]:
            raise ValueError('Wrong identity/colour')
        if not np.allclose(pose['position'][:2],entry[:2],atol=1e-6,rtol=0):
            raise ValueError('Stage/layout position mismatch')
        if not np.allclose(obj['xyz'][:2],entry[:2],atol=1e-8,rtol=0):
            raise ValueError('Label/layout position mismatch')
        q=pose['orientation']
        if abs(q[1])+abs(q[2])>1e-5 or abs(pose['position'][2]-cam['table_top_z'])>1e-5:
            raise ValueError('Cube is not upright/on the known surface')
        actual_yaw=math.degrees(2*math.atan2(q[3],q[0]))
        wanted=generation['cube_yaws_deg'][i]
        # USD's float quaternion extraction quantizes near-zero angles (the
        # known capture reports 0 for an authored .010deg rotation). Use the
        # saved quaternion for geometry; allow .05deg numerical agreement.
        if abs((actual_yaw-wanted+180)%360-180)>.05 or abs(obj['yaw_deg']-wanted)>1e-8:
            raise ValueError('Stage/label yaw mismatch')
        hull=cube_hull(cam,pose,edge)
        lo=np.floor(hull.min(0)).astype(int)-4
        hi=np.ceil(hull.max(0)).astype(int)+5
        if np.any(lo<0) or hi[0]>image.shape[1] or hi[1]>image.shape[0]:
            raise ValueError('Clipped cube')
        x0,y0=lo; x1,y1=hi
        crop=image[y0:y1,x0:x1]
        predicted=raster_hull(hull-lo,crop.shape[:2])
        mask=colour_mask(crop,obj['colour'])
        n, labels, stats, centres=cv2.connectedComponentsWithStats(mask,8)
        candidates=[j for j in range(1,n) if stats[j,4]>=20]
        if not candidates:
            raise ValueError('No matching rendered colour')
        component=max(candidates,key=lambda j: np.logical_and(labels==j,predicted).sum())
        observed=labels==component
        iou=float(np.logical_and(predicted,observed).sum()/max(1,np.logical_or(predicted,observed).sum()))
        if iou<.85:
            raise ValueError(f"{obj['name']} rendered silhouette IoU {iou:.3f} below .85")
        ys,xs=np.nonzero(observed)
        box=[int(x0+xs.min()),int(y0+ys.min()),int(x0+xs.max()+1),int(y0+ys.max()+1)]
        if any(max(box[0],b[0])<min(box[2],b[2]) and max(box[1],b[1])<min(box[3],b[3]) for b in selected):
            raise ValueError('Overlapping cube boxes require an occlusion-aware exporter')
        selected.append(box)
        w,h=cam['resolution']; a,b,c,d=box
        rows.append(f"{CLASSES[obj['colour']]} {(a+c)/2/w:.9f} {(b+d)/2/h:.9f} {(c-a)/w:.9f} {(d-b)/h:.9f}")
        records.append(dict(name=obj['name'],colour=obj['colour'],class_id=CLASSES[obj['colour']],
            bbox_xyxy_pixels=box,projected_hull_pixels=hull.tolist(),silhouette_iou=iou,
            world_origin_bottom_m=pose['position'],world_orientation_wxyz=q,
            world_center_m=(np.array(pose['position'])+rotate(np.array([0,0,edge/2]),q)).tolist()))
        cv2.rectangle(overlay,(a,b),(c-1,d-1),(0,255,0),2)
        cv2.putText(overlay,obj['colour'],(a,b-6),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
    summary=dict(seed=generation['seed'],cube_count=len(cubes),objects=records,passed=True,
        bbox_source='rendered_colour_component_verified_against_bottom_origin_USD_geometry',
        occlusion_policy='reject silhouette IoU below .85 or overlapping cube boxes',
        manual_review_complete=False)
    if write_outputs:
        (folder/'cube_labels.txt').write_text('\n'.join(rows)+('\n' if rows else ''))
        (folder/'annotation_qa.json').write_text(json.dumps(summary,indent=2)+'\n')
        if not cv2.imwrite(str(folder/'labels_overlay.png'),overlay):
            raise ValueError('Overlay write failed')
    result=dict(cube_count=len(cubes),minimum_silhouette_iou=min((r['silhouette_iou'] for r in records),default=None))
    if not write_outputs:
        result.update(annotations=summary,yolo_rows=rows)
    return result
