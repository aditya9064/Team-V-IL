"""RGB-only perception. No stage, object wrappers, configured layouts or truth."""
import itertools
import cv2
import numpy as np
from backproject import CameraModel
from cube_pose import refine_pose
import scene_config as C

COLOURS=('red','blue','yellow')

def colour_mask(image,colour):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);h,s,v=cv2.split(hsv)
    gate=((h<=10)|(h>=170)) if colour=='red' else ((h>=95)&(h<=135)) if colour=='blue' else ((h>=18)&(h<=40))
    return (gate&(s>=90 if colour!='yellow' else s>=70)&(v>=45)).astype(np.uint8)

def detect(image,camera):
    """Existing HSV proposal + unchanged full-yaw silhouette refinement gates."""
    cam=CameraModel(camera);height,width=image.shape[:2]
    if [width,height]!=camera['resolution']:raise ValueError('Camera/image resolution mismatch')
    found=[];rejected=[]
    for colour in COLOURS:
        mask=colour_mask(image,colour)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area=cv2.contourArea(contour)
            if not 40<=area<=1800:continue
            (u,v),(w,h),_=cv2.minAreaRect(contour)
            if min(w,h)<=0 or max(w,h)/min(w,h)>1.8 or v<height*.50:continue
            point=cam.to_plane(u,v,z=cam.table_z+C.CUBE_EDGE/2)
            if point is None:continue
            if not (.18<=point[0]<=.52 and -.40<=point[1]<=.32):continue
            xy,yaw,fit=refine_pose(image,cam,{'colour':colour},point[:2],0.,search_half_width=45)
            item=dict(colour=colour,xy=list(map(float,xy)),yaw_deg=float(yaw),
                      silhouette_iou=float(fit),pixel_xy=[float(u),float(v)])
            if fit<.85:
                rejected.append(item);continue
            if any(np.linalg.norm(np.asarray(o['xy'])-xy)<.012 for o in found):
                raise ValueError('Duplicate/ambiguous geometric detection')
            found.append(item)
    return found,rejected

def cube_region(camera,centre,yaw=0.,padding=4):
    cam=CameraModel(camera);angle=np.deg2rad(yaw)
    rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    points=[]
    for x,y,z in itertools.product((-1,1),repeat=3):
        xy=np.asarray(centre[:2])+rot@np.array([x,y])*C.CUBE_EDGE/2
        point=cam.project([*xy,centre[2]+z*C.CUBE_EDGE/2])
        if point is None:return None
        points.append(point)
    hull=cv2.convexHull(np.asarray(points,np.float32)).reshape(-1,2)
    lo=np.floor(hull.min(0)).astype(int)-padding;hi=np.ceil(hull.max(0)).astype(int)+padding+1
    w,h=camera['resolution']
    if min(lo)<0 or hi[0]>w or hi[1]>h:return None
    x0,y0=lo;x1,y1=hi
    raster=np.zeros((y1-y0,x1-x0),np.uint8)
    cv2.fillConvexPoly(raster,np.round(hull-lo).astype(np.int32),1)
    return [int(x0),int(y0),int(x1),int(y1)],raster.astype(bool)

def colour_evidence(image,camera,centre,colour,yaw=0.,reference=None):
    """Bounded projected-volume evidence; thresholds are provisional simulation gates.

    Held cubes may be partially hidden by fingers. Require a coherent new colour
    component, not source disappearance, and compare with a same-pose empty hand.
    """
    region=cube_region(camera,centre,yaw)
    if region is None:return dict(positive=False,empty_reference_match=False,reason='volume_out_of_view')
    box,predicted=region;x0,y0,x1,y1=box;crop=image[y0:y1,x0:x1]
    masks={c:colour_mask(crop,c).astype(bool) for c in COLOURS}
    before=None
    if reference is not None:
        before={c:colour_mask(reference[y0:y1,x0:x1],c).astype(bool) for c in COLOURS}
    area=max(1,int(predicted.sum()));scores={};raw={}
    for c,mask in masks.items():
        raw[c]=int((mask&predicted).sum())
        # Tolerate at most a one-pixel empty-hand edge shift, never subtract RGB truth.
        if before is not None:
            old=cv2.dilate(before[c].astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
            mask=mask&~old
        n,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
        overlap=max((int(((labels==i)&predicted).sum()) for i in range(1,n)),default=0)
        scores[c]=overlap/area
    threshold=.35 if reference is not None else .65
    positive=scores[colour]>=threshold and scores[colour]*area>=8
    wrong=[c for c in COLOURS if c!=colour and scores[c]>=threshold and scores[c]*area>=8]
    empty=before is not None and max(scores.values())<.08
    return dict(positive=bool(positive and not wrong),wrong_colour=wrong,
                empty_reference_match=bool(empty),colour_coverage=scores,
                raw_colour_pixels=raw,projected_area_px=area,box=box,
                threshold=threshold,reference_used=reference is not None)
