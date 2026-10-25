"""Markerless held-base estimate from calibrated RGB contours and CAD bottom edges.
No simulator part pose is accepted by this module. Dimensions are meters externally.
"""
import cv2, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.ndimage import map_coordinates

def project(points, model):
    points=np.asarray(points,float)
    x=np.asarray(model['camera_to_world'],float)
    p=np.c_[points,np.ones(len(points))]@np.linalg.inv(x)
    w,h=model['resolution'];fx=w*model['focal_length_mm']/model['horizontal_aperture_mm'];fy=h*model['focal_length_mm']/model['vertical_aperture_mm']
    if np.any(p[:,2]>=-.01):raise ValueError('CAD behind camera or inside near plane')
    return np.c_[w/2+fx*p[:,0]/-p[:,2],h/2-fy*p[:,1]/-p[:,2]]

def front_edges(vertices_mm, nominal, model):
    v=np.asarray(vertices_mm,float);lo=v.min(0);hi=v.max(0)
    if abs(lo[2])>1e-6:raise ValueError('Unexpected floor datum')
    c=np.array([[lo[0],lo[1],0],[hi[0],lo[1],0],[hi[0],hi[1],0],[lo[0],hi[1],0]])*.001
    eye=np.asarray(model['camera_to_world'])[3,:3];centre=np.asarray(nominal)
    normals=np.array([[0,-1,0],[1,0,0],[0,1,0],[-1,0,0]])
    # Far floor edges form the external lower contour when viewed from below.
    keep=np.where(normals@(eye-centre)<0)[0]
    if len(keep)!=2 or eye[2]>=centre[2]-.002:raise ValueError('Inspection cameras must see two lower front edges from below')
    # Exclude corner antialiasing, retain distributed samples along each full edge.
    return np.concatenate([c[i]+np.linspace(.04,.96,70)[:,None]*(c[(i+1)%4]-c[i]) for i in keep])

def estimate(images, models, vertices_mm, nominal_position):
    """Only RGB, static camera calibration, CAD and a bounded nominal ROI enter."""
    nominal=np.asarray(nominal_position,float)
    if set(images)!=set(models) or len(images)!=2 or nominal.shape!=(3,) or not np.isfinite(nominal).all():raise ValueError('Two views/calibration and finite nominal pose required')
    views=[]
    for name in sorted(images):
        image=np.asarray(images[name]);model=models[name];w,h=model['resolution']
        if image.shape!=(h,w,3) or image.dtype!=np.uint8:raise ValueError('Invalid RGB image')
        p=project(np.asarray(vertices_mm)*.001+nominal,model)
        x0,y0=np.maximum(np.floor(p.min(0)-60).astype(int),0);x1,y1=np.minimum(np.ceil(p.max(0)+60).astype(int),[w,h])
        r,g,b=image[:,:,0].astype(float),image[:,:,1].astype(float),image[:,:,2].astype(float)
        blue=((b>r*1.3)&(b>g*1.08)&(b-r>25)).astype(np.uint8)
        roi=np.zeros((h,w),np.uint8);roi[y0:y1,x0:x1]=blue[y0:y1,x0:x1]
        contours,_=cv2.findContours(roi,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        if not contours:raise ValueError('Held base not visible in '+name)
        contour=max(contours,key=cv2.contourArea)
        if cv2.contourArea(contour)<1000:raise ValueError('Insufficient held-base image area')
        edge=np.zeros((h,w),np.uint8);cv2.drawContours(edge,[contour],-1,1,1)
        distance=cv2.distanceTransform(1-edge,cv2.DIST_L2,cv2.DIST_MASK_PRECISE).astype(float)
        views.append((name,model,front_edges(vertices_mm,nominal,model),distance,contour))
    def residual(a):
        rot=Rotation.from_euler('xyz',a[3:],degrees=True).as_matrix();pos=nominal+a[:3]*.001
        values=[]
        for _,m,v,d,_ in views:
            uv=project(v@rot.T+pos,m)
            if (uv<0).any() or (uv[:,0]>=d.shape[1]-1).any() or (uv[:,1]>=d.shape[0]-1).any():return np.ones(280)*1000
            values.extend(map_coordinates(d,[uv[:,1],uv[:,0]],order=1,mode='constant',cval=1000))
        return np.array(values)
    fit=least_squares(residual,np.zeros(6),bounds=([-2,-2,-2,-3,-3,-3],[2,2,2,3,3,3]),diff_step=.01,x_scale=[1,1,1,1,1,1],max_nfev=200,ftol=1e-8,xtol=1e-8,gtol=1e-8)
    errors=residual(fit.x);scores={}
    for i,(name,model,v,d,contour) in enumerate(views):
        e=errors[i*140:(i+1)*140];scores[name]={'edge_rms_px':float(np.sqrt(np.mean(e**2))),'edge_p95_px':float(np.percentile(e,95)),'edge_coverage_within_2px':float(np.mean(e<=2)),'observed_contour_points':len(contour)}
    # Failed/obscured fits may never dispatch a placement.
    if not fit.success or any(v['edge_p95_px']>2 or v['edge_coverage_within_2px']<.95 for v in scores.values()) or np.any(np.abs(fit.x[:3])>=1.95) or np.any(np.abs(fit.x[3:])>=2.95):raise ValueError('Held-pose fit uncertain: '+str(scores))
    rot=Rotation.from_euler('xyz',fit.x[3:],degrees=True).as_matrix();pos=nominal+fit.x[:3]*.001
    return {'position_world_m':pos.tolist(),'rotation_world':rot.tolist(),'offset_mm_and_xyz_deg':fit.x.tolist(),'views':scores,'rms_px':float(np.sqrt(np.mean(errors**2))),'function_evaluations':fit.nfev,'source':'two calibrated RGB lower-edge contours + unchanged CAD; no runtime object coordinates'}

def placement_from_held(estimated_position, estimated_rotation, actual_tip_world, actual_tool_rotation, support_origin, vertices_mm, gap_mm=.10):
    p,R,tip,T,dest=map(lambda v:np.asarray(v,float),(estimated_position,estimated_rotation,actual_tip_world,actual_tool_rotation,support_origin))
    if any(not np.isfinite(x).all() for x in [p,R,tip,T,dest]) or not 0<gap_mm<=.1:raise ValueError('Invalid held placement input')
    if abs(np.linalg.det(R)-1)>1e-5 or abs(np.linalg.det(T)-1)>1e-5:raise ValueError('Invalid rigid orientation')
    # Fixed part-in-tool transform. Desired part is level and at recipe yaw zero.
    part_in_tool=T.T@R;origin_in_tool=T.T@(p-tip)
    desired_tool=part_in_tool.T
    # Level CAD floor lies on local z=0. Positive gap avoids a forced contact.
    desired_origin=dest+np.array([0.,0.,gap_mm*.001])
    desired_tip=desired_origin-desired_tool@origin_in_tool
    return {'tip_world_m':desired_tip.tolist(),'tool_rotation_world':desired_tool.tolist(),'part_origin_in_tool_m':origin_in_tool.tolist(),'part_rotation_in_tool':part_in_tool.tolist(),'desired_part_origin_world_m':desired_origin.tolist(),'clearance_mm':gap_mm,'method':'rigid held transform from RGB + measured FK; level base, positive floor clearance'}
