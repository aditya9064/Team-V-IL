"""Markerless held-base estimate from calibrated RGB contours and CAD bottom edges.
No simulator part pose is accepted by this module. Dimensions are meters externally.
"""
import json,hashlib
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

def image_problem(images, models, vertices_mm, nominal_position):
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
    return nominal,views,residual

FROZEN_PROTOCOL_SHA256='b22cb0055a6f462454a0933086f3c33513de573cb5db7f918cf7755e4fbf85c1'
SOLVER_OPTIONS={'bounds':([-2,-2,-2,-3,-3,-3],[2,2,2,3,3,3]),'diff_step':.01,'x_scale':[1,1,1,1,1,1],'max_nfev':400,'ftol':1e-8,'xtol':1e-8,'gtol':1e-8}

def fixed_seed_bank():
 values=[np.zeros(6)]
 for magnitude in (.5,1.):
  for axis in range(6):
   for sign in (-1,1):
    v=np.zeros(6);v[axis]=sign*magnitude;values.append(v)
 return [v.tolist() for v in values]

def eligibility_reasons(optimizer,scores,x):
 reasons=[]
 if not optimizer['success']:reasons.append('solver_not_successful')
 for name,v in scores.items():
  if v['edge_p95_px']>2:reasons.append(name+':p95_exceeds_2px')
  if v['edge_coverage_within_2px']<.95:reasons.append(name+':coverage_below_.95')
 if np.any(np.abs(x[:3])>=1.95):reasons.append('translation_near_original_bound')
 if np.any(np.abs(x[3:])>=2.95):reasons.append('rotation_near_original_bound')
 return reasons

def select_image_only(candidates):
 bank=fixed_seed_bank()
 if len(candidates)!=25 or [c['seed_index'] for c in candidates]!=list(range(25)) or [c['seed_parameters_mm_xyz_deg'] for c in candidates]!=bank:raise ValueError('Incomplete or altered fixed25 bank')
 eligible=[]
 for c in candidates:
  if c['image_eligible']:
   if c['rejection_reasons'] or c['cost'] is None or not np.isfinite(c['cost']):raise ValueError('Invalid eligible candidate')
   eligible.append(c)
 if not candidates[0]['image_eligible']:return None
 return None if not eligible else min(eligible,key=lambda c:(c['cost'],c['seed_index']))['seed_index']

def estimate(images,models,vertices_mm,nominal_position):
 """Complete frozen25 bank, imagecost ranking only; no truth/previousfit input."""
 nominal,views,residual=image_problem(images,models,vertices_mm,nominal_position)
 candidates=[]
 for index,seed in enumerate(fixed_seed_bank()):
  c={'seed_index':index,'seed_parameters_mm_xyz_deg':seed,'offset_mm_and_xyz_deg':None,'position_world_m':None,'rotation_world':None,'cost':None,'optimizer':None,'views':None,'rms_px':None,'image_eligible':False,'rejection_reasons':[]}
  try:
   fit=least_squares(residual,np.array(seed),**SOLVER_OPTIONS);x=np.asarray(fit.x,float)
   if x.shape!=(6,) or not np.isfinite(x).all():raise ValueError('Invalid solver pose')
   errors=residual(x)
   if errors.shape!=(280,) or not np.isfinite(errors).all() or not np.isfinite(fit.cost) or fit.cost<0 or not np.isfinite(fit.optimality):raise ValueError('Invalid solver output')
   if not np.isclose(float(fit.cost),float(errors@errors/2),atol=1e-8,rtol=1e-8) or not isinstance(fit.success,(bool,np.bool_)) or not 0<int(fit.nfev)<=400 or not 0<int(fit.njev)<=400:raise ValueError('Inconsistent solver output')
   scores={}
   for i,(name,model,v,d,contour) in enumerate(views):
    e=errors[i*140:(i+1)*140];scores[name]={'edge_rms_px':float(np.sqrt(np.mean(e**2))),'edge_p95_px':float(np.percentile(e,95)),'edge_coverage_within_2px':float(np.mean(e<=2)),'observed_contour_points':len(contour)}
   optimizer={'success':bool(fit.success),'status':int(fit.status),'message':str(fit.message),'nfev':int(fit.nfev),'njev':int(fit.njev),'max_nfev':400,'cost':float(fit.cost),'optimality':float(fit.optimality)}
   reasons=eligibility_reasons(optimizer,scores,x);R=Rotation.from_euler('xyz',x[3:],degrees=True).as_matrix();position=nominal+x[:3]*.001
   c.update(offset_mm_and_xyz_deg=x.tolist(),position_world_m=position.tolist(),rotation_world=R.tolist(),cost=float(fit.cost),optimizer=optimizer,views=scores,rms_px=float(np.sqrt(np.mean(errors**2))),image_eligible=not reasons,rejection_reasons=reasons)
  except Exception as e:c['rejection_reasons']=['invalid_solver_or_output:'+type(e).__name__+':'+str(e)]
  candidates.append(c)
 selected=select_image_only(candidates)
 audit={'protocol_sha256':FROZEN_PROTOCOL_SHA256,'fixed_seed_parameters_mm_xyz_deg':fixed_seed_bank(),'solver_options':SOLVER_OPTIONS,'candidate_count':25,'original_zero_seed_gate_required':True,'original_zero_seed_gate_pass':candidates[0]['image_eligible'],'candidates':candidates,'selected_seed_index_IMAGE_ONLY':selected,'ranking':'minimum original imagecost among original eligible candidates; tie by seed_index','truth_inputs_or_selection':False,'early_stop':False,'adaptive_seed_bank':False}
 if selected is None:raise ValueError('Held-pose multistart HOLD: '+json.dumps(audit,allow_nan=False))
 c=candidates[selected]
 return {'nominal_position_m':nominal.tolist(),'image_RGB_sha256':{k:hashlib.sha256(np.ascontiguousarray(images[k]).tobytes()).hexdigest() for k in sorted(images)},'camera_model_sha256':{k:hashlib.sha256(json.dumps(models[k],sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest() for k in sorted(models)},'CAD_vertices_mm_sha256':hashlib.sha256(np.ascontiguousarray(vertices_mm,dtype=np.float64).tobytes()).hexdigest(),'position_world_m':c['position_world_m'],'rotation_world':c['rotation_world'],'offset_mm_and_xyz_deg':c['offset_mm_and_xyz_deg'],'views':c['views'],'rms_px':c['rms_px'],'function_evaluations':c['optimizer']['nfev'],'optimizer':c['optimizer'],'source':'two calibrated RGB lower-edge contours + unchanged CAD; fixed25 original-objective image-only multistart; no runtime object coordinates','multistart':audit}
