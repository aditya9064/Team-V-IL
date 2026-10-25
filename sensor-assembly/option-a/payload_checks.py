"""Independent pose checks for a loose PCB, never a motion correction."""
import math
import numpy as np
from assembly_checks import role_pair


def rotation(quaternion):
    q=np.asarray(quaternion,float)
    if q.shape!=(4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1)>1e-3:
        raise ValueError('Invalid payload quaternion')
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def payload_score(carrier,pcb,floor_m=.002):
    a=np.asarray(carrier['position_world_m'],float);b=np.asarray(pcb['position_world_m'],float)
    if a.shape!=(3,) or b.shape!=(3,) or not np.all(np.isfinite([a,b])):
        raise ValueError('Invalid payload position')
    rc=rotation(carrier['orientation_wxyz']);rp=rotation(pcb['orientation_wxyz'])
    local=rc.T@(b-a);relative=rc.T@rp
    score={'relative_position_carrier_m':local.tolist(),'xy_mm':float(np.linalg.norm(local[:2])*1000),
           'z_mm':float(abs(local[2]-floor_m)*1000),
           'tilt_deg':math.degrees(math.acos(float(np.clip(relative[2,2],-1,1)))),
           'yaw_deg':abs(math.degrees(math.atan2(relative[1,0],relative[0,0])))}
    score['pass']=score['xy_mm']<=.75 and score['z_mm']<=.35 and score['tilt_deg']<=1 and score['yaw_deg']<=1
    return score


def initial_supports(preplaced_base=True):
    supports={role_pair(('carrier','floor'),('pcb','substrate')),role_pair(('carrier','pcb_keeper'),('pcb','substrate')),
            role_pair(('carrier','wall'),('pcb','substrate'))}
    if preplaced_base:supports.add(role_pair(('nest','floor'),('base','floor')))
    return supports


def placement_supports():
    return {
        'base':{role_pair(('base',r),('nest','floor')) for r in ['floor','ear']},
        'carrier':{role_pair(('base','floor'),('carrier','floor'))},
        'frame':{role_pair(('frame',r),('pcb','substrate')) for r in ['ring','ear']} |
                {role_pair(('frame',r),('carrier','wall')) for r in ['ring','ear']},
        'cover':{role_pair(('cover','ear'),('base','floor'))}}


def horizontal_seating_normals(normals):
    """Extra guard for the CAD-proven tab-bottom/base-floor seating pair."""
    n=np.asarray(normals,float)
    return bool(n.ndim==2 and n.shape[1]==3 and len(n)>0 and np.all(np.isfinite(n))
                and np.all(np.abs(np.linalg.norm(n,axis=1)-1)<1e-3)
                and np.all(np.abs(n[:,2])>=math.cos(math.radians(1))))


def horizontal_support_pairs():
    return {role_pair(('carrier','floor'),('base','floor')),role_pair(('base','floor'),('nest','floor')),role_pair(('base','ear'),('nest','floor')),role_pair(('cover','ear'),('base','floor'))}


def initial_supports_for_case(case):
    if case not in ('loaded-carrier','full-assembly'):raise ValueError('Unknown mechanical case')
    return initial_supports(preplaced_base=case=='loaded-carrier')
