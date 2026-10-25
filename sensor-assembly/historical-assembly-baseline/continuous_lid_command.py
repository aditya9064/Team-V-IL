"""Prospective command geometry only: RGB-held transform, FK and fixed CAD."""
import math
import numpy as np
IK_POSITION_TOL_M=1e-6
FOOT_COMMAND_RESIDUAL_MM=.002
ORIGINAL_IK_SHA256='6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'
ORIGINAL_FK_SHA256='f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed'

def vector(x,n):
 a=np.asarray(x,float)
 if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('Invalid vector')
 return a

def rotation(x):
 a=np.asarray(x,float)
 if a.shape!=(3,3) or not np.isfinite(a).all() or not np.allclose(a.T@a,np.eye(3),atol=1e-5,rtol=0) or not math.isclose(float(np.linalg.det(a)),1.,abs_tol=1e-5):raise ValueError('Invalid rotation')
 return a

def held_foot_points(estimate,tip_world_m,tool_rotation_world,foot_corners_mm):
 R=rotation(tool_rotation_world);p=vector(estimate['position_world_m'],3);B=rotation(estimate['rotation_world']);tip=vector(tip_world_m,3);corners=np.asarray(foot_corners_mm,float)
 if corners.shape!=(16,3) or not np.isfinite(corners).all():raise ValueError('All16 feet required')
 return ((corners*.001)@B.T+p-tip)@R

def foot_residual_mm(points_tool_m,desired_tip_robot_m,desired_R,solved_tip_robot_m,solved_R):
 points=np.asarray(points_tool_m,float)
 if points.shape!=(16,3) or not np.isfinite(points).all():raise ValueError('All16 held foot points required')
 desired=points@rotation(desired_R).T+vector(desired_tip_robot_m,3)
 solved=points@rotation(solved_R).T+vector(solved_tip_robot_m,3)
 return np.linalg.norm(solved-desired,axis=1)*1000

def make_command_record(index,next_step,sim_time,desired_tip,desired_R,solved_q,solved_tip,solved_R,jaw_targets,points_tool_m,ik_ok):
 if not isinstance(index,int) or isinstance(index,bool) or index<1 or not isinstance(next_step,int) or isinstance(next_step,bool) or not math.isfinite(sim_time):raise ValueError('Invalid command sequence/time')
 if set(jaw_targets)!={'jaw_l','jaw_r'} or not all(math.isfinite(v) for v in jaw_targets.values()):raise ValueError('Invalid jaw targets')
 residual=foot_residual_mm(points_tool_m,desired_tip,desired_R,solved_tip,solved_R)
 return {'index':index,'next_step':next_step,'sim_time_before_issue':float(sim_time),'desired_tip_robot_m':vector(desired_tip,3).tolist(),'desired_rotation_world':rotation(desired_R).tolist(),'solved_q_rad':vector(solved_q,6).tolist(),'solved_tip_robot_m':vector(solved_tip,3).tolist(),'solved_rotation_world':rotation(solved_R).tolist(),'jaw_joint_targets_m':dict(jaw_targets),'per_corner_residual_mm':residual.tolist(),'maximum_foot_residual_mm':float(residual.max()),'residual_limit_mm':FOOT_COMMAND_RESIDUAL_MM,'ik_position_tol_m':IK_POSITION_TOL_M,'ik_ok':bool(ik_ok),'prospective_geometry_eligible':bool(ik_ok and residual.max()<=FOOT_COMMAND_RESIDUAL_MM),'issued':False}

def independent_fk(q,tool_offset,joint_dtype='float64'):
 """Read-only checker FK, same explicit chain as source pinned above."""
 if joint_dtype not in ('float32','float64'):raise ValueError('Unsupported joint dtype')
 vector(q,6);q=np.asarray(q,dtype=joint_dtype);R=np.eye(3);p=np.zeros(3)
 for i,(off,axis,sign) in enumerate([((0,0,.33),'z',1),((0,0,0),'y',1),((0,0,.26),'y',-1),((0,0,.02),'x',-1),((.29,0,0),'y',-1),((0,0,0),'x',-1)]):
  p=p+R@np.array(off);a=sign*q[i];c=np.cos(a);s=np.sin(a)
  A=np.array([[1,0,0],[0,c,-s],[0,s,c]]) if axis=='x' else np.array([[c,0,s],[0,1,0],[-s,0,c]]) if axis=='y' else np.array([[c,-s,0],[s,c,0],[0,0,1]])
  R=R@A
 return R,p+R@vector(tool_offset,3)
