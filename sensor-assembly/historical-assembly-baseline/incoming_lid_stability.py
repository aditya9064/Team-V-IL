"""Ideal-simulation readiness veto only; no targets, waits, or simulator APIs."""
import math
import numpy as np
HOLD_STEPS=120
WINDOW_SAMPLES=25

def finite_vector(value,n):
 a=np.asarray(value,float)
 if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('Invalid finite vector')
 return a

def rotation(value):
 a=np.asarray(value,float)
 if a.shape!=(3,3) or not np.isfinite(a).all() or not np.allclose(a.T@a,np.eye(3),atol=1e-5,rtol=0) or not math.isclose(float(np.linalg.det(a)),1.,abs_tol=1e-5):raise ValueError('Invalid rotation')
 return a

def make_sample(step,sim_time,part_position_world_m,part_rotation_world,tip_world_m,tool_rotation_world,foot_corners_mm,commanded_arm_joints,issued_jaw_joint_targets_m):
 if isinstance(step,bool) or not isinstance(step,int) or step<1 or not math.isfinite(sim_time):raise ValueError('Invalid sample time/step')
 p=finite_vector(part_position_world_m,3);tip=finite_vector(tip_world_m,3);R=rotation(part_rotation_world);T=rotation(tool_rotation_world)
 corners=np.asarray(foot_corners_mm,float)
 if corners.shape!=(16,3) or not np.isfinite(corners).all():raise ValueError('Expected all16 CAD foot corners')
 arm=finite_vector(commanded_arm_joints,6)
 if set(issued_jaw_joint_targets_m)!={'jaw_l','jaw_r'} or not all(math.isfinite(v) for v in issued_jaw_joint_targets_m.values()):raise ValueError('Invalid jaw commands')
 local=((corners*.001)@R.T+p-tip)@T*1000
 return {'step':step,'sim_time':float(sim_time),'part_position_world_m':p.tolist(),'part_rotation_world':R.tolist(),'tip_world_m':tip.tolist(),'tool_rotation_world':T.tolist(),'corners_tool_mm':local.tolist(),'commanded_arm_joints':arm.tolist(),'issued_jaw_joint_targets_m':dict(issued_jaw_joint_targets_m)}

def qualify(samples,expected_arm_joints,expected_jaw_joint_targets_m,budget_mm,foot_corners_mm):
 """Always last25 of FIXED first120 plus EVERY later sample, never rolling."""
 expected_arm=finite_vector(expected_arm_joints,6)
 if not math.isfinite(budget_mm) or budget_mm<=0:raise ValueError('Invalid unchanged drift budget')
 if len(samples)<HOLD_STEPS:raise ValueError('Missing fixed hold samples')
 previous=None
 for x in samples:
  recomputed=make_sample(x['step'],x['sim_time'],x['part_position_world_m'],x['part_rotation_world'],x['tip_world_m'],x['tool_rotation_world'],foot_corners_mm,x['commanded_arm_joints'],x['issued_jaw_joint_targets_m'])
  a=np.asarray(x['corners_tool_mm'],float)
  if a.shape!=(16,3) or not np.isfinite(a).all() or not np.allclose(a,recomputed['corners_tool_mm'],atol=1e-7,rtol=0):raise ValueError('Invalid/missing/reconstructed corner data')
  if list(x['commanded_arm_joints'])!=expected_arm.tolist() or x['issued_jaw_joint_targets_m']!=expected_jaw_joint_targets_m:raise ValueError('Hold command changed')
  if isinstance(x['step'],bool) or not isinstance(x['step'],int) or not math.isfinite(x['sim_time']):raise ValueError('Invalid time/step')
  if previous is not None and (x['step']!=previous['step']+1 or abs(x['sim_time']-previous['sim_time']-1/60)>2e-6):raise ValueError('Missing/nonconsecutive physics sample')
  previous=x
 window=samples[HOLD_STEPS-WINDOW_SAMPLES:]
 points=np.asarray([x['corners_tool_mm'] for x in window]);per_corner=np.linalg.norm(points[:,None]-points[None,:],axis=3).max(axis=(0,1));maximum=float(per_corner.max())
 return {'pass':maximum<=budget_mm,'budget_mm':float(budget_mm),'maximum_pairwise_corner_diameter_mm':maximum,'per_corner_diameter_mm':per_corner.tolist(),'hold_samples':HOLD_STEPS,'fixed_window_start_index':HOLD_STEPS-WINDOW_SAMPLES,'fixed_window_samples':WINDOW_SAMPLES,'subsequent_samples':len(samples)-HOLD_STEPS,'evaluated_samples':len(window),'window_first_step':window[0]['step'],'window_last_step':window[-1]['step'],'window_first_sim_time':window[0]['sim_time'],'window_last_sim_time':window[-1]['sim_time'],'truth_used_for_targets':False,'adaptive_wait':False}
