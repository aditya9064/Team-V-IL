"""Fixed fixture intervention and read-only comparison; never a control target."""
import math
import numpy as np

def fixture_offset(condition):
    if condition=='present':return np.zeros(3)
    if condition=='removed':return np.array([1.,0.,0.])
    raise ValueError('Unknown support condition')

def classify_terminal(r):
    if r.get('diagnostic',{}).get('lowering_started') is not True:return 'INCONCLUSIVE_BEFORE_LOWERING'
    if r.get('status')=='DIAGNOSTIC_LOWERING_COMPLETED':
        if r.get('unexpected_contacts') or r.get('contact_monitor_errors'):raise ValueError('Completion with strict violation')
        return 'LOWERING_COMPLETED_WITHOUT_STRICT_STOP'
    if r.get('status')=='FAILED':
        return 'STRICT_STOP_DURING_LOWERING' if r.get('diagnostic_lower_trace') else 'INCONCLUSIVE_MISSING_TRACE'
    return 'INCONCLUSIVE_NONTERMINAL'

def comparison_pose(r):
    shot=next(c for c in r['checkpoints'] if c['phase']=='1_base_carry')
    b=shot['poses']['base'];tip=np.asarray(shot['tip_robot_m'],float)+[0,0,r['diagnostic']['table_top_z_m']]
    return dict(relative_mm=(np.asarray(b['position_world_m'])-tip)*1000,tip_mm=np.asarray(shot['tip_robot_m'])*1000,
                tilt=b['tilt_deg'],yaw=b['yaw_deg'],jaws_mm={n:v*1000 for n,v in shot['jaw_joints'].items()})

def comparable(a,b):
    x,y=comparison_pose(a),comparison_pose(b)
    v=dict(tcp_difference_mm=float(np.linalg.norm(x['tip_mm']-y['tip_mm'])),
           held_relative_difference_mm=float(np.linalg.norm(x['relative_mm']-y['relative_mm'])),
           tilt_difference_deg=abs(x['tilt']-y['tilt']),yaw_difference_deg=abs((x['yaw']-y['yaw']+180)%360-180),
           maximum_jaw_difference_mm=max(abs(x['jaws_mm'][n]-y['jaws_mm'][n]) for n in x['jaws_mm']))
    ceilings=dict(tcp_difference_mm=.1,held_relative_difference_mm=.15,tilt_difference_deg=.25,yaw_difference_deg=.25,maximum_jaw_difference_mm=.05)
    if not all(math.isfinite(n) for n in v.values()):raise ValueError('Nonfinite comparison')
    return dict(values=v,ceilings=ceilings,comparable=all(v[k]<=ceilings[k] for k in v))

def predict_free_held_placement(carry,lower_goal_robot,table_top_z,support_top_z,base_vertices_mm,cad_tip_offset_mm):
    """Read-only prediction assuming fixed tool orientation and unchanged held transform."""
    b=carry['poses']['base'];p=np.asarray(b['position_world_m'],float);tip=np.asarray(carry['tip_robot_m'],float);goal=np.asarray(lower_goal_robot,float)
    vertices=np.asarray(base_vertices_mm,float)*.001;q=np.asarray(b['orientation_wxyz'],float)
    if not all(np.all(np.isfinite(v)) for v in [p,tip,goal,vertices,q]) or q.shape!=(4,) or abs(np.linalg.norm(q)-1)>1e-3:raise ValueError('Nonfinite/invalid diagnostic pose')
    w,x,y,z=q;rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    predicted=p+(goal-tip);held=(tip[2]+table_top_z-p[2])*1000
    return dict(assumption='Tool orientation and held-part transform unchanged; score-only, never a command correction',captured_tip_above_base_origin_mm=float(held),cad_tip_above_base_origin_mm=float(cad_tip_offset_mm),captured_minus_cad_mm=float(held-cad_tip_offset_mm),requested_tip_above_support_mm=float((goal[2]+table_top_z-support_top_z)*1000),predicted_free_base_origin_world_m=predicted.tolist(),predicted_base_origin_gap_to_support_mm=float((predicted[2]-support_top_z)*1000),predicted_lowest_base_gap_to_support_mm=float((np.min(vertices@rot[2,:]+predicted[2])-support_top_z)*1000))
