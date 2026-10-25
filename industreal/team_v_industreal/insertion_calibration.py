"""Frozen ideal-simulator calibration for a diagnostic controller, never a policy."""
import torch
from .control import quaternion_multiply,quaternion_rotate


def inverse(q):
    return torch.cat((q[...,:1],-q[...,1:]),dim=-1)/q.square().sum(-1,keepdim=True)


def calibrate(tcp_pos,tcp_quat,peg_pos,peg_quat,fixture_pos,fixture_quat):
    # Called once after privileged episode setup; only this frozen record may
    # influence targets. Live part truth is reserved for STOP/scoring.
    grasp_q=quaternion_multiply(inverse(tcp_quat),peg_quat)
    grasp_p=quaternion_rotate(inverse(tcp_quat),peg_pos-tcp_pos)
    z=torch.tensor([0.,0.,1.],device=tcp_pos.device,dtype=tcp_pos.dtype).repeat(len(tcp_pos),1)
    peg_axis=quaternion_rotate(peg_quat,z);fixture_axis=quaternion_rotate(fixture_quat,z)
    dot=(peg_axis*fixture_axis).sum(-1,keepdim=True)
    if (dot<0).any():raise ValueError('Benchmark requires initially upward peg')
    swing=torch.cat((1.+dot,torch.linalg.cross(peg_axis,fixture_axis,dim=-1)),dim=-1)
    swing=swing/torch.linalg.vector_norm(swing,dim=-1,keepdim=True)
    goal_peg_q=quaternion_multiply(swing,peg_quat)
    goal_tcp_q=quaternion_multiply(goal_peg_q,inverse(grasp_q))
    return {'grasp_position_tcp_m':grasp_p.detach().clone(),
            'grasp_quaternion_wxyz':grasp_q.detach().clone(),
            'fixture_position_m':fixture_pos.detach().clone(),
            'fixture_axis':fixture_axis.detach().clone(),
            'goal_tcp_quaternion_wxyz':goal_tcp_q.detach().clone()}


def target_tcp(calibration,peg_root_height_m):
    q=calibration['goal_tcp_quaternion_wxyz']
    peg=calibration['fixture_position_m']+float(peg_root_height_m)*calibration['fixture_axis']
    return peg-quaternion_rotate(q,calibration['grasp_position_tcp_m']),q.clone()


def interpolate(start_pos,start_quat,end_pos,end_quat,fraction):
    if not 0<=fraction<=1:raise ValueError('Interpolation fraction outside0..1')
    end_quat=torch.where((start_quat*end_quat).sum(-1,keepdim=True)<0,-end_quat,end_quat)
    q=(1-fraction)*start_quat+fraction*end_quat
    return (1-fraction)*start_pos+fraction*end_pos,q/torch.linalg.vector_norm(q,dim=-1,keepdim=True)


def release_state(joints,forces,requested):
    """Measured jaw displacement and per-pair contact load, never command alone."""
    import math
    opening=[.006+float(joints[6]),.006-float(joints[7])]
    loads={name:math.sqrt(sum(float(v)**2 for v in forces.get(name,[0.,0.,0.]))) for name in ('JawL','JawR')}
    values=opening+list(loads.values())
    if not all(math.isfinite(v) for v in values):raise ValueError('Invalid release measurement')
    return {'opening_per_jaw_m':opening,'grip_force_n':loads,
            'released':bool(requested and min(opening)>=.0055 and max(loads.values())<=.001)}


def gradual_release_opening(grasp_opening_m, fraction):
    """Smooth finite jaw command, bounded by the original6mm per-jaw stroke."""
    import math
    if not math.isfinite(grasp_opening_m) or not 0<=grasp_opening_m<=.006:
        raise ValueError('Invalid grasp opening')
    if not math.isfinite(fraction) or not 0<=fraction<=1:
        raise ValueError('Invalid release progress')
    smooth=fraction*fraction*(3.-2.*fraction)
    return grasp_opening_m+(.006-grasp_opening_m)*smooth
