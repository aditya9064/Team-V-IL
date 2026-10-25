"""Six-axis position adapter. Tensor-only; contains no simulator or hardware IO."""
import torch


def _finite(name, value):
    if not torch.isfinite(value).all():
        raise ValueError(f"Non-finite {name}")


def quaternion_multiply(a, b):
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack((aw*bw-ax*bx-ay*by-az*bz,
                        aw*bx+ax*bw+ay*bz-az*by,
                        aw*by-ax*bz+ay*bw+az*bx,
                        aw*bz+ax*by-ay*bx+az*bw), -1)


def quaternion_rotate(q, vector):
    _finite("quaternion", q)
    norm = torch.linalg.vector_norm(q, dim=-1, keepdim=True)
    if (norm < 1e-8).any():
        raise ValueError("Zero quaternion")
    q = q / norm
    xyz = q[..., 1:]
    uv = torch.linalg.cross(xyz, vector, dim=-1)
    return vector + 2 * (q[..., :1] * uv + torch.linalg.cross(xyz, uv, dim=-1))


def pose_error(position, quaternion, target_position, target_quaternion):
    for name, value in [("position", position), ("quaternion", quaternion),
                        ("target position", target_position), ("target quaternion", target_quaternion)]:
        _finite(name, value)
    if (torch.linalg.vector_norm(quaternion, dim=-1) < 1e-8).any() or (torch.linalg.vector_norm(target_quaternion, dim=-1) < 1e-8).any():
        raise ValueError("Zero quaternion")
    q = quaternion / torch.linalg.vector_norm(quaternion, dim=-1, keepdim=True)
    target = target_quaternion / torch.linalg.vector_norm(target_quaternion, dim=-1, keepdim=True)
    inverse = q.clone()
    inverse[..., 1:] *= -1
    error = quaternion_multiply(target, inverse)
    error = torch.where(error[..., :1] < 0, -error, error)
    norm = torch.linalg.vector_norm(error[..., 1:], dim=-1, keepdim=True)
    angle = 2 * torch.atan2(norm, error[..., :1].clamp_min(0))
    rotation = error[..., 1:] * torch.where(norm > 1e-8, angle / norm.clamp_min(1e-8), 2.0)
    return torch.cat((target_position-position, rotation), -1)


def shift_jacobian(jacobian, world_offset):
    """Translate a wrist Jacobian to a TCP without changing rotational axes."""
    if jacobian.shape[-2:] != (6, 6) or world_offset.shape != jacobian.shape[:-2]+(3,):
        raise ValueError("Expected (...,6,6) Jacobian and (...,3) TCP offset")
    result = jacobian.clone()
    omega = jacobian[..., 3:, :].transpose(-1, -2)
    delta = torch.linalg.cross(omega, world_offset.unsqueeze(-2).expand_as(omega), dim=-1)
    result[..., :3, :] += delta.transpose(-1, -2)
    return result


def joint_position_step(joints, jacobian, error, lower, upper, max_step_rad=0.005,
                        damping=0.05, max_translation_m=0.001, max_rotation_rad=0.01):
    """Damped least-squares position step with finite, pose and joint bounds."""
    if joints.shape[-1] != 6 or jacobian.shape != joints.shape[:-1]+(6, 6) or error.shape != joints.shape:
        raise ValueError("Expected six arm joints, a 6x6 Jacobian and six pose errors")
    for name, value in [("joints", joints), ("Jacobian", jacobian), ("pose error", error), ("lower", lower), ("upper", upper)]:
        _finite(name, value)
    parameters=torch.tensor([damping,max_step_rad,max_translation_m,max_rotation_rad])
    if not torch.isfinite(parameters).all() or (parameters<=0).any() or (lower >= upper).any():
        raise ValueError("Invalid joint bounds or DLS parameters")
    if ((joints < lower-1e-5) | (joints > upper+1e-5)).any():
        raise ValueError("Measured arm state is outside joint limits")
    bounded = error.clone()
    for section, maximum in [(slice(0, 3), max_translation_m), (slice(3, 6), max_rotation_rad)]:
        norm = torch.linalg.vector_norm(bounded[..., section], dim=-1, keepdim=True)
        bounded[..., section] *= (maximum/norm.clamp_min(1e-12)).clamp_max(1)
    jt = jacobian.transpose(-1, -2)
    matrix = jacobian @ jt + damping*damping*torch.eye(6, device=joints.device, dtype=joints.dtype)
    delta = (jt @ torch.linalg.solve(matrix, bounded.unsqueeze(-1))).squeeze(-1)
    target = joints + delta.clamp(-max_step_rad, max_step_rad)
    return torch.maximum(lower, torch.minimum(upper, target))


def jaw_targets(opening_per_jaw_m):
    if not 0 <= opening_per_jaw_m <= 0.006:
        raise ValueError("SMC opening must be between 0 and 6 mm per jaw")
    displacement = 0.006 - opening_per_jaw_m
    return (-displacement, displacement)


def integrate_joint_target(measured,previous,increment_target,lower,upper,gain=.15,maximum_offset_rad=.005):
    """Bounded command integration for the diagnostic position-servo adapter.

    Compensates static load bias without changing authored joint drives.
    The proposed command is limited relative to measured joints to prevent
    integrator windup against contact or a position limit.
    """
    if any(x.shape!=measured.shape for x in (previous,increment_target,lower,upper)):
        raise ValueError('Incompatible command integration shapes')
    for name,value in [('measured',measured),('previous',previous),('increment target',increment_target),('lower',lower),('upper',upper)]:_finite(name,value)
    if not 0<gain<=1 or not 0<maximum_offset_rad<.1:raise ValueError('Invalid integration limits')
    if (lower>=upper).any():raise ValueError('Invalid integration joint bounds')
    lo=torch.maximum(lower,measured-maximum_offset_rad);hi=torch.minimum(upper,measured+maximum_offset_rad)
    return torch.maximum(lo,torch.minimum(hi,previous+gain*(increment_target-measured)))
