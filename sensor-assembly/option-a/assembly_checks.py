"""Pure contact checks, shared by the simulation and its negative controls."""
import math


def role_pair(a,b):return tuple(sorted([tuple(a),tuple(b)]))


def classify_contact(actors,colliders,roles,state,support_pairs,robot_root):
    for actor,role in zip(actors,roles):
        if actor.startswith('/World/Assembly_') and role is None:raise ValueError('Unmapped assembly collider')
    if actors[0]==actors[1]:return True,'same_rigid_body'
    if all(r is not None for r in roles):
        allowed=role_pair(*roles) in support_pairs
        return allowed,'part_support' if allowed else 'part_interference'
    for side in (0,1):
        other=1-side;r=roles[side]
        if r is None:continue
        if colliders[other]=='/World/Mat' and r[1] in ('floor','ear','panel','ring'):
            allowed=r[0]!=state['active'] or state['allow_active_table']
            return allowed,'table_support' if allowed else 'active_part_table'
        if actors[other].startswith(robot_root+'/Jaw'):
            allowed=r[0]==state['active'] and state['allow_grip'] and r[1] in ('grip_stem','grip_cap','grip_support')
            return allowed,'grip_contact' if allowed else 'finger_interference'
    return False,'unintended'


def contact_violation(allowed,minsep_m,impulse_Ns,maximum_intended_penetration_mm):
    if not math.isfinite(minsep_m) or not math.isfinite(impulse_Ns):raise ValueError('Non-finite contact data')
    return ((not allowed and (minsep_m < -1e-6 or impulse_Ns>1e-8))
            or (allowed and minsep_m*1000 < -maximum_intended_penetration_mm))


def grasp_command_m(contact_per_jaw_mm,extra_travel_mm,penetration_limit_mm):
    """Nominal grip preload must leave margin below the collision stop limit."""
    values=(contact_per_jaw_mm,extra_travel_mm,penetration_limit_mm)
    if not all(math.isfinite(v) for v in values):raise ValueError('Non-finite grip command')
    if not 0 < extra_travel_mm <= penetration_limit_mm/2:raise ValueError('Grip preload exceeds reserved penetration margin')
    if contact_per_jaw_mm<=extra_travel_mm:raise ValueError('Grip command outside usable jaw range')
    return (contact_per_jaw_mm-extra_travel_mm)*.001
