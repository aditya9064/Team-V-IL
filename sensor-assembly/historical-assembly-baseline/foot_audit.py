"""Independent CAD foot/support geometry audit. No simulator or motion API imports.

Meters externally, millimeters in CAD and reported distances. Predictions accept
RGB-estimated part pose and measured tool FK. Ground-truth report auditing is a
separate, explicitly offline operation and never returns motion commands.
"""
import hashlib
import json
import math
from pathlib import Path


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def transpose(a):
    return [list(v) for v in zip(*a)]


def mv(a, b):
    return [dot(row, b) for row in a]


def mmul(a, b):
    return [[dot(row, col) for col in transpose(b)] for row in a]


def vec(v, n=3):
    if not isinstance(v, (list, tuple)) or len(v) != n:
        raise ValueError('Wrong vector shape')
    v = [float(x) for x in v]
    if not all(math.isfinite(x) for x in v):
        raise ValueError('Nonfinite vector')
    return v


def rigid_rotation(r):
    r = [vec(row) for row in r]
    if len(r) != 3:
        raise ValueError('Wrong rotation shape')
    e = mmul(transpose(r), r)
    if max(abs(e[i][j]-(i == j)) for i in range(3) for j in range(3)) > 1e-5:
        raise ValueError('Nonorthonormal rotation')
    det = dot(r[0], [r[1][1]*r[2][2]-r[1][2]*r[2][1],
                    r[1][2]*r[2][0]-r[1][0]*r[2][2],
                    r[1][0]*r[2][1]-r[1][1]*r[2][0]])
    if abs(det-1) > 1e-5:
        raise ValueError('Improper rotation')
    return r


def quaternion_rotation(q):
    w, x, y, z = vec(q, 4)
    s = w*w+x*x+y*y+z*z
    if abs(math.sqrt(s)-1) > 1e-3:
        raise ValueError('Quaternion not unit')
    return rigid_rotation([
        [1-2*(y*y+z*z)/s, 2*(x*y-z*w)/s, 2*(x*z+y*w)/s],
        [2*(x*y+z*w)/s, 1-2*(x*x+z*z)/s, 2*(y*z-x*w)/s],
        [2*(x*z-y*w)/s, 2*(y*z+x*w)/s, 1-2*(x*x+y*y)/s]])


def pose(value):
    p = vec(value['position_world_m'])
    r = (rigid_rotation(value['rotation_world']) if 'rotation_world' in value
         else quaternion_rotation(value['orientation_wxyz']))
    return p, r


def usd_matrix_pose(matrix):
    if len(matrix) != 4:
        raise ValueError('Wrong USD matrix shape')
    m = [vec(row, 4) for row in matrix]
    if max(abs(m[i][3]) for i in range(3)) > 1e-8 or abs(m[3][3]-1) > 1e-8:
        raise ValueError('Not a USD rigid row matrix')
    return {'position_world_m': m[3][:3],
            'rotation_world': rigid_rotation(transpose([row[:3] for row in m[:3]]))}


def transform(p, r, point_mm):
    return [a+b for a, b in zip(p, mv(r, [x*.001 for x in point_mm]))]


def box(piece):
    v = [vec(p) for p in piece['vertices_mm']]
    if len(v) != 8 or len({tuple(p) for p in v}) != 8:
        raise ValueError('Support must retain eight distinct actual CAD box corners')
    lo, hi = [[f(p[i] for p in v) for i in range(3)] for f in (min, max)]
    expected = {(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])}
    if {tuple(p) for p in v} != expected or any(b <= a for a, b in zip(lo, hi)):
        raise ValueError('Support is not the actual axis-aligned CAD box')
    return lo, hi


def support_geometry(mesh):
    """Match all four current lid feet to actual corresponding floor pads."""
    pairs = []
    for ci, bi in zip(range(4, 8), range(8, 12)):
        foot = mesh['parts']['cover']['colliders'][ci]
        pad = mesh['parts']['base']['colliders'][bi]
        if foot['role'] != 'ear' or pad['role'] != 'floor':
            raise ValueError('Current support identities changed')
        fl, fh = box(foot)
        pl, ph = box(pad)
        if abs(fl[2]) > 1e-8 or abs(ph[2]-3) > 1e-8:
            raise ValueError('Current foot/pad support datums changed')
        if any(abs((fl[i]+fh[i])-(pl[i]+ph[i])) > 1e-8 for i in (0, 1)):
            raise ValueError('Foot/pad centers do not match')
        corners = [[x, y, fl[2]] for x in (fl[0], fh[0]) for y in (fl[1], fh[1])]
        pairs.append({'foot_index': ci, 'pad_index': bi, 'corners_mm': corners,
                      'pad_bounds_mm': [pl, ph]})
    if len({tuple(v['corners_mm'][0]) for v in pairs}) != 4:
        raise ValueError('Four independent feet required')
    return pairs


def all_foot_gaps(mesh, cover_pose, base_pose):
    cp, cr = pose(cover_pose)
    bp, br = pose(base_pose)
    inverse_base = transpose(br)
    rows = []
    for pair in support_geometry(mesh):
        lo, hi = pair['pad_bounds_mm']
        local = [[x*1000 for x in mv(inverse_base, [a-b for a, b in zip(transform(cp, cr, v), bp)])]
                 for v in pair['corners_mm']]
        gaps = [v[2]-hi[2] for v in local]
        containment = [min(v[0]-lo[0], hi[0]-v[0], v[1]-lo[1], hi[1]-v[1]) for v in local]
        rows.append({**pair, 'foot_corners_in_base_mm': local, 'corner_gaps_mm': gaps,
                     'minimum_gap_mm': min(gaps), 'maximum_gap_mm': max(gaps),
                     'minimum_pad_edge_margin_mm': min(containment),
                     'foot_fully_over_pad': min(containment) >= 0})
    return {'feet': rows, 'minimum_gap_mm': min(x['minimum_gap_mm'] for x in rows),
            'all_feet_over_pads': all(x['foot_fully_over_pad'] for x in rows)}


def predict_from_held(estimated_pose, current_tip_world_m, current_tool_rotation,
                      target_tip_world_m, target_tool_rotation):
    """Rigid prediction only: estimated RGB pose + FK; no truth argument."""
    p, r = pose(estimated_pose)
    tip = vec(current_tip_world_m)
    t = rigid_rotation(current_tool_rotation)
    target_tip = vec(target_tip_world_m)
    target_t = rigid_rotation(target_tool_rotation)
    relative_r = mmul(transpose(t), r)
    relative_p = mv(transpose(t), [a-b for a, b in zip(p, tip)])
    result_p = [a+b for a, b in zip(target_tip, mv(target_t, relative_p))]
    return {'position_world_m': result_p, 'rotation_world': mmul(target_t, relative_r),
            'source': 'RGB estimated part pose plus measured tool FK and proposed target'}


def error_budget_at_feet(mesh, translation_bounds_mm, rotation_bound_deg,
                         calibration_height_error_mm, hold_drift_height_error_mm,
                         support_height_error_mm, tcp_tracking_height_error_mm):
    """Conservative norm bound for every foot; caller must justify each bound.

    Rotation error is about part origin. Unknown/missing bounds must never be
    replaced by zero. This is a geometric bound, not a statistical confidence.
    """
    xyz = vec(translation_bounds_mm)
    scalars = vec([rotation_bound_deg, calibration_height_error_mm,
                   hold_drift_height_error_mm, support_height_error_mm,
                   tcp_tracking_height_error_mm], 5)
    if min(xyz+scalars) < 0 or rotation_bound_deg > 180:
        raise ValueError('Invalid uncertainty bound')
    # Conservative for any support normal, including a tilted actual base.
    translation = math.sqrt(dot(xyz, xyz))
    extras = sum(scalars[1:])
    rows = []
    for pair in support_geometry(mesh):
        radius = max(math.sqrt(dot(v, v)) for v in pair['corners_mm'])
        angle = 2*radius*math.sin(math.radians(rotation_bound_deg)/2)
        rows.append({'foot_index': pair['foot_index'], 'radius_mm': radius,
                     'translation_error_mm': translation, 'rotation_error_mm': angle,
                     'other_error_mm': extras, 'total_height_error_mm': translation+angle+extras})
    return {'feet': rows, 'maximum_height_error_mm': max(x['total_height_error_mm'] for x in rows),
            'qualification': 'Bounds require separate fit/calibration/drift/support evidence; not inferred from pixel residuals'}


def tcp_tracking_bounds_at_feet(mesh, part_origin_in_tool_m, part_rotation_in_tool,
                               tcp_translation_error_mm, tcp_rotation_error_deg):
    """TCP orientation errors rotate about TCP, using the full foot/TCP lever."""
    origin = vec(part_origin_in_tool_m)
    rotation = rigid_rotation(part_rotation_in_tool)
    translation, angle = vec([tcp_translation_error_mm, tcp_rotation_error_deg], 2)
    if min(translation, angle) < 0 or angle > 180:
        raise ValueError('Invalid TCP tracking error')
    rows = []
    for foot in support_geometry(mesh):
        points = [[1000*(a+b) for a, b in zip(origin, mv(rotation, [x*.001 for x in v]))]
                  for v in foot['corners_mm']]
        radius = max(math.sqrt(dot(v, v)) for v in points)
        error = 2*radius*math.sin(math.radians(angle)/2)
        rows.append({'foot_index': foot['foot_index'], 'foot_to_tcp_radius_mm': radius,
                     'tcp_rotation_height_error_mm': error,
                     'tcp_translation_height_error_mm': translation,
                     'total_tracking_height_error_mm': translation+error})
    return {'feet': rows,
            'maximum_height_error_mm': max(v['total_tracking_height_error_mm'] for v in rows)}


def observed_tcp_tracking_foot_errors(mesh, planned_cover_pose, target_tip_world_m,
                                      target_tool_rotation, actual_tip_world_m,
                                      actual_tool_rotation, base_pose):
    """Exact observed FK discrepancy under the predicted rigid grasp, no truth.

    This measures this instant, not an upper bound on future dynamic tracking.
    """
    actual_prediction = predict_from_held(planned_cover_pose, target_tip_world_m,
                                         target_tool_rotation, actual_tip_world_m,
                                         actual_tool_rotation)
    planned = all_foot_gaps(mesh, planned_cover_pose, base_pose)
    actual = all_foot_gaps(mesh, actual_prediction, base_pose)
    differences = []
    for a, p in zip(actual['feet'], planned['feet']):
        errors = [[x-y for x, y in zip(av, pv)] for av, pv in
                  zip(a['foot_corners_in_base_mm'], p['foot_corners_in_base_mm'])]
        differences.append({'foot_index': a['foot_index'],
            'corner_gap_errors_mm': [x-y for x, y in zip(a['corner_gaps_mm'], p['corner_gaps_mm'])],
            'corner_position_errors_mm': [math.sqrt(dot(v, v)) for v in errors]})
    return {'feet': differences,
            'maximum_absolute_height_error_mm': max(abs(v) for f in differences for v in f['corner_gap_errors_mm']),
            'maximum_position_error_mm': max(v for f in differences for v in f['corner_position_errors_mm']),
            'meaning': 'Observed FK discrepancy only; future tracking/drift still requires qualification'}


def direct_error_budget(mesh, fit_foot_height_error_mm, calibration_height_error_mm,
                        hold_drift_height_error_mm, support_z_error_mm,
                        support_tilt_deg, tcp_tracking_height_error_mm,
                        support_xy_error_mm, part_foot_xy_error_mm,
                        source_assumptions, support_yaw_deg=1.):
    """Direct per-foot fit bound for the bounded precontact diagnostic.

    At a queried XY coordinate the tilted pad-top plane height varies by
    radius*tan(theta) + pad_z*(sec(theta)-1), plus its origin Z error.
    Radius includes both base XY placement and the declared part-foot XY error.
    The fit-height bound may be an explicitly limited empirical joint-SE(3)
    envelope; it is not replaced by independently worst Euler/position bounds.
    """
    values = vec([fit_foot_height_error_mm, calibration_height_error_mm,
                  hold_drift_height_error_mm, support_z_error_mm,
                  support_tilt_deg, tcp_tracking_height_error_mm,
                  support_xy_error_mm, part_foot_xy_error_mm, support_yaw_deg], 9)
    if min(values) < 0 or support_tilt_deg >= 90 or support_yaw_deg > 180 or not source_assumptions:
        raise ValueError('Explicit nonnegative sourced diagnostic bounds required')
    theta = math.radians(support_tilt_deg)
    lateral = support_xy_error_mm+part_foot_xy_error_mm
    # Conservative boundary displacement for pad yaw plus tilt. This is a
    # separate pad-containment allowance; yaw does not raise the origin or
    # change the independent bound on support-normal tilt used below.
    pads = [v for foot in support_geometry(mesh)
            for v in mesh['parts']['base']['colliders'][foot['pad_index']]['vertices_mm']]
    pad_radius = max(math.sqrt(dot(v, v)) for v in pads)
    angular_displacement = 2*pad_radius*math.sin(math.radians(min(180., support_yaw_deg+support_tilt_deg))/2)
    rows = []
    for foot in support_geometry(mesh):
        radius = max(math.hypot(v[0], v[1]) for v in foot['corners_mm'])+lateral
        support = support_z_error_mm + radius*math.tan(theta) + abs(foot['pad_bounds_mm'][1][2])*(1/math.cos(theta)-1)
        total = fit_foot_height_error_mm+calibration_height_error_mm+hold_drift_height_error_mm+tcp_tracking_height_error_mm+support
        rows.append({'foot_index': foot['foot_index'], 'evaluation_radius_mm': radius,
                     'fit_height_error_mm': fit_foot_height_error_mm,
                     'calibration_height_error_mm': calibration_height_error_mm,
                     'hold_drift_height_error_mm': hold_drift_height_error_mm,
                     'tcp_tracking_height_error_mm': tcp_tracking_height_error_mm,
                     'support_height_error_mm': support,
                     'total_height_error_mm': total})
    return {'feet': rows, 'maximum_height_error_mm': max(v['total_height_error_mm'] for v in rows),
            'maximum_horizontal_error_mm': lateral+angular_displacement,
            'support_pad_angular_displacement_mm': angular_displacement,
            'support_yaw_deg': support_yaw_deg,
            'source_assumptions': source_assumptions,
            'qualification': 'Bounded known-pose simulation precontact diagnostic only; empirical fit envelope is not a physical guarantee'}


def qualify_prediction(gaps, budget):
    bound = float(budget['maximum_height_error_mm'])
    if not math.isfinite(bound) or bound < 0:
        raise ValueError('Invalid budget')
    lower = gaps['minimum_gap_mm']-bound
    horizontal = float(budget.get('maximum_horizontal_error_mm', 0))
    if not math.isfinite(horizontal) or horizontal < 0:
        raise ValueError('Invalid horizontal uncertainty')
    edge_lower = min(v['minimum_pad_edge_margin_mm'] for v in gaps['feet'])-horizontal
    return {'positive_clearance_lower_bound_mm': lower,
            'pad_edge_margin_lower_bound_mm': edge_lower,
            'geometric_dispatch_eligible': bool(gaps['all_feet_over_pads'] and lower > 0 and edge_lower > 0),
            'scope': 'One necessary geometric condition only; never assembly success or certified safety'}


def audit_raw_contacts(mesh, report):
    """Offline evidence only. Keep pre/post timing explicit; never select best fit."""
    diagnostics = report.get('contact_timing_diagnostics', {})
    trace = diagnostics.get('pre_step_trace', [])
    snapshots = []
    if trace:
        snapshots.append(('last_recorded_pre_step', trace[-1]))
    if diagnostics.get('post_step'):
        snapshots.append(('recorded_post_step', diagnostics['post_step']))
    results = []
    for contact in report.get('unexpected_contacts', []):
        paths = contact['colliders']
        if not (any('/Assembly_cover/C_' in p and p.endswith('_ear') for p in paths)
                and any('/Assembly_base/C_' in p and p.endswith('_floor') for p in paths)):
            continue
        foot_path = next(p for p in paths if '/Assembly_cover/' in p)
        foot_index = int(foot_path.rsplit('/', 1)[1].split('_')[1])
        raw_depth = -float(contact['minimum_separation_mm'])
        frames = []
        for name, snapshot in snapshots:
            matrices = snapshot['actor_world_matrices']
            gaps = all_foot_gaps(mesh, usd_matrix_pose(matrices['/World/Assembly_cover']),
                                 usd_matrix_pose(matrices['/World/Assembly_base']))
            foot = next(v for v in gaps['feet'] if v['foot_index'] == foot_index)
            depth = -foot['minimum_gap_mm']
            frames.append({'timing': name, 'step': snapshot['step'],
                           'foot_plane_depth_mm': depth,
                           'raw_minus_geometry_mm': raw_depth-depth,
                           'all_foot_gaps': gaps})
        limit = report['limits']['maximum_intended_penetration_mm']
        results.append({'raw_contact_step': contact['step'], 'phase': contact['phase'],
                        'raw_depth_mm': raw_depth, 'raw_limit_mm': limit,
                        'original_raw_failure_retained': raw_depth > limit,
                        'timed_reconstructions': frames})
    return {'source': 'OFFLINE report truth for independent verification; prohibited as runtime steering',
            'contacts': results}


def load_mesh(path, expected_sha256=None):
    data = Path(path).read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError('CAD source hash mismatch')
    mesh = json.loads(data)
    support_geometry(mesh)
    return mesh, digest


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--expected-mesh-sha256')
    args = parser.parse_args()
    mesh, digest = load_mesh(args.mesh, args.expected_mesh_sha256)
    report = json.loads(Path(args.report).read_text())
    print(json.dumps({'mesh_sha256': digest, **audit_raw_contacts(mesh, report)}, indent=2))
