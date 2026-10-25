"""Two-view, actual-CAD silhouette fit. No simulator pose or object ID input.

Fit acceptance means image agreement only. It does not certify a contact clearance.
Camera matrices use the row-vector USD convention used by the existing trial.
"""
import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

# Fixed-family CAD datum: shell panel occupies z33..35mm; grasp structure is
# above it and is partially hidden by the unchanged fingers in archived views.
BODY_PANEL_TOP_MM = 35.


def project(points, model):
    points = np.asarray(points, float)
    camera = np.asarray(model['camera_to_world'], float)
    p = np.einsum('ij,jk->ik', np.c_[points, np.ones(len(points))], np.linalg.inv(camera))
    w, h = model['resolution']
    fx = w * model['focal_length_mm'] / model['horizontal_aperture_mm']
    fy = h * model['focal_length_mm'] / model['vertical_aperture_mm']
    if np.any(p[:, 2] >= -.01):
        raise ValueError('CAD behind camera or within near plane')
    return np.c_[w/2 + fx*p[:, 0]/-p[:, 2], h/2 - fy*p[:, 1]/-p[:, 2]]


def _world(points, rotation, position):
    return np.einsum('ij,kj->ik', points, rotation) + position


def _mesh(mesh):
    v = np.asarray(mesh['vertices_mm'], float) * .001
    f = np.asarray(mesh['faces'], int)
    if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all():
        raise ValueError('Invalid CAD vertices')
    if f.ndim != 2 or f.shape[1] != 3 or f.min() < 0 or f.max() >= len(v):
        raise ValueError('Triangulated actual CAD required')
    adjacency = {}
    for face_id, face in enumerate(f):
        for a, b in zip(face, np.roll(face, -1)):
            adjacency.setdefault(tuple(sorted((int(a), int(b)))), []).append(face_id)
    if any(len(faces) != 2 for faces in adjacency.values()):
        raise ValueError('Watertight CAD required for silhouette extraction')
    normals = np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]])
    return v, f, adjacency, normals


def render_mask(mesh, position, rotation, model):
    """Actual triangle union; preserves slots, feet and wall-bottom datum."""
    v = np.asarray(mesh['vertices_mm'], float)*.001
    uv = project(_world(v, rotation, position), model)
    w, h = model['resolution']
    mask = np.zeros((h, w), np.uint8)
    for face in mesh['faces']:
        cv2.fillConvexPoly(mask, np.round(uv[face]*16).astype(np.int32), 1, shift=4)
    return mask


def _signed_distance(mask):
    # Zero lies between foreground/background pixel centres rather than on an
    # arbitrarily selected foreground boundary pixel.
    return (cv2.distanceTransform(1-mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
            - cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)).astype(float)


def _sample(field, uv):
    return map_coordinates(field, [uv[:, 1], uv[:, 0]], order=1,
                           mode='constant', cval=1000.)


def _silhouette_points(mesh_data, mesh, position, rotation, model):
    v, faces, adjacency, normals = mesh_data
    eye = np.asarray(model['camera_to_world'], float)[3, :3]
    eye_local = np.asarray(rotation).T @ (eye-position)
    facing = np.einsum('ij,ij->i', normals, eye_local-v[faces[:, 0]])
    points = []
    for (a, b), adjacent in adjacency.items():
        if (facing[adjacent[0]] > 0) == (facing[adjacent[1]] > 0):
            continue
        # Sample actual CAD edges at approximately one rendered pixel spacing.
        endpoints = project(_world(v[[a, b]], rotation, position), model)
        count = max(3, int(np.ceil(np.linalg.norm(endpoints[1]-endpoints[0]))))
        points.append(v[a] + np.linspace(.03, .97, count)[:, None]*(v[b]-v[a]))
    if not points:
        raise ValueError('No CAD silhouette edges visible')
    points = np.concatenate(points)
    points = points[points[:, 2] <= BODY_PANEL_TOP_MM*.001+1e-9]
    uv = project(_world(points, rotation, position), model)
    model_mask = render_mask(mesh, position, rotation, model)
    # Reject self-occluded silhouette candidates: union rendering includes all
    # triangles, so a projected internal/hidden edge is well inside this mask.
    distance = np.abs(_sample(_signed_distance(model_mask), uv))
    points = points[distance <= 1.5]
    if len(points) < 140:  # existing base fitter uses 140 CAD points per view
        raise ValueError('Insufficient unoccluded CAD edge evidence')
    # Deterministic bounded sampling avoids overweighting high triangle density.
    if len(points) > 1000:
        points = points[np.linspace(0, len(points)-1, 1000).astype(int)]
    return points


def _observed_mask(image, mesh, nominal, model):
    w, h = model['resolution']
    if image.shape != (h, w, 3) or image.dtype != np.uint8:
        raise ValueError('Invalid RGB image dimensions/type')
    uv = project(np.asarray(mesh['vertices_mm'])*.001+nominal, model)
    lo = np.maximum(np.floor(uv.min(0)-60).astype(int), 0)
    hi = np.minimum(np.ceil(uv.max(0)+60).astype(int), [w, h])
    r, g, b = image.transpose(2, 0, 1).astype(float)
    # Light-blue lid differs from saturated blue base: archived RGB has e.g.
    # [172,189,200], and illumination varies across its sides. Derive chroma
    # separation from nominal-CAD interior and surrounding background, rather
    # than importing the base's unsuitable b/r and absolute contrast cutoffs.
    # This is a simulation material cue, not a general object detector.
    chroma = np.clip(b-r, 0, 255).astype(np.uint8)
    expected = render_mask(mesh, nominal, np.eye(3), model)
    # Twenty pixels removes nominal-pose/antialias contamination for statistics;
    # final fit uses the full observed boundary, without morphological repair.
    kernel = np.ones((41, 41), np.uint8)
    interior = cv2.erode(expected, kernel).astype(bool)
    exterior = cv2.dilate(expected, kernel) == 0
    roi_region = np.zeros((h, w), bool); roi_region[lo[1]:hi[1], lo[0]:hi[0]] = True
    foreground_low = float(np.percentile(chroma[interior], 1))
    background_high = float(np.percentile(chroma[exterior & roi_region], 99))
    if foreground_low <= background_high:
        raise ValueError('Lid/background chroma not separated in nominal ROI')
    threshold = (foreground_low + background_high)/2
    blue = ((b > g) & (g > r) & (chroma > threshold)).astype(np.uint8)
    roi = np.zeros((h, w), np.uint8)
    roi[lo[1]:hi[1], lo[0]:hi[0]] = blue[lo[1]:hi[1], lo[0]:hi[0]]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(roi, 8)
    if count <= 1:
        raise ValueError('Lid not visible in calibrated ROI')
    label = 1+int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[label, cv2.CC_STAT_AREA] < 1000:
        raise ValueError('Insufficient lid image area')
    mask = (labels == label).astype(np.uint8)
    yy, xx = np.where(mask)
    if xx.min() <= lo[0] or yy.min() <= lo[1] or xx.max() >= hi[0]-1 or yy.max() >= hi[1]-1:
        raise ValueError('Lid silhouette clipped by frame/ROI')
    return mask


def _pose(a, nominal):
    return nominal+a[:3]*.001, Rotation.from_euler('xyz', a[3:], degrees=True).as_matrix()


def _jacobian(fun, a):
    # Numerical derivative step is an optimizer setting, not an accuracy gate.
    cols = []
    for i in range(6):
        step = np.zeros(6); step[i] = .001
        cols.append((fun(a+step)-fun(a-step))/.002)
    return np.column_stack(cols)


def estimate(images, models, visual_mesh, nominal_position):
    """Fit near-nominal held lid from two calibrated RGB views and actual CAD.

    Existing ±2mm/±3degree search, 2px edge quality and 95% coverage gates are
    retained. Returned sensitivity diagnostics are not a certified error bound.
    """
    nominal = np.asarray(nominal_position, float)
    if nominal.shape != (3,) or not np.isfinite(nominal).all() or set(images) != set(models) or len(images) != 2:
        raise ValueError('Two calibrated views and finite nominal position required')
    cameras = [np.asarray(m['camera_to_world'], float) for m in models.values()]
    if any(c.shape != (4, 4) or not np.isfinite(c).all() for c in cameras):
        raise ValueError('Invalid camera calibration')
    if np.allclose(cameras[0][3, :3], cameras[1][3, :3]):
        raise ValueError('Two distinct camera viewpoints required')
    data = _mesh(visual_mesh)
    masks = {n: _observed_mask(np.asarray(images[n]), visual_mesh, nominal, models[n]) for n in sorted(images)}
    distances = {n: _signed_distance(m) for n, m in masks.items()}
    a = np.zeros(6)
    rounds = []
    # Re-evaluate actual visibility as the pose changes, rather than assuming
    # base-floor or fixed AABB edges remain the visible lid silhouette.
    for _ in range(3):
        pos, rot = _pose(a, nominal)
        points = {n: _silhouette_points(data, visual_mesh, pos, rot, models[n]) for n in sorted(images)}
        def residual(x):
            p, R = _pose(x, nominal)
            return np.concatenate([_sample(distances[n], project(_world(points[n], R, p), models[n])) for n in sorted(images)])
        fit = least_squares(residual, a, jac=lambda x: _jacobian(residual, x),
                            bounds=([-2]*3+[-3]*3, [2]*3+[3]*3), x_scale=np.ones(6),
                            loss='soft_l1', f_scale=2.,
                            max_nfev=400, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        a = fit.x
        rounds.append({'success': bool(fit.success), 'nfev': int(fit.nfev), 'cost': float(fit.cost)})
        if not fit.success:
            raise ValueError('Lid optimizer failed: '+str(rounds))
    p, R = _pose(a, nominal)
    scores = {}
    for n in sorted(images):
        errors = np.abs(_sample(distances[n], project(_world(points[n], R, p), models[n])))
        predicted = render_mask(visual_mesh, p, R, models[n])
        observed_contours, _ = cv2.findContours(masks[n], cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        observed_uv = np.concatenate(observed_contours).reshape(-1, 2)
        # CAD-defined exclusion, chosen before observing residuals: the grip
        # structure is finger-occluded and is not a measurement surface.
        vertices = np.asarray(visual_mesh['vertices_mm'], float)
        grip_vertices = vertices[vertices[:, 2] > BODY_PANEL_TOP_MM]
        if len(grip_vertices):
            grip_uv = project(_world(grip_vertices*.001, R, p), models[n])
            grip_lo, grip_hi = grip_uv.min(0)-2, grip_uv.max(0)+2
            in_grip = np.all((observed_uv >= grip_lo) & (observed_uv <= grip_hi), axis=1)
            observed_uv = observed_uv[~in_grip]
        reverse_errors = np.abs(_sample(_signed_distance(predicted), observed_uv))
        scores[n] = {'edge_rms_px': float(np.sqrt(np.mean(errors**2))),
                     'edge_p95_px': float(np.percentile(errors, 95)),
                     'edge_coverage_within_2px': float(np.mean(errors <= 2)),
                     'reverse_edge_p95_px': float(np.percentile(reverse_errors, 95)),
                     'reverse_edge_coverage_within_2px': float(np.mean(reverse_errors <= 2)),
                     'measurement_domain': 'actual shell and feet z<=35mm; CAD-projected finger-occluded grip excluded',
                     'cad_edge_points': len(points[n]), 'observed_contour_points': len(observed_uv)}
    if np.any(np.abs(a[:3]) >= 1.95) or np.any(np.abs(a[3:]) >= 2.95) or any(
        s['edge_p95_px'] > 2 or s['edge_coverage_within_2px'] < .95
        or s['reverse_edge_p95_px'] > 2 or s['reverse_edge_coverage_within_2px'] < .95 for s in scores.values()):
        raise ValueError('Lid image-fit uncertain: '+str({'views': scores, 'offset': a.tolist()}))
    J = _jacobian(residual, a)
    singular = np.linalg.svd(J, compute_uv=False)
    return {'position_world_m': p.tolist(), 'rotation_world': R.tolist(),
            'offset_mm_and_xyz_deg': a.tolist(), 'views': scores,
            'rms_px': float(np.sqrt(np.mean(residual(a)**2))),
            'optimizer': {'success': bool(fit.success), 'status': int(fit.status), 'rounds': rounds},
            'uncertainty_diagnostics': {'jacobian_singular_values': singular.tolist(),
                'jacobian_rank': int(np.linalg.matrix_rank(J)),
                'meaning': 'local image-fit sensitivity only; calibration, drift and systematic image bias unbounded',
                'clearance_qualified': False},
            'source': 'two calibrated RGB silhouettes + actual triangulated lid CAD; no runtime object coordinates'}


def placement_from_held(estimated_position, estimated_rotation, actual_tip_world,
                        actual_tool_rotation, support_origin, vertices_mm, gap_mm=.10):
    """Same rigid transform as base helper; accepts an independently qualified fit."""
    p, R, tip, T, dest = [np.asarray(v, float) for v in
        (estimated_position, estimated_rotation, actual_tip_world, actual_tool_rotation, support_origin)]
    if any(not np.isfinite(v).all() for v in (p, R, tip, T, dest)) or not 0 < gap_mm <= .1:
        raise ValueError('Invalid held placement input')
    for rotation in (R, T):
        if rotation.shape != (3, 3) or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5):
            raise ValueError('Invalid rigid rotation')
    A, local = T.T@R, T.T@(p-tip)
    target_T = A.T
    target_p = dest + [0, 0, gap_mm*.001]
    return {'tip_world_m': (target_p-target_T@local).tolist(), 'tool_rotation_world': target_T.tolist(),
            'part_origin_in_tool_m': local.tolist(), 'part_rotation_in_tool': A.tolist(),
            'desired_part_origin_world_m': target_p.tolist(), 'clearance_mm': gap_mm,
            'method': 'rigid RGB held transform + measured FK; level lid, positive foot clearance'}
