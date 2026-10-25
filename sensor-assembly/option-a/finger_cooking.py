"""Preserve supplied finger vertices while avoiding tiny convex cooking inputs.

No profile, joint, drive, material or physical dimensions are modified.
The cooked result must pass a source-profile ray audit before any pick.
"""
import math
import numpy as np


def configure_source_fidelity(stage):
    from pxr import UsdGeom, PhysxSchema, Gf
    shapes = [p for p in stage.Traverse()
              if '/Jaw' in str(p.GetPath()) and '/Collision/Part' in str(p.GetPath())
              and p.IsA(UsdGeom.Mesh)]
    if len(shapes) != 138:
        raise RuntimeError('Unexpected supplied finger collider count: ' + str(len(shapes)))
    maximum = 0.0
    for prim in shapes:
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), float)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        before = np.array([list(matrix.Transform(Gf.Vec3d(*p))) for p in points])
        mesh.GetPointsAttr().Set([Gf.Vec3f(*(p * 100)) for p in points])
        UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(.01, .01, .01))
        hull = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        hull.CreateMinThicknessAttr().Set(0.)
        hull.CreateHullVertexLimitAttr().Set(255)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        after = np.array([list(matrix.Transform(Gf.Vec3d(*p)))
                          for p in np.asarray(mesh.GetPointsAttr().Get(), float)])
        maximum = max(maximum, float(np.max(np.linalg.norm(after - before, axis=1))))
        if hull.GetMinThicknessAttr().Get() != 0 or hull.GetHullVertexLimitAttr().Get() != 255:
            raise RuntimeError('Finger hull cooking readback differs')
    if not math.isfinite(maximum) or maximum > 1e-7:
        raise RuntimeError('Numerical cooking representation changed source geometry')
    return {'mode': 'source_fidelity_cooking', 'collider_count': len(shapes),
            'maximum_world_source_vertex_change_m': maximum,
            'maximum_allowed_world_source_vertex_change_m': 1e-7,
            'min_thickness_m': 0., 'hull_vertex_limit': 255,
            'local_vertex_multiplier': 100, 'uniform_child_scale': .01,
            'source_profile_dimensions_unchanged': True,
            'rigid_collision_note': 'Oblong hulls may fall back to CPU rigid collision; not a particle/deformable test.'}


def audit_cooked_finger_surfaces(stage, robot_prim, profile, query):
    from pxr import UsdGeom, Gf
    probes = []
    for sign, tag in [(1, 'L'), (-1, 'R')]:
        prefix = robot_prim + '/Jaw' + tag + '/Collision/'
        prim = stage.GetPrimAtPath(prefix + 'Part00')
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        for u in [.0445, .050, .056, .061, .068, .0733]:
            crossings = []
            for p0, p1 in zip(profile, profile[1:] + profile[:1]):
                if p0[0] != p1[0] and min(p0[0], p1[0]) <= u <= max(p0[0], p1[0]):
                    crossings.append(p0[1] + (u-p0[0])/(p1[0]-p0[0])*(p1[1]-p0[1]))
            if not crossings:
                raise RuntimeError('No source profile intersection')
            v = sign * min(crossings)
            start = np.array([u, v - sign * .002, 0.])
            end = start + np.array([0., sign * .002, 0.])
            origin = np.array(matrix.Transform(Gf.Vec3d(*(start * 100))))
            target = np.array(matrix.Transform(Gf.Vec3d(*(end * 100))))
            direction = target - origin
            expected = float(np.linalg.norm(direction))
            direction /= expected
            hit = query.raycast_closest(tuple(origin), tuple(direction), .004)
            collider = str(hit.get('collision', ''))
            distance = float(hit.get('distance', float('nan')))
            error = (expected - distance) * 1000
            probes.append({'jaw': tag, 'u_m': u,
                           'expected_source_distance_mm': expected * 1000,
                           'actual_cooked_distance_mm': distance * 1000 if math.isfinite(distance) else None,
                           'inward_error_mm': error if math.isfinite(error) else None,
                           'same_jaw_hit': bool(hit.get('hit')) and collider.startswith(prefix),
                           'collider': collider})
    passed = all(p['same_jaw_hit'] and p['inward_error_mm'] is not None
                 and abs(p['inward_error_mm']) <= .03 for p in probes)
    return {'probes': probes, 'source_surface_match': passed,
            'maximum_allowed_absolute_error_mm': .03,
            'maximum_absolute_error_mm': max((abs(p['inward_error_mm']) for p in probes
                                              if p['inward_error_mm'] is not None), default=None)}
