"""OFFLINE convex SAT audit of archived authored world meshes and timed poses.

Reports source geometry, not an unobserved cooked PhysX hull or intra-step pose.
All raw failures remain failures. No simulator/motion API is imported.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull


def unique_axes(values):
    result = {}
    for value in values:
        value = np.asarray(value, float)
        length = np.linalg.norm(value)
        if length < 1e-10:
            continue
        value = value/length
        pivot = next((v for v in value if abs(v) > 1e-9), 1)
        if pivot < 0:
            value = -value
        result.setdefault(tuple(np.round(value, 9)), value)
    return list(result.values())


def hull_geometry(vertices_m):
    v = np.asarray(vertices_m, float)*1000
    if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all():
        raise ValueError('Invalid archived vertices')
    hull = ConvexHull(v)
    edges = []
    for face in hull.simplices:
        for a, b in zip(face, np.roll(face, -1)):
            edges.append(v[b]-v[a])
    return v, unique_axes(hull.equations[:, :3]), unique_axes(edges)


def axis_gap(a, b, normal):
    normal = np.asarray(normal, float)
    normal /= np.linalg.norm(normal)
    ap = np.einsum('ij,j->i', a, normal)
    bp = np.einsum('ij,j->i', b, normal)
    return float(max(bp.min()-ap.max(), ap.min()-bp.max()))


def sat(vertices_a_m, vertices_b_m, raw_normal):
    a, na, ea = hull_geometry(vertices_a_m)
    b, nb, eb = hull_geometry(vertices_b_m)
    axes = unique_axes(na+nb+[np.cross(u, v) for u in ea for v in eb])
    gap, n = max((axis_gap(a, b, v), v.tolist()) for v in axes)
    return {'maximum_separating_gap_mm': gap,
            'convex_source_geometries_overlap': gap < 0,
            'minimum_translation_depth_mm': max(0, -gap),
            'max_gap_axis_world': n,
            'separation_on_raw_contact_normal_mm': axis_gap(a, b, raw_normal),
            'candidate_axes': len(axes)}


def actor_for(path, matrices):
    choices = [name for name in matrices if path.startswith(name+'/')]
    if not choices:
        raise ValueError('No actor transform for '+path)
    return max(choices, key=len)


def audit(report_path):
    raw = Path(report_path).read_bytes()
    report = json.loads(raw)
    diag = report['contact_timing_diagnostics']
    post = report['failed_stop_contact_meshes']
    post_matrices = diag['post_step']['actor_world_matrices']
    rows = []
    for contact in report['unexpected_contacts']:
        paths = contact['colliders']
        if not all(path in post for path in paths):
            rows.append({'contact': contact, 'geometry_available': False})
            continue
        normal = contact['contact_points'][0]['normal_world']
        local = {}
        for path in paths:
            actor = actor_for(path, post_matrices)
            vertices = np.c_[np.asarray(post[path]['world_vertices_m']), np.ones(len(post[path]['world_vertices_m']))]
            local[path] = (actor, np.einsum('ij,jk->ik', vertices, np.linalg.inv(np.asarray(post_matrices[actor]))))
        results = []
        for snapshot in diag['pre_step_trace']+[diag['post_step']]:
            name = ('post_step' if snapshot is diag['post_step'] else 'pre_step')
            vertices = []
            for path in paths:
                actor, points = local[path]
                vertices.append(np.einsum('ij,jk->ik', points, np.asarray(snapshot['actor_world_matrices'][actor]))[:, :3])
            result = sat(*vertices, normal)
            result.update(timing=name, step=snapshot['step'], sim_time=snapshot.get('sim_time'))
            result['raw_depth_minus_source_mtd_mm'] = -contact['minimum_separation_mm']-result['minimum_translation_depth_mm']
            results.append(result)
        # Also audit explicitly captured meshes independently of reconstruction.
        explicit = {}
        for name, saved in [('last_pre_step_authored', diag.get('pre_step_contact_meshes', {})),
                            ('post_step_authored', post)]:
            if all(path in saved for path in paths):
                explicit[name] = sat(saved[paths[0]]['world_vertices_m'], saved[paths[1]]['world_vertices_m'], normal)
        rows.append({'colliders': paths, 'raw_depth_mm': -contact['minimum_separation_mm'],
                     'raw_limit_mm': report['limits']['maximum_intended_penetration_mm'],
                     'raw_failure_retained': -contact['minimum_separation_mm'] > report['limits']['maximum_intended_penetration_mm'],
                     'phase': contact['phase'], 'contact_step': contact['step'],
                     'raw_normal': normal, 'geometry_available': True,
                     'timed_source_convex_sat': results, 'explicit_authored_mesh_sat': explicit})
    return {'scope': __doc__, 'report_sha256': hashlib.sha256(raw).hexdigest(),
            'report_path': str(report_path), 'source_cooking': report.get('finger_cooking'),
            'contacts': rows}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report')
    parser.add_argument('--output-name')
    args = parser.parse_args()
    result = audit(args.report)
    if args.output_name:
        if Path(args.output_name).name != args.output_name:
            raise ValueError('Output must stay inside audit directory')
        (Path(__file__).parent/args.output_name).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
