"""Finite analytical convex-union ray witnesses, in part-local coordinates.

This checks cooked collision surfaces later; it is not a grasp dynamics proof.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull


def ray_distance(equations, start, direction, maximum):
    lo,hi=0.,float(maximum)
    for a,b in zip(equations[:,:3],equations[:,3]):
        origin=float(a@start+b);rate=float(a@direction)
        if abs(rate)<1e-12:
            if origin>1e-8:return None
        elif rate>0:hi=min(hi,-origin/rate)
        else:lo=max(lo,-origin/rate)
        if lo>hi+1e-9:return None
    return lo if lo<=maximum and hi>=0 else None


def build(parts,design):
    result=[];coverage={}
    for name,item in parts['parts'].items():
        hulls=[ConvexHull(np.asarray(c['vertices_mm'],float)) for c in item['colliders']]
        equations=[h.equations for h in hulls]
        coverage[name]=[]
        for index,(c,hull) in enumerate(zip(item['colliders'],hulls)):
            points=np.asarray(c['vertices_mm'],float);found=None
            # Fixed candidate order; prefer the largest exposed triangle.
            candidates=[]
            for triangle,eq in zip(hull.simplices,hull.equations):
                xyz=points[triangle];area=np.linalg.norm(np.cross(xyz[1]-xyz[0],xyz[2]-xyz[0]))/2
                candidates.append((-area,xyz.mean(0),eq[:3]))
            for _,center,n in sorted(candidates,key=lambda x:x[0]):
                start=center+n*2.;direction=-n
                distances=[ray_distance(eq,start,direction,3.) for eq in equations]
                finite=[v for v in distances if v is not None]
                nearest=min(finite) if finite else None
                if nearest is not None and abs(nearest-2.)<1e-5:
                    found={'part':name,'label':f'collider_{index:03d}_{c["role"]}_exposed_face',
                           'collider_index':index,'start_m':(start*.001).tolist(),'direction':direction.tolist(),
                           'maximum_distance_m':.003,'hit':True,'expected_distance_mm':nearest}
                    break
            if found is not None:result.append(found);coverage[name].append(index)
        if name in design['grip']:
            g=design['grip'][name];x,y=g['handle_centre_xy_mm'];z=g['tcp_above_part_bottom_mm']
            for dy in (-6.,-5.,5.,6.):
                start=np.array([x,y+dy,z+16.]);direction=np.array([0.,0.,-1.])
                if any(ray_distance(eq,start,direction,10.) is not None for eq in equations):
                    raise ValueError('Authored blade-slot negative ray is not clear')
                result.append({'part':name,'label':'empty_blade_slot_y_'+str(dy),
                    'start_m':(start*.001).tolist(),'direction':direction.tolist(),
                    'maximum_distance_m':.010,'hit':False})
    return result,coverage


if __name__=='__main__':
    root=Path(__file__).parent
    probes,coverage=build(json.loads((root/'parts.json').read_text()),json.loads((root/'design.json').read_text()))
    (root/'geometry-probes.json').write_text(json.dumps(probes,indent=2,allow_nan=False)+'\n')
    (root/'geometry-probe-coverage.json').write_text(json.dumps({'exposed_collider_indices':coverage,
        'probes':len(probes),'distance_limit_mm':.03,'scope':'Exposed-face rays and12emptyblade-slot rays; not everyfacet or continuous dynamics'},indent=2)+'\n')
    print(json.dumps({'probes':len(probes),'exposed_colliders':{k:len(v) for k,v in coverage.items()}}))
