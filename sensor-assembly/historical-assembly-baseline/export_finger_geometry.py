"""Offline source-authoring map; no USD/Isaac import or live physics query."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np

def export(starter,destination):
    starter=Path(starter);sys.path.insert(0,str(starter))
    import scene_config as C,gripper_geometry as gg
    root=Path(__file__).parent
    carriage=C.FLANGE_OFFSET+C.ADAPTER_T+C.ZCYL_LEN+C.ZFLANGE_T+C.BODY_LEN+C.PLATE_T
    finger=carriage+C.CARRIAGE_LEN-C.CARRIAGE_OVERLAP;colliders={}
    for sign,tag in [(1,'L'),(-1,'R')]:
        profile=[(u,sign*v) for u,v in gg.finger_profile()];boundary=C.FINGER_LENGTH-C.SOFT_TIP_LENGTH
        sections=[(gg.clip_profile(profile,boundary,False),'PHYS_MAT_PATH'),(gg.clip_profile(profile,boundary,True),'SOFT_TIP_MAT_PATH')]
        pieces=[(section,tri,material) for section,material in sections for tri in gg.triangulate(section)]
        for number,(section,tri,material) in enumerate(pieces):
            triangle=[section[i] for i in tri]
            points=np.array([(u,v,z) for z in (-C.FINGER_THICK/2,C.FINGER_THICK/2) for u,v in triangle],dtype=np.float32).astype(float)
            cooked=(points*100).astype(np.float32).astype(float);scale=float(np.float32(.01));translation=[finger-carriage,-sign*C.FINGER_WIDTH/2,0]
            path=C.ROBOT_PRIM+'/Jaw'+tag+'/Collision/Part'+format(number,'02d')
            colliders[path]={'jaw_root':C.ROBOT_PRIM+'/Jaw'+tag,'source_prism_vertices_m':points.tolist(),'cooking_authored_vertices':cooked.tolist(),
                'collider_to_jaw_row_matrix':[[scale,0,0,0],[0,scale,0,0],[0,0,scale,0],[*translation,1]],
                'source_vertices_in_jaw_m':(points+translation).tolist(),'authored_cooking_vertices_in_jaw_m':(cooked*scale+translation).tolist(),
                'section_material_source_symbol':material,'cooking':'convexHull; authored input geometry only, not a readback of generated cooked hull'}
    value={'scope':'OFFLINE STATIC SOURCE MAP; no live USD or physics query, no steering; authored cooking inputs are not exact cooked-hull proof',
        'SDK_source_sha256':{n:hashlib.sha256((starter/n).read_bytes()).hexdigest() for n in ['cell.py','scene_config.py','gripper_geometry.py']},
        'finger_cooking_script_sha256':hashlib.sha256((root/'finger_cooking.py').read_bytes()).hexdigest(),
        'algorithm':'Exact original cell.build finger sections/triangulate PartXX ordering/prism float32/translation; existing source-fidelity100x/.01 authored cooking',
        'collider_count':len(colliders),'colliders':colliders}
    assert len(colliders)==138;Path(destination).write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--starter',required=True);p.add_argument('--out',required=True);a=p.parse_args();export(a.starter,a.out)
