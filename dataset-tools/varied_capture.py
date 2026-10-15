"""Small, seeded diversity pilot using one Isaac app and unchanged starter files."""
import argparse
import copy
import gc
import hashlib
import json
import math
import random
import sys
import time
from pathlib import Path
from persistent_capture import prepare_capture

COUNTS=(0,1,3,6,8,6)


def configuration(C, original_camera, seed, profile):
    rng=random.Random(seed)
    caps=[(x+rng.uniform(-.018,.018),y+rng.uniform(-.018,.018)) for x,y in C.SORT_CAPS]
    vials=[(x+rng.uniform(-.012,.012),y+rng.uniform(-.012,.012),yaw+rng.uniform(-8,8))
           for x,y,yaw in C.SORT_VIALS]
    count=COUNTS[profile%len(COUNTS)]
    colours=(['yellow','blue','red']*3)[:count]
    if count==1:
        colours=[('yellow','blue','red')[seed%3]]
    rng.shuffle(colours); cubes=[]
    for colour in colours:
        for attempt in range(10000):
            xy=(rng.uniform(.24,.46),rng.uniform(-.15,.28))
            if any(math.dist(xy,p[:2])<.035 for p in cubes):
                continue
            if any(math.dist(xy,p)<.045 for p in caps+[C.CUBE_STACK_XY]):
                continue
            blocked=False
            for x,y,yaw in vials:
                ax,ay=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
                t=max(-C.VIAL_LENGTH/2,min(C.VIAL_LENGTH/2,(xy[0]-x)*ax+(xy[1]-y)*ay))
                if math.dist(xy,(x+t*ax,y+t*ay))<.035:
                    blocked=True
            if not blocked:
                cubes.append((*xy,colour));break
        else:
            raise ValueError('Could not place separated objects')
    camera=copy.deepcopy(original_camera)
    camera['eye']=tuple(v+rng.uniform(-s,s) for v,s in zip(camera['eye'],(.015,.015,.008)))
    camera['look_at']=tuple(v+rng.uniform(-.008,.008) if i<2 else v for i,v in enumerate(camera['look_at']))
    camera['focal_length']*=rng.uniform(.97,1.03)
    shade=(.72,.84,.94)[profile%3]
    return dict(seed=seed,profile=profile,cube_count=count,cube_layout=cubes,
        cube_yaws_deg=[rng.uniform(-45,45) for _ in cubes],caps=caps,vials=vials,
        table_camera_config=camera,light_intensity_scale=rng.uniform(.75,1.25),
        backdrop_diffuse_rgb=[shade,shade,shade],backdrop_roughness=rng.uniform(.3,.8),
        yellow_positions_fixed=False,source='fanuc_er4ia_hackathon_starter_v05',
        mode='diversity_pilot_v2',real_camera_matched=False,reachability_verified=False,
        object_origin='cube root at bottom-face centre; dimensions unchanged',
        limits=['upright 10mm cubes only','separated objects; occlusions rejected',
                'camera jitter is synthetic, not a physical calibration update'])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--starter',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--count',type=int,default=6)
    parser.add_argument('--start-seed',type=int,default=525101)
    args=parser.parse_args()
    if not 1<=args.count<=12:
        parser.error('Diversity pilot limited to 1–12 scenes')
    starter,out=args.starter.resolve(),args.out.resolve()
    if out.exists():
        parser.error('Use fresh output directory')
    source=starter/'capture_scene.py'
    code=prepare_capture(source.read_text())
    sys.path.insert(0,str(starter));import scene_config as C
    keys=('SORT_CUBES','SORT_CUBE_YAWS_DEG','SORT_CAPS','SORT_VIALS','TABLE_CAM')
    original={k:copy.deepcopy(getattr(C,k)) for k in keys}
    out.mkdir(parents=True)
    report=dict(mode='diversity_pilot_v2',requested_count=args.count,samples=[],complete=False,
        training_ready=False,manual_label_review_required=True,
        source_sha256={n:hashlib.sha256((starter/n).read_bytes()).hexdigest()
                       for n in ('capture_scene.py','cell.py','scene_config.py','render_settings.py')})
    app=None;namespace=None;original_argv=sys.argv[:];start=time.monotonic()
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True});report['startup_s']=time.monotonic()-start
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import create_new_stage
        import cell
        from varied_labels import verify_labels
        paper_original=copy.deepcopy(cell._MATERIAL_SPEC['paper_white'])
        for index in range(args.count):
            for k,v in original.items():setattr(C,k,copy.deepcopy(v))
            generation=configuration(C,original['TABLE_CAM'],args.start_seed+index,index)
            generation=json.loads(json.dumps(generation))
            C.SORT_CUBES=generation['cube_layout'];C.SORT_CUBE_YAWS_DEG=generation['cube_yaws_deg']
            C.SORT_CAPS=generation['caps'];C.SORT_VIALS=generation['vials']
            C.TABLE_CAM=generation['table_camera_config']
            cell._MATERIAL_SPEC['paper_white']=(tuple(generation['backdrop_diffuse_rgb']),0.,generation['backdrop_roughness'])
            folder=out/f"seed-{generation['seed']}";folder.mkdir()
            (folder/'generation.json').write_text(json.dumps(generation,indent=2)+'\n')
            record=dict(seed=generation['seed'],valid=False);report['samples'].append(record)
            sample_start=time.monotonic()
            try:
                if namespace is not None:
                    namespace['world'].stop();namespace['cam'].pause();namespace.clear()
                    World.clear_instance();gc.collect()
                create_new_stage();app.update()
                sys.argv=[str(source),'--camera','table','--out',str(folder)]
                namespace={'__name__':'__main__','__file__':str(source),'SHARED_APP':app,
                           'LIGHT_SCALE':generation['light_intensity_scale']}
                exec(compile(code,str(source),'exec'),namespace)
                record.update(verify_labels(folder,generation,C.CUBE_EDGE));record['valid']=True
            except Exception as e:
                record['error']=f'{type(e).__name__}: {e}';raise
            finally:
                record['elapsed_s']=time.monotonic()-sample_start
                (out/'batch_report.json').write_text(json.dumps(report,indent=2)+'\n')
                print('[varied]',json.dumps(record),flush=True)
        report['complete']=all(s['valid'] for s in report['samples']) and len(report['samples'])==args.count
    except Exception as e:
        report['error']=f'{type(e).__name__}: {e}';raise
    finally:
        report['total_s_before_close']=time.monotonic()-start
        (out/'batch_report.json').write_text(json.dumps(report,indent=2)+'\n')
        for k,v in original.items():setattr(C,k,v)
        if 'paper_original' in locals():cell._MATERIAL_SPEC['paper_white']=paper_original
        sys.argv=original_argv
        if namespace is not None:namespace.clear()
        if app is not None:app.close()


if __name__=='__main__':main()
