"""CPU check of nominal benchmark targets using supplied FANUC FK/IK only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--starter',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    import numpy as np
    sys.path.insert(0,str(a.starter.resolve()))
    import scene_config as C,ik,fk
    # cell.py imports USD, so derive the exact same sum from host configuration.
    tip=C.TOOL_LENGTH
    limits=np.deg2rad([C.JOINT_LIMITS_DEG['J'+str(i)] for i in range(1,7)])
    seed=np.deg2rad([C.HOME_POSE_DEG['J'+str(i)] for i in range(1,7)])
    ik.set_tool_axis(C.TOOL_AXIS);ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
    rows=[]
    for x,y,h in [(x,y,h) for x in (.348,.35,.352) for y in (-.002,0.,.002) for h in (.053,.073)]:
        target=np.array([x,y,C.SURFACE_DZ+.025+h])
        result=ik.solve(seed,target,np.array([0.,0.,-1.]),limits,iters=300,tol=1e-5,
                        tool_offset=np.array(C.TOOL_AXIS)*tip,spread_dir=np.array([0.,1.,0.]))
        q=result[0] if isinstance(result,tuple) else result
        r,pos=fk.tool_pose(q,np.array(C.TOOL_AXIS)*tip)
        translation=float(np.linalg.norm(pos-target))
        approach_error=float(np.linalg.norm(r@np.array(C.TOOL_AXIS)-[0,0,-1]))
        spread_error=float(np.linalg.norm(r@np.array(C.FINGER_SPREAD_AXIS)-[0,1,0]))
        passed=translation<=.001 and approach_error<=.001 and spread_error<=.001 and bool(((q>=limits[:,0])&(q<=limits[:,1])).all())
        rows.append({'target_robot_m':target.tolist(),'joints_rad':q.tolist(),'position_error_m':translation,
                     'approach_axis_error':approach_error,'spread_axis_error':spread_error,'reachable':passed})
    report={'status':'CPU_REACHABILITY_PASS' if all(r['reachable'] for r in rows) else 'CPU_REACHABILITY_FAILED',
            'cases':rows,'tcp_m':tip,'source_hashes':{n:hashlib.sha256((a.starter/n).read_bytes()).hexdigest() for n in ['scene_config.py','fk.py','ik.py']},
            'scope':'numerical reachability only; no collision, grip, simulation, training or hardware claim',
            'simulation_executed':False,'full_assembly_accepted':False}
    a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'cases':len(rows),'maximum_position_error_m':max(r['position_error_m'] for r in rows)}))
    raise SystemExit(0 if report['status']=='CPU_REACHABILITY_PASS' else 1)


if __name__=='__main__':main()
