"""Separate offline outcome audit. Does not certify mechanical success.

Full ordered raw trace/report remains on Rosie. Compact output selects an
explicit terminal window; acquisition checker separately covers every row.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from payload_convex_geometry import sat
from check_payload_contact_trace import validate_trace

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def transformed(points,matrix):return (np.c_[np.asarray(points,float),np.ones(len(points))]@np.asarray(matrix,float))[:,:3]
def audit(path,source):
 path=Path(path);source=Path(source);r=json.loads(path.read_text());trace=r.get('payload_contact_trace',{});cooked=r.get('payload_cooked_geometry',{});rows=trace.get('rows',[]);phases={};worst=None
 try:integrity={'pass':True,'result':validate_trace(r,source)}
 except Exception as exc:integrity={'pass':False,'error':repr(exc)}
 for row in rows:
  group=phases.setdefault(row['phase'],{'first_step':row['state_step'],'last_step':row['state_step'],'steps':0,'headers':0,'points':0,'no_reported_header_steps':0,'minimum_separation_mm':None,'event_types':{}})
  group['last_step']=row['state_step'];group['steps']+=1;group['no_reported_header_steps']+=int(not row['headers'])
  for h in row['headers']:
   group['headers']+=1;group['points']+=len(h['points']);event=str(h['event_type_code']);group['event_types'][event]=group['event_types'].get(event,0)+1
   for point in h['points']:
    sep=point['separation_mm'];group['minimum_separation_mm']=sep if group['minimum_separation_mm'] is None else min(sep,group['minimum_separation_mm'])
    if worst is None or sep<worst['point']['separation_mm']:worst={'step':row['state_step'],'phase':row['phase'],'header':{k:v for k,v in h.items() if k!='points'},'point':point}
 repr_match={}
 for p,item in cooked.get('colliders',{}).items():
  authored=np.asarray(item['authored_points_mesh_local'],float);source_world=transformed(authored,item['mesh_to_world_matrix'])
  results=[]
  for convex in item['convexes']:
   world=transformed(convex['vertices_mesh_local'],item['mesh_to_world_matrix']);dist=np.linalg.norm(world[:,None]-source_world[None,:],axis=-1)
   results.append({'cooked_vertices':len(world),'authored_vertices':len(source_world),'bidirectional_vertex_set_distance_mm':float(max(dist.min(0).max(),dist.min(1).max())*1000),'coordinate_semantics':'mesh local transformed as NVIDIA 110 demo; not active PxShape identity'})
  repr_match[p]=results
 failure_sat=[]
 for contact in r.get('unexpected_contacts',[]):
  paths=contact['colliders']
  if not all(p in cooked.get('colliders',{}) and cooked['colliders'][p]['convexes'] for p in paths):continue
  normal=contact['contact_points'][0]['normal_world'];results=[]
  snapshots=[('pre',s) for s in r['contact_timing_diagnostics']['pre_step_trace']]+[('post',r['contact_timing_diagnostics']['post_step'])]
  for timing,snapshot in snapshots:
   verts=[]
   for p in paths:
    item=cooked['colliders'][p];matrix=np.asarray(item['mesh_to_actor_matrix'])@np.asarray(snapshot['actor_world_matrices'][item['actor']]);verts.append([transformed(c['vertices_mesh_local'],matrix) for c in item['convexes']])
   results.append({'timing':timing,'step':snapshot['step'],'sim_time':snapshot['sim_time'],'convex_pairs':[sat(a,b,normal) for a in verts[0] for b in verts[1]]})
  failure_sat.append({'original_raw_contact':contact,'post_terminal_cooking_representation_sat_at_archived_poses':results,'raw_gate_unchanged':True,'exact_active_PxShape_claim':False})
 last=rows[-1]['state_step'] if rows else None
 compact=[row for row in rows if row['state_step']>=last-11] if rows else []
 poses=[s for s in r.get('payload_contact_pose_trace',[]) if last is not None and s['step']>=last-11]
 return {'scope':'PCB/tray diagnostic only; no full assembly acceptance','report_sha256':digest(path),'report_bytes':path.stat().st_size,'runtime_status':r['status'],'error':r.get('error'),'full_assembly_accepted':False,'original_report_full_acceptance':r.get('full_assembly_accepted'),'runtime_sha256':r['trial_script_sha256'],'source_manifest_sha256':digest(source/'snapshot-sha256.json'),'acquisition_integrity':integrity,'phases':phases,'worst_recorded_payload_contact':worst,'part_results':r.get('part_results'),'limits':r.get('limits'),'payload_motion_audit':r.get('payload_motion_audit'),'original_unexpected_contacts':r.get('unexpected_contacts'),'cooked_complete':cooked.get('complete'),'cooked_errors':cooked.get('errors'),'cooking_representation_source_vertex_comparison':repr_match,'failure_cooked_geometry_audit':failure_sat,'terminal_raw_trace_window':compact,'terminal_pre_pose_window':poses,'window_scope':'last 12 recorded steps for compact review only; full acquisition has no decimation','lid_capture_scope_reached':r.get('lid_contact_trace',{}).get('capture_scope_reached'),'lid_contact_rows':len(r.get('lid_contact_trace',{}).get('rows',[])),'postrun_source_integrity':{n:digest(source/n)==h for n,h in json.loads((source/'snapshot-sha256.json').read_text()).items()}}
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('report');ap.add_argument('--output',required=True);a=ap.parse_args();value=audit(a.report,Path(__file__).parent);Path(a.output).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');print(json.dumps({k:value[k] for k in ('runtime_status','error','acquisition_integrity','full_assembly_accepted')},allow_nan=False))
