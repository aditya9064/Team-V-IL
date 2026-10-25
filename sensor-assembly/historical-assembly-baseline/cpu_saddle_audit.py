"""Independent CPU data audit; no simulator, targets or hardware operations."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

def need(v,message):
 if not v:raise ValueError(message)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def integrals(item):
 v=np.asarray(item['vertices_mm'],float);faces=np.asarray(item['faces'],int);need(np.isfinite(v).all(),'Nonfinite mesh')
 V=0.;F=np.zeros(3);S=np.zeros((3,3))
 for a,b,c in v[faces]:
  dv=float(np.dot(a,np.cross(b,c))/6);s=a+b+c;V+=dv;F+=dv*s/4;S+=dv*(np.outer(s,s)+np.outer(a,a)+np.outer(b,b)+np.outer(c,c))/20
 need(V>0,'Nonpositive oriented volume');return V,F,S

def ray(entries,origin,direction,maximum):
 o=np.asarray(origin)*1000;D=np.asarray(direction,float);D/=np.linalg.norm(D);found=[]
 for e in entries:
  h=ConvexHull(np.asarray(e['vertices_mm']));a=np.einsum('ij,j->i',h.equations[:,:3],D);b=np.einsum('ij,j->i',h.equations[:,:3],o)+h.equations[:,3];lo=0.;hi=maximum*1000
  for aa,bb in zip(a,b):
   if abs(aa)<1e-10:
    if bb>1e-7:hi=-1;break
   elif aa>0:hi=min(hi,-bb/aa)
   else:lo=max(lo,-bb/aa)
  if lo<=hi+1e-7 and hi>=0:found.append(max(lo,0.))
 return min(found) if found else None

def audit(source):
 p=Path(source);old=json.loads((p/'nest-mesh.json').read_text());new=json.loads((p/'lid-saddle-mesh.json').read_text());d0=json.loads((p/'nest-design.json').read_text());d=json.loads((p/'lid-saddle-design.json').read_text());proof=json.loads((p/'supported-saddle-proof.json').read_text())
 need(proof['new_mesh_sha256']==sha(p/'lid-saddle-mesh.json') and proof['new_design_sha256']==sha(p/'lid-saddle-design.json') and proof['new_mass_reference_sha256']==sha(p/'lid-saddle-mass-reference.json'),'Candidate proof file binding changed')
 a=old['parts']['cover']['colliders'];b=new['parts']['cover']['colliders'];need(len(a)==57 and len(b)==58 and b[:53]==a[:53] and b[53]==a[54],'Unapproved old lid modification')
 need(all(x['role']=='grip_support' for x in b[54:]),'Wrong new feature roles')
 need(all(new['parts'][n]==old['parts'][n] for n in old['parts'] if n!='cover'),'Other product changed')
 for k in ['limits','payload_limits','grip_commands','grip_command','layout_m','payload_initial_gap_mm','finger_profile_sha256']:need(new[k]==old[k],'Original command/limit/layout/source changed:'+k)
 for k in ['handle_centre_xy','tcp_above_bottom_mm','side_entry_mm']:need(d['cover'][k]==d0['cover'][k],'Motion target changed:'+k)
 need(d['assembly']==d0['assembly'],'Assembly recipe changed')
 V,F,S=integrals(new['parts']['cover']['visual']);mass=V*1e-9*1240;need(abs(V-new['parts']['cover']['volume_mm3'])<1e-5,'Visual volume mismatch')
 vals=[integrals(c) for c in b];vol=sum(q[0] for q in vals);first=sum((q[1] for q in vals),np.zeros(3));second=sum((q[2] for q in vals),np.zeros((3,3)));centre=first/vol;Q=second-vol*np.outer(centre,centre);I=(np.eye(3)*np.trace(Q)-Q)*mass/vol*1e-6
 ref=json.loads((p/'lid-saddle-mass-reference.json').read_text())['cover'];q=np.array(ref['principal_axes_wxyz']);R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();actualI=R@np.diag(ref['diagonal_inertia_kg_m2'])@R.T
 need(abs(mass-ref['mass_kg'])<1e-11 and np.linalg.norm(centre*.001-ref['centre_of_mass_local_m'])<1e-10 and np.allclose(I,actualI,rtol=1e-9,atol=1e-15),'New natural mass/COM/full inertia prediction mismatch')
 for probe in new['geometry_probes']:
  t=ray(new['parts'][probe['part']]['colliders'],probe['start'],probe['direction'],probe['distance']);need((t is not None)==probe['hit'],'Ray hit mismatch:'+probe['label'])
  if probe['hit']:need(abs(t-probe['expected_distance_mm'])<=probe['maximum_distance_error_mm']==.03,'Ray distance/tolerance mismatch:'+probe['label'])
 return {'candidate_CAD_and_natural_mass_and79_probes_validated':True,'mass_kg':mass,'probe_count':len(new['geometry_probes']),'old_fingers_other_parts_and_limits_unchanged':True,'no_dynamic_or_forceclosure_claim':True}
