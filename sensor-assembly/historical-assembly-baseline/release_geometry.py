import numpy as np

def separating_gap_mm(ma,mb):
 def geometry(m):
  v=np.array(m['world_vertices_m'],float)*1000;edges=[];normals=[];off=0
  for count in m['face_counts']:
   face=m['face_indices'][off:off+count];off+=count
   for i in range(len(face)):edges.append(v[face[(i+1)%len(face)]]-v[face[i]])
   for i in range(1,len(face)-1):
    n=np.cross(v[face[i]]-v[face[0]],v[face[i+1]]-v[face[0]])
    if np.linalg.norm(n)>1e-10:normals.append(n/np.linalg.norm(n))
  return v,edges,normals
 a,b=geometry(ma),geometry(mb);axes=a[2]+b[2]
 for e in a[1]:
  for f in b[1]:
   n=np.cross(e,f)
   if np.linalg.norm(n)>1e-10:axes.append(n/np.linalg.norm(n))
 return max(float(max((b[0]@n).min()-(a[0]@n).max(),(a[0]@n).min()-(b[0]@n).max())) for n in axes)
