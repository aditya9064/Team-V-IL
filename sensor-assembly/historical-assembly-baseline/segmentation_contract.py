"""Geometry-equivalent foot segmentation and reduced-fixture mass contract."""
import numpy as np
from pathlib import Path
import json
CASES=('fixed_original','fixed_split','movable_original','movable_split')
SELECTED={'cover':6,'base':10}
SPLIT_MM=2.

def properties(piece,mass):
 v=np.asarray(piece['vertices_mm'],float);lo=v.min(0);hi=v.max(0);s=(hi-lo)*.001
 return {'mass_kg':float(mass),'centre_of_mass_local_m':((hi+lo)*.0005).tolist(),'diagonal_inertia_kg_m2':(mass/12*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2])).tolist(),'principal_axes_wxyz':[1.,0.,0.,0.]}

def box_piece(lo,hi,role):
 # Same vertex/triangle convention as a convex box, units remain millimeters.
 lo=np.asarray(lo,float);hi=np.asarray(hi,float)
 v=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
 faces=[[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]]
 return {'role':role,'vertices_mm':v.tolist(),'faces':faces}

def pieces(geometry,name,split):
 if name=='nest':return [(0,box_piece([18,-26,0],[30,-14,4],'floor'))]
 i=SELECTED[name];piece=geometry['parts'][name]['colliders'][i]
 if name!='cover' or not split:return [(i,piece)]
 v=np.asarray(piece['vertices_mm'],float);lo=v.min(0);hi=v.max(0);cut=lo[2]+SPLIT_MM
 a=hi.copy();a[2]=cut;b=lo.copy();b[2]=cut
 return [(6,box_piece(lo,a,'ear')),(7,box_piece(b,hi,'ear'))]

def expected_paths(split):
 return ['/World/Assembly_base/C_010_floor']+(['/World/Assembly_cover/C_006_ear','/World/Assembly_cover/C_007_ear'] if split else ['/World/Assembly_cover/C_006_ear'])
