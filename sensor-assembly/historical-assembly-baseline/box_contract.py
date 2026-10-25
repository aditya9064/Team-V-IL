"""Same box-shaped product regions, native primitive versus original convex mesh."""
import numpy as np
from segmentation_contract import SELECTED,pieces,properties,expected_paths
CASES=('fixed_original','fixed_box','movable_original','movable_box')
def bounds(piece):
 v=np.asarray(piece['vertices_mm'],float);lo=v.min(0);hi=v.max(0)
 expected=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
 if v.shape!=(8,3) or not np.allclose(np.sort(v,axis=0),np.sort(expected,axis=0),atol=1e-12) or len({tuple(x) for x in v})!=8:raise ValueError('Native-box substitution requires the exact eight box corners')
 return {'centre_m':((lo+hi)*.0005).tolist(),'size_m':((hi-lo)*.001).tolist()}
