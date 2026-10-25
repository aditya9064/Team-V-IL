"""Exact contact plan for a physical, static nest; never attaches the base."""
import json,math
from pathlib import Path
import numpy as np

def guide_contact(colliders,roles,state,normals,contracts=None):
    """Return None for non-guide contacts; False for any unplanned guide touch."""
    guides=[i for i,r in enumerate(roles) if r==('nest','fixture_guide')]
    if not guides:return None
    if len(guides)!=1:return False
    i=guides[0];j=1-i
    if not state.get('fixture_entry_enabled',False) or roles[j] not in [('base','floor'),('base','wall')]:return False
    contracts=contracts if contracts is not None else json.loads(Path(__file__).with_name('nest-proof.json').read_text())['guide_pair_contracts']
    for c in contracts:
        if colliders[i]!=f'/World/Assembly_nest/C_{c["index"]:03d}_fixture_guide':continue
        if colliders[j] not in [f'/World/Assembly_base/C_{k:03d}_'+('floor' if k<12 else 'wall') for k in c['allowed_base_indices']]:return False
        a=np.asarray(normals,float);expected=np.asarray(c['normals'],float);expected/=np.linalg.norm(expected,axis=1)[:,None]
        return bool(a.ndim==2 and a.shape[1]==3 and len(a) and np.all(np.isfinite(a)) and np.all(abs(np.linalg.norm(a,axis=1)-1)<1e-3) and np.all(np.max(abs(a@expected.T),axis=1)>=math.cos(math.radians(1))))
    return False
