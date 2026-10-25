"""Read-only body-origin interval estimates from already captured USD transforms."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
class PassiveVelocity:
 def __init__(self,paths):self.paths=dict(paths);self.previous=None
 def sample(self,t,matrices):
  t=float(t);current={n:np.asarray(matrices[p],float) for n,p in self.paths.items()}
  if not math.isfinite(t) or any(m.shape!=(4,4) or not np.isfinite(m).all() for m in current.values()):raise ValueError('Invalid passive pose/time')
  dt=None if self.previous is None else t-self.previous[0]
  available=dt is not None and dt>0
  if dt is not None and dt<0:raise ValueError('Passive snapshot time moved backwards')
  out={}
  for name,m in current.items():
   value={'available':available,'source':'finite_difference_existing_USD_pose_snapshots','linear_reference':'body origin, not center of mass','angular_reference':'world log(Rnow Rprev.T)/dt','interval_s':dt,'linear_world_m_s':None,'angular_world_rad_s':None}
   if available:
    before=self.previous[1][name];R=m[:3,:3].T;B=before[:3,:3].T
    value['linear_world_m_s']=((m[3,:3]-before[3,:3])/dt).tolist()
    value['angular_world_rad_s']=(Rotation.from_matrix(R@B.T).as_rotvec()/dt).tolist()
   else:value['reason']='first_sample' if self.previous is None else 'same_simulation_time'
   out[name]=value
  if self.previous is None or available:self.previous=(t,current)
  return out
