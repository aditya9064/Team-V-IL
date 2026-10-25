import json,unittest,math,itertools
from pathlib import Path
import numpy as np
class NestClearanceChecks(unittest.TestCase):
 def test_cooked_wall_inputs_match_declared_clearance(self):
  g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text());walls=[np.array(v['vertices_mm']) for v in g['parts']['nest']['colliders'] if v['role']=='wall'];x=next(v for v in walls if v[:,0].min()>0);y=next(v for v in walls if v[:,1].min()>0);self.assertAlmostEqual(x[:,0].min(),34.3,places=4);self.assertAlmostEqual(y[:,1].min(),31.8,places=4);self.assertEqual(g['limits']['placement_xy_mm'],.75);self.assertEqual(g['limits']['placement_yaw_deg'],1.)
 def test_all_original_placement_tolerance_corners_clear_contact_offsets(self):
  p=json.loads(Path(__file__).with_name('nest-clearance-proof.json').read_text());self.assertLess(p['old_worst_xy_yaw_margin_mm'],0)
  for dx,dy,degrees in itertools.product([-.75,0,.75],[-.75,0,.75],[-1,0,1]):
   a=math.radians(degrees);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]);v=np.array(list(itertools.product([-32.5,32.5],[-30.,30.])))@rot.T+[dx,dy];self.assertGreater(34.3-np.abs(v[:,0]).max(),.4);self.assertGreater(31.8-np.abs(v[:,1]).max(),.4)
