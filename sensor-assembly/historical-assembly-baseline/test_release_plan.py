import unittest,json
from pathlib import Path
import numpy as np
from release_plan import square_release_plan
class StagedRelease(unittest.TestCase):
 def test_recipe_tip_and_axes(self):
  p=square_release_plan([.3,.1,.8979],[16.5,0],13.228201,[1,0,0],[0,1,0],'Y');np.testing.assert_allclose(p['tip_world_m'],[.3165,.1,.911128201]);R=np.array(p['tool_rotation_world']);np.testing.assert_allclose(R@np.array([1,0,0]),[0,0,-1]);np.testing.assert_allclose(R@np.array([0,1,0]),[0,1,0]);self.assertAlmostEqual(np.linalg.det(R),1.)
 def test_bad_axes_and_nonfinite_rejected(self):
  with self.assertRaises(ValueError):square_release_plan([0,0,0],[0,0],13,[1,0,0],[1,0,0],'X')
  with self.assertRaises(ValueError):square_release_plan([0,0,np.nan],[0,0],13,[1,0,0],[0,1,0],'Y')
 def test_staged_sweep_has_no_unintended_interference_and_open_is_clear(self):
  p=json.loads(Path(__file__).with_name('staged-release-plan-proof.json').read_text());self.assertTrue(p['no_gate_changes']);self.assertEqual(len(p['samples']),33)
  for v in p['samples']:
   for k in ['wall','floor','grip_root']:self.assertLess(v['overlap_mm3_by_role'][k],1e-6)
   if v['stage']=='open':self.assertTrue(all(n<1e-5 for n in v['overlap_mm3_by_role'].values()))
