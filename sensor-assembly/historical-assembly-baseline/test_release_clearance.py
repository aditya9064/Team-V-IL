import unittest,json,copy
from pathlib import Path
import numpy as np
from release_geometry import separating_gap_mm
class ReleaseClearanceChecks(unittest.TestCase):
 def test_recorded_failed_pose_interferes_old_guide_and_clears_wider_guide(self):
  p=json.loads(Path(__file__).with_name('release-failure-fixture.json').read_text());self.assertTrue(p['runtime_failure_preserved']);self.assertLess(separating_gap_mm(p['finger'],p['old_guide']),0)
  m=copy.deepcopy(p['old_guide']);m['world_vertices_m']=(np.array(m['world_vertices_m'])+np.array(p['guide_shift_world_mm'])*.001).tolist();self.assertGreater(separating_gap_mm(p['finger'],m),.08)
 def test_full_opening_clear_and_misalignment_negative_detected(self):
  p=json.loads(Path(__file__).with_name('release-clearance-proof.json').read_text());self.assertTrue(p['read_only_counterfactual_not_runtime_pass']);self.assertTrue(all(v['full_opening_interference_mm3']<1e-5 for v in p['opening_sweep'][:3]));self.assertGreater(p['opening_sweep'][-1]['full_opening_interference_mm3'],1)
