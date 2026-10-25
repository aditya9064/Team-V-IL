import unittest,json,hashlib
from pathlib import Path
import numpy as np
from box_contract import bounds
class NativeAssembly(unittest.TestCase):
 def setUp(self):self.p=Path(__file__).parent;self.g=json.loads((self.p/'nest-mesh.json').read_text())
 def test_only_declared_collision_mass_readback_and_metadata_patch(self):
  s=(self.p/'native_nest_full_assembly_trial.py').read_text()
  for a,b in reversed(json.loads((self.p/'native-runtime-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(self.p/'nest_full_assembly_trial.py').read_text())
 def test_all_eight_exact_box_regions_no_volume_removed(self):
  for name,ids in {'cover':[4,5,6,7],'base':[8,9,10,11]}.items():
   for i in ids:
    q=self.g['parts'][name]['colliders'][i];b=bounds(q);self.assertAlmostEqual(float(np.prod(b['size_m'])),float(np.prod(np.ptp(q['vertices_mm'],axis=0)))*1e-9,16)
 def test_all_motion_and_numeric_acceptance_assertions_retained(self):
  s=(self.p/'check_native_nest_full.py').read_text();extra=s[s.index("    assert len(r['native_primitive_audit'])"):s.index("    assert r['physics']==")];s=s.replace(extra,'').replace("('native_nest_full_assembly_trial.py','trial_script_sha256')","('nest_full_assembly_trial.py','trial_script_sha256')");self.assertEqual(s,(self.p/'check_nest_full.py').read_text())
 def test_verified_full_seating_reference_complete(self):
  r=json.loads((self.p/'verified-seating-mass-reference.json').read_text());self.assertEqual(set(r),{'base','carrier','pcb','cover'})
  for v in r.values():self.assertTrue(v['validated']);self.assertTrue(all(x>0 for x in v['diagonal_inertia_kg_m2']))
if __name__=='__main__':unittest.main()
