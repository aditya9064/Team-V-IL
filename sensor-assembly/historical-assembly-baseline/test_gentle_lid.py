from pathlib import Path
import hashlib,json,unittest,ast
import numpy as np
from lid_release_schedule import lid_release_samples,lid_release_bounds
class GentleLid(unittest.TestCase):
 def setUp(self):self.p=Path(__file__).parent
 def test_only_declared_jaw_profile_and_duration_change(self):
  s=(self.p/'gentle_lid_full_assembly_trial.py').read_text()
  for a,b in reversed(json.loads((self.p/'gentle-lid-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(self.p/'bounded_nest_full_assembly_trial.py').read_text())
 def test_all_original_acceptance_checks_preserved(self):
  s=(self.p/'check_gentle_lid_full.py').read_text()
  for a,b in reversed(json.loads((self.p/'gentle-lid-checker-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(self.p/'check_bounded_nest_full.py').read_text())
 def test_hardware_geometry_and_estimators_unchanged(self):
  for n,h in json.loads((self.p/'gentle-lid-contract.json').read_text())['geometry_fingers_unchanged'].items():self.assertEqual(hashlib.sha256((self.p/n).read_bytes()).hexdigest(),h,n)
 def test_profile_same_endpoints_monotonic_and_fixed_cadence(self):
  v=lid_release_samples(3.,1/60);self.assertEqual(len(v),180);np.testing.assert_allclose(np.diff([0]+[t for t,s in v]),1/60,atol=1e-15);self.assertEqual(v[-1],(3.,1.));self.assertTrue(np.all(np.diff([0]+[s for t,s in v])>=0))
  for u in (0,1):self.assertEqual(30*u*u-60*u**3+30*u**4,0);self.assertEqual(60*u-180*u*u+120*u**3,0)
 def test_lower_speed_acceleration_and_no_distance_change(self):
  b=lid_release_bounds(.004);self.assertLess(b['speed_ratio'],.42);self.assertLess(b['acceleration_ratio'],.107);self.assertEqual(b['same_jaw_opening_distance_m'],.004)
  for v in (float('nan'),0,-1,.006):
   with self.assertRaises(ValueError):lid_release_bounds(v)
  with self.assertRaises(ValueError):lid_release_samples(1.,1/60)
 def test_contact_and_capture_functions_exact_parent(self):
  sources=[ast.parse((self.p/n).read_text()) for n in ('bounded_nest_full_assembly_trial.py','gentle_lid_full_assembly_trial.py')]
  for name in ('on_contacts','advance','fail_contacts','save','measure_held','score_placed','check_inventory','feedback'):
   nodes=[next(n for n in ast.walk(s) if isinstance(n,ast.FunctionDef) and n.name==name) for s in sources];self.assertEqual(ast.dump(nodes[0]),ast.dump(nodes[1]),name)
if __name__=='__main__':unittest.main()
