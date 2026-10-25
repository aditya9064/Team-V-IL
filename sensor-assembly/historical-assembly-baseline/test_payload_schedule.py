import ast,hashlib,json,unittest
from pathlib import Path
import numpy as np
from payload_schedule import payload_carry_samples,profile_bounds
class PayloadSchedule(unittest.TestCase):
 def test_bounds_zero_endpoint_acceleration_and_lower_excitation(self):
  t=np.array([[0,0],*payload_carry_samples(4.,1/60)]);x=t[:,1]*np.hypot(.1,.26);v=np.diff(x)*60;acc=np.diff(v)*60;b=profile_bounds(np.hypot(.1,.26));self.assertEqual(len(t),241);self.assertAlmostEqual(t[-1,1],1);self.assertTrue(np.all(np.diff(t[:,1])>0));self.assertLess(max(abs(acc)),.101);self.assertLess(abs(acc[0]),.005);self.assertLess(abs(acc[-1]),.005);self.assertLess(b['peak_acceleration_m_s2']/b['old_peak_acceleration_m_s2'],.241)
 def test_invalid_durations_rejected(self):
  for d in [0,2,3,float('nan')]:
   with self.assertRaises(ValueError):payload_carry_samples(d,1/60)
 def test_exact_reversal_to_failed_parent_and_physics_cad_checks_unchanged(self):
  p=Path(__file__).parent;proof=json.loads((p/'gentle-carry-provenance.json').read_text());s=(p/'gentle_full_assembly_trial.py').read_text()
  for a,b in reversed(proof['replacements']):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(p/'full_assembly_trial.py').read_text())
  for n,h in proof['unchanged_files'].items():self.assertEqual(hashlib.sha256((p/n).read_bytes()).hexdigest(),h,n)
 def test_contact_monitor_payload_release_and_camera_gates_identical(self):
  p=Path(__file__).parent;sources=[ast.parse((p/n).read_text()) for n in ['full_assembly_trial.py','gentle_full_assembly_trial.py']]
  for name in ['on_contacts','fail_contacts','advance','command','feedback','save','poses','score_placed','check_inventory','measure_held']:
   functions=[next(n for n in ast.walk(s) if isinstance(n,ast.FunctionDef) and n.name==name) for s in sources];self.assertEqual(ast.dump(functions[0]),ast.dump(functions[1]),name)
