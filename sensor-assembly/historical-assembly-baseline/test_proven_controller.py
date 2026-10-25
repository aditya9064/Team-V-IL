import json,hashlib,unittest
from pathlib import Path
class ProvenController(unittest.TestCase):
 def test_current_pinned_controller_and_original_checker(self):
  r=Path(__file__).parent;p=json.loads((r/'proven-controller-proof.json').read_text())
  for n,key in [('holder_assembly_trial.py','controller_sha256'),('check_holder_assembly.py','checker_sha256')]:self.assertEqual(hashlib.sha256((r/n).read_bytes()).hexdigest(),p[key])
  self.assertTrue(p['original_phase_durations_restored'] and p['all_numeric_thresholds_unchanged'])
 def test_physical_gates_and_new_support_surface_are_pinned(self):
  r=Path(__file__).parent;p=json.loads((r/'proven-controller-proof.json').read_text())
  for n,h in p['source_physical_gate_sha256'].items():self.assertEqual(hashlib.sha256((r/n).read_bytes()).hexdigest(),h)
  self.assertTrue(p['payload_only_declared_horizontal_floor_pair_changed'])
