import ast,hashlib,json,unittest
from pathlib import Path
class FullMigration(unittest.TestCase):
 def setUp(self):self.root=Path(__file__).parent;self.proof=json.loads((self.root/'full-sequence-provenance.json').read_text())
 def test_exact_accepted_parent_restored_and_cad_unchanged(self):
  s=(self.root/'full_assembly_trial.py').read_text()
  for a,b in reversed(self.proof['replacements']):
   self.assertEqual(s.count(b),1,b[:100]);s=s.replace(b,a)
  self.assertEqual(s,(self.root/'held_base_seating_trial.py').read_text());self.assertEqual(hashlib.sha256(s.encode()).hexdigest(),self.proof['parent_sha256'])
  for n,h in self.proof['unchanged_files'].items():self.assertEqual(hashlib.sha256((self.root/n).read_bytes()).hexdigest(),h,n)
 def test_all_protected_motion_contact_capture_and_payload_gates_identical(self):
  a=(self.root/'held_base_seating_trial.py').read_text().replace('SIMULATION | RGB held-pose base seating/release | current fingers','SIMULATION | base > loaded tray > lid | current fingers');b=(self.root/'full_assembly_trial.py').read_text()
  for name in ['on_contacts','fail_contacts','advance','move','command','feedback','save','poses','score_placed','check_inventory','measure_held']:
   values=[next(n for n in ast.walk(ast.parse(s)) if isinstance(n,ast.FunctionDef) and n.name==name) for s in [a,b]];self.assertEqual(ast.dump(values[0]),ast.dump(values[1]),name)
 def test_base_measurement_and_staged_release_branches_are_exact_parent(self):
  a=(self.root/'held_base_seating_trial.py').read_text();b=(self.root/'full_assembly_trial.py').read_text()
  for start,end in [("            observed=measure_held('1_base_held_inspection')","            state['held_reference']=None"),("            release_plan=square_release_plan(","            release=move(prefix+'_release'")]:
   original=a[a.index(start):a.index(end,a.index(start))];nested=''.join('    '+line+'\n' for line in original.rstrip().splitlines());self.assertIn("            if name=='base':\n"+nested,b)
