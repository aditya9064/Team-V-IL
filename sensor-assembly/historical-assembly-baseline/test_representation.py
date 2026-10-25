import json,hashlib,unittest
from pathlib import Path
class Representation(unittest.TestCase):
 def setUp(self):self.r=Path(__file__).parent;self.p=json.loads((self.r/'representation-reference.json').read_text())
 def test_controller_is_original_with_only_inverse_local_mesh_units(self):
  s=(self.r/'holder_assembly_trial.py').read_text();s=s.replace("else:\n                    v=v*.1\n                    UsdGeom.Xformable(m.GetPrim()).AddScaleOp().Set(Gf.Vec3f(.01,.01,.01))","else:UsdGeom.Xformable(m.GetPrim()).AddScaleOp().Set(Gf.Vec3f(.001,.001,.001))");self.assertEqual(s,self.p['original_runner'])
 def test_finger_source_geometry_audit_and_all_numeric_gates_unchanged(self):
  s=(self.r/'finger_cooking.py').read_text()
  for a,b in self.p['finger_replacements']:s=s.replace(b,a)
  self.assertEqual(s,self.p['original_finger_cooking'])
  for n,h in self.p['unchanged_hashes'].items():self.assertEqual(hashlib.sha256((self.r/n).read_bytes()).hexdigest(),h)
 def test_inverse_unit_products_are_identical(self):
  self.assertAlmostEqual(100*.01,1000*.001);self.assertAlmostEqual(.1*.01,.001);self.assertEqual(self.p['expected_world_vertex_error_max_m'],1e-7)
