import unittest,tempfile,shutil
from pathlib import Path
from team_v_industreal.fixture import verified_fixture
class DeclaredFixtureTests(unittest.TestCase):
 def test_measured_bore_and_peg_clearance_match_original_fit_limit(self):
  path,record=verified_fixture();self.assertTrue(Path(path).exists());self.assertAlmostEqual(.0081/2-record['peg_measured_max_radius_m'],.000057,places=8)
 def test_modified_mesh_cannot_pass_provenance(self):
  source=Path(__file__).parents[1]/'fixtures'
  with tempfile.TemporaryDirectory() as tmp:
   for p in source.iterdir():shutil.copy2(p,tmp)
   p=Path(tmp)/'factory_hole_declared_8p1mm.usda';p.write_bytes(p.read_bytes()+b'\n# changed\n')
   with self.assertRaisesRegex(ValueError,'checksum'):verified_fixture(tmp)
