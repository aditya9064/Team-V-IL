import ast,hashlib,json,tempfile,unittest
from pathlib import Path
from json_token_equivalence import assert_equivalent
HERE=Path(__file__).resolve().parent
class CompactRuntime(unittest.TestCase):
 def test_exact_single_change(self):
  parent=(HERE/'v58b_original_parent.py').read_text(); child=(HERE/'original_lid_placement_trial.py').read_text()
  self.assertEqual(hashlib.sha256(parent.encode()).hexdigest(),'0504fb48d663a9226a597d2ce31e87a1374e5bbafe1d4399f7cd9f9ad9baccaa')
  self.assertEqual(child.replace("json.dumps(report,separators=(',',':'))","json.dumps(report,indent=2)"),parent)
 def test_real_persist_data_equivalence(self):
  report={'poses':[[0.,-0.,.05],{'name':'a  b\\n'}],'errors':[],'state':None}
  with tempfile.TemporaryDirectory() as d:
   texts=[]
   for name in ['v58b_original_parent.py','original_lid_placement_trial.py']:
    node=next(n for n in ast.walk(ast.parse((HERE/name).read_text())) if isinstance(n,ast.FunctionDef) and n.name=='persist')
    scope={'report':report,'out':Path(d),'json':json};exec(compile(ast.Module(body=[node],type_ignores=[]),name,'exec'),scope);scope['persist']();texts.append((Path(d)/'report.json').read_text())
   assert_equivalent(*texts);self.assertEqual(json.loads(texts[0]),json.loads(texts[1]))
