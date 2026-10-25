import json,subprocess,sys,tempfile,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
class ExternalTiming(unittest.TestCase):
 def test_mirrors_output_and_exit(self):
  with tempfile.TemporaryDirectory() as d:
   events=Path(d)/'timing.jsonl'
   r=subprocess.run([sys.executable,str(HERE/'execution_timing.py'),'--events',str(events),'--',sys.executable,'-u','-c',"print('[assembly] test');raise SystemExit(7)"],text=True,capture_output=True)
   self.assertEqual(r.returncode,7);self.assertEqual(r.stdout,'[assembly] test\n');rows=[json.loads(s) for s in events.read_text().splitlines()]
   self.assertEqual([v['kind'] for v in rows],['process_start','runtime_marker','process_exit']);self.assertEqual(rows[-1]['returncode'],7);self.assertTrue(all(v['elapsed_wall_s']>=0 for v in rows))
 def test_existing_evidence_not_overwritten(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'events';p.write_text('preserve')
   r=subprocess.run([sys.executable,str(HERE/'execution_timing.py'),'--events',str(p),'--',sys.executable,'-c','raise SystemExit(0)'],capture_output=True)
   self.assertNotEqual(r.returncode,0);self.assertEqual(p.read_text(),'preserve')
