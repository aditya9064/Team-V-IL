"""Host-only controller tests; do not establish Isaac/GPU performance."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

RUNNER = Path(__file__).with_name('benchmark_capture.py').resolve()

class BenchmarkTests(unittest.TestCase):
    def test_capture_validation_and_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            starter = root/'starter'
            starter.mkdir()
            for name in ('capture_scene.py','scene_config.py','cell.py','render_settings.py'):
                (starter/name).write_text('# host test fixture only\n')
            fixture=root/'fixture.py'
            fixture.write_text('''import argparse,json
from pathlib import Path
from PIL import Image
p=argparse.ArgumentParser()
p.add_argument('--starter');p.add_argument('--out');p.add_argument('--seed',type=int)
a=p.parse_args();o=Path(a.out);o.mkdir()
im=Image.new('RGB',(3840,2160));im.putpixel((0,0),(255,255,255));im.save(o/'world_camera.png')
layout=[[0.2+i*0.02,0.1,'yellow'] for i in range(6)]
(o/'generation.json').write_text(json.dumps({'seed':a.seed,'cube_layout':layout,'cube_yaws_deg':[0]*6}))
(o/'ground_truth.json').write_text(json.dumps({'objects':[{'category':'cube','colour':x[2],'xyz':x[:2]+[0.005],'yaw_deg':0} for x in layout]}))
(o/'camera.json').write_text(json.dumps({'resolution':[3840,2160]}))
(o/'scene_state.json').write_text('{}')
''')
            common=[sys.executable,str(RUNNER),'--python',sys.executable,
                    '--starter',str(starter),'--capture',str(fixture),'--host-label','host-test']
            good=root/'good'
            result=subprocess.run(common+['--out',str(good)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr+result.stdout)
            summary=json.loads((good/'benchmark.json').read_text())
            self.assertTrue(summary['complete'])
            self.assertEqual(summary['valid_samples'],3)
            fixture.write_text(fixture.read_text()+"\n(o/'ground_truth.json').write_text('{}')\n")
            bad=root/'bad'
            result=subprocess.run(common+['--out',str(bad)],capture_output=True,text=True)
            self.assertEqual(result.returncode,1)
            summary=json.loads((bad/'benchmark.json').read_text())
            self.assertFalse(summary['complete'])
            self.assertEqual(len(summary['samples']),1)
            self.assertIsNotNone(summary['samples'][0]['error'])

if __name__=='__main__':
    unittest.main()
