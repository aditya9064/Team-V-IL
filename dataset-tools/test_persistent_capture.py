"""Host checks for source compatibility and saved-data rejection, not GPU validation."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from persistent_capture import prepare_capture, validate_capture

ROOT = Path(__file__).resolve().parents[1]
STARTER = Path(os.environ.get('FANUC_STARTER',ROOT/'data/external/fanuc-v05/fanuc_er4ia_sim'))
SAMPLE = Path(os.environ.get('TEAM_V_CAPTURE_SAMPLE',ROOT/'benchmark-results/brev-l40s-20261014/team-v-benchmark-results/seed-424201'))

class PersistentCaptureTests(unittest.TestCase):
    @unittest.skipUnless((STARTER/'capture_scene.py').is_file(),
                         'Optional integration fixture: set FANUC_STARTER to the official starter')
    def test_known_source_and_changed_lifecycle(self):
        source = (STARTER/'capture_scene.py').read_text()
        prepared = prepare_capture(source)
        self.assertNotIn('sim_app = SimulationApp(', prepared)
        self.assertNotIn('sim_app.close()', prepared)
        self.assertIn('for _ in range(90)', prepared)
        self.assertIn('gt = {"objects": [], "bins": {}}', prepared)
        with self.assertRaises(ValueError):
            prepare_capture(source.replace('sim_app.close()', 'sim_app.close(wait_for_replicator=True)'))

    @unittest.skipUnless((SAMPLE/'generation.json').is_file(),
                         'Optional integration fixture: set TEAM_V_CAPTURE_SAMPLE to a saved capture')
    def test_real_saved_capture_and_stage_mismatch(self):
        generation = json.loads((SAMPLE/'generation.json').read_text())
        self.assertEqual(validate_capture(SAMPLE,generation)['cube_count'],6)
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'sample'
            target.mkdir()
            from PIL import Image
            image=Image.new('RGB',(3840,2160)); image.putpixel((0,0),(255,255,255))
            image.save(target/'world_camera.png')
            for name in ('generation.json','camera.json','ground_truth.json','scene_state.json'):
                (target/name).write_bytes((SAMPLE/name).read_bytes())
            state=json.loads((target/'scene_state.json').read_text())
            state['objects']['SortCube0']['position'][0] += 0.01
            (target/'scene_state.json').write_text(json.dumps(state))
            with self.assertRaisesRegex(ValueError,'captured stage disagree'):
                validate_capture(target,generation)
            (target/'scene_state.json').write_bytes((SAMPLE/'scene_state.json').read_bytes())
            truth=json.loads((target/'ground_truth.json').read_text())
            truth['objects'][0]['colour']='wrong'
            (target/'ground_truth.json').write_text(json.dumps(truth))
            with self.assertRaisesRegex(ValueError,'colour mismatch'):
                validate_capture(target,generation)

if __name__=='__main__':
    unittest.main()
