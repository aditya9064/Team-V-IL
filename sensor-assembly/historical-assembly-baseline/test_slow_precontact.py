import ast,copy,hashlib,json,unittest
from pathlib import Path
import numpy as np
from motion_schedule import motion_samples
from check_slow_lid_precontact import ORIGINAL_PHASE_DURATIONS,validate_slow_precontact

class SlowPrecontact(unittest.TestCase):
 def setUp(self):self.p=Path(__file__).parent
 def test_reversible_runtime_and_frozen_v47(self):
  old=(self.p/'passive_lid_precontact_trial.py').read_text();s=(self.p/'slow_lid_precontact_trial.py').read_text()
  for a,b in reversed(json.loads((self.p/'slow-runtime-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,old);self.assertEqual(hashlib.sha256(old.encode()).hexdigest(),'de2fdc9f20685a1308f84da8077d32001ca2d957d5868c332bb964d2700ecbaa')
 def test_reversible_checker(self):
  s=(self.p/'check_slow_lid_precontact.py').read_text()
  for a,b in reversed(json.loads((self.p/'slow-checker-patches.json').read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(self.p/'check_passive_lid_precontact.py').read_text())
 def test_all_inherited_control_functions_identical(self):
  trees=[ast.parse((self.p/n).read_text()) for n in ('passive_lid_precontact_trial.py','slow_lid_precontact_trial.py')]
  for name in ('advance','move','command','feedback','poses','contact_pose_snapshot','measure_held','measure_lid','lid_clearance_guard','on_contacts','fail_contacts','score_placed','check_inventory'):
   nodes=[next(x for x in ast.walk(t) if isinstance(x,ast.FunctionDef) and x.name==name) for t in trees];self.assertEqual(ast.dump(nodes[0]),ast.dump(nodes[1]),name)
 def test_only_motion_call_duration_changes(self):
  trees=[ast.parse((self.p/n).read_text()) for n in ('passive_lid_precontact_trial.py','slow_lid_precontact_trial.py')]
  calls=[[n for n in ast.walk(t) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='move'] for t in trees]
  self.assertEqual(len(calls[0]),len(calls[1]));changes=[]
  for a,b in zip(*calls):
   if ast.dump(a)!=ast.dump(b):
    self.assertEqual(ast.unparse(a.args[0]),"prefix + '_precontact'");self.assertEqual(a.args[3].value,2.);self.assertEqual(b.args[3].value,4.);changes.append(a)
    b=copy.deepcopy(b);b.args[3]=copy.deepcopy(a.args[3]);self.assertEqual(ast.dump(a),ast.dump(b))
  self.assertEqual(len(changes),1)
 def test_original_curve_only_resampled_slower(self):
  before=motion_samples(2.,1/60);after=motion_samples(4.,1/60);self.assertEqual((len(before),len(after)),(120,240))
  for (ta,sa),(tb,sb) in zip(before,after[1::2]):self.assertAlmostEqual(tb,2*ta);self.assertAlmostEqual(sb,sa,places=14)
 def fixture(self):
  tree=ast.parse((self.p/'slow_lid_precontact_trial.py').read_text())
  node=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and len(n.targets)==1 and ast.unparse(n.targets[0])=="report['slow_precontact_profile']")
  env={'report':{},'goal':np.array([.3,.1,.15]),'precontact':np.array([.3,.1,.035]),'path_length':.115}
  exec(compile(ast.Module(body=[node],type_ignores=[]),'metadata-only','exec'),env)
  rows=[{'phase':phase,'requested_duration_s':4. if phase=='3_cover_precontact' else duration,'commands':round((4. if phase=='3_cover_precontact' else duration)*60)} for phase,duration in ORIGINAL_PHASE_DURATIONS.items()]
  return {'motion_sampling':{'phases':rows},'slow_precontact_profile':env['report']['slow_precontact_profile']}
 def test_runtime_analytic_bounds_and_all_duration_contracts(self):
  r=self.fixture();validate_slow_precontact(r);self.assertEqual(len(r['motion_sampling']['phases']),37);v=r['slow_precontact_profile'];self.assertAlmostEqual(v['peak_speed_m_s'],.043125);self.assertAlmostEqual(v['peak_acceleration_m_s2'],.043125);self.assertAlmostEqual(v['interior_jerk_m_s3'],.0215625)
 def test_negative_duration_commands_and_other_phase_budgets(self):
  good=self.fixture()
  for phase in ORIGINAL_PHASE_DURATIONS:
   bad=copy.deepcopy(good);row=next(x for x in bad['motion_sampling']['phases'] if x['phase']==phase);row['requested_duration_s']+=1/60
   with self.assertRaises(AssertionError,msg=phase):validate_slow_precontact(bad)
  for key,value in [('commands',120),('requested_duration_s',2.)]:
   bad=copy.deepcopy(good);bad['motion_sampling']['phases'][-1][key]=value
   with self.assertRaises(AssertionError):validate_slow_precontact(bad)
  for key,value in [('duration_ratio',1.),('peak_acceleration_m_s2',.01),('path_length_m',float('nan')),('grasp_reference_unchanged',False),('endpoint_acceleration_discontinuity_retained',False)]:
   bad=copy.deepcopy(good);bad['slow_precontact_profile'][key]=value
   with self.assertRaises(AssertionError,msg=key):validate_slow_precontact(bad)
if __name__=='__main__':unittest.main()
