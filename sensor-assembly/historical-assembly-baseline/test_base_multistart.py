import ast,copy,inspect,json,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2,numpy as np
from scipy.spatial.transform import Rotation
import held_pose_multistart as multi
from check_multistart_lid_precontact import validate_multistart_observation
class BaseMultistart(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.p=Path(__file__).parent;cls.evidence=cls.p/'fit-regression-286834';cls.meta=json.loads((cls.evidence/'report-subset.json').read_text());cls.geometry=json.loads((cls.p/'nest-mesh.json').read_text());cls.design=json.loads((cls.p/'nest-design.json').read_text());cls.vertices=cls.geometry['parts']['base']['visual']['vertices_mm'];cls.results=[]
  for obs in cls.meta['held_pose_observations']:
   images={k:cv2.cvtColor(cv2.imread(str(cls.evidence/(obs['label']+'-'+k+'.png'))),cv2.COLOR_BGR2RGB) for k in obs['camera_models']};nominal=np.array(obs['tip_world_m'])-np.array([*np.array(cls.design['base']['handle_centre_xy'])*.001,cls.design['base']['tcp_above_bottom_mm']*.001]);value=multi.estimate(images,obs['camera_models'],cls.vertices,nominal);value.update(label=obs['label'],tip_world_m=obs['tip_world_m'],camera_models=obs['camera_models']);cls.results.append((value,images,nominal))
 def test_failed834_pair_image_rank_old_accuracy_and_repeatability(self):
  for v,images,nominal in self.results:
   validate_multistart_observation(v,self.evidence,self.vertices,self.design);truth=next(s['poses']['base'] for s in self.meta['checkpoints'] if s['phase']==v['label']);q=truth['orientation_wxyz'];R=Rotation.from_quat([*q[1:],q[0]]).as_matrix();self.assertLessEqual(np.linalg.norm((np.array(v['position_world_m'])-truth['position_world_m'])*1000),.15);self.assertLessEqual(Rotation.from_matrix(np.array(v['rotation_world']).T@R).magnitude()*180/np.pi,.25)
  a,b=[x[0] for x in self.results];self.assertLessEqual(np.linalg.norm((np.array(a['position_world_m'])-b['position_world_m'])*1000),.1);self.assertLessEqual(Rotation.from_matrix(np.array(a['rotation_world']).T@b['rotation_world']).magnitude()*180/np.pi,.15)
 def test_wrong_color_blank_missing_view_and_occluded_edge(self):
  v,images,nominal=self.results[0];models=v['camera_models']
  for bad in [{k:np.zeros_like(a) for k,a in images.items()},{k:np.full_like(a,[255,0,0]) for k,a in images.items()},{next(iter(images)):next(iter(images.values()))}]:
   with self.assertRaises(ValueError):multi.estimate(bad,models,self.vertices,nominal)
  obscured={k:x.copy() for k,x in images.items()};name=next(iter(images));uv=multi.project(np.array(self.vertices)*.001+nominal,models[name]);x0,y0=np.floor(uv.min(0)).astype(int);x1,y1=np.ceil(uv.max(0)).astype(int);obscured[name][(y0+y1)//2:,:,:]=150
  with self.assertRaises(ValueError):multi.estimate(obscured,models,self.vertices,nominal)
 def mock_problem(self):
  return (np.zeros(3),[(name,None,None,None,np.zeros((2000,1,2))) for name in ('x','y')],lambda x:np.zeros(280))
 def perfect_fit(self,*args,**kwargs):return SimpleNamespace(x=np.zeros(6),cost=0.,optimality=0.,success=True,status=1,message='perfect',nfev=1,njev=1)
 def test_full25_even_first_perfect_and_seedindex_tie(self):
  with patch.object(multi,'image_problem',return_value=self.mock_problem()),patch.object(multi,'least_squares',side_effect=self.perfect_fit) as solve:
   v=multi.estimate({'x':np.zeros((1,1,3),np.uint8),'y':np.zeros((1,1,3),np.uint8)},{'x':{},'y':{}},[[0,0,0]],np.zeros(3))
  self.assertEqual(solve.call_count,25);self.assertEqual([call.args[1].tolist() for call in solve.call_args_list],multi.fixed_seed_bank());self.assertEqual(v['multistart']['selected_seed_index_IMAGE_ONLY'],0)
  for c in solve.call_args_list:self.assertEqual(c.kwargs,multi.SOLVER_OPTIONS)
 def test_invalid_solver_and_zeroeligible_hold_logs_complete_bank(self):
  cases=[{'success':False},{'cost':float('nan')},{'cost':-1.},{'x':np.full(6,np.nan)},{'optimality':float('nan')},{'x':np.array([2.,0,0,0,0,0])}]
  for overrides in cases:
   def fake(*a,**k):v=self.perfect_fit();v.__dict__.update(overrides);return v
   with patch.object(multi,'image_problem',return_value=self.mock_problem()),patch.object(multi,'least_squares',side_effect=fake) as solve:
    with self.assertRaisesRegex(ValueError,'multistart HOLD') as err:multi.estimate({'x':np.zeros((1,1,3),np.uint8),'y':np.zeros((1,1,3),np.uint8)},{'x':{},'y':{}},[[0,0,0]],np.zeros(3))
   self.assertEqual(solve.call_count,25);audit=json.loads(str(err.exception).split('HOLD: ',1)[1]);self.assertIsNone(audit['selected_seed_index_IMAGE_ONLY']);self.assertEqual(len(audit['candidates']),25);self.assertTrue(all(not c['image_eligible'] for c in audit['candidates']))
 def test_zero_ineligible_others_eligible_still_HOLD_full25(self):
  calls=[]
  def fake(*args,**kwargs):
   result=self.perfect_fit();result.success=bool(calls);calls.append(args[1].tolist());return result
  with patch.object(multi,'image_problem',return_value=self.mock_problem()),patch.object(multi,'least_squares',side_effect=fake):
   with self.assertRaisesRegex(ValueError,'multistart HOLD') as err:multi.estimate({'x':np.zeros((1,1,3),np.uint8),'y':np.zeros((1,1,3),np.uint8)},{'x':{},'y':{}},[[0,0,0]],np.zeros(3))
  audit=json.loads(str(err.exception).split('HOLD: ',1)[1]);self.assertEqual(len(calls),25);self.assertFalse(audit['candidates'][0]['image_eligible']);self.assertTrue(all(c['image_eligible'] for c in audit['candidates'][1:]));self.assertIsNone(audit['selected_seed_index_IMAGE_ONLY']);self.assertFalse(audit['original_zero_seed_gate_pass'])
 def test_exact32px_loweredge_occlusion_old_and_new_both_reject(self):
  import held_pose_bounded as original
  v,images,nominal=self.results[1];models=v['camera_models'];bad={k:x.copy() for k,x in images.items()}
  for k in bad:
   uv=np.rint(multi.project(multi.front_edges(self.vertices,nominal,models[k])+nominal,models[k])).astype(np.int32)
   cv2.polylines(bad[k],[uv[:70]],False,(0,0,0),32);cv2.polylines(bad[k],[uv[70:]],False,(0,0,0),32)
  with self.assertRaises(ValueError):original.estimate(bad,models,self.vertices,nominal)
  with self.assertRaisesRegex(ValueError,'multistart HOLD') as err:multi.estimate(bad,models,self.vertices,nominal)
  audit=json.loads(str(err.exception).split('HOLD: ',1)[1]);self.assertEqual(len(audit['candidates']),25);self.assertFalse(audit['candidates'][0]['image_eligible']);self.assertTrue(any(c['image_eligible'] for c in audit['candidates'][1:]));self.assertIsNone(audit['selected_seed_index_IMAGE_ONLY'])
 def test_checker_rank_rawpixel_hash_and_nan_score_negatives(self):
  v,images,nominal=self.results[0]
  for mutate in [lambda x:x['multistart'].update(selected_seed_index_IMAGE_ONLY=(x['multistart']['selected_seed_index_IMAGE_ONLY']+1)%25),lambda x:x['multistart']['candidates'][0].update(image_eligible=False),lambda x:x['multistart']['candidates'][0]['views'][next(iter(x['camera_models']))].update(edge_p95_px=float('nan')),lambda x:x['multistart']['candidates'][0].update(cost=0),lambda x:x['image_RGB_sha256'].update({next(iter(x['camera_models'])):'bad'}),lambda x:x['camera_model_sha256'].update({next(iter(x['camera_models'])):'bad'}),lambda x:x.update(CAD_vertices_mm_sha256='bad')]:
   bad=copy.deepcopy(v);mutate(bad)
   with self.assertRaises((AssertionError,ValueError)):validate_multistart_observation(bad,self.evidence,self.vertices,self.design)
  bad=copy.deepcopy(v['multistart']['candidates']);bad.pop()
  with self.assertRaises(ValueError):multi.select_image_only(bad)
 def test_no_truth_input_and_original_problem_text_exact(self):
  self.assertEqual(list(inspect.signature(multi.estimate).parameters),['images','models','vertices_mm','nominal_position'])
  old=(self.p/'held_pose_bounded.py').read_text();new=(self.p/'held_pose_multistart.py').read_text();body=old[old.index('    nominal=np.asarray(nominal_position,float)'):old.index('    fit=least_squares(')];self.assertIn(body,new)
  tree=ast.parse(new)
  for n in ast.walk(tree):
   if isinstance(n,ast.Call) and isinstance(n.func,ast.Name):self.assertNotIn(n.func.id,('open','eval','exec'))
 def test_reversible_runtime_checker_and_all_control_AST(self):
  for child,parent,pf in [('multistart_lid_precontact_trial.py','continuous_lid_precontact_trial.py','multistart-runtime-patches.json'),('check_multistart_lid_precontact.py','check_continuous_lid_precontact.py','multistart-checker-patches.json')]:
   s=(self.p/child).read_text()
   for a,b in reversed(json.loads((self.p/pf).read_text())):self.assertEqual(s.count(b),1);s=s.replace(b,a)
   self.assertEqual(s,(self.p/parent).read_text())
  trees=[ast.parse((self.p/n).read_text()) for n in ('continuous_lid_precontact_trial.py','multistart_lid_precontact_trial.py')];fns=[{n.name:ast.dump(n) for n in ast.walk(t) if isinstance(n,ast.FunctionDef)} for t in trees];self.assertEqual(set(fns[0]),set(fns[1]))
  for name in fns[0]:
   if name!='main':self.assertEqual(fns[0][name],fns[1][name],name)
if __name__=='__main__':unittest.main()
