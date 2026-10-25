import hashlib,inspect,json,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
from scipy.spatial.transform import Rotation
import held_pose_bounded as candidate
import test_held_pose as fixtures
rendered=fixtures.rendered

class HeldBudget(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  fixtures.HeldPoseTests.setUpClass();cls.mesh=fixtures.HeldPoseTests.mesh;cls.nominal=fixtures.HeldPoseTests.nominal;cls.models=fixtures.HeldPoseTests.models
 def images(self):
  R=Rotation.from_euler('xyz',[.6,-1.4,.4],degrees=True).as_matrix();pos=self.nominal+[.00012,-.00008,-.0006]
  return {n:rendered(self.mesh,pos,R,m) for n,m in self.models.items()},pos,R
 def test_only_budget_and_diagnostic_changes(self):
  p=Path(__file__).parent;c=json.loads((p/'held-budget-contract.json').read_text());s=(p/'held_pose_bounded.py').read_text()
  for old,new in reversed(c['patches']):self.assertEqual(s.count(new),1);s=s.replace(new,old)
  self.assertEqual(s,(p/'held_pose.py').read_text())
  for n,h in c['historical_sources_unchanged'].items():self.assertEqual(hashlib.sha256((p/n).read_bytes()).hexdigest(),h,n)
 def test_only_runner_import_changes(self):
  p=Path(__file__).parent;c=json.loads((p/'held-budget-contract.json').read_text());s=(p/'bounded_nest_full_assembly_trial.py').read_text()
  for a,b in reversed(c['runner_patches']):self.assertEqual(s.count(b),1);s=s.replace(b,a)
  self.assertEqual(s,(p/'native_nest_full_assembly_trial.py').read_text())
 def test_all_original_independent_acceptance_checks_retained(self):
  p=Path(__file__).parent;s=(p/'check_bounded_nest_full.py').read_text();extra="        assert v['optimizer']['success'] and v['optimizer']['status']>0 and 0<v['optimizer']['nfev']<=v['optimizer']['max_nfev']==400\n";self.assertEqual(s.count(extra),1);s=s.replace(extra,'').replace("    assert r['held_pose_estimator_sha256']==digest(source/'held_pose_bounded.py')\n",'').replace("('bounded_nest_full_assembly_trial.py','trial_script_sha256')","('native_nest_full_assembly_trial.py','trial_script_sha256')");self.assertEqual(s,(p/'check_native_nest_full.py').read_text())
 def test_rgb_fit_has_original_accuracy_and_capped_budget(self):
  images,pos,R=self.images();fit=candidate.estimate(images,self.models,self.mesh['vertices_mm'],self.nominal)
  self.assertLessEqual(np.linalg.norm((np.array(fit['position_world_m'])-pos)*1000),.15);self.assertLessEqual(Rotation.from_matrix(np.array(fit['rotation_world']).T@R).magnitude()*180/np.pi,.25)
  self.assertTrue(fit['optimizer']['success']);self.assertEqual(fit['optimizer']['max_nfev'],400);self.assertLessEqual(fit['function_evaluations'],400)
 def test_optimizer_exhaustion_rejected_despite_good_edges(self):
  images,_,_=self.images();original=candidate.least_squares
  def exhausted(*a,**k):
   self.assertEqual(k['max_nfev'],400);fit=original(*a,**k);fit.success=False;fit.status=0;fit.message='Test controlled exhaustion';return fit
  with patch.object(candidate,'least_squares',side_effect=exhausted):
   with self.assertRaisesRegex(ValueError,'Test controlled exhaustion'):candidate.estimate(images,self.models,self.mesh['vertices_mm'],self.nominal)
 def test_empty_images_rejected(self):
  with self.assertRaises(ValueError):candidate.estimate({n:np.zeros((960,1280,3),np.uint8) for n in self.models},self.models,self.mesh['vertices_mm'],self.nominal)
 def test_occlusion_rejected(self):
  images={n:rendered(self.mesh,self.nominal,np.eye(3),m) for n,m in self.models.items()};images['x'][480:,:,:]=150
  with self.assertRaises(ValueError):candidate.estimate(images,self.models,self.mesh['vertices_mm'],self.nominal)
 def test_truth_free_api_unchanged(self):self.assertEqual(list(inspect.signature(candidate.estimate).parameters),['images','models','vertices_mm','nominal_position'])
if __name__=='__main__':unittest.main()
