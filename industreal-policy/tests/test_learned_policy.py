import tempfile,unittest
from pathlib import Path
import torch
from team_v_industreal.learned_policy import Policy,warm_start,gae,action_target,observation,save_checkpoint,load_checkpoint
class LearnedPolicyTests(unittest.TestCase):
 def test_gae_terminal_masks_bootstrap_and_matches_returns(self):
  a,r=gae(torch.tensor([1.,2.]),torch.tensor([.5,.6,99.]),torch.tensor([0.,1.]),gamma=1,tau=1)
  self.assertTrue(torch.allclose(r,torch.tensor([3.,2.])));self.assertTrue(torch.allclose(a,torch.tensor([2.5,1.4])))
 def test_actions_are_bounded_and_zero_rotation_is_finite(self):
  p,q=action_target(torch.zeros(1,3),torch.tensor([[1.,0,0,0]]),torch.tensor([[100.,-100.,100.,0.,0.,0.]]))
  self.assertAlmostEqual(float(p.norm()),.002,places=8);self.assertTrue(bool((p*torch.tensor([[1.,-1.,1.]])>0).all()));self.assertTrue(torch.allclose(q,torch.tensor([[1.,0,0,0]])))
  with self.assertRaises(ValueError):action_target(torch.zeros(1,3),q,torch.full((1,6),float('nan')))
 def test_observation_needs_tcp_goal_and_forces_not_live_peg_pose(self):
  q=torch.tensor([[1.,0,0,0]]);x=observation(torch.zeros(1,3),q,torch.tensor([[.001,0,0]]),q,torch.zeros(1,6),torch.zeros(1,6))
  self.assertEqual(x.shape,(1,18));self.assertAlmostEqual(float(x[0,0]),.5)
 def test_fitted_checkpoint_drives_actions_and_roundtrip_is_exact(self):
  with tempfile.TemporaryDirectory() as d:
   m=Policy();warm_start(m,d,count=512);x=torch.randn(12,18)
   path=Path(d)/'model.pt';save_checkpoint(m,path,{'fitted_policy':True});loaded,_=load_checkpoint(path,'cpu')
   self.assertTrue(torch.equal(m.mean(x),loaded.mean(x)))
   with torch.no_grad():loaded.actor.weight.zero_();loaded.actor.bias.zero_()
   self.assertFalse(torch.equal(m.mean(x),loaded.mean(x)))
 def test_split_registry_has_no_overlapping_episode_seeds(self):
  import json
  plan=json.loads((Path(__file__).parents[1]/'experiment-plan.json').read_text());groups=[set(plan[k]) for k in ['training_seeds','validation_seeds','test_seeds']]
  self.assertFalse(any(a&b for i,a in enumerate(groups) for b in groups[i+1:]));self.assertEqual(len(groups[2]),30)

 def test_ppo_updates_actor_from_aligned_rollout(self):
  from team_v_industreal.learned_policy import ppo_update
  torch.manual_seed(7);m=Policy();rollout=[]
  for i in range(40):
   x=torch.randn(1,18)
   with torch.no_grad():
    d=m.distribution(x);a=d.sample();log=d.log_prob(a).sum(-1)
   rollout.append({'obs':x,'latent_action':a,'log_prob':log,'reward':torch.tensor(float(i%7))})
  r=ppo_update(m,rollout);self.assertEqual(r['actual_simulator_transitions'],40);self.assertGreater(r['actor_parameter_l2_change'],0.)

 def test_insertion_velocity_cap_preserves_zero_action_and_direction(self):
  p=torch.zeros(1,3);q=torch.tensor([[1.,0,0,0]])
  target,_=action_target(p,q,torch.tensor([[1.,1.,1.,0,0,0]]),max_translation_m=.0004)
  self.assertAlmostEqual(float(target.norm()),.0004,places=8)
  zero,_=action_target(p,q,torch.zeros(1,6),max_translation_m=.0004);self.assertTrue(torch.equal(zero,p))

 def test_canonical_yaw_changes_frozen_goal_without_changing_grasp(self):
  from team_v_industreal.insertion_calibration import calibrate,canonical_peg_yaw
  from team_v_industreal.control import quaternion_multiply
  import math
  identity=torch.tensor([[1.,0,0,0]]);peg=torch.tensor([[math.cos((math.pi+.05)/2),0,0,math.sin((math.pi+.05)/2)]])
  c=calibrate(torch.zeros(1,3),identity,torch.zeros(1,3),peg,torch.zeros(1,3),identity);out=canonical_peg_yaw(c,identity)
  result=quaternion_multiply(out['goal_tcp_quaternion_wxyz'],out['grasp_quaternion_wxyz'])
  self.assertTrue(torch.allclose(result.abs(),torch.tensor([[0.,0,0,1.]]),atol=1e-6));self.assertTrue(torch.equal(c['grasp_quaternion_wxyz'],out['grasp_quaternion_wxyz']))
