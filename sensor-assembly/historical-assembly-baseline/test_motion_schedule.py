import unittest
from motion_schedule import motion_samples
class PhysicsRateMotionChecks(unittest.TestCase):
 def test_every_original_phase_duration_preserved(self):
  for duration in [1.,1.5,2.,3.]:
   s=motion_samples(duration,1/60)
   self.assertEqual(len(s),round(duration*60));self.assertEqual(s[-1],(duration,1.))
   self.assertTrue(all(abs(t-(i+1)/60)<1e-12 for i,(t,_) in enumerate(s)))
 def test_old_render_rate_sampling_has_larger_position_jumps(self):
  fine=motion_samples(2.,1/60);old=motion_samples(2.,1/30)
  def largest(s):
   p=[0.]+[v for _,v in s];return max(b-a for a,b in zip(p,p[1:]))
  self.assertLess(largest(fine),.51*largest(old))
  self.assertTrue(all(b[1]>a[1] for a,b in zip(fine,fine[1:])))
 def test_invalid_or_fractional_step_duration_fails(self):
  for duration,dt in [(0,1/60),(float('nan'),1/60),(1,0),(.011,1/60)]:
   with self.assertRaises(ValueError):motion_samples(duration,dt)
if __name__=='__main__':unittest.main()
