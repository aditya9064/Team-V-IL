import unittest
from passive_sleep_state import PassiveSleepState, BODY_NAMES

class Interface:
 def __init__(self): self.stage=7;self.calls=[];self.value=False;self.query_error=None;self.switch=False
 def get_attached_stage(self):
  self.calls.append('stage');return self.stage
 def is_sleeping(self, stage, body):
  self.calls.append(('sleep',stage,body))
  if self.query_error:raise self.query_error
  if self.switch:self.stage=8
  return self.value

class PassiveTests(unittest.TestCase):
 def observer(self, api=None):
  api=api or Interface();return api,PassiveSleepState(api,7,dict(zip(sorted(BODY_NAMES),range(10,14))),{n:'/World/Assembly_'+n for n in BODY_NAMES})
 def test_all_awake_is_valid_not_missing(self):
  api,o=self.observer();r=o.capture(1,1/60,'release');self.assertTrue(r['valid']);self.assertEqual(r['sleeping'],dict.fromkeys(BODY_NAMES,False));self.assertEqual(len(api.calls),6)
 def test_all_asleep_does_not_satisfy_force_gate(self):
  api,o=self.observer();api.value=True;r=o.capture(1,1/60,'final');self.assertTrue(r['valid']);self.assertTrue(all(r['sleeping'].values()));self.assertFalse(o.evidence()['force_gate_satisfied_by_sleep'])
 def test_wrong_stage_is_unknown(self):
  api,o=self.observer();api.stage=8;r=o.capture(1,1/60,'final');self.assertFalse(r['valid']);self.assertIsNone(r['sleeping']);self.assertEqual(api.calls,['stage'])
 def test_missing_stage_is_unknown(self):
  api,o=self.observer();api.stage=0;self.assertFalse(o.capture(1,1/60,'final')['valid'])
 def test_stage_switch_during_read_is_unknown(self):
  api,o=self.observer();api.switch=True;self.assertFalse(o.capture(1,1/60,'final')['valid'])
 def test_query_error_is_unknown_does_not_raise_or_retry(self):
  api,o=self.observer();api.query_error=RuntimeError('missing actor');r=o.capture(1,1/60,'final');self.assertFalse(r['valid']);self.assertIsNone(r['sleeping']);self.assertEqual(len(api.calls),2)
 def test_nonboolean_is_unknown(self):
  for value in [0,1,None,'false',[],{}]:
   api,o=self.observer();api.value=value;self.assertFalse(o.capture(1,1/60,'final')['valid'])
 def test_missing_step_invalidates_trace(self):
  api,o=self.observer();o.capture(1,1/60,'hold');self.assertFalse(o.capture(3,3/60,'hold')['valid']);self.assertFalse(o.evidence()['all_samples_valid'])
 def test_duplicate_step_is_invalid(self):
  api,o=self.observer();o.capture(1,1/60,'hold');self.assertFalse(o.capture(1,1/60,'hold')['valid'])
 def test_bad_time_or_step_does_not_query(self):
  for step,t in [(True,1.),(0,1.),(1,float('nan')),(1,float('inf')),(1,-1.0),(1,True)]:
   api,o=self.observer();self.assertFalse(o.capture(step,t,'hold')['valid']);self.assertEqual(api.calls,[])
 def test_stage_boolean_or_aliased_bodies_rejected(self):
  api=Interface();paths={n:'/World/Assembly_'+n for n in BODY_NAMES}
  with self.assertRaises(ValueError):PassiveSleepState(api,True,dict.fromkeys(BODY_NAMES,1),paths)
  with self.assertRaises(ValueError):PassiveSleepState(api,7,dict.fromkeys(BODY_NAMES,1),paths)
 def test_no_measurement_is_not_evidence(self):
  api,o=self.observer();self.assertFalse(o.evidence()['all_samples_valid'])
 def test_unknown_then_good_sample_keeps_trace_invalid(self):
  api,o=self.observer();api.query_error=ValueError('bad');o.capture(1,1/60,'hold');api.query_error=None;self.assertTrue(o.capture(2,2/60,'hold')['valid']);self.assertFalse(o.evidence()['all_samples_valid'])
 def test_canonical_body_name_swap_rejected(self):
  api=Interface();paths={n:'/World/Assembly_'+n for n in BODY_NAMES};paths['base'],paths['cover']=paths['cover'],paths['base']
  with self.assertRaises(ValueError):PassiveSleepState(api,7,dict(zip(sorted(BODY_NAMES),range(10,14))),paths)
 def test_true_false_readouts_remain_distinct(self):
  api,o=self.observer();o.capture(1,1/60,'hold');api.value=True;o.capture(2,2/60,'hold');self.assertEqual([v['sleeping']['cover'] for v in o.rows],[False,True]);self.assertTrue(o.evidence()['all_samples_valid'])
if __name__=='__main__':unittest.main()
