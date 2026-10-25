import unittest
from build_contact_trace_review import append_acquisition
class AcquisitionReview(unittest.TestCase):
 def test_acquisition_cannot_promote_failed_motion(self):
  parent={'runtime_status':'FAILED','independent_stage_acceptance':False,'independent_full_acceptance':False,'full_assembly_accepted':False,'error':'Lid grasp per-foot3D drift veto'}
  actual=append_acquisition(parent,0,'telemetry complete','')
  for k,v in parent.items():self.assertEqual(actual[k],v)
  self.assertFalse(actual['contact_telemetry_acquisition']['mechanical_success_checked_here'])
 def test_acquisition_failure_cannot_mutate_original_scope(self):
  parent={'runtime_status':'DIAGNOSTIC_PASS','independent_stage_acceptance':False,'independent_full_acceptance':False,'full_assembly_accepted':False}
  actual=append_acquisition(parent,1,'','invalid header')
  for k,v in parent.items():self.assertEqual(actual[k],v)
  self.assertFalse(actual['contact_telemetry_acquisition']['acquisition_integrity_checked'])
