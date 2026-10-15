import copy
import unittest
from supervisor import (CALIBRATION_ID,MODEL_ID,Hold,recipe,validate_frame,choose,recheck,
                        pickup_decision,placement_decision,sensor_identifier)

def detection(colour='blue',xy=(.32,.10),yaw=0.):
    return dict(colour=colour,xy=list(xy),yaw_deg=yaw,silhouette_iou=.95)

def frame(objects=None,seq=1):
    return dict(observation_id=seq,captured_monotonic=100.,calibration_id=CALIBRATION_ID,
                model_id=MODEL_ID,robot_idle=True,detections=objects or [],
                views={n:dict(sensor_time=float(seq),sensor_frame=seq) for n in ('table','side','wrist')})

class SupervisorTests(unittest.TestCase):
    def test_isaac6_fabric_frame_identifier(self):
        self.assertEqual(sensor_identifier({'referenceTimeNumerator':150,'referenceTimeDenominator':60}),2.5)
        self.assertEqual(sensor_identifier(10),10.)
        with self.assertRaises(Hold):sensor_identifier({'referenceTimeNumerator':1,'referenceTimeDenominator':0})
    def test_empty_and_missing_inventory_wait(self):
        slots=recipe(['red'])
        self.assertEqual(choose(frame(),slots,[]),(None,'WAIT_FOR_PART'))
        self.assertEqual(choose(frame([detection()]),slots,[]),(None,'WAIT_FOR_PART'))

    def test_arrival_becomes_eligible_in_new_frame(self):
        slots=recipe(['blue']);self.assertEqual(choose(frame(),slots,[])[1],'WAIT_FOR_PART')
        proposal,state=choose(frame([detection()],seq=2),slots,[])
        self.assertEqual(state,'PROPOSED');self.assertEqual(proposal['observation_id'],2)

    def test_stale_and_missing_camera_hold(self):
        for update in ({'captured_monotonic':0.},{'calibration_id':'wrong'}, {'model_id':'wrong'}, {'robot_idle':False}):
            f=frame();f.update(update)
            with self.assertRaises(Hold):validate_frame(f,101.)
        f=frame();del f['views']['wrist']
        with self.assertRaises(Hold):validate_frame(f,101.)

    def test_reused_or_skewed_camera_frame_hold(self):
        before=frame(seq=1);after=frame(seq=2)
        validate_frame(after,101.,previous=before)
        for key in ('sensor_frame','sensor_time'):
            bad=copy.deepcopy(after);bad['views']['side'][key]=1
            with self.assertRaises(Hold):validate_frame(bad,101.,previous=before)
        bad=copy.deepcopy(after);bad['views']['side']['sensor_time']+=.1
        with self.assertRaises(Hold):validate_frame(bad,101.)

    def test_shifted_and_rotated_target_invalidated(self):
        proposal,_=choose(frame([detection()]),recipe(['blue']),[])
        self.assertTrue(recheck(proposal,frame([detection()],seq=2)))
        self.assertFalse(recheck(proposal,frame([detection(xy=(.345,.1))],seq=2)))
        self.assertFalse(recheck(proposal,frame([detection(yaw=8.)],seq=2)))
        self.assertFalse(recheck(proposal,frame([detection(colour='red')],seq=2)))

    def test_ambiguous_or_occupied_destination_holds(self):
        slots=recipe(['blue'])
        with self.assertRaises(Hold):choose(frame([detection(),detection(xy=(.325,.1))]),slots,[])
        with self.assertRaises(Hold):choose(frame([detection(xy=slots[0]['xy'],colour='red')]),slots,[])
        with self.assertRaises(Hold):choose(frame(),slots,['slot-0'])
        shifted=detection(xy=(slots[0]['xy'][0]+.005,slots[0]['xy'][1]))
        with self.assertRaises(Hold):choose(frame([shifted]),slots,['slot-0'])

    def test_source_disappearance_is_not_pickup_evidence(self):
        self.assertEqual(pickup_decision(False,{'side':{'positive':False}}),'UNCERTAIN')
        self.assertEqual(pickup_decision(False,{'side':{'positive':True}}),'VERIFIED')
        self.assertEqual(pickup_decision(True,{'side':{'positive':True}}),'UNCERTAIN')

    def test_empty_grasp_needs_positive_source_and_reference(self):
        evidence={'side':dict(positive=False,empty_reference_match=True)}
        self.assertEqual(pickup_decision(True,evidence),'CONFIRMED_EMPTY')
        self.assertEqual(pickup_decision(False,evidence),'UNCERTAIN')
        self.assertEqual(pickup_decision(True,{'side':dict(positive=False)}),'UNCERTAIN')

    def test_wrong_held_colour_always_holds(self):
        self.assertEqual(pickup_decision(False,{'side':dict(positive=True),
                                              'wrist':dict(wrong_colour=['red'])}),'UNCERTAIN')

    def test_placement_needs_two_frames_colour_pose_and_side(self):
        slot=recipe(['blue'])[0]
        f=frame([detection(xy=slot['xy'])]);g=frame([detection(xy=slot['xy'])],seq=2)
        good=[dict(positive=True),dict(positive=True)]
        self.assertTrue(placement_decision([f,g],slot,good))
        self.assertFalse(placement_decision([f],slot,good[:1]))
        self.assertFalse(placement_decision([f,f],slot,good))
        self.assertFalse(placement_decision([f,g],slot,[dict(positive=False)]*2))
        g['detections'][0]['colour']='red'
        self.assertFalse(placement_decision([f,g],slot,good))

    def test_requested_backend_identity_is_pinned(self):
        f=frame();f['model_id']='pinned-checkpoint-identity'
        with self.assertRaises(Hold):validate_frame(f,101.)
        validate_frame(f,101.,expected_model_id='pinned-checkpoint-identity')
        with self.assertRaises(Hold):validate_frame(f,101.,expected_model_id='different-checkpoint')

if __name__=='__main__':unittest.main()
