import math,unittest
import numpy as np
from payload_checks import rotation,payload_score,initial_supports,placement_supports
from assembly_checks import classify_contact


def pose(xyz,q=(1,0,0,0)):
    return {'position_world_m':list(xyz),'orientation_wxyz':list(q)}


class PayloadChecks(unittest.TestCase):
    def test_common_translation_and_rotation_preserve_payload(self):
        angle=math.radians(37);q=(math.cos(angle/2),0,0,math.sin(angle/2));c=pose([.4,-.2,.9],q)
        p=pose(np.asarray(c['position_world_m'])+rotation(q)@np.array([0,0,.002]),q)
        self.assertTrue(payload_score(c,p)['pass'])
    def test_payload_left_on_source_fails(self):self.assertFalse(payload_score(pose([.4,0,1]),pose([.2,0,.902]))['pass'])
    def test_two_degree_relative_tilt_fails(self):self.assertFalse(payload_score(pose([0,0,0]),pose([0,0,.002],[math.cos(math.radians(1)),math.sin(math.radians(1)),0,0]))['pass'])
    def test_lost_floor_support_fails(self):self.assertFalse(payload_score(pose([0,0,0]),pose([0,0,.003]))['pass'])
    def test_rim_corner_clearance_within_gate(self):self.assertTrue(payload_score(pose([0,0,0]),pose([.0005,.0005,.002]))['pass'])
    def test_displacement_outside_gate_fails(self):self.assertFalse(payload_score(pose([0,0,0]),pose([.0008,0,.002]))['pass'])
    def test_nonfinite_feedback_fails(self):
        with self.assertRaises(ValueError):payload_score(pose([0,0,0]),pose([float('nan'),0,.002]))
    def test_wrong_relative_yaw_fails(self):self.assertFalse(payload_score(pose([0,0,0]),pose([0,0,.002],[math.cos(math.radians(1)),0,0,math.sin(math.radians(1))]))['pass'])


class PayloadContacts(unittest.TestCase):
    def setUp(self):self.state={'active':'carrier','allow_grip':True,'allow_active_table':False}
    def contact(self,a,b,supports):
        return classify_contact(['/World/Assembly_'+a[0],'/World/Assembly_'+b[0]],['a','b'],[a,b],self.state,supports,'/World/ER4iA')[0]
    def test_board_floor_and_rim_are_real_supports(self):
        for role in ['floor','wall']:self.assertTrue(self.contact(('pcb','substrate'),('carrier',role),initial_supports()))
    def test_pcb_components_not_carrier_support(self):self.assertFalse(self.contact(('pcb','component'),('carrier','wall'),initial_supports()))
    def test_ledge_contact_requires_placement_stage(self):
        self.assertFalse(self.contact(('carrier','floor'),('base','floor'),initial_supports()))
        self.assertTrue(self.contact(('carrier','floor'),('base','floor'),initial_supports()|placement_supports()['carrier']))
    def test_frame_has_two_actual_support_surfaces(self):
        for p in ['ring','ear']:
            self.assertTrue(self.contact(('frame',p),('pcb','substrate'),placement_supports()['frame']))
            self.assertTrue(self.contact(('frame',p),('carrier','wall'),placement_supports()['frame']))
    def test_pcb_finger_contact_always_forbidden(self):
        self.assertFalse(classify_contact(['/World/ER4iA/JawL','/World/Assembly_pcb'],['finger','board'],[None,('pcb','substrate')],self.state,initial_supports(),'/World/ER4iA')[0])
    def test_pcb_table_contact_forbidden(self):
        self.assertFalse(classify_contact(['/World/Assembly_pcb','/World/Mat'],['board','/World/Mat'],[('pcb','substrate'),None],self.state,initial_supports(),'/World/ER4iA')[0])


if __name__=='__main__':unittest.main()
