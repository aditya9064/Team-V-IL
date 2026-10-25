import unittest
from assembly_checks import classify_contact,contact_violation,role_pair,grasp_command_m


class ContactChecks(unittest.TestCase):
    def setUp(self):
        self.state={'active':'dummy_board','allow_grip':False,'allow_active_table':True}
        self.jaw='/World/ER4iA/JawL';self.body='/World/Assembly_dummy_board'
    def finger(self,role,part='dummy_board'):
        return classify_contact([self.jaw,'/World/Assembly_'+part],['jaw_shape','part_shape'],[None,(part,role)],self.state,set(),'/World/ER4iA')[0]
    def test_approach_cannot_contact_grip(self):self.assertFalse(self.finger('grip_stem'))
    def test_close_can_contact_target_stem(self):
        self.state['allow_grip']=True;self.assertTrue(self.finger('grip_stem'))
    def test_support_shoulder_only_during_active_grasp(self):
        self.assertFalse(self.finger('grip_support'))
        self.state['allow_grip']=True
        self.assertTrue(self.finger('grip_support'))
        self.assertFalse(self.finger('grip_support','cover'))
    def test_target_panel_is_not_a_grip(self):
        self.state['allow_grip']=True;self.assertFalse(self.finger('panel'))
    def test_grip_root_is_not_permitted_contact(self):
        self.state['allow_grip']=True;self.assertFalse(self.finger('grip_root'))
    def test_wrong_part_grip_forbidden(self):
        self.state['allow_grip']=True;self.assertFalse(self.finger('grip_stem','cover'))
    def test_passive_table_support(self):
        self.assertTrue(classify_contact(['/World/Assembly_cover',''],['c','/World/Mat'],[('cover','panel'),None],self.state,set(),'/World/ER4iA')[0])
    def test_held_part_table_forbidden(self):
        self.state['allow_active_table']=False
        self.assertFalse(classify_contact([self.body,''],['c','/World/Mat'],[('dummy_board','panel'),None],self.state,set(),'/World/ER4iA')[0])
    def test_support_requires_explicit_stage(self):
        args=([self.body,'/World/Assembly_base'],['a','b'],[('dummy_board','panel'),('base','ledge')],self.state)
        self.assertFalse(classify_contact(*args,set(),'/World/ER4iA')[0])
        self.assertTrue(classify_contact(*args,{role_pair(('base','ledge'),('dummy_board','panel'))},'/World/ER4iA')[0])
    def test_guide_pin_contact_is_not_wall_support(self):
        self.assertFalse(classify_contact(['/World/Assembly_cover','/World/Assembly_base'],['a','b'],[('cover','panel'),('base','guide_pin')],self.state,{role_pair(('cover','panel'),('base','wall'))},'/World/ER4iA')[0])
    def test_unknown_part_collider_fails(self):
        with self.assertRaises(ValueError):classify_contact([self.jaw,self.body],['a','b'],[None,None],self.state,set(),'/World/ER4iA')
    def test_zero_impulse_positive_separation_is_only_proximity(self):self.assertFalse(contact_violation(False,.0001,0,.25))
    def test_unintended_penetration_fails(self):self.assertTrue(contact_violation(False,-.00001,0,.25))
    def test_unintended_contact_impulse_fails(self):self.assertTrue(contact_violation(False,.0001,.00001,.25))
    def test_excess_intended_penetration_fails(self):self.assertTrue(contact_violation(True,-.0003,0,.25))
    def test_nonfinite_contact_fails(self):
        with self.assertRaises(ValueError):contact_violation(True,float('nan'),0,.25)
    def test_grip_preload_leaves_collision_margin(self):self.assertAlmostEqual(grasp_command_m(3.,.1,.25),.0029)
    def test_old_grip_overtravel_rejected(self):
        with self.assertRaises(ValueError):grasp_command_m(3.,.3,.25)
    def test_invalid_grip_values_rejected(self):
        for values in [(float('nan'),.1,.25),(3.,-.1,.25),(.05,.1,.25)]:
            with self.subTest(values=values),self.assertRaises(ValueError):grasp_command_m(*values)


if __name__=='__main__':unittest.main()
