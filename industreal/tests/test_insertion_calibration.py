import unittest,torch,math
from team_v_industreal.insertion_calibration import calibrate,target_tcp,interpolate,release_state
from team_v_industreal.control import quaternion_multiply,quaternion_rotate,integrate_joint_target
from team_v_industreal.contacts import ContactAudit


class FrozenCalibrationTests(unittest.TestCase):
    def test_target_reconstructs_seated_peg_and_removes_tilt_without_yaw_flip(self):
        tcp_p=torch.tensor([[.35,.002,.055]],dtype=torch.float64)
        tcp_q=torch.tensor([[0.,1.,0.,0.]],dtype=torch.float64)
        axis_q=torch.tensor([[math.cos(.003),math.sin(.003),0.,0.]],dtype=torch.float64)
        peg_q=quaternion_multiply(axis_q,torch.tensor([[0.,0.,0.,1.]],dtype=torch.float64))
        peg_p=tcp_p+torch.tensor([[.0002,.0003,-.020]],dtype=torch.float64)
        fixed_p=torch.tensor([[.349,0.,-.0051]],dtype=torch.float64);fixed_q=torch.tensor([[1.,0.,0.,0.]],dtype=torch.float64)
        c=calibrate(tcp_p,tcp_q,peg_p,peg_q,fixed_p,fixed_q);p,q=target_tcp(c,.00005)
        actual_peg=p+quaternion_rotate(q,c['grasp_position_tcp_m'])
        actual_q=quaternion_multiply(q,c['grasp_quaternion_wxyz'])
        self.assertTrue(torch.allclose(actual_peg,fixed_p+torch.tensor([[0.,0.,.00005]],dtype=torch.float64),atol=1e-12))
        self.assertTrue(torch.allclose(quaternion_rotate(actual_q,torch.tensor([[0.,0.,1.]],dtype=torch.float64)),torch.tensor([[0.,0.,1.]],dtype=torch.float64),atol=1e-12))
        self.assertGreater(float((actual_q*peg_q).sum().abs()),.99)
        before=target_tcp(c,.030)[0].clone();peg_p+=1.;fixed_p-=1.;tcp_p+=.25
        self.assertTrue(torch.equal(before,target_tcp(c,.030)[0]))

    def test_quaternion_interpolation_uses_short_path(self):
        pos=torch.zeros((1,3));q=torch.tensor([[1.,0.,0.,0.]])
        middle=interpolate(pos,q,pos,-q,.5)[1]
        self.assertTrue(torch.equal(middle,q))
        with self.assertRaises(ValueError):interpolate(pos,q,pos,q,1.1)

    def test_open_command_or_one_free_jaw_cannot_certify_release(self):
        closed=[0.]*6+[-.0022,.0022];opened=[0.]*8
        self.assertFalse(release_state(closed,{},True)['released'])
        self.assertFalse(release_state(opened,{},False)['released'])
        self.assertFalse(release_state(opened,{'JawL':[0.,.01,0.]},True)['released'])
        self.assertTrue(release_state(opened,{'JawL':[0.,0.,0.],'JawR':[0.,0.,0.]},True)['released'])

    def test_support_permission_does_not_survive_reset(self):
        a=ContactAudit();a.allow_table_support=True
        a.ingest_tensor(['Table'],[1],[0],[0.],[.18639])
        self.assertFalse(a.invalid)
        a.reset();a.ingest_tensor(['Table'],[1],[0],[0.],[.18639])
        self.assertTrue(a.invalid)


if __name__=='__main__':unittest.main()


class PositionServoIntegrationTests(unittest.TestCase):
    def test_static_load_bias_is_rejected_without_live_peg_feedback(self):
        # Reference static plant: actual angle = command - load deflection.
        # A measured-joint incremental command leaves the load bias behind.
        bias=torch.tensor([[.003,-.002,0.,0.,0.,0.]])
        cmd=torch.zeros((1,6));lo=torch.full_like(cmd,-1.);hi=-lo
        for _ in range(100):
            measured=cmd-bias
            cmd=integrate_joint_target(measured,cmd,torch.zeros_like(cmd),lo,hi)
        self.assertLess(float((cmd-bias).abs().max()),1e-7)

    def test_command_windup_is_bounded_at_contact_and_joint_limits(self):
        measured=torch.zeros((1,6));cmd=measured.clone();lo=torch.full_like(cmd,-.003);hi=-lo
        for _ in range(100):
            cmd=integrate_joint_target(measured,cmd,torch.ones_like(cmd),lo,hi)
        self.assertTrue(torch.all(cmd<=hi));self.assertTrue(torch.all(cmd-measured<=.005))
        with self.assertRaises(ValueError):integrate_joint_target(measured,cmd,cmd,lo,hi,gain=float('nan'))


class CaptureVersusFinalScoreTests(unittest.TestCase):
    def test_capture_permission_cannot_relax_the_final_released_fit(self):
        from team_v_industreal.evaluation import within_capture_envelope,score
        self.assertTrue(within_capture_envelope(.000063,.00005,.001))
        samples=[dict(time_s=t,radial_error_m=.000063,depth_error_m=.00005,tilt_rad=.001,
                      max_contact_depth_m=.0001,released=True,contact_monitor_valid=True,socket_contact_observed=True) for t in [0.,.6]]
        self.assertIn('radial_clearance_exceeded',score(samples)['reasons'])
        for r,z,a in [(.0004,0.,0.),(.00005,.025,0.),(.00005,0.,.1),(float('nan'),0.,0.)]:
            self.assertFalse(within_capture_envelope(r,z,a))


class GradualReleaseTests(unittest.TestCase):
    def test_release_is_smooth_bounded_monotonic_and_reaches_full_stroke(self):
        from team_v_industreal.insertion_calibration import gradual_release_opening
        values=[gradual_release_opening(.0036,i/60) for i in range(61)]
        self.assertEqual(values[0],.0036);self.assertAlmostEqual(values[-1],.006)
        self.assertTrue(all(.0036<=v<=.006 for v in values))
        self.assertTrue(all(a<=b for a,b in zip(values,values[1:])))
        self.assertLess(max(b-a for a,b in zip(values,values[1:])),.000061)
        self.assertLess(values[1]-values[0],.000002)
        for opening,fraction in [(float('nan'),0),(.007,.5),(.0036,-.01),(.0036,1.01),(.0036,float('inf'))]:
            with self.subTest(opening=opening,fraction=fraction),self.assertRaises(ValueError):gradual_release_opening(opening,fraction)

    def test_release_api_cannot_override_training_or_exceed_stroke(self):
        import ast,math,types
        from pathlib import Path
        tree=ast.parse((Path(__file__).parents[1]/'team_v_industreal/environment.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FanucInsertionEnv')
        nodes=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in {'begin_release','set_release_opening_per_jaw'}]
        namespace={'math':math};exec(compile(ast.Module(body=nodes,type_ignores=[]),'<diagnostic-release-api>','exec'),namespace)
        env=types.SimpleNamespace(cfg=types.SimpleNamespace(insertion_baseline=False),binding={'grasp_opening_per_jaw_m':.0036},release_requested=False,diagnostic_release_opening=None)
        with self.assertRaises(RuntimeError):namespace['begin_release'](env,gradual=True)
        namespace['begin_release'](env);self.assertIsNone(env.diagnostic_release_opening)
        env.cfg.insertion_baseline=True;namespace['begin_release'](env,gradual=True)
        namespace['set_release_opening_per_jaw'](env,.005);self.assertEqual(env.diagnostic_release_opening,.005)
        for value in [float('nan'),.00359,.00601]:
            with self.subTest(value=value),self.assertRaises(ValueError):namespace['set_release_opening_per_jaw'](env,value)
