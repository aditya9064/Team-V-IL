import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import subprocess
import sys
import os
import tempfile

import numpy as np
import torch

from team_v_industreal.control import (joint_position_step,pose_error,quaternion_rotate,
                                      shift_jacobian,jaw_targets)
from team_v_industreal.contacts import ContactAudit
from team_v_industreal.curriculum import get_new_max_disp
from team_v_industreal.evaluation import score
from team_v_industreal.doctor import inspect
from team_v_industreal.runtime import inspect_runtime
from launch import validate_smoke_transition

ROOT=Path(__file__).resolve().parents[1]


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.q=torch.zeros((1,6),dtype=torch.float64)
        self.j=torch.eye(6,dtype=torch.float64).unsqueeze(0)
        self.lo=torch.full_like(self.q,-1.)
        self.hi=torch.full_like(self.q,1.)

    def test_zero_action_is_finite_and_stationary(self):
        target=joint_position_step(self.q,self.j,self.q,self.lo,self.hi)
        torch.testing.assert_close(target,self.q)

    def test_singular_jacobian_does_not_explode(self):
        target=joint_position_step(self.q,torch.zeros_like(self.j),torch.ones_like(self.q),self.lo,self.hi)
        torch.testing.assert_close(target,self.q)

    def test_displacement_and_joint_bounds(self):
        error=torch.tensor([[100.,100.,100.,100.,100.,100.]],dtype=self.q.dtype)
        q=torch.full_like(self.q,0.9999)
        result=joint_position_step(q,self.j,error,self.lo,self.hi)
        self.assertTrue(bool((result<=self.hi).all()))
        self.assertLessEqual(float((result-q).abs().max()),0.0050001)
        self.assertLess(float(torch.linalg.vector_norm((result-q)[:,:3])),0.00101)

    def test_invalid_step_parameters_rejected(self):
        for parameters in ({'max_rotation_rad':0},{'damping':float('nan')},{'max_translation_m':-1}):
            with self.assertRaises(ValueError):
                joint_position_step(self.q,self.j,self.q,self.lo,self.hi,**parameters)

    def test_nonfinite_and_wrong_dof_fail_before_commands(self):
        for bad in (float('nan'),float('inf')):
            e=self.q.clone();e[0,0]=bad
            with self.assertRaises(ValueError):joint_position_step(self.q,self.j,e,self.lo,self.hi)
        with self.assertRaises(ValueError):joint_position_step(torch.zeros((1,7)),self.j,self.q,self.lo,self.hi)

    def test_antipodal_quaternions_are_same_pose(self):
        q=torch.tensor([[1.,0.,0.,0.]])
        error=pose_error(torch.zeros((1,3)),q,torch.zeros((1,3)),-q)
        torch.testing.assert_close(error,torch.zeros_like(error))

    def test_zero_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            pose_error(torch.zeros((1,3)),torch.zeros((1,4)),torch.zeros((1,3)),torch.tensor([[1.,0.,0.,0.]]))

    def test_tcp_rotation_maps_z_to_original_tool_x(self):
        q=torch.tensor([[math.sqrt(.5),0.,math.sqrt(.5),0.]],dtype=torch.float64)
        result=quaternion_rotate(q,torch.tensor([[0.,0.,1.]],dtype=q.dtype))
        torch.testing.assert_close(result,torch.tensor([[1.,0.,0.]],dtype=q.dtype))

    def test_tcp_jacobian_matches_finite_difference_rotating_wrist(self):
        offset=torch.tensor([[.293,0.,0.]],dtype=torch.float64)
        jac=torch.zeros((1,6,6),dtype=torch.float64);jac[0,5,0]=1
        shifted=shift_jacobian(jac,offset)
        eps=1e-7
        rotation=torch.tensor([[math.cos(eps/2),0.,0.,math.sin(eps/2)]],dtype=torch.float64)
        finite=(quaternion_rotate(rotation,offset)-offset)/eps
        torch.testing.assert_close(shifted[0,:3,0],finite[0],atol=2e-8,rtol=1e-7)

    def test_smc_signs_and_six_mm_stroke(self):
        self.assertEqual(jaw_targets(.006),(-0.,0.))
        self.assertEqual(jaw_targets(0.),(-.006,.006))
        for value in (-.0001,.0061,float('nan')):
            with self.assertRaises(ValueError):jaw_targets(value)


class ContactTests(unittest.TestCase):
    def header(self,one='HeldAsset',two='FixedAsset',count=1,start=0):
        return SimpleNamespace(actor0='/World/envs/env_0/'+one+'/body',actor1='/World/envs/env_0/'+two+'/body',num_contact_data=count,contact_data_offset=start)

    def test_all_indexed_points_include_deepest(self):
        audit=ContactAudit()
        audit.ingest([self.header(count=3)],[SimpleNamespace(separation=s) for s in (-.0001,-.0004,-.00005)],str)
        self.assertTrue(audit.invalid)
        self.assertEqual(audit.point_count,3)
        self.assertEqual(audit.max_depth_m,.0004)

    def test_invalid_raw_data_latches_error(self):
        for data in ([],[SimpleNamespace(separation=float('nan'))]):
            audit=ContactAudit();audit.ingest([self.header()],data,str)
            self.assertTrue(audit.invalid)
            self.assertTrue(audit.errors)

    def test_robot_fixture_contact_is_rejected(self):
        audit=ContactAudit();audit.ingest([self.header('Robot','FixedAsset')],[SimpleNamespace(separation=0.)],str)
        self.assertTrue(audit.invalid)

    def test_missing_headers_are_not_socket_contact_proof(self):
        audit=ContactAudit();audit.ingest([],[],str)
        self.assertFalse(audit.socket_contact_observed)
        self.assertIn('never proof',audit.report()['no_header_means'])


class AcceptanceTests(unittest.TestCase):
    def sample(self,time):
        return dict(time_s=time,radial_error_m=.00001,depth_error_m=.0001,tilt_rad=.001,
                    max_contact_depth_m=.0001,released=True,contact_monitor_valid=True,socket_contact_observed=True)

    def test_only_scoped_release_benchmark_can_pass(self):
        result=score([self.sample(0),self.sample(.6)])
        self.assertEqual(result['status'],'INSERTION_BENCHMARK_PASS')
        self.assertFalse(result['full_assembly_accepted'])
        self.assertFalse(result['physical_hardware_tested'])

    def test_low_height_outside_hole_is_not_insertion(self):
        samples=[self.sample(0),self.sample(.6)]
        for s in samples:s['radial_error_m']=.005
        self.assertIn('radial_clearance_exceeded',score(samples)['reasons'])

    def test_missing_unreleased_nan_and_short_hold_fail(self):
        for key,value in [('released',False),('contact_monitor_valid',False),('socket_contact_observed',False),('max_contact_depth_m',float('nan'))]:
            samples=[self.sample(0),self.sample(.6)]
            for s in samples:s[key]=value
            self.assertEqual(score(samples)['status'],'FAILED',key)
        self.assertEqual(score([self.sample(0),self.sample(.1)])['status'],'FAILED')
        self.assertEqual(score([])['status'],'FAILED')


class UpstreamIntegrationTests(unittest.TestCase):
    def test_auto_reset_cannot_hide_a_failed_contact_step(self):
        audit=ContactAudit();audit.errors.append('contact failure before automatic episode reset')
        terminated=torch.tensor([audit.invalid])
        audit.reset()  # The actual pinned DirectRLEnv resets before returning.
        self.assertFalse(audit.invalid)
        with self.assertRaisesRegex(RuntimeError,'auto-reset'):
            validate_smoke_transition({'policy':torch.zeros((1,19))},torch.tensor([-1.]),terminated,torch.tensor([False]))

    def test_smoke_requires_uninterrupted_finite_observations_and_reward(self):
        obs={'policy':torch.zeros((1,19)),'critic':torch.zeros((1,40))}
        reward=torch.zeros(1);flags=torch.zeros(1,dtype=torch.bool)
        validate_smoke_transition(obs,reward,flags,flags)
        for bad_obs,bad_reward,timeout in [
            ({'policy':torch.tensor([[float('nan')]])},reward,flags),
            (obs,torch.tensor([float('inf')]),flags),
            (obs,reward,torch.ones_like(flags)),
        ]:
            with self.assertRaises(RuntimeError):
                validate_smoke_transition(bad_obs,bad_reward,flags,timeout)

    def test_sim6_lab3_runtime_is_rejected_before_simulator_import(self):
        result=inspect_runtime(
            versions={'isaacsim':'6.0.0.0','isaaclab':'3.0.0','numpy':'2.3.1',
                      'gymnasium':'1.2.0','torch':'2.10.0','rl-games':'1.6.1'},
            python_version=(3,12),isaaclab_root=Path('/absent/isaaclab'))
        self.assertFalse(result['runtime_profile_matches'])
        self.assertTrue(any('Sim 5.1.0' in e for e in result['runtime_errors']))
        self.assertTrue(any('distribution must' in e for e in result['runtime_errors']))
        self.assertFalse(result['simulation_executed'])

    def test_matching_versions_do_not_hide_changed_lab_quaternion_math(self):
        with tempfile.TemporaryDirectory() as folder:
            for name in ('envs/direct_rl_env.py','utils/math.py'):
                path=Path(folder)/name;path.parent.mkdir(exist_ok=True)
                path.write_text('changed API or quaternion convention')
            result=inspect_runtime(
                versions={'isaacsim':'5.1.0.0','isaaclab':'0.47.2','numpy':'1.26.0',
                          'gymnasium':'1.2.0','torch':'2.7.0','rl-games':'1.6.1'},
                python_version=(3,11),isaaclab_root=Path(folder))
            self.assertFalse(result['runtime_profile_matches'])
            self.assertTrue(any('utils/math.py' in e for e in result['runtime_errors']))
            self.assertFalse(result['runtime_compatibility_verified'])

    def test_original_curriculum_functions_are_ast_identical(self):
        original=ast.parse((ROOT/'vendor/industreal_algo_utils.py').read_text())
        port=ast.parse((ROOT/'team_v_industreal/curriculum.py').read_text())
        names={'get_new_max_disp','get_curriculum_reward_scale'}
        def bodies(tree):return {n.name:ast.dump(n,include_attributes=False) for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names}
        self.assertEqual(bodies(original),bodies(port))

    def test_curriculum_changes_difficulty_but_not_acceptance(self):
        cfg=SimpleNamespace(rl=SimpleNamespace(curriculum_height_bound=[0.,.02],curriculum_height_step=[-.001,.002],curriculum_success_thresh=.8,curriculum_failure_thresh=.5))
        self.assertAlmostEqual(get_new_max_disp(.9,cfg,.02),.019)
        self.assertAlmostEqual(get_new_max_disp(.1,cfg,.019),.02)
        self.assertEqual(get_new_max_disp(.7,cfg,.01),.01)

    def test_six_joint_reset_has_no_franka_joint_slice(self):
        module=ast.parse((ROOT/'team_v_industreal/environment.py').read_text())
        cls=next(n for n in module.body if isinstance(n,ast.ClassDef))
        reset=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_randomize_fanuc_episode')
        for node in ast.walk(reset):
            if isinstance(node,ast.Subscript) and isinstance(node.value,ast.Attribute) and node.value.attr in ('joint_pos','ctrl_target_joint_pos'):
                self.assertNotIn('Constant(value=7)',ast.dump(node))
        text=ast.unparse(reset)
        self.assertIn('ik_attempt >= 8',text)

    def test_doctor_proves_source_only_and_lists_runtime_limits(self):
        result=inspect()
        self.assertEqual(result['source_errors'],[])
        self.assertFalse(result['simulation_executed'])
        self.assertFalse(result['runtime_compatibility_verified'])

    def test_launch_refuses_a_login_host_before_opening_isaac(self):
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder)/'must-not-exist'
            env=dict(os.environ);env.pop('SLURM_JOB_ID',None)
            result=subprocess.run([sys.executable,str(ROOT/'launch.py'),'--mode','smoke','--binding','absent.json','--out',str(out)],env=env,capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('allocated GPU session',result.stderr)
            self.assertFalse(out.exists())

    def test_unvalidated_source_export_is_never_a_simulator_pass(self):
        code=ast.parse((ROOT/'export_robot.py').read_text())
        text=ast.unparse(code)
        self.assertIn('ASSET_EXPORTED_NOT_SIMULATOR_VALIDATED',text)
        self.assertNotIn('simulation_executed=True',text)


if __name__=='__main__':unittest.main()
