"""Failure injection into the actual runtime advance body; no simulator claim."""
import ast
from pathlib import Path
import types
import unittest
import numpy as np
from post_step_guards import record_payload_then_check_contacts
from payload_checks import payload_score


class World:
    current_time=0.
    steps=0
    def step(self,render=False):self.steps+=1;self.current_time+=1/60
    def render(self):raise AssertionError('No render expected on first physics step')


class Arm:
    def get_joint_positions(self):return np.zeros(8,dtype=np.float32)
    def get_joint_velocities(self):return np.zeros(8,dtype=np.float32)


class RuntimeFailureTests(unittest.TestCase):
    def make(self,payload_offset=0.,contact_error=False):
        tree=ast.parse(Path(__file__).with_name('option_a_pickup.py').read_text())
        advance=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='advance')
        report={'diagnostic_trace':[],'full_motion_observations':[],'video':{'frames':0}}
        world=World();arm=Arm();state={'step':0,'phase':'carrier_close'}
        def poses():
            return {name:{'position_world_m':[.36+ (payload_offset if name=='pcb' else 0),-.12,.9+(.004 if name=='pcb' else 0)],
                          'orientation_wxyz':[1.,0,0,0],'tilt_deg':0.,'yaw_deg':0.}
                    for name in ['base','carrier','pcb','cover','nest']}
        def snapshot():return {'step':state['step'],'sim_time':world.current_time,'joint_positions':arm.get_joint_positions().tolist()}
        def stop():
            if contact_error:raise RuntimeError('injected original contact stop')
        env={'state':state,'report':report,'world':world,'arm':arm,'snapshot':snapshot,'np':np,'ai':list(range(6)),
             'q':np.zeros(6),'issued':{'jaw_l':-.006,'jaw_r':.006},'C':types.SimpleNamespace(TABLE_TOP_Z=.9),
             'fk':types.SimpleNamespace(tool_pose=lambda q,tool:(np.eye(3),np.array([.36,-.12,.10]))),
             'tool':(.293,0,0),'poses':poses,'record_payload_then_check_contacts':record_payload_then_check_contacts,
             'payload_score':payload_score,'stop_contacts':stop,'held_reference':None,'a':types.SimpleNamespace(part='carrier')}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[advance],type_ignores=[])),'actual-runtime-advance','exec'),env)
        return env['advance'],report,world

    def test_contact_failure_preserves_same_step_PCB_and_terminal_row(self):
        advance,report,world=self.make(contact_error=True)
        with self.assertRaisesRegex(RuntimeError,'original contact stop'):advance()
        self.assertEqual(world.steps,1)
        self.assertEqual(len(report['full_motion_observations']),1)
        self.assertEqual(report['payload_motion_audit']['samples'],1)
        self.assertEqual(report['full_motion_last_snapshot']['step'],1)
        self.assertAlmostEqual(report['full_motion_last_snapshot']['sim_time'],1/60)

    def test_simultaneous_payload_failure_keeps_contact_priority(self):
        advance,report,world=self.make(payload_offset=.001,contact_error=True)
        with self.assertRaisesRegex(RuntimeError,'original contact stop'):advance()
        self.assertEqual(world.steps,1)
        self.assertAlmostEqual(report['payload_motion_audit']['maximum_xy_mm'],1.)
        self.assertEqual(report['payload_motion_audit']['samples'],1)

    def test_payload_failure_stops_without_extra_physics(self):
        advance,report,world=self.make(payload_offset=.001)
        with self.assertRaisesRegex(RuntimeError,'Loose PCB guard failed'):advance()
        self.assertEqual(world.steps,1)
        self.assertEqual(report['payload_motion_audit']['samples'],1)


if __name__=='__main__':unittest.main()
