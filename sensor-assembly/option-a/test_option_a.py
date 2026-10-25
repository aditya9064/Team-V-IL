import ast
import copy
import hashlib
import json
from pathlib import Path
import unittest
import numpy as np
from pickup_protocol import protocol, pickup_score, SDK_PINS
from assembly_checks import classify_contact, contact_violation
from geometry_probe_plan import ray_distance, build
from scipy.spatial import ConvexHull

ROOT=Path(__file__).parent


class OptionATests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design=json.loads((ROOT/'design.json').read_text())
        cls.parts=json.loads((ROOT/'parts.json').read_text())

    def test_original_sdk_bytes(self):
        for name,digest in SDK_PINS.items():
            self.assertEqual(hashlib.sha256((ROOT/'pinned-sdk'/name).read_bytes()).hexdigest(),digest)

    def test_cad09_identity(self):
        self.assertEqual(hashlib.sha256((ROOT/'design.json').read_bytes()).hexdigest(),'d88ac777e0f57c48255278894cafc2cc6386a7479cdeb8fe5a40c1f598f25cb7')
        self.assertEqual(hashlib.sha256((ROOT/'parts.json').read_bytes()).hexdigest(),'819e147f08a1a992ab5b7d24a8d9443f619d3989e000095306b88381b00892b9')

    def test_fixed_recipe_binary_and_part_goals(self):
        for name,offset,z in [('base',18,29.58820100784302),('carrier',-18,29.58820100784302),('cover',18,77.58820100784303)]:
            p=protocol(self.design,name)
            self.assertEqual([x['role'] for x in p['phases']],['orient','above','approach','side_entry','close','lift'])
            self.assertEqual([x['duration_s'] for x in p['phases']],[1,2,2,1.5,3,1.5])
            self.assertEqual(p['limits'],self.design['limits_unchanged'])
            self.assertEqual(p['jaw_targets'],{'open':{'jaw_l':0.,'jaw_r':0.},'closed':{'jaw_l':-.006,'jaw_r':.006}})
            goals=np.array([v['target_robot_m'] for v in p['phases']])
            self.assertAlmostEqual(goals[2,0]-goals[3,0],offset*.001)
            self.assertAlmostEqual(goals[3,2],-.0051+z*.001)
            np.testing.assert_allclose(goals[5]-goals[3],[0,0,.04],rtol=0,atol=1e-14)
            self.assertEqual(p['hold_steps'],120)
            self.assertFalse(p['camera_guided'])

    def test_authorized_scope_rejects_wrong_cad_or_part(self):
        for name in ['frame','all','pcb']:
            with self.assertRaises(ValueError):protocol(self.design,name)
        altered=copy.deepcopy(self.design);altered['gripper_changed']=True
        with self.assertRaises(ValueError):protocol(altered,'base')

    def test_separate_payload_floor_and_spawn(self):
        for name in ['base','carrier','cover']:
            p=protocol(self.design,name);c=p['authored_spawn']['carrier'];b=p['authored_spawn']['pcb']
            np.testing.assert_allclose(np.array(b['position_world_m'])-c['position_world_m'],[0,0,.004],atol=1e-15)
            self.assertEqual(p['payload_floor_m'],.004)
            self.assertEqual(p['payload_limits'],{'xy_mm':.75,'z_mm':.35,'relative_tilt_deg':1.,'relative_yaw_deg':1.})

    def test_original_contact_limits(self):
        self.assertFalse(contact_violation(True,-.0002499,1.,.25))
        self.assertTrue(contact_violation(True,-.0002501,0.,.25))
        self.assertTrue(contact_violation(False,.0002,1.01e-8,.25))
        self.assertTrue(contact_violation(False,-1.01e-6,0.,.25))
        with self.assertRaises(ValueError):contact_violation(True,float('nan'),0,.25)

    def test_gripper_only_intended_contact_roles(self):
        actors=['/World/Assembly_base','/World/ER4iA/JawL'];cols=['/World/Assembly_base/C_020_grip_stem','/World/ER4iA/JawL/Collision/Part0']
        state={'active':'base','allow_grip':True,'allow_active_table':True}
        self.assertTrue(classify_contact(actors,cols,[('base','grip_stem'),None],state,set(),'/World/ER4iA')[0])
        self.assertFalse(classify_contact(actors,cols,[('base','floor'),None],state,set(),'/World/ER4iA')[0])
        state['allow_grip']=False
        self.assertFalse(classify_contact(actors,cols,[('base','grip_stem'),None],state,set(),'/World/ER4iA')[0])

    def test_pickup_rejects_low_lift_tilt_and_slip(self):
        initial={'position_world_m':[.36,-.12,.8949]}
        good={'position_world_m':[.36,-.12,.9349],'tilt_deg':0.,'yaw_deg':0.}
        self.assertTrue(pickup_score(initial,good)['pass'])
        for key,value in [('tilt_deg',5.001),('yaw_deg',3.001),('position_world_m',[.363,-.12,.9349]),('position_world_m',[.36,-.12,.9298])]:
            altered=dict(good);altered[key]=value;self.assertFalse(pickup_score(initial,altered)['pass'])

    def test_rays_hit_and_empty_slot(self):
        expected=json.loads((ROOT/'geometry-probes.json').read_text())
        self.assertEqual(len(expected),154)
        self.assertEqual(sum(not r['hit'] for r in expected),12)
        coverage={name:set() for name in self.parts['parts']}
        hulls={name:[ConvexHull(np.asarray(c['vertices_mm'],float)).equations for c in item['colliders']]
               for name,item in self.parts['parts'].items()}
        for ray in expected:
            start=np.asarray(ray['start_m'])*1000;direction=np.asarray(ray['direction'])
            self.assertAlmostEqual(np.linalg.norm(direction),1.,places=10)
            distances=[ray_distance(eq,start,direction,ray['maximum_distance_m']*1000) for eq in hulls[ray['part']]]
            hits=[d for d in distances if d is not None]
            if ray['hit']:
                self.assertTrue(hits)
                self.assertAlmostEqual(min(hits),ray['expected_distance_mm'],places=5)
                self.assertAlmostEqual(distances[ray['collider_index']],min(hits),places=5)
                coverage[ray['part']].add(ray['collider_index'])
            else:self.assertFalse(hits)
        for name,p in self.parts['parts'].items():
            self.assertEqual(coverage[name],set(range(len(p['colliders']))))

    def test_analytic_ray_negative_control(self):
        cube=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
        hull=ConvexHull(cube).equations
        self.assertAlmostEqual(ray_distance(hull,np.array([0,0,3]),np.array([0,0,-1]),4),2)
        self.assertIsNone(ray_distance(hull,np.array([2,0,3]),np.array([0,0,-1]),4))

    def test_no_truth_or_rgb_in_protocol(self):
        tree=ast.parse((ROOT/'pickup_protocol.py').read_text())
        node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='protocol')
        attrs={n.attr for n in ast.walk(node) if isinstance(n,ast.Attribute)}
        self.assertFalse(attrs&{'get_world_pose','get_current_frame','get_rgba','set_world_pose'})

    def test_runtime_binary_commands_and_fixed_hold_structure(self):
        code=(ROOT/'option_a_pickup.py').read_text();tree=ast.parse(code)
        self.assertNotIn('grasp_command_m',code)
        self.assertNotIn('set_world_pose(',code)
        self.assertNotIn('set_kinematic',code)
        self.assertEqual(code.count('arm.set_joint_positions('),1) #source startup only
        command=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='command')
        self.assertIn("C.JAW_OPEN if value=='open' else C.JAW_CLOSED",ast.get_source_segment(code,command))
        self.assertIn("for _ in range(120):command('closed');advance()",code)
        self.assertIn('held_reference',code)


if __name__=='__main__':unittest.main()
