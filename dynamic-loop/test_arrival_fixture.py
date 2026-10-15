"""Verify arrival transforms are authored before physics reset, not runtime truth."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import numpy as np

class Attribute:
    def __init__(self,value):self.value=value
    def Set(self,value):self.value=value
    def Get(self):return self.value
class Translation(Attribute):
    def GetOpType(self):return 'translate'
class Prim:
    def __init__(self,name):self.name=name;self.translation=Translation([.3,.1,.9]);self.kinematic=Attribute(False)
    def GetName(self):return self.name
    def HasAPI(self,api):return True
    def GetPath(self):return '/World/'+self.name
class Stage:
    def __init__(self):self.prims=[Prim('SortCube0'),Prim('SortCube1'),Prim('SortCube2')]
    def Traverse(self):return iter(self.prims)
    def GetPrimAtPath(self,path):return next(p for p in self.prims if str(p.GetPath())==path)
class Actor:
    def __init__(self,prim):self.prim=prim;self.prim_path=prim.GetPath();self.moves=[]
    def get_world_pose(self):return np.array(self.prim.translation.Get()),np.array([1.,0,0,0])
    def set_world_pose(self,pos,quat):
        assert self.prim.kinematic.Get() is False,'Restore only after releasing kinematic mode'
        self.moves.append((pos.copy(),quat.copy()))
    def set_linear_velocity(self,v):pass
    def set_angular_velocity(self,v):pass

class ArrivalFixtureTest(unittest.TestCase):
    def test_authored_parking_precedes_reset_and_wait_releases_one(self):
        config=types.SimpleNamespace(SORT_CUBES=[(.3,.1,'blue'),(.4,.1,'blue'),(.4,.2,'red')],SORT_CUBE_YAWS_DEG=[0,0,0])
        physics=types.SimpleNamespace(RigidBodyAPI=lambda p:types.SimpleNamespace(CreateKinematicEnabledAttr=lambda:p.kinematic))
        geom=types.SimpleNamespace(Xformable=lambda p:types.SimpleNamespace(GetOrderedXformOps=lambda:[p.translation]),XformOp=types.SimpleNamespace(TypeTranslate='translate'))
        pxr=types.SimpleNamespace(UsdPhysics=physics,UsdGeom=geom,Gf=types.SimpleNamespace(Vec3d=lambda *v:list(v)))
        with patch.dict(sys.modules,{'scene_config':config,'pxr':pxr}):
            spec=importlib.util.spec_from_file_location('isolated_harness',Path(__file__).with_name('trial_harness.py'))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            harness=module.Harness('late_arrival');stage=Stage();harness.freeze(stage)
            self.assertEqual(len(harness.arrival_originals),2)
            self.assertTrue(all(p.translation.Get()[0]>1 for p in stage.prims[:2]))
            self.assertEqual(stage.prims[2].translation.Get(),[.3,.1,.9])
            actors={p.name:Actor(p) for p in stage.prims};harness._actor=lambda n:actors[n]
            harness.unfreeze(stage)
            self.assertTrue(all(p.kinematic.Get() for p in stage.prims[:2]))
            self.assertFalse(stage.prims[2].kinematic.Get())
            plant=types.SimpleNamespace(settle=lambda *args,**kwargs:None)
            harness.on_wait(plant);harness.on_wait(plant)
            self.assertFalse(stage.prims[0].kinematic.Get());self.assertTrue(stage.prims[1].kinematic.Get())
            self.assertEqual(len(actors['SortCube0'].moves),1)
            np.testing.assert_allclose(actors['SortCube0'].moves[0][0],[.3,.1,.9])
            self.assertEqual(len(actors['SortCube1'].moves),0)
            self.assertEqual([i['kind'] for i in harness.injections],['parts_parked_before_physics_reset','part_arrived_after_wait'])

if __name__=='__main__':unittest.main()
