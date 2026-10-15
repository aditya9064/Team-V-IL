"""Simulation setup/fault injection/truth scoring. Never perception or control."""
import math
import numpy as np
import scene_config as C

class Harness:
    def __init__(self,scenario):
        self.scenario=scenario;self.attempt=0;self.moved=False;self.injections=[]
        self.layout=list(C.SORT_CUBES);self.yaws=list(C.SORT_CUBE_YAWS_DEG)
        if scenario=='empty':C.SORT_CUBES=[];C.SORT_CUBE_YAWS_DEG=[]
        if scenario=='missing':
            keep=[i for i,p in enumerate(self.layout) if p[2]!='red']
            C.SORT_CUBES=[self.layout[i] for i in keep];C.SORT_CUBE_YAWS_DEG=[self.yaws[i] for i in keep]
        self.names={f'SortCube{i}':p[2] for i,p in enumerate(C.SORT_CUBES)}

    def freeze(self,stage):
        from pxr import UsdPhysics
        self.bodies=[]
        for prim in stage.Traverse():
            if prim.GetName().startswith(('SortCube','SortCap','SortVial')) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
                self.bodies.append(str(prim.GetPath()))

    def unfreeze(self,stage):
        from pxr import UsdPhysics
        for path in self.bodies:UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr().Set(False)

    def precheck(self,plant,proposal):
        if self.scenario!='moved' or self.moved:return
        from isaacsim.core.prims import SingleRigidPrim
        wrappers=[]
        for name in self.names:
            actor=SingleRigidPrim('/World/'+name);actor.initialize()
            pos,quat=actor.get_world_pose();wrappers.append((actor,np.array(pos),quat))
        actor,pos,quat=min(wrappers,key=lambda item:np.linalg.norm(item[1][:2]-proposal['target']['xy']))
        for delta in ([.025,0.],[-.025,0.],[0.,.03],[0.,-.03]):
            candidate=pos.copy();candidate[:2]+=delta
            if .23<candidate[0]<.47 and -.17<candidate[1]<.29 and all(
                other is actor or np.linalg.norm(candidate[:2]-p[:2])>.025 for other,p,_ in wrappers):break
        else:raise RuntimeError('No valid target-shift test location')
        actor.set_world_pose(candidate,quat);actor.set_linear_velocity(np.zeros(3));actor.set_angular_velocity(np.zeros(3))
        self.moved=True;plant.settle(30,render=True)
        self.injections.append(dict(kind='target_shift_after_proposal',delta_m=delta))

    def begin_attempt(self):self.attempt+=1

    def actuator(self,phase,jaw):
        if self.scenario in ('miss_once','miss_twice') and (self.attempt==1 or self.scenario=='miss_twice'):
            if phase in ('close','dwell','lift'):return C.JAW_OPEN
        return jaw

    def score(self,slots,completed):
        from isaacsim.core.prims import SingleXFormPrim
        poses={}
        for name,colour in self.names.items():
            pos,q=SingleXFormPrim('/World/'+name).get_world_pose();w,x,y,z=q
            rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                          [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                          [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            centre=np.array(pos)+rot@np.array([0,0,C.CUBE_EDGE/2])
            poses[name]=dict(colour=colour,centre=centre.tolist(),
                             yaw_deg=float(np.rad2deg(np.arctan2(rot[1,0],rot[0,0]))),
                             tilt_deg=float(np.rad2deg(np.arccos(np.clip(rot[2,2],-1,1)))))
        results=[]
        for slot in slots:
            near=[(n,p) for n,p in poses.items() if math.dist(p['centre'][:2],slot['xy'])<.010/3]
            expected_z=C.TABLE_TOP_Z+C.SURFACE_DZ+C.CUBE_EDGE/2
            correct=(len(near)==1 and near[0][1]['colour']==slot['colour']
                     and abs(near[0][1]['centre'][2]-expected_z)<.0015
                     and abs((near[0][1]['yaw_deg']+45)%90-45)<8
                     and near[0][1]['tilt_deg']<5)
            results.append(dict(slot_id=slot['slot_id'],camera_completed=slot['slot_id'] in completed,
                                truth_correct=bool(correct),objects=near))
        return dict(evaluation_only=True,object_poses=poses,slots=results,injections=self.injections)
