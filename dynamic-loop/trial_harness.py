"""Simulation setup/fault injection/truth scoring. Never perception or control."""
import math
import numpy as np
import scene_config as C

class Harness:
    def __init__(self,scenario,slots=None):
        self.scenario=scenario;self.attempt=0;self.moved=False;self.injections=[]
        self.slots=slots or [];self.injected=False;self.parked=[];self.arrival_originals={}
        self.layout=list(C.SORT_CUBES);self.yaws=list(C.SORT_CUBE_YAWS_DEG)
        if scenario=='empty':C.SORT_CUBES=[];C.SORT_CUBE_YAWS_DEG=[]
        if scenario=='missing':
            keep=[i for i,p in enumerate(self.layout) if p[2]!='red']
            C.SORT_CUBES=[self.layout[i] for i in keep];C.SORT_CUBE_YAWS_DEG=[self.yaws[i] for i in keep]
        self.names={f'SortCube{i}':p[2] for i,p in enumerate(C.SORT_CUBES)}

    def freeze(self,stage):
        from pxr import UsdPhysics,UsdGeom,Gf
        self.bodies=[]
        for prim in stage.Traverse():
            if prim.GetName().startswith(('SortCube','SortCap','SortVial')) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
                self.bodies.append(str(prim.GetPath()))
                if self.scenario=='late_arrival' and self.names.get(prim.GetName())=='blue':
                    # Author the fixture BEFORE physics initialization. A pose
                    # write on an already-kinematic actor can be restored from
                    # its authored transform by the next physics update.
                    translate=next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()
                                   if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
                    original=np.array(translate.Get(),dtype=float)
                    self.arrival_originals[str(prim.GetPath())]=original
                    parked=original.copy();parked[:2]=[1.5+.03*len(self.arrival_originals),1.5]
                    translate.Set(Gf.Vec3d(*parked))

    def unfreeze(self,stage):
        from pxr import UsdPhysics
        self.stage=stage
        for path in self.bodies:
            name=path.rsplit('/',1)[-1]
            body=UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path))
            if self.scenario=='late_arrival' and self.names.get(name)=='blue':
                actor=self._actor(name);_,quat=actor.get_world_pose()
                self.parked.append((actor,self.arrival_originals[path].copy(),np.array(quat)))
            else:body.CreateKinematicEnabledAttr().Set(False)
        if self.scenario=='late_arrival':
            if len(self.parked)!=2:raise RuntimeError('Late-arrival fixture requires exactly two parked blue parts')
            self.injections.append(dict(kind='parts_parked_before_physics_reset',count=len(self.parked),fixture_only=True))
        if self.scenario=='occupied_destination':
            name=next(n for n,c in self.names.items() if c=='red')
            actor=self._actor(name);pos,_=actor.get_world_pose();pos=np.array(pos)
            pos[:2]=self.slots[0]['xy'];self._move(actor,pos,[1.,0.,0.,0.])
            self.injections.append(dict(kind='destination_occupied',colour='red',slot_id=self.slots[0]['slot_id'],fixture_only=True))

    def _actor(self,name):
        from isaacsim.core.prims import SingleRigidPrim
        actor=SingleRigidPrim('/World/'+name);actor.initialize()
        return actor

    def _move(self,actor,position,orientation,clear_velocity=True):
        actor.set_world_pose(np.asarray(position),np.asarray(orientation))
        if clear_velocity:
            actor.set_linear_velocity(np.zeros(3));actor.set_angular_velocity(np.zeros(3))

    def before_capture(self,plant,label):
        if self.scenario=='missing_view' and label=='predispatch' and not self.injected:
            class MissingRGB:
                def __init__(self,wrapped):self.wrapped=wrapped
                def __getattr__(self,key):return getattr(self.wrapped,key)
                def get_rgba(self):return None
            plant.cameras['side']=MissingRGB(plant.cameras['side'])
            self.injected=True
            self.injections.append(dict(kind='rgb_stream_missing',view='side',at=label,fixture_only=True))

    def on_wait(self,plant):
        if self.scenario!='late_arrival' or self.injected:return
        from pxr import UsdPhysics,UsdGeom,Gf
        actor,pos,quat=self.parked[0]
        prim=self.stage.GetPrimAtPath(actor.prim_path)
        translate=next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()
                       if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        translate.Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False)
        self._move(actor,pos,quat)
        actor.set_linear_velocity(np.zeros(3));actor.set_angular_velocity(np.zeros(3))
        plant.settle(30,render=True);self.injected=True
        self.injections.append(dict(kind='part_arrived_after_wait',colour='blue',fixture_only=True))

    def before_pickup_check(self,plant):
        if self.scenario!='occluded_view' or self.injected:return
        from pxr import UsdGeom,Gf
        shield=UsdGeom.Cube.Define(plant.stage,plant.paths['side']+'/TestOccluder')
        shield.CreateSizeAttr(1.)
        shield.AddTranslateOp().Set(Gf.Vec3d(0,0,-.10))
        shield.AddScaleOp().Set(Gf.Vec3f(.5,.5,.002))
        shield.CreateDisplayColorAttr([Gf.Vec3f(.25,.25,.25)])
        self.injected=True
        self.injections.append(dict(kind='verification_view_obstructed',view='side',fixture_only=True))

    def after_placement(self,plant,slot):
        if self.scenario!='placed_moved' or self.injected:return
        actors=[]
        for name in self.names:
            actor=self._actor(name);pos,q=actor.get_world_pose()
            actors.append((actor,np.asarray(pos),q))
        actor,pos,q=min(actors,key=lambda x:math.dist(x[1][:2],slot['xy']))
        if math.dist(pos[:2],slot['xy'])>.010/3:raise RuntimeError('Placed-move fixture found no just-placed part')
        moved=pos.copy();moved[0]+=.015
        self._move(actor,moved,q);plant.settle(30,render=True);self.injected=True
        self.injections.append(dict(kind='completed_placement_displaced',slot_id=slot['slot_id'],delta_m=[.015,0],fixture_only=True))

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
