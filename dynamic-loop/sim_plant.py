"""Simulation adapter: cameras, robot feedback and bounded physical motions.

No object-pose lookup is exposed to the supervisor. Scenario injection and
truth scoring live in trial_harness.py and cannot verify a runtime outcome.
"""
import hashlib
import json
import time
from pathlib import Path
import cv2
import numpy as np
import scene_config as C
import fk,ik,cell
from sort_scene import ease,turn,segments
from supervisor import CALIBRATION_ID,MODEL_ID,Hold,sensor_identifier
from vision import detect

class Plant:
    def __init__(self,folder,harness):
        import omni.usd
        from pxr import UsdGeom
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.sensors.camera import Camera
        self.UsdGeom=UsdGeom;self.folder=Path(folder);self.harness=harness
        self.world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/30)
        self.stage=omni.usd.get_context().get_stage()
        cell.build(self.stage);harness.freeze(self.stage)
        paths={'table':cell.TABLE_CAM_PATH,'side':cell.SIDE_CAM_PATH,'wrist':cell.WRIST_CAM_PATH}
        resolutions={'table':(3840,2160),'side':(1920,1080),'wrist':(1280,720)}
        self.cameras={name:Camera(prim_path=path,resolution=resolutions[name]) for name,path in paths.items()}
        self.paths=paths;self.resolutions=resolutions
        cell.enable_ccd(self.world);self.world.reset()
        for cam in self.cameras.values():cam.initialize()
        self.arm=SingleArticulation(prim_path=C.ROBOT_PRIM,name='er4ia');self.arm.initialize()
        names=list(self.arm.dof_names);self.indices={n:i for i,n in enumerate(names)}
        self.arm_idx=[self.indices[f'J{i}'] for i in range(1,7)]
        self.limits=np.deg2rad([C.JOINT_LIMITS_DEG[f'J{i}'] for i in range(1,7)])
        self.tool=tuple(np.asarray(C.TOOL_AXIS)*cell.tip_offset())
        ik.set_tool_axis(C.TOOL_AXIS);ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
        self.q=np.deg2rad([C.CAPTURE_POSE_DEG[f'J{i}'] for i in range(1,7)])
        self.home=fk.tool_pose(self.q,self.tool)[1]
        full=self.arm.get_joint_positions().copy();full[self.arm_idx]=self.q
        for joint,value in cell.jaw_targets(C.JAW_OPEN).items():full[self.indices[joint]]=value
        self.arm.set_joint_positions(full)
        self.jaw=C.JAW_OPEN;self.observation=0;self.command_frames=0;self.render_intervals=[]
        for _ in range(120):self.command(self.q,self.jaw);self.world.step(render=True)
        harness.unfreeze(self.stage)
        self.settle(30,render=True)

    def command(self,q,jaw):
        from isaacsim.core.utils.types import ArticulationAction
        full=self.arm.get_joint_positions().copy();full[self.arm_idx]=q
        for joint,value in cell.jaw_targets(jaw).items():full[self.indices[joint]]=value
        self.arm.apply_action(ArticulationAction(joint_positions=full))

    def settle(self,steps=24,render=False):
        for _ in range(steps):
            self.command(self.q,self.jaw)
            before=time.monotonic();sim_before=float(self.world.current_time)
            self.world.step(render=render)
            if render:
                self.render_intervals.append((sim_before,float(self.world.current_time),before,time.monotonic()))
                self.render_intervals=self.render_intervals[-64:]

    def tip(self):
        measured=self.arm.get_joint_positions()[self.arm_idx]
        return fk.tool_pose(measured,self.tool)

    def camera_model(self,name):
        prim=self.stage.GetPrimAtPath(self.paths[name]);gcam=self.UsdGeom.Camera(prim)
        xf=self.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        return dict(camera=name,resolution=list(self.resolutions[name]),
            focal_length_mm=float(gcam.GetFocalLengthAttr().Get()),
            horizontal_aperture_mm=float(gcam.GetHorizontalApertureAttr().Get()),
            vertical_aperture_mm=float(gcam.GetVerticalApertureAttr().Get()),
            camera_to_world=[[float(xf[r][c]) for c in range(4)] for r in range(4)],
            table_top_z=float(C.TABLE_TOP_Z+C.SURFACE_DZ),robot_base_xy=[0.,0.])

    def capture(self,label,perceive=True):
        # Render while testing stability: rendering advances physics, so a
        # non-rendering stability sample cannot certify a later camera frame.
        self.settle(18,render=True)
        stable=0
        for _ in range(240):
            self.settle(1,render=True)
            velocity=self.arm.get_joint_velocities()[self.arm_idx]
            still=bool(np.all(np.isfinite(velocity)) and np.max(np.abs(velocity))<.02)
            stable=stable+1 if still else 0
            if stable>=6:break
        else:raise Hold('robot_settle_timeout')
        self.observation+=1
        folder=self.folder/'observations'/f'{self.observation:03}-{label}'
        folder.mkdir(parents=True)
        images={};models={};views={}
        for name,cam in self.cameras.items():
            frame=cam.get_current_frame();rgba=cam.get_rgba()
            if rgba is None or np.asarray(rgba).ndim!=3:raise Hold('camera_image_missing:'+name)
            image=cv2.cvtColor(np.asarray(rgba)[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR)
            path=folder/(name+'.png')
            if not cv2.imwrite(str(path),image):raise RuntimeError('Image write failed')
            images[name]=image;models[name]=self.camera_model(name)
            (folder/(name+'-camera.json')).write_text(json.dumps(models[name],indent=2)+'\n')
            sensor_time=frame.get('rendering_time');sensor_frame=frame.get('rendering_frame')
            if sensor_time is None or sensor_frame is None:
                raise Hold('camera_capture_metadata_missing:'+name+':'+','.join(frame))
            reference=({key:int(sensor_frame[key]) for key in ('referenceTimeNumerator','referenceTimeDenominator')}
                       if isinstance(sensor_frame,dict) else None)
            # Bind the sensor's simulation capture time to a measured render-call
            # interval. Use its START as a conservative wall-clock age bound;
            # image readback and inference time must not make an old frame fresh.
            render_step=min(self.render_intervals,key=lambda interval:min(
                abs(float(sensor_time)-interval[0]),abs(float(sensor_time)-interval[1])))
            time_error=min(abs(float(sensor_time)-render_step[0]),abs(float(sensor_time)-render_step[1]))
            if time_error>1/30+.000001:raise Hold('capture_time_not_in_recent_render:'+name)
            views[name]=dict(sensor_time=float(sensor_time),sensor_frame=sensor_identifier(sensor_frame),
                             render_reference_time=reference,
                             capture_monotonic_lower_bound=render_step[2],
                             capture_wall_interval_s=[render_step[2],render_step[3]],
                             path=str(path.relative_to(self.folder)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        detected,rejected=detect(images['table'],models['table']) if perceive else ([],[])
        velocities=self.arm.get_joint_velocities()[self.arm_idx]
        idle=bool(np.all(np.isfinite(velocities)) and np.max(np.abs(velocities))<.02)
        result=dict(observation_id=self.observation,
                    captured_monotonic=min(v['capture_monotonic_lower_bound'] for v in views.values()),
                    received_unix=time.time(),calibration_id=CALIBRATION_ID,model_id=MODEL_ID,
                    views=views,robot_idle=idle,detections=detected,rejected=rejected,
                    world_time_at_readback=float(self.world.current_time),
                    robot_joint_velocity_max=float(np.max(np.abs(velocities))))
        (folder/'observation.json').write_text(json.dumps(result,indent=2)+'\n')
        overlay=images['table'].copy()
        for obj in detected:
            u,v=map(round,obj['pixel_xy']);cv2.circle(overlay,(u,v),20,(0,255,0),2)
            cv2.putText(overlay,obj['colour']+' %.3f'%obj['silhouette_iou'],(u+22,v),
                        cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),2)
        cv2.imwrite(str(folder/'table-overlay.png'),overlay)
        return result,images,models

    def prepare(self,target,slot):
        if C.CUBE_EDGE>=2*C.JAW_OPEN:raise Hold('cube_exceeds_gripper_opening')
        if not .85<=target['silhouette_iou']<=1:raise Hold('cube_pose_fit_uncertain')
        if not (.18<=target['xy'][0]<=.52 and -.40<=target['xy'][1]<=.32):raise Hold('outside_workspace')
        theta=np.deg2rad(target['yaw_deg']);spread=np.array([-np.sin(theta),np.cos(theta),0.])
        current=fk.fk(self.q)[0]@np.asarray(C.FINGER_SPREAD_AXIS)
        if np.dot(current,spread)<0:spread=-spread
        destination=np.array([0.,1.,0.])
        if np.dot(spread,destination)<0:destination=-destination
        pick=dict(category='cube',xy=target['xy'],pick_z=C.SURFACE_DZ+C.CUBE_TIP_CLEARANCE,
                  place_xy=slot['xy'],place_z=C.SURFACE_DZ+C.CUBE_TIP_CLEARANCE,
                  spread_dir=spread.tolist(),place_spread_dir=destination.tolist())
        q=self.q.copy();path={}
        for phase,p0,p1,duration,j0,j1,direction in segments(pick,self.home):
            initial=None
            if direction is not None and phase in ('approach','carry'):
                initial=fk.fk(q)[0]@np.asarray(C.FINGER_SPREAD_AXIS)
                initial[2]=0;initial/=np.linalg.norm(initial)
            points=[];n=round(duration*30)
            for i in range(n):
                t=ease((i+1)/n);goal=p0+(p1-p0)*t
                orient=turn(initial,direction,t) if initial is not None else direction
                q,ok=ik.solve(q,goal,[0,0,-1],self.limits,tool_offset=self.tool,spread_dir=orient)
                if not ok:q,ok=ik.solve(q,goal,[0,0,-1],self.limits,tool_offset=self.tool,
                                         spread_dir=orient,iters=500,damping=.02)
                r,actual=fk.tool_pose(q,self.tool)
                aligned=orient is None or abs(np.dot(r@np.asarray(C.FINGER_SPREAD_AXIS),orient))>.999
                if not ok or np.linalg.norm(actual-goal)>.001 or not aligned or np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):
                    raise Hold('trajectory_ik_preflight_failed:'+phase)
                points.append((q.copy(),j0+(j1-j0)*t,goal.copy()))
            path[phase]=points
        return dict(pick=pick,path=path)

    def move(self,plan,phases,event):
        for phase in phases:
            for q,jaw,goal in plan['path'][phase]:
                self.q=q.copy();self.jaw=self.harness.actuator(phase,jaw)
                self.command(self.q,self.jaw)
                for _ in range(2):self.world.step(render=False)
                self.command_frames+=1
            _,arrival=self.tip();arrival_error=float(np.linalg.norm(arrival-goal))
            # Stop at each commanded phase endpoint. Require measured convergence
            # before the next phase; never loosen the 2 mm tracking limit to
            # compensate for servo lag at the end of an eased trajectory.
            stable=0;settle_steps=0
            for _ in range(120):
                self.settle(1,render=False);settle_steps+=1
                _,actual=self.tip();error=float(np.linalg.norm(actual-goal))
                velocity=self.arm.get_joint_velocities()[self.arm_idx]
                converged=(np.isfinite(error) and error<=.002 and
                           np.all(np.isfinite(velocity)) and np.max(np.abs(velocity))<.02)
                stable=stable+1 if converged else 0
                if stable>=6:break
            self.world.render()
            r,actual=self.tip();error=float(np.linalg.norm(actual-goal))
            event.setdefault('motion',[]).append(dict(phase=phase,tip_error_mm=error*1000,
                                                       arrival_tip_error_mm=arrival_error*1000,
                                                       settling_steps=settle_steps,endpoint_stable=stable>=6,
                                                       commanded_tip_robot_m=goal.tolist(),actual_tip_robot_m=actual.tolist()))
            if not np.isfinite(error) or error>.002:raise Hold('actual_tip_tracking_error:'+phase)
            if stable<6:raise Hold('phase_endpoint_settle_timeout:'+phase)

    def reference(self):
        open_frame=self.capture('empty-hand-open',perceive=False)
        self.jaw=C.CUBE_JAW_HALF_TARGET;self.settle(45)
        closed_frame=self.capture('empty-hand-closed',perceive=False)
        self.jaw=C.JAW_OPEN;self.settle(45)
        return open_frame,closed_frame

    def retreat_empty(self,plan,event):
        self.jaw=C.JAW_OPEN;self.settle(30)
        # At the original lift/check pose. Return directly along the preflight
        # approach in reverse, preserving its validated joint limits and height.
        for q,_,goal in reversed(plan['path']['approach']):
            self.q=q.copy();self.command(self.q,self.jaw)
            for _ in range(2):self.world.step(render=False)
        self.settle(30)
