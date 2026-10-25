"""Loaded-carrier/full PCB-holder mechanical trials. No learned or real control.

Simulator poses are independent scoring/stop evidence, never target corrections.
Video stays in the remote run directory; no download operation is implemented.
"""
import argparse,hashlib,json,math,sys,traceback
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from held_pose import estimate,placement_from_held,project,front_edges
from release_plan import square_release_plan
from motion_schedule import motion_samples
from payload_schedule import payload_carry_samples,profile_bounds
from assembly_checks import classify_contact,contact_violation,grasp_command_m
from fixture_contacts import guide_contact
from finger_cooking import configure_source_fidelity,audit_cooked_finger_surfaces
from payload_checks import payload_score,initial_supports,placement_supports,horizontal_seating_normals,horizontal_support_pairs,initial_supports_for_case

ORDER=('base',)
COLORS=((.12,.43,.70),(.08,.58,.30),(.86,.48,.07),(.49,.64,.79))


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    global ORDER
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('starter','mesh','design','out'):ap.add_argument('--'+name,required=True)
    ap.add_argument('--case',choices=['loaded-carrier','full-assembly'],required=True)
    a=ap.parse_args();starter=Path(a.starter).resolve();meshpath=Path(a.mesh).resolve();designpath=Path(a.design).resolve();out=Path(a.out).resolve()
    ORDER=('base','carrier','cover')
    if a.case!='full-assembly':raise ValueError('Full assembly only')
    physical_parts=['base','carrier','pcb','nest'] if a.case=='loaded-carrier' else ['base','carrier','pcb','cover','nest']
    out.mkdir(parents=True,exist_ok=False)
    geometry=json.loads(meshpath.read_text());design=json.loads(designpath.read_text());limits=geometry['limits'];layout=geometry['layout_m']
    if geometry['units']!='millimeter' or design['units']!='millimeter' or digest(designpath)!=geometry['design_sha256']:raise ValueError('Design/mesh units or identity mismatch')
    if design['assembly']['sequence']!=['base','carrier','cover']:raise ValueError('Unexpected full assembly order')
    grip=geometry['grip_command']
    expected_jaw=grasp_command_m(grip['nominal_contact_per_jaw_mm'],grip['extra_travel_per_jaw_mm'],limits['maximum_intended_penetration_mm'])
    if not math.isclose(expected_jaw,geometry['closed_grasp_command_per_jaw_m'],abs_tol=1e-12):raise ValueError('Grip command audit differs')
    commands=geometry.get('grip_commands',{name:dict(grip,closed_grasp_command_per_jaw_m=expected_jaw) for name in ORDER})
    for name,g in commands.items():
        expected=grasp_command_m(g['nominal_contact_per_jaw_mm'],g['extra_travel_per_jaw_mm'],limits['maximum_intended_penetration_mm'])
        if not math.isclose(expected,g['closed_grasp_command_per_jaw_m'],abs_tol=1e-12) or not 0<expected<.006:raise ValueError('Per-part grip command identity/range mismatch')
    report={'status':'STARTING','task':'full_assembly_with_verified_base_held_pose_fix','case':a.case,'vision_guided':True,'physical_hardware_test':False,
        'design_sha256':digest(designpath),'design_revision':design['revision'],'mesh_sha256':digest(meshpath),'trial_script_sha256':digest(__file__),
        'part_order':list(ORDER),'physical_parts':physical_parts,'limits':limits,'payload_limits':geometry['payload_limits'],'layout_m':layout,'collision_geometry_audit':geometry['collision_audit'],'grip_command':grip,'grip_commands':commands,
        'checkpoints':[],'part_results':[],'unexpected_contacts':[],'contact_monitor_errors':[],'contact_summary':{},
        'assumptions':{'material_density_kg_m3':1240,'solid_density_not_actual_print_mass':True,'friction':'unchanged supplied generic 0.9/0.8 and estimated high-friction fingers','all_product_parts_dynamic':True,'static_nest_fixture':True,'fixture_or_kinematic_attachment':False,'grasps_or_poses_corrected_from_truth':False},
        'payload_initial_gap_mm':geometry['payload_initial_gap_mm'],'structural_subassembly_assumption':design.get('rigid_subassembly_assumption'),'scope':'Known-pose base, loaded tray, lid assembly. Base uses RGB held-pose correction and staged release from accepted286139. Tray/lid use existing CAD recipe targets. PCB separate loose dynamic body. No trained detector, random-pose, print or physical-hardware qualification.',
        'initial_preparation':{'product_parts_preassembled_together':False,'pcb_preloaded_in_carrier':True,'only_preplaced_fixture':(['base','nest'] if a.case=='loaded-carrier' else ['nest']),'robot_placement_order':list(ORDER)},
        'video':{'download_to_laptop':False,'camera':'angled overview camera; continuous actual simulation frames','fps':30,'resolution':[1280,960],'frames':0},'print_release':False,'motion_sampling':{'command_hz':60,'physics_steps_per_command':1,'path_and_phase_durations_unchanged':False,'original_phase_durations_retained':False,'only_duration_change':{'2_carrier_carry':{'before_s':2.,'after_s':4.}},'changes':'Verified base RGB leveling/measured target/staged release; existing tray and lid CAD paths retained','phases':[]}}
    def persist():(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    report['physical_nest']={'contract_sha256':digest(Path(__file__).with_name('nest-proof.json')),'contact_checker_sha256':digest(Path(__file__).with_name('fixture_contacts.py')),'base_dynamic_throughout':True,'product_attachments':False,'no_clamp_or_preload':True}
    report['grip_collision_union_audit']=geometry['grip_collision_union_audit']
    persist();sys.path.insert(0,str(starter))
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True});writer=None;contact_sub=None
    try:
        import cv2,imageio_ffmpeg,omni.usd,omni.physx
        from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,PhysicsSchemaTools
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation,SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera
        import scene_config as C,cell,ik,fk,gripper_geometry
        report['starter_hashes']={name:digest(starter/name) for name in ['cell.py','scene_config.py','gripper_geometry.py','ik.py','fk.py']}
        if report['starter_hashes']['gripper_geometry.py']!=geometry['finger_profile_sha256']:raise ValueError('Finger profile differs from source audit')
        world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/30)
        stage=omni.usd.get_context().get_stage();cell.build(stage,with_objects=False)
        report['finger_cooking']=configure_source_fidelity(stage)
        report['finger_cooking_script_sha256']=digest(Path(__file__).with_name('finger_cooking.py'))
        surface=C.TABLE_TOP_Z+C.SURFACE_DZ;roots={};collider_roles={}
        for index,name in enumerate(physical_parts):
            path='/World/Assembly_'+name;root=UsdGeom.Xform.Define(stage,path).GetPrim();roots[name]=path
            xy=layout['assembly_body_centre'] if name in ('base','nest') and a.case=='loaded-carrier' else layout['pickup_body_centres'][name]
            z=surface+(0. if name=='nest' else .0033 if name=='base' and a.case=='loaded-carrier' else .0003)
            if name=='pcb':z+=design['carrier']['floor_thickness']*.001+geometry['payload_initial_gap_mm']*.001
            UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(float(xy[0]),float(xy[1]),z))
            for label,data,visible in [('Visual',geometry['parts'][name]['visual'],True)]+[
                    (f'C_{i:03d}_{r["role"]}',r,False) for i,r in enumerate(geometry['parts'][name]['colliders'])]:
                m=UsdGeom.Mesh.Define(stage,path+'/'+label);v=np.asarray(data['vertices_mm'],float);f=np.asarray(data['faces'],int)
                if visible:v=v*.001
                else:
                    v=v*.1
                    UsdGeom.Xformable(m.GetPrim()).AddScaleOp().Set(Gf.Vec3f(.01,.01,.01))
                m.CreatePointsAttr([Gf.Vec3f(*p) for p in v]);m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel().tolist());m.CreateSubdivisionSchemeAttr('none')
                if visible:m.CreateDisplayColorAttr([Gf.Vec3f(*{'nest':(.4,.4,.4),'base':(.12,.43,.70),'carrier':(.58,.65,.68),'pcb':(.08,.58,.30),'frame':(.86,.48,.07),'cover':(.49,.64,.79)}[name])])
                else:
                    UsdGeom.Imageable(m).CreateVisibilityAttr().Set('invisible');UsdPhysics.CollisionAPI.Apply(m.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr().Set('convexHull')
                    hull=PhysxSchema.PhysxConvexHullCollisionAPI.Apply(m.GetPrim());hull.CreateHullVertexLimitAttr().Set(255);hull.CreateMinThicknessAttr().Set(0.)
                    collision=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());collision.CreateContactOffsetAttr().Set(.0002);collision.CreateRestOffsetAttr().Set(0.)
                    cell.bind_physics_material(m.GetPrim());collider_roles[str(m.GetPath())]=(name,data['role'])
            mass=geometry['parts'][name]['volume_mm3']*1e-9*1240;
            if name!='nest':cell.make_dynamic(root,mass,collide_self=False)
            report.setdefault('part_masses_kg',{})[name]=mass
        report['separate_body_audit']={}
        for name,path in roots.items():
            body=stage.GetPrimAtPath(path);api=UsdPhysics.RigidBodyAPI(body)
            enabled=bool(api.GetRigidBodyEnabledAttr().Get()) if name!='nest' else False;kinematic=bool(api.GetKinematicEnabledAttr().Get()) if name!='nest' else False
            gravity_disabled=bool(PhysxSchema.PhysxRigidBodyAPI(body).GetDisableGravityAttr().Get()) if name!='nest' else False
            if name!='nest' and (not enabled or kinematic or gravity_disabled):raise RuntimeError('Part is not freely dynamic: '+name)
            report['separate_body_audit'][name]={'root':path,'dynamic':enabled,'kinematic':kinematic,'gravity_disabled':gravity_disabled,'mass_kg':report['part_masses_kg'][name],'fixture_static':name=='nest'}
        report['part_attachment_joints']=[]
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint):
                j=UsdPhysics.Joint(prim);targets=list(j.GetBody0Rel().GetTargets())+list(j.GetBody1Rel().GetTargets())
                if any(str(t).startswith('/World/Assembly_') for t in targets):report['part_attachment_joints'].append(str(prim.GetPath()))
        if report['part_attachment_joints']:raise RuntimeError('Hidden part attachment is forbidden')
        report['new_part_convex_cooking']={'hull_vertex_limit':255,'min_thickness_m':0.,'contact_offset_m':.0002,'rest_offset_m':0.,'input_coordinates':'millimeter vertices, explicit uniform child scale 0.001 to meters','note':'World source geometry preserved. Supplied finger hulls use the separately audited source-fidelity numerical cooking representation.'}
        report['physical_collider_counts']={name:sum(v[0]==name for v in collider_roles.values()) for name in physical_parts}
        cell.camera(stage,'/World/AssemblyTop',dict(eye=(.32,-.02,surface+.9),look_at=(.32,-.02,surface),up=(0,1,0),resolution=(1280,960),focal_length=32.,horiz_aperture=20.955))
        cell.camera(stage,'/World/AssemblyClose',dict(eye=(.56,-.20,surface+.29),look_at=(.30,.10,surface+.04),up=(0,0,1),resolution=(1280,720),focal_length=36.,horiz_aperture=20.955))
        cell.camera(stage,'/World/AssemblyOverview',dict(eye=(.65,-.55,surface+.48),look_at=(.32,-.02,surface+.04),up=(0,0,1),resolution=(1280,960),focal_length=27.,horiz_aperture=20.955))
        cameras={'top':Camera(prim_path='/World/AssemblyTop',resolution=(1280,960)),
                 'overview':Camera(prim_path='/World/AssemblyOverview',resolution=(1280,960)),
                 'side':Camera(prim_path=cell.SIDE_CAM_PATH,resolution=(960,540)),
                 'assembly':Camera(prim_path='/World/AssemblyClose',resolution=(1280,720))}
        inspection_nominal=np.array([*layout['assembly_body_centre'],surface+layout['travel_tip_height_above_table']-design['base']['tcp_above_bottom_mm']*.001])
        for key,offset in [('inspection_x',[.19,-.08,-.05]),('inspection_y',[-.08,-.19,-.05])]:
            path='/World/'+key
            cell.camera(stage,path,dict(eye=tuple(inspection_nominal+offset),look_at=tuple(inspection_nominal+[0,0,.01]),up=(0,0,1),resolution=(1920,1440),focal_length=35.,horiz_aperture=20.955))
            cameras[key]=Camera(prim_path=path,resolution=(1920,1440))
        state={'phase':'initial_settling','active':None,'allow_active_table':False,'allow_grip':False,'step':0,'held_reference':None,'released_reference':None,'fixture_entry_enabled':False}
        support_pairs=initial_supports_for_case(a.case);placed=['nest'];contact_count=0
        # Read-only before-step evidence: never feeds commands or pass/fail gates.
        diagnostic_roots=[C.ROBOT_PRIM+'/JawL',C.ROBOT_PRIM+'/JawR']+[roots[n] for n in ORDER]+[roots['pcb']]
        contact_pose_trace=[]
        def contact_pose_snapshot():
            return {'step':state['step'],'sim_time':float(world.current_time),
                'actor_world_matrices':{path:[list(row) for row in UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)] for path in diagnostic_roots},
                'joint_positions':arm.get_joint_positions().tolist(),
                'joint_velocities':arm.get_joint_velocities().tolist(),
                'commanded_arm_joints':q.tolist(),'commanded_jaw_per_side_m':float(jaw)}
        def pair(a,b):return tuple(sorted([tuple(a),tuple(b)]))
        planned_supports=placement_supports()
        def fail_contacts():
            if report['contact_monitor_errors']:raise RuntimeError('Contact monitor error: '+report['contact_monitor_errors'][0])
            if report['unexpected_contacts']:raise RuntimeError('Unintended contact: '+json.dumps(report['unexpected_contacts'][0]))
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        def on_contacts(headers,data,*unused):
            nonlocal contact_count
            for h in headers:
                contact_count+=1
                try:
                    if not h.num_contact_data:continue
                    actors=[str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))]
                    colliders=[str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))]
                    roles=[collider_roles.get(c) for c in colliders]
                    if all(r is not None for r in roles) and {r[0] for r in roles}=={'carrier','pcb'}:
                        report['payload_contact_events']=report.get('payload_contact_events',0)+1
                    allowed,category=classify_contact(actors,colliders,roles,state,support_pairs,C.ROBOT_PRIM)
                    relevant=any(r is not None for r in roles) or any(x.startswith(C.ROBOT_PRIM) for x in actors)
                    if not relevant:continue
                    points=[data[i] for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                    if all(r is not None for r in roles) and pair(*roles) in horizontal_support_pairs():
                        normals_ok=horizontal_seating_normals([[float(x) for x in v.normal] for v in points])
                        audit=report.setdefault('horizontal_seating_contacts',{'events':0,'only_horizontal':True,'cad_support_proof_sha256':digest(Path(__file__).with_name('seating-support-proof.json')),'covered_pairs':[str(v) for v in horizontal_support_pairs()]})
                        audit['events']+=1;audit['only_horizontal']=audit['only_horizontal'] and normals_ok
                        allowed=allowed and normals_ok

                    fixture_allowed=guide_contact(colliders,roles,state,[[float(x) for x in v.normal] for v in points])
                    if fixture_allowed is not None:
                        allowed=fixture_allowed;category='planned_nest_guide' if allowed else 'unplanned_nest_guide'
                        report.setdefault('nest_guide_contacts',[]).append({'step':state['step'],'phase':state['phase'],'colliders':colliders,'allowed':allowed,'normals':[[float(x) for x in v.normal] for v in points],'minimum_separation_mm':min(float(v.separation) for v in points)*1000,'maximum_impulse_Ns':max(float(np.linalg.norm(v.impulse)) for v in points)})
                    minsep=min(float(v.separation) for v in points)
                    impulse=max(float(np.sqrt(sum(float(x)**2 for x in v.impulse))) for v in points)
                    if not math.isfinite(minsep) or not math.isfinite(impulse):raise ValueError('Non-finite contact data')
                    key=state['phase']+'|'+category+'|'+str(roles)+'|'+str(colliders if not any(roles) else '')
                    summary=report['contact_summary'].setdefault(key,{'events':0,'allowed_pair':allowed,'minimum_separation_mm':float('inf'),'maximum_impulse_Ns':0.})
                    summary['events']+=1;summary['minimum_separation_mm']=min(summary['minimum_separation_mm'],minsep*1000);summary['maximum_impulse_Ns']=max(summary['maximum_impulse_Ns'],impulse)
                    if any(r==('base','grip_support') for r in roles) and category=='grip_contact':
                        b=report.setdefault('base_shoulder_contact_observations',{'events':0,'positive_impulse_events':0,'max_abs_normal_z':0.})
                        b['events']+=1;b['positive_impulse_events']+=int(impulse>1e-8);b['max_abs_normal_z']=max(b['max_abs_normal_z'],max(abs(float(v.normal[2])) for v in points))
                    if any(r==('carrier','grip_support') for r in roles) and category=='grip_contact':
                        a=report.setdefault('carrier_shoulder_contact_observations',{'events':0,'positive_impulse_events':0,'max_abs_normal_z':0.})
                        a['events']+=1;a['positive_impulse_events']+=int(impulse>1e-8);a['max_abs_normal_z']=max(a['max_abs_normal_z'],max(abs(float(v.normal[2])) for v in points))
                    violation=contact_violation(allowed,minsep,impulse,limits['maximum_intended_penetration_mm'])
                    if violation and len(report['unexpected_contacts'])<50:
                        report['unexpected_contacts'].append({'phase':state['phase'],'step':state['step'],'actors':actors,'colliders':colliders,'roles':roles,'allowed_pair':allowed,'minimum_separation_mm':minsep*1000,'maximum_impulse_Ns':impulse,
                            'contact_points':[{'position_world_m':[float(x) for x in v.position],
                                'normal_world':[float(x) for x in v.normal],
                                'separation_mm':float(v.separation)*1000,
                                'impulse_Ns':[float(x) for x in v.impulse]} for v in points]})
                except Exception as e:
                    if len(report['contact_monitor_errors'])<10:report['contact_monitor_errors'].append(repr(e))
        report['payload_contact_events']=0
        contact_sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
        report['contact_monitor_setup']={'api_and_subscription_before_reset':True,'runtime_checks_active_during_reset':True}

        physics=world.get_physics_context();cell.enable_ccd(world);physics.set_solver_type('PGS');world.reset()
        report['contact_monitor_setup']['headers_seen_during_reset']=contact_count
        fail_contacts()
        if physics.get_solver_type()!='PGS':raise RuntimeError('PGS readback failed')
        arm=SingleArticulation(prim_path=C.ROBOT_PRIM,name='assembly_er4ia');arm.initialize()
        names=list(arm.dof_names);indices={n:i for i,n in enumerate(names)};ai=[indices['J'+str(i)] for i in range(1,7)]
        report['physics']={'solver':physics.get_solver_type(),'physics_dt':float(world.get_physics_dt()),'render_dt':float(world.get_rendering_dt()),
            'position_iterations':int(arm.get_solver_position_iteration_count()),'velocity_iterations':int(arm.get_solver_velocity_iteration_count())}
        if (report['physics']['position_iterations'],report['physics']['velocity_iterations'])!=(32,1):raise RuntimeError('Original articulation iterations changed')
        if not math.isclose(report['physics']['physics_dt'],1/60,abs_tol=1e-9) or not math.isclose(report['physics']['render_dt'],1/30,abs_tol=1e-9):raise RuntimeError('Physics/render timestep readback differs')
        report['step_timing_audit']={'samples':0,'maximum_step_error_s':0.,'maximum_render_advance_s':0.,'method':'one physics-only step; separate render every two steps'}
        for cam in cameras.values():cam.initialize()
        prims={name:SingleXFormPrim(path) for name,path in roots.items()}
        ik.set_tool_axis(C.TOOL_AXIS);ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
        tool=tuple(np.array(C.TOOL_AXIS)*cell.tip_offset());joint_limits=np.deg2rad([C.JOINT_LIMITS_DEG['J'+str(i)] for i in range(1,7)])
        down=np.array([0.,0.,-1.]);spread=np.array([0.,1.,0.]);goal=np.array([.30,-.10,C.SURFACE_DZ+layout['travel_tip_height_above_table']]);jaw=C.JAW_OPEN
        q=np.deg2rad([C.HOME_POSE_DEG['J'+str(i)] for i in range(1,7)])
        q,ok=ik.solve(q,goal,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
        if not ok:raise RuntimeError('Initial home IK failed')
        def command(j):
            full=arm.get_joint_positions().copy();full[ai]=q
            for name,value in cell.jaw_targets(j).items():full[indices[name]]=value
            arm.apply_action(ArticulationAction(joint_positions=full))
        full=arm.get_joint_positions().copy();full[ai]=q
        for name,value in cell.jaw_targets(jaw).items():full[indices[name]]=value
        arm.set_joint_positions(full)
        partial=out/'angled-view-recording.partial.mp4'
        writer=imageio_ffmpeg.write_frames(str(partial),(1280,960),fps=30,codec='libx264',quality=7,
            output_params=['-preset','fast','-threads','2','-movflags','+faststart'])
        writer.send(None)
        def advance(n=1):
            for _ in range(n):
                state['step']+=1;render=state['step']%2==0
                contact_pose_trace.append(contact_pose_snapshot())
                del contact_pose_trace[:-6]
                before_step=float(world.current_time);world.step(render=False)
                after_step=float(world.current_time);step_error=abs(after_step-before_step-1/60)
                timing=report['step_timing_audit'];timing['samples']+=1;timing['maximum_step_error_s']=max(timing['maximum_step_error_s'],step_error)
                if step_error>2e-6:raise RuntimeError('Physics step duration differs from 1/60 s')
                if render:
                    world.render();render_advance=abs(float(world.current_time)-after_step)
                    timing['maximum_render_advance_s']=max(timing['maximum_render_advance_s'],render_advance)
                    if render_advance>1e-9:raise RuntimeError('Rendering advanced physics unexpectedly')
                fail_contacts()
                score=payload_score(poses()['carrier'],poses()['pcb'],design['carrier']['floor_thickness']*.001)
                audit=report.setdefault('payload_motion_audit',{'samples':0,'maximum_xy_mm':0.,'maximum_z_mm':0.,'maximum_relative_tilt_deg':0.,'maximum_relative_yaw_deg':0.})
                audit['samples']+=1
                for key,resultkey in [('xy_mm','maximum_xy_mm'),('z_mm','maximum_z_mm'),('tilt_deg','maximum_relative_tilt_deg'),('yaw_deg','maximum_relative_yaw_deg')]:audit[resultkey]=max(audit[resultkey],score[key])
                if not score['pass']:
                    report['payload_failure']={'phase':state['phase'],'step':state['step'],'score':score}
                    raise RuntimeError('Loose PCB shifted/tilted outside carrier limits during '+state['phase'])
                if state['released_reference'] is not None:
                    current=np.asarray(poses()[state['active']]['position_world_m']);drift=float(np.linalg.norm(current-np.asarray(state['released_reference']))*1000)
                    audit=report.setdefault('release_motion_audits',{}).setdefault(state['active'],{'samples':0,'maximum_drift_mm':0.})
                    audit['samples']+=1;audit['maximum_drift_mm']=max(audit['maximum_drift_mm'],drift)
                    if drift>limits['release_follow_mm']:raise RuntimeError('Released base dragged during '+state['phase'])
                if state['held_reference'] is not None:
                    held_pose=poses()[state['active']];held_tip,*_=feedback();relative=np.asarray(held_pose['position_world_m'])-held_tip-np.array([0,0,C.TABLE_TOP_Z])
                    slip=float(np.linalg.norm(relative-np.asarray(state['held_reference']))*1000)
                    audit=report.setdefault('held_motion_audits',{}).setdefault(state['active'],{'samples':0,'maximum_relative_slip_mm':0.,'maximum_tilt_deg':0.,'maximum_yaw_deg':0.})
                    audit['samples']+=1;audit['maximum_relative_slip_mm']=max(audit['maximum_relative_slip_mm'],slip);audit['maximum_tilt_deg']=max(audit['maximum_tilt_deg'],held_pose['tilt_deg']);audit['maximum_yaw_deg']=max(audit['maximum_yaw_deg'],abs(held_pose['yaw_deg']))
                    if slip>limits['maximum_pickup_xy_slip_mm'] or held_pose['tilt_deg']>limits['maximum_pickup_tilt_deg'] or abs(held_pose['yaw_deg'])>limits['maximum_pickup_yaw_deg']:raise RuntimeError('Held part slipped/tilted during '+state['phase'])
                if placed and render:
                    current=poses()
                    for prior in placed:
                        score=score_placed(prior,current[prior])
                        if not score['pass']:
                            report['transient_inventory_failure']={'part':prior,'phase':state['phase'],'step':state['step'],'score':score}
                            raise RuntimeError('Previously placed '+prior+' moved during '+state['phase'])
                if render:
                    rgba=cameras['overview'].get_rgba()
                    if rgba is None or not np.asarray(rgba).size:
                        if report['video']['frames']:raise RuntimeError('Overview camera disappeared during recording')
                        continue
                    rgb=np.ascontiguousarray(np.asarray(rgba[:,:,:3],dtype=np.uint8).copy())
                    if rgb.shape!=(960,1280,3):raise RuntimeError('Unexpected video camera size')
                    cv2.rectangle(rgb,(0,0),(1280,64),(20,28,38),-1)
                    cv2.putText(rgb,'SIMULATION | base > loaded tray > lid | current fingers',(18,25),cv2.FONT_HERSHEY_SIMPLEX,.62,(245,245,245),1,cv2.LINE_AA)
                    cv2.putText(rgb,state['phase'].replace('_',' '),(18,51),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,210,110),1,cv2.LINE_AA)
                    writer.send(rgb);report['video']['frames']+=1
        def feedback():
            joints=arm.get_joint_positions()[ai];speeds=arm.get_joint_velocities()[ai];R,tip=fk.tool_pose(joints,tool)
            if not np.all(np.isfinite(joints)) or not np.all(np.isfinite(speeds)) or not np.all(np.isfinite(tip)):raise RuntimeError('Non-finite robot feedback')
            axis=R@np.asarray(C.TOOL_AXIS);angle=math.degrees(math.acos(float(np.clip(np.dot(axis,down),-1,1))))
            actual_spread=R@np.asarray(C.FINGER_SPREAD_AXIS);spread_angle=math.degrees(math.acos(float(np.clip(np.dot(actual_spread,spread),-1,1))))
            return np.asarray(tip),float(np.max(np.abs(speeds))),angle,spread_angle
        def poses():
            values={}
            for name,prim in prims.items():
                pos,quat=prim.get_world_pose();pos=np.asarray(pos,float);quat=np.asarray(quat,float)
                if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(quat)) or abs(np.linalg.norm(quat)-1)>1e-3:raise RuntimeError('Invalid object pose')
                w,x,y,z=quat;tilt=math.degrees(math.acos(float(np.clip(1-2*(x*x+y*y),-1,1))));yaw=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
                values[name]={'position_world_m':pos.tolist(),'orientation_wxyz':quat.tolist(),'tilt_deg':tilt,'yaw_deg':yaw}
            return values
        def save(label):
            stable=0;trace=[]
            for _ in range(180):
                command(jaw);advance(2);tip,speed,angle,spread_angle=feedback()
                stable=stable+1 if speed<limits['camera_speed_rad_s'] else 0
                trace.append({'step':state['step'],'sim_time':float(world.current_time),'maximum_raw_joint_speed_rad_s':speed,'tip_robot_m':tip.tolist()})
                if stable>=limits['camera_samples']:break
            if stable<limits['camera_samples']:raise RuntimeError('Camera stillness gate failed at '+label)
            shot={'phase':label,'step':state['step'],'sim_time':float(world.current_time),'poses':poses(),'stillness_trace':trace[-limits['camera_samples']:],
                'tip_robot_m':tip.tolist(),'tool_axis_error_deg':angle,'spread_axis_error_deg':spread_angle,'payload_score':payload_score(poses()['carrier'],poses()['pcb'],design['carrier']['floor_thickness']*.001),'jaw_joints':{n:float(arm.get_joint_positions()[indices[n]]) for n in cell.jaw_targets(jaw)}}
            report['checkpoints'].append(shot)
            for name,cam in cameras.items():
                rgba=cam.get_rgba()
                if rgba is None or not np.asarray(rgba).size:raise RuntimeError('Missing camera '+name)
                if not cv2.imwrite(str(out/(label+'-'+name+'.png')),cv2.cvtColor(np.asarray(rgba[:,:,:3],np.uint8),cv2.COLOR_RGB2BGR)):raise RuntimeError('Camera evidence write failed')
            persist();print('[assembly]',label,flush=True);return shot
        def move(label,newgoal,newjaw,duration,allow_table=False,new_support=False,new_spread=None,new_orientation=None):
            nonlocal q,jaw,goal,spread,down
            state['phase']=label;start=goal.copy();j0=jaw;samples=(payload_carry_samples(duration,1/60) if label=='2_carrier_carry' else motion_samples(duration,1/60));state['allow_active_table']=allow_table
            phase_start=float(world.current_time);command_times=[]
            start_angle=math.atan2(spread[1],spread[0]);end=spread if new_spread is None else np.asarray(new_spread)
            delta_angle=(math.atan2(end[1],end[0])-start_angle+math.pi)%(2*math.pi)-math.pi
            active=state['active']
            orientation_path=None
            if new_orientation is not None:
                start_R,_=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                orientation_path=Slerp([0,1],Rotation.from_matrix(np.array([start_R,new_orientation])))
            for elapsed,s in samples:
                t=elapsed/duration;target=start+(newgoal-start)*s;j=j0+(newjaw-j0)*s
                angle=start_angle+delta_angle*s;spread=np.array([math.cos(angle),math.sin(angle),0.])
                if orientation_path is not None:
                    desired_R=orientation_path(s).as_matrix();down=desired_R@np.asarray(C.TOOL_AXIS);spread=desired_R@np.asarray(C.FINGER_SPREAD_AXIS)
                if 'lift' in label and t>.2:state['allow_active_table']=False
                if new_support and abs(float(target[2]-newgoal[2]))<=.0025:
                    support_pairs.update(planned_supports[active])
                    if active=='base':
                        state['allow_active_table']=True;state['fixture_entry_enabled']=True
                q,ok=ik.solve(q,target,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
                if not ok:raise RuntimeError('IK failed at '+label)
                command_times.append(float(world.current_time));command(j);advance(1)
            phase_duration=float(world.current_time)-phase_start
            if abs(phase_duration-duration)>2e-6 or any(abs(b-a-1/60)>2e-6 for a,b in zip(command_times,command_times[1:])):raise RuntimeError('Motion command cadence/duration mismatch')
            report['motion_sampling']['phases'].append({'phase':label,'requested_duration_s':duration,'actual_duration_s':phase_duration,'commands':len(samples),'maximum_command_interval_s':max((b-a for a,b in zip(command_times,command_times[1:])),default=0.)})
            jaw=float(newjaw);goal=np.asarray(newgoal).copy();good=0;trace=[]
            for _ in range(240):
                command(jaw);advance(1);tip,speed,angle,spread_angle=feedback();error=float(np.linalg.norm(tip-goal)*1000)
                okay=error<=limits['endpoint_mm'] and angle<=1. and spread_angle<=1.
                good=good+1 if okay else 0;trace.append({'step':state['step'],'tip_error_mm':error,'tool_axis_error_deg':angle,'spread_axis_error_deg':spread_angle,'max_raw_joint_speed_rad_s':speed})
                if good>=limits['endpoint_samples']:break
            if good<limits['endpoint_samples']:raise RuntimeError('Measured endpoint gate failed at '+label)
            report.setdefault('endpoint_traces',{})[label]=trace[-limits['endpoint_samples']:]
            return save(label)
        def measure_held(label):
            estimates=[]
            for sample in ('a','b'):
                shot=save(label+'_'+sample)
                tip_world=np.asarray(shot['tip_robot_m'])+np.array([0,0,C.TABLE_TOP_Z])
                nominal=tip_world-np.array([*np.asarray(design['base']['handle_centre_xy'])*.001,design['base']['tcp_above_bottom_mm']*.001])
                images={};models={}
                for key in ('inspection_x','inspection_y'):
                    cam=cameras[key];rgba=cam.get_rgba();images[key]=np.ascontiguousarray(rgba[:,:,:3],dtype=np.uint8)
                    gc=UsdGeom.Camera(stage.GetPrimAtPath('/World/'+key));xf=UsdGeom.Xformable(gc.GetPrim()).ComputeLocalToWorldTransform(0)
                    models[key]={'resolution':[1920,1440],'focal_length_mm':float(gc.GetFocalLengthAttr().Get()),'horizontal_aperture_mm':float(gc.GetHorizontalApertureAttr().Get()),'vertical_aperture_mm':float(gc.GetVerticalApertureAttr().Get()),'camera_to_world':[list(row) for row in xf]}
                observation=estimate(images,models,geometry['parts']['base']['visual']['vertices_mm'],nominal)
                observation.update(label=label+'_'+sample,step=state['step'],tip_world_m=tip_world.tolist(),camera_models=models)
                R=np.asarray(observation['rotation_world']);p=np.asarray(observation['position_world_m'])
                for key in images:
                    overlay=images[key].copy();edges=front_edges(geometry['parts']['base']['visual']['vertices_mm'],nominal,models[key]);uv=project(edges@R.T+p,models[key])
                    for u,v in uv:cv2.circle(overlay,(round(u),round(v)),2,(255,60,20),-1)
                    cv2.imwrite(str(out/(label+'_'+sample+'-'+key+'-fit.png')),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))
                estimates.append(observation);report.setdefault('held_pose_observations',[]).append(observation);persist()
            delta=float(np.linalg.norm(np.array(estimates[0]['position_world_m'])-np.array(estimates[1]['position_world_m']))*1000)
            angle=float(Rotation.from_matrix(np.array(estimates[0]['rotation_world']).T@np.array(estimates[1]['rotation_world'])).magnitude()*180/np.pi)
            report.setdefault('held_pose_repeatability',[]).append({'label':label,'position_difference_mm':delta,'orientation_difference_deg':angle,'pass':delta<=.1 and angle<=.15})
            if delta>.1 or angle>.15:raise RuntimeError('Held-pose two-frame agreement failed')
            # Independent simulator score is read-only acceptance/stop evidence. It never adjusts the estimated pose or target.
            actual=poses()['base'];actual_R=__import__('payload_checks').rotation(actual['orientation_wxyz'])
            score={'label':label,'position_error_mm':float(np.linalg.norm(np.array(actual['position_world_m'])-p)*1000),'orientation_error_deg':float(Rotation.from_matrix(R.T@actual_R).magnitude()*180/np.pi),'purpose':'independent read-only stop/scoring; never target correction'}
            report.setdefault('held_pose_independent_scores',[]).append(score);persist()
            if score['position_error_mm']>.15 or score['orientation_error_deg']>.25:raise RuntimeError('RGB held-pose independent accuracy check failed')
            return estimates[-1]
        def score_placed(name,pose):
            target=np.array([*layout['assembly_body_centre'],surface+design['assembly']['part_bottom_z'][name]*.001]);pos=np.asarray(pose['position_world_m'])
            error={'xy_mm':float(np.linalg.norm(pos[:2]-target[:2])*1000),'z_mm':float(abs(pos[2]-target[2])*1000),'tilt_deg':pose['tilt_deg'],'yaw_deg':abs(pose['yaw_deg'])}
            error['pass']=error['xy_mm']<=limits['placement_xy_mm'] and error['z_mm']<=limits['placement_z_mm'] and error['tilt_deg']<=limits['placement_tilt_deg'] and error['yaw_deg']<=limits['placement_yaw_deg']
            return error
        def check_inventory(shot):
            results={name:score_placed(name,shot['poses'][name]) for name in placed}
            report.setdefault('inventory_audits',[]).append({'phase':shot['phase'],'results':results})
            if not all(v['pass'] for v in results.values()):raise RuntimeError('A previously placed part moved outside its tolerance')
        report['status']='RUNNING';persist()
        for _ in range(120):command(jaw);advance(1)
        initial=save('initial');initial_poses=initial['poses']
        if a.case=='loaded-carrier':
            placed.extend(['base','nest']);report['preplaced_fixture_parts']=['base','nest'];check_inventory(initial)
        else:
            report['preplaced_fixture_parts']=['nest'];check_inventory(initial)
        # Probe cooked PhysX shapes as well as the offline Boolean audit. This
        # catches missing small hulls and guide holes accidentally filled in.
        query=omni.physx.get_physx_scene_query_interface();probes=[]
        report['finger_surface_audit']=audit_cooked_finger_surfaces(stage,C.ROBOT_PRIM,gripper_geometry.finger_profile(),query)
        persist()
        if not report['finger_surface_audit']['source_surface_match']:raise RuntimeError('Cooked finger surface differs from source profile')
        print('FINGER_SOURCE_SURFACE_PROBES_PASS',len(report['finger_surface_audit']['probes']),flush=True)
        def probe(name,local_start,direction,distance,expect_hit,label,expected_distance_mm=None,maximum_distance_error_mm=.03):
            pose=initial_poses[name];body=np.asarray(pose['position_world_m']);w,x,y,z=pose['orientation_wxyz']
            R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            origin=body+R@np.asarray(local_start);ray=R@np.asarray(direction)
            hits=[];other_hits=[]
            def hit_callback(hit):
                value={'collision':str(hit.collision),'distance':float(hit.distance)}
                (hits if value['collision'].startswith(roots[name]+'/') else other_hits).append(value)
                return True
            query.raycast_all(tuple(float(v) for v in origin),tuple(float(v) for v in ray),float(distance),hit_callback)
            h=min(hits,key=lambda v:v['distance']) if hits else {}
            collider=str(h.get('collision',''));actual=bool(hits)
            item={'part':name,'label':label,'expected_part_hit':expect_hit,'actual_part_hit':actual,'collider':collider,'other_collider_hits_ignored_for_geometry_only':len(other_hits)}
            if expected_distance_mm is not None:
                measured=float(h.get('distance',float('nan')))*1000
                item.update(expected_distance_mm=expected_distance_mm,actual_distance_mm=measured if math.isfinite(measured) else None,maximum_distance_error_mm=maximum_distance_error_mm,distance_pass=math.isfinite(measured) and abs(measured-expected_distance_mm)<=maximum_distance_error_mm)
            probes.append(item)
            report['cooked_geometry_probes']=probes
        for item in geometry['geometry_probes']:
            if item['part'] not in physical_parts:continue
            probe(item['part'],item['start'],item['direction'],item['distance'],item['hit'],item['label'],item.get('expected_distance_mm'),item.get('maximum_distance_error_mm',.03))
        failed_probes=[v for v in probes if v['expected_part_hit']!=v['actual_part_hit'] or not v.get('distance_pass',True)]
        if failed_probes:raise RuntimeError('Cooked collision geometry probes failed: '+str(len(failed_probes))+'; first '+failed_probes[0]['part']+' '+failed_probes[0]['label'])
        report['cooked_geometry_probe_pass']=True;persist();print('COOKED_PART_GEOMETRY_PROBES_PASS',len(probes),flush=True)
        for number,name in enumerate(ORDER,1):
            state['active']=name;state['allow_grip']=False;state['allow_active_table']=True
            prefix=f'{number}_{name}'
            # Nominal fixture coordinates only, no simulator-pose correction.
            body=np.asarray(layout['pickup_body_centres'][name]);handle=np.asarray(design[name]['handle_centre_xy'])*.001
            source_xy=body+handle;dest_xy=np.asarray(layout['assembly_body_centre'])+handle
            desired_spread=np.array([0.,1.,0.]) if design['assembly']['preferred_jaw_closing_axis'][name]=='Y' else np.array([1.,0.,0.])
            # Rotate while high, then travel to a clear point above the handle.
            move(prefix+'_orient',goal.copy(),C.JAW_OPEN,1.,allow_table=True,new_spread=desired_spread)
            entry=np.array([design[name].get('side_entry_mm',0)*.001,0.])
            above=np.array([*(source_xy+entry),C.SURFACE_DZ+layout['travel_tip_height_above_table']])
            move(prefix+'_above',above,C.JAW_OPEN,2.,allow_table=True)
            grasp=np.array([*source_xy,C.SURFACE_DZ+design[name]['tcp_above_bottom_mm']*.001])
            offset_grasp=grasp+np.array([*entry,0.])
            move(prefix+'_approach',offset_grasp,C.JAW_OPEN,2.,allow_table=True)
            if np.linalg.norm(entry)>0:move(prefix+'_side_entry',grasp,C.JAW_OPEN,1.5,allow_table=True)
            state['allow_grip']=True
            close=move(prefix+'_close',grasp,commands[name]['closed_grasp_command_per_jaw_m'],3.,allow_table=True)
            lift=grasp+np.array([0.,0.,.04]);held=move(prefix+'_lift',lift,jaw,1.5,allow_table=True)
            before=np.asarray(initial_poses[name]['position_world_m']);after=np.asarray(held['poses'][name]['position_world_m']);delta=after-before
            pickup={'lift_mm':float(delta[2]*1000),'xy_slip_mm':float(np.linalg.norm(delta[:2])*1000),'tilt_deg':held['poses'][name]['tilt_deg'],'yaw_deg':abs(held['poses'][name]['yaw_deg'])}
            pickup['pass']=pickup['lift_mm']>=limits['minimum_lift_mm'] and pickup['xy_slip_mm']<=limits['maximum_pickup_xy_slip_mm'] and pickup['tilt_deg']<=limits['maximum_pickup_tilt_deg'] and pickup['yaw_deg']<=limits['maximum_pickup_yaw_deg']
            result={'part':name,'pickup':pickup,'placement_verified':False,'release_verified':False};report['part_results'].append(result);persist()
            if not pickup['pass']:raise RuntimeError(name+' pickup/slip/tilt/yaw check failed')
            if name=='carrier':
                pcb_delta=np.asarray(held['poses']['pcb']['position_world_m'])-np.asarray(initial_poses['pcb']['position_world_m'])
                result['separate_pcb_pickup_lift_mm']=float(pcb_delta[2]*1000)
                if result['separate_pcb_pickup_lift_mm']<limits['minimum_lift_mm']:raise RuntimeError('PCB did not lift with carrier')
            state['held_reference']=(after-np.asarray(held['tip_robot_m'])-np.array([0,0,C.TABLE_TOP_Z])).tolist()
            travel=lift.copy();travel[2]=C.SURFACE_DZ+layout['travel_tip_height_above_table']
            move(prefix+'_raise_for_transfer',travel,jaw,1.5)
            above_dest=np.array([*dest_xy,travel[2]])
            if name=='carrier':
                report['payload_transfer_schedule']=profile_bounds(float(np.linalg.norm(above_dest-travel)));persist()
            move(prefix+'_carry',above_dest,jaw,4. if name=='carrier' else 2.)
            if name=='base':
                observed=measure_held('1_base_held_inspection')
                actual_R,actual_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                support_origin=np.array([*layout['assembly_body_centre'],surface+design['assembly']['part_bottom_z']['base']*.001])
                planned=placement_from_held(observed['position_world_m'],observed['rotation_world'],np.asarray(actual_tip)+[0,0,C.TABLE_TOP_Z],actual_R,support_origin,geometry['parts']['base']['visual']['vertices_mm'])
                level_R=np.asarray(planned['tool_rotation_world']);local=np.asarray(planned['part_origin_in_tool_m'])
                # Level at carry height, preserving the observed part origin. No object coordinates enter this target.
                level_tip=np.asarray(observed['position_world_m'])-level_R@local-np.array([0,0,C.TABLE_TOP_Z])
                move(prefix+'_level_held',level_tip,jaw,2.,new_orientation=level_R)
                observed=measure_held('1_base_leveled_inspection')
                actual_R,actual_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                planned=placement_from_held(observed['position_world_m'],observed['rotation_world'],np.asarray(actual_tip)+[0,0,C.TABLE_TOP_Z],actual_R,support_origin,geometry['parts']['base']['visual']['vertices_mm'])
                report['measured_placement_plan']=planned;report['placement_input_contract']='RGB estimate + static camera calibration + unchanged CAD + measured arm FK + known fixture recipe only';persist()
                place=np.asarray(planned['tip_world_m'])-np.array([0,0,C.TABLE_TOP_Z])
                lower=move(prefix+'_lower',place,jaw,2.,new_support=True,new_orientation=np.asarray(planned['tool_rotation_world']))
            else:
                allowance=.10 if design[name].get('support_shoulder') else 0.
                report.setdefault('designed_placement_seating_allowance_mm',{})[name]=allowance
                place=np.array([*dest_xy,C.SURFACE_DZ+design['assembly']['part_bottom_z'][name]*.001+design[name]['tcp_above_bottom_mm']*.001+allowance*.001])
                lower=move(prefix+'_lower',place,jaw,2.,new_support=True)
            state['held_reference']=None
            state['released_reference']=lower['poses'][name]['position_world_m']
            if name=='base':
                release_plan=square_release_plan(support_origin,design['base']['handle_centre_xy'],design['base']['tcp_above_bottom_mm'],C.TOOL_AXIS,C.FINGER_SPREAD_AXIS,design['assembly']['preferred_jaw_closing_axis']['base'])
                report['staged_release_plan']=release_plan;persist()
                move(prefix+'_relax_clamp',place,release_plan['relaxed_per_jaw_m'],2.,allow_table=True)
                square=np.asarray(release_plan['tip_world_m'])-np.array([0,0,C.TABLE_TOP_Z])
                move(prefix+'_square_fingers',square,jaw,2.,allow_table=True,new_orientation=np.asarray(release_plan['tool_rotation_world']))
                place=square
            release=move(prefix+'_release',place,C.JAW_OPEN,1.,allow_table=name=='base')
            # Keep the pre-opening reference through exit/retreat; do not reset release drift.
            exit_goal=place+np.array([*entry,0.])
            if np.linalg.norm(entry)>0:move(prefix+'_side_exit',exit_goal,C.JAW_OPEN,1.5,allow_table=name=='base')
            state['allow_grip']=False
            retreat=exit_goal.copy();retreat[2]=C.SURFACE_DZ+layout['travel_tip_height_above_table']
            final=move(prefix+'_retreat',retreat,C.JAW_OPEN,2.,allow_table=name=='base')
            pos=np.asarray(final['poses'][name]['position_world_m']);released=np.asarray(lower['poses'][name]['position_world_m']);follow=float(np.linalg.norm(pos-released)*1000)
            placement=score_placed(name,final['poses'][name]);result.update(placement=placement,placement_verified=placement['pass'],release_follow_mm=follow,release_verified=follow<=limits['release_follow_mm'] and all(abs(v)<=.00025 for v in final['jaw_joints'].values()))
            if not result['placement_verified'] or not result['release_verified']:raise RuntimeError(name+' placement/release tolerance failed')
            placed.append(name)
            if name=='carrier':
                placed.append('pcb');result['separate_pcb_placement']=score_placed('pcb',final['poses']['pcb'])
                if not result['separate_pcb_placement']['pass']:raise RuntimeError('Separate PCB final placement failed')
            state['released_reference']=None;state['active']=None;state['allow_grip']=False;check_inventory(final);persist()
            print('[assembly] VERIFIED',name,json.dumps(result),flush=True)
        state['phase']='final_assembly_inspection'
        for _ in range(120):command(jaw);advance(1)
        final=save('final_assembly_inspection');check_inventory(final);fail_contacts()
        report['final_part_scores']={name:score_placed(name,final['poses'][name]) for name in placed}
        if set(placed)!=set(physical_parts) or len(report['part_results'])!=len(ORDER) or not all(r['pickup']['pass'] and r['placement_verified'] and r['release_verified'] for r in report['part_results']) or not all(s['pass'] for s in report['final_part_scores'].values()):raise RuntimeError('Incomplete assembly results')
        if not contact_count:raise RuntimeError('No contact reports received')
        writer.close();writer=None
        video=out/'full-assembly-SIMULATION-PASS.mp4';partial.rename(video)
        report['video'].update(path=str(video),duration_s=report['video']['frames']/30,bytes=video.stat().st_size,sha256=digest(video))
        report['contact_headers_seen']=contact_count;report['full_assembly_accepted']=True;report['status']='SIMULATION_PASS'
        print('TEAM_V_FULL_HELD_FIX_ASSEMBLY_PASS',flush=True)
    except Exception as e:
        report['status']='FAILED';report['error']=str(e);traceback.print_exc()
        try:
            report['failed_stop_robot_feedback']={'tip_speed_axis':feedback(),'q_actual':arm.get_joint_positions().tolist(),'q_velocity':arm.get_joint_velocities().tolist(),'q_command_arm':q.tolist(),'jaw_command_per_side_m':jaw,'goal_robot_m':goal.tolist(),'part_poses':poses()}
            report['failed_stop_robot_feedback']['tip_speed_axis']=[v.tolist() if isinstance(v,np.ndarray) else v for v in report['failed_stop_robot_feedback']['tip_speed_axis']]
            report['failed_stop_contact_meshes']={}
            report['contact_timing_diagnostics']={'purpose':'read-only pre/post-step geometry; no gate or command changes',
                'pre_step_trace':contact_pose_trace,'post_step':contact_pose_snapshot(),
                'pre_step_contact_meshes':{}}
            for item in report['unexpected_contacts']:
                for path in item['colliders']:
                    prim=stage.GetPrimAtPath(path)
                    if not prim or not prim.IsA(UsdGeom.Mesh):continue
                    m=UsdGeom.Mesh(prim);matrix=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                    report['failed_stop_contact_meshes'][path]={'world_vertices_m':[list(matrix.Transform(Gf.Vec3d(*v))) for v in m.GetPointsAttr().Get()],'face_counts':list(m.GetFaceVertexCountsAttr().Get()),'face_indices':list(m.GetFaceVertexIndicesAttr().Get())}
                    actor=next((root for root in diagnostic_roots if path.startswith(root+'/')),None)
                    if actor is not None:
                        root_after=UsdGeom.Xformable(stage.GetPrimAtPath(actor)).ComputeLocalToWorldTransform(0)
                        collider_to_actor=matrix*root_after.GetInverse()
                        root_before=Gf.Matrix4d(*[x for row in contact_pose_trace[-1]['actor_world_matrices'][actor] for x in row])
                        before_matrix=collider_to_actor*root_before
                        report['contact_timing_diagnostics']['pre_step_contact_meshes'][path]={
                            'actor':actor,'world_vertices_m':[list(before_matrix.Transform(Gf.Vec3d(*v))) for v in m.GetPointsAttr().Get()],
                            'face_counts':list(m.GetFaceVertexCountsAttr().Get()),'face_indices':list(m.GetFaceVertexIndicesAttr().Get())}

        except Exception as diagnostics_error:report['failed_stop_diagnostics_error']=repr(diagnostics_error)
        # Render the stopped state without another physics step. Preserve the
        # real failure view, with an explicitly labelled frozen tail in video.
        try:
            if 'cameras' in locals() and 'world' in locals():
                world.render()
                for camera_name,cam in cameras.items():
                    rgba=cam.get_rgba()
                    if rgba is not None and np.asarray(rgba).size:
                        cv2.imwrite(str(out/('FAILED-stop-'+camera_name+'.png')),cv2.cvtColor(np.asarray(rgba[:,:,:3],np.uint8),cv2.COLOR_RGB2BGR))
                if writer is not None:
                    rgba=cameras['overview'].get_rgba()
                    if rgba is not None and np.asarray(rgba).size:
                        rgb=np.ascontiguousarray(np.asarray(rgba[:,:,:3],np.uint8).copy());cv2.rectangle(rgb,(0,0),(1280,90),(80,15,15),-1)
                        cv2.putText(rgb,'SIMULATION STOPPED - FAILED TRIAL',(18,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2,cv2.LINE_AA)
                        cv2.putText(rgb,str(e)[:105],(18,57),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
                        cv2.putText(rgb,'Frozen failure view; no further motion',(18,80),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,230,150),1,cv2.LINE_AA)
                        for _ in range(60):writer.send(rgb);report['video']['frames']+=1
                        report['video']['frozen_failure_tail_frames']=60
        except Exception as capture_error:report['failed_stop_capture_error']=repr(capture_error)
        raise
    finally:
        report['contact_headers_seen']=locals().get('contact_count',0)
        if writer is not None:
            try:writer.close()
            except Exception as e:report['video']['encoder_close_error']=repr(e)
        if report['status']!='SIMULATION_PASS':
            partial=out/'angled-view-recording.partial.mp4'
            if partial.exists():
                failed=out/('FAILED-'+a.case+'-angled-view.mp4');partial.rename(failed);report['video']['failed_recording_path']=str(failed)
        persist();app.close()


if __name__=='__main__':main()
