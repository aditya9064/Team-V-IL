"""Approved Option A: isolated known-fixture binary pickup; no assembly claim.

Targets use authored CAD/layout only. Object poses are scoring/STOP evidence.
The loose PCB remains an independent gravity body. MP4 stays on Rosie.
"""
import argparse
import hashlib
import json
import math
import signal
from pathlib import Path
import sys
import traceback
import numpy as np
from scipy.spatial.transform import Rotation
from assembly_checks import classify_contact, contact_violation
from payload_checks import initial_supports, payload_score, rotation
from post_step_guards import record_payload_then_check_contacts
from motion_schedule import motion_samples
from finger_cooking import configure_source_fidelity, audit_cooked_finger_surfaces
from full_mass_readback import query_mass
from pickup_protocol import protocol, pickup_score, SDK_PINS


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    def interrupted(signum, frame):
        raise TimeoutError('Finite experiment watchdog interrupted runtime')
    signal.signal(signal.SIGTERM, interrupted)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--starter', required=True)
    ap.add_argument('--part', choices=['base', 'carrier', 'cover'], required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    source = Path(__file__).resolve().parent
    manifest=json.loads((source/'snapshot-sha256.json').read_text())
    actual={str(f.relative_to(source)) for f in source.rglob('*') if f.is_file() and f.name!='snapshot-sha256.json'}
    if actual!=set(manifest) or any(sha(source/name)!=value for name,value in manifest.items()):
        raise ValueError('Sealed source bytes differ before simulator creation')
    starter = Path(a.starter).resolve()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    design = json.loads((source/'design.json').read_text())
    geometry = json.loads((source/'parts.json').read_text())
    if geometry['design_sha256'] != sha(source/'design.json'):
        raise ValueError('CAD identity mismatch')
    p = protocol(design, a.part)
    expected_protocol = json.loads((source/('protocol-'+a.part+'.json')).read_text())
    if p != expected_protocol:
        raise ValueError('Externally sealed protocol differs')
    limits = p['limits']
    report = {'status': 'STARTING', 'task': 'option_a_known_fixture_binary_pickup',
        'selected_part': a.part, 'protocol': p, 'protocol_sha256': sha(source/('protocol-'+a.part+'.json')),
        'source_manifest_sha256': sha(source/'snapshot-sha256.json'),
        'trial_script_sha256': sha(__file__), 'design_sha256': sha(source/'design.json'),
        'mesh_sha256': sha(source/'parts.json'), 'limits': limits,
        'payload_limits': p['payload_limits'], 'payload_floor_m': .004,
        'physical_parts': ['base', 'carrier', 'pcb', 'cover', 'nest'],
        'authored_spawn': {}, 'checkpoints': [], 'endpoint_traces': {},
        'full_motion_observations': [], 'diagnostic_trace': [],
        'contact_records': [], 'contact_summary': {}, 'unexpected_contacts': [],
        'contact_monitor_errors': [], 'contact_headers_seen': 0, 'relevant_contact_headers_seen': 0,
        'motion_sampling': {'command_hz': 60, 'physics_steps_per_command': 1, 'phases': []},
        'scope': 'Separate known-fixture mechanical pickup, 40mm lift and fixed2s hold; not placement/release/full assembly/trained perception/print/hardware/reliability.',
        'vision_guided': False, 'camera_freshness_qualified': False,
        'physical_hardware_test': False, 'full_assembly_accepted': False, 'print_release': False,
        'video': {'download_to_laptop': False, 'camera': 'recording-only angled view',
                  'fps': 30, 'resolution': [1280, 960], 'frames': 0, 'frame_ledger': []},
        'part_results': [], 'commands': []}
    def persist():
        (out/'report.json').write_text(json.dumps(report, separators=(',', ':'), allow_nan=False)+'\n')
    persist()
    sys.path.insert(0, str(starter))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    writer = None
    contact_sub = None
    world = None
    try:
        import cv2
        import imageio_ffmpeg
        import omni.usd
        import omni.physx
        from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, PhysicsSchemaTools
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera
        import cell, scene_config as C, ik, fk, gripper_geometry
        report['starter_hashes'] = {n: sha(starter/n) for n in SDK_PINS}
        if report['starter_hashes'] != SDK_PINS:
            raise RuntimeError('Original SDK source changed')
        if geometry['finger_profile_sha256'] != SDK_PINS['gripper_geometry.py']:
            raise RuntimeError('Finger profile changed')
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/30)
        stage = omni.usd.get_context().get_stage()
        cell.build(stage, with_objects=False)
        report['finger_cooking'] = configure_source_fidelity(stage)
        roots = {}
        roles = {}
        report['part_masses_kg'] = {}
        report['authored_mass_properties'] = design['mass_properties']
        colors = {'base': (.12,.43,.70), 'carrier': (.58,.65,.68),
                  'pcb': (.08,.58,.30), 'cover': (.49,.64,.79), 'nest': (.4,.4,.4)}
        for name in report['physical_parts']:
            path = '/World/Assembly_'+name
            root = UsdGeom.Xform.Define(stage, path).GetPrim()
            roots[name] = path
            xyz = p['authored_spawn'][name]['position_world_m']
            UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(*xyz))
            UsdGeom.Xformable(root).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.,Gf.Vec3d(0,0,0)))
            report['authored_spawn'][name] = dict(p['authored_spawn'][name],
                actor_world_matrix=[list(row) for row in UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0)])
            values = [('Visual', geometry['parts'][name]['visual'], True)] + [
                (f'C_{i:03d}_{v["role"]}', v, False) for i,v in enumerate(geometry['parts'][name]['colliders'])]
            for label, data, visible in values:
                vertices = np.asarray(data['vertices_mm'], float)
                faces = np.asarray(data['faces'], int)
                native = not visible and ((name == 'cover' and label in [f'C_{i:03d}_ear' for i in (4,5,6,7)]) or
                                          (name == 'base' and label in [f'C_{i:03d}_floor' for i in (8,9,10,11)]))
                if native:
                    lo,hi = vertices.min(0)*.001, vertices.max(0)*.001
                    m = UsdGeom.Cube.Define(stage, path+'/'+label)
                    m.CreateSizeAttr(1.)
                    xf = UsdGeom.Xformable(m.GetPrim())
                    xf.AddTranslateOp().Set(Gf.Vec3d(*((lo+hi)/2)))
                    xf.AddScaleOp().Set(Gf.Vec3f(*(hi-lo)))
                    corners = np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
                    distance = np.linalg.norm(corners[:,None]-vertices[None,:]*.001, axis=-1)
                    error = float(max(distance.min(0).max(), distance.min(1).max()))
                    if error > 1e-7:
                        raise RuntimeError('Native box differs from CAD')
                    report.setdefault('native_primitive_audit',[]).append({'part':name,'path':str(m.GetPath()),'geometry_identity_error_m':error})
                else:
                    m = UsdGeom.Mesh.Define(stage, path+'/'+label)
                    if visible:
                        vertices = vertices*.001
                    else:
                        vertices = vertices*.1
                        UsdGeom.Xformable(m.GetPrim()).AddScaleOp().Set(Gf.Vec3f(.01,.01,.01))
                    m.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
                    m.CreateFaceVertexCountsAttr([3]*len(faces))
                    m.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
                    m.CreateSubdivisionSchemeAttr('none')
                if visible:
                    m.CreateDisplayColorAttr([Gf.Vec3f(*colors[name])])
                else:
                    UsdGeom.Imageable(m).CreateVisibilityAttr().Set('invisible')
                    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
                    if not native:
                        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr().Set('convexHull')
                        hull = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(m.GetPrim())
                        hull.CreateHullVertexLimitAttr().Set(255)
                        hull.CreateMinThicknessAttr().Set(0.)
                    col = PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim())
                    col.CreateContactOffsetAttr().Set(.0002)
                    col.CreateRestOffsetAttr().Set(0.)
                    cell.bind_physics_material(m.GetPrim())
                    roles[str(m.GetPath())] = (name, data['role'])
            mp = design['mass_properties'][name]
            report['part_masses_kg'][name] = mp['mass_kg']
            if name != 'nest':
                cell.make_dynamic(root, mp['mass_kg'], collide_self=False)
                vals,vecs = np.linalg.eigh(np.asarray(mp['inertia_tensor_kg_m2']))
                if np.linalg.det(vecs) < 0:vecs[:,0] *= -1
                quat = Rotation.from_matrix(vecs).as_quat()
                mass = UsdPhysics.MassAPI(root)
                mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(*mp['centre_of_mass_local_m']))
                mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*vals))
                mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*quat[:3])))
        report['collider_roles'] = {k:list(v) for k,v in roles.items()}
        report['separate_body_audit'] = {}
        for name,path in roots.items():
            prim = stage.GetPrimAtPath(path)
            rb = UsdPhysics.RigidBodyAPI(prim)
            value = {'root':path,'dynamic':bool(rb.GetRigidBodyEnabledAttr().Get()) if name!='nest' else False,
                     'kinematic':bool(rb.GetKinematicEnabledAttr().Get()) if name!='nest' else False,
                     'gravity_disabled':bool(PhysxSchema.PhysxRigidBodyAPI(prim).GetDisableGravityAttr().Get()) if name!='nest' else False,
                     'fixture_static':name=='nest','mass_kg':report['part_masses_kg'][name]}
            report['separate_body_audit'][name] = value
            if name!='nest' and (not value['dynamic'] or value['kinematic'] or value['gravity_disabled']):
                raise RuntimeError('Product must be a separate loose gravity body')
        attachments=[]
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint):
                joint=UsdPhysics.Joint(prim)
                targets=list(joint.GetBody0Rel().GetTargets())+list(joint.GetBody1Rel().GetTargets())
                if any(str(t).startswith('/World/Assembly_') for t in targets):attachments.append(str(prim.GetPath()))
        report['part_attachment_joints']=attachments
        if attachments:raise RuntimeError('Product attachments forbidden')
        report['physical_collider_counts']={name:sum(v[0]==name for v in roles.values()) for name in roots}
        report['installed_physics_materials']={}
        report['installed_jaw_drives']={}
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                mat=UsdPhysics.MaterialAPI(prim)
                report['installed_physics_materials'][str(prim.GetPath())]={
                    'static_friction':mat.GetStaticFrictionAttr().Get(),
                    'dynamic_friction':mat.GetDynamicFrictionAttr().Get(),
                    'restitution':mat.GetRestitutionAttr().Get()}
            if prim.IsA(UsdPhysics.PrismaticJoint) and 'jaw' in str(prim.GetPath()).lower():
                joint=UsdPhysics.PrismaticJoint(prim);drive=UsdPhysics.DriveAPI(prim,'linear')
                report['installed_jaw_drives'][str(prim.GetPath())]={
                    'axis':joint.GetAxisAttr().Get(),'lower_limit':joint.GetLowerLimitAttr().Get(),
                    'upper_limit':joint.GetUpperLimitAttr().Get(),'stiffness':drive.GetStiffnessAttr().Get(),
                    'damping':drive.GetDampingAttr().Get(),'max_force':drive.GetMaxForceAttr().Get()}
        surface = C.TABLE_TOP_Z+C.SURFACE_DZ
        cell.camera(stage,'/World/PickupOverview',dict(eye=(.65,-.55,surface+.48),
            look_at=(.36,-.12,surface+.06),up=(0,0,1),resolution=(1280,960),focal_length=27.,horiz_aperture=20.955))
        cameras={'overview':Camera(prim_path='/World/PickupOverview',resolution=(1280,960)),
                 'table':Camera(prim_path=cell.TABLE_CAM_PATH,resolution=(640,360)),
                 'side':Camera(prim_path=cell.SIDE_CAM_PATH,resolution=(640,360)),
                 'wrist':Camera(prim_path=cell.WRIST_CAM_PATH,resolution=(640,360))}
        state={'phase':'initial_settling','step':0,'active':None,'allow_active_table':False,'allow_grip':False}
        support_pairs=initial_supports(preplaced_base=False)
        report['support_pairs']=[list(v) for v in sorted(support_pairs)]
        callback_count=0
        def stop_contacts():
            if report['contact_monitor_errors']:raise RuntimeError('Contact monitor error: '+report['contact_monitor_errors'][0])
            if report['unexpected_contacts']:raise RuntimeError('Contact violation: '+json.dumps(report['unexpected_contacts'][0]))
        def on_contacts(headers,data,*unused):
            nonlocal callback_count
            callback_count+=1
            for ordinal,h in enumerate(headers):
                report['contact_headers_seen']+=1
                try:
                    actors=[str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))]
                    colliders=[str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))]
                    mapped=[roles.get(c) for c in colliders]
                    relevant=any(r is not None for r in mapped) or any(x.startswith(C.ROBOT_PRIM) for x in actors)
                    if not relevant:continue
                    report['relevant_contact_headers_seen']+=1
                    points=[data[i] for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                    record={'callback':callback_count,'header_ordinal':ordinal,'step':state['step'],
                        'sim_time':float(world.current_time),'state':dict(state),'actors':actors,'colliders':colliders,
                        'roles':mapped,'num_contact_data':int(h.num_contact_data),'points':[{'position_world_m':[float(x) for x in v.position],
                            'normal_world':[float(x) for x in v.normal],'separation_m':float(v.separation),
                            'impulse_Ns':[float(x) for x in v.impulse]} for v in points]}
                    report['contact_records'].append(record)
                    if not points:continue
                    allowed,category=classify_contact(actors,colliders,mapped,state,support_pairs,C.ROBOT_PRIM)
                    depth=min(float(v.separation) for v in points)
                    impulse=max(float(np.linalg.norm(v.impulse)) for v in points)
                    bad=contact_violation(allowed,depth,impulse,limits['maximum_intended_penetration_mm'])
                    record.update(allowed_pair=allowed,category=category,violation=bad)
                    key=state['phase']+'|'+category+'|'+str(mapped)
                    entry=report['contact_summary'].setdefault(key,{'events':0,'minimum_separation_mm':depth*1000,'maximum_impulse_Ns':0.})
                    entry['events']+=1;entry['minimum_separation_mm']=min(entry['minimum_separation_mm'],depth*1000)
                    entry['maximum_impulse_Ns']=max(entry['maximum_impulse_Ns'],impulse)
                    if all(v is not None for v in mapped) and {v[0] for v in mapped}=={'carrier','pcb'}:
                        report['payload_contact_events']=report.get('payload_contact_events',0)+1
                    if bad and len(report['unexpected_contacts'])<50:
                        report['unexpected_contacts'].append(record)
                except Exception as error:
                    report['contact_monitor_errors'].append(repr(error))
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        contact_sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
        report['contact_monitor_setup']={'api_and_subscription_before_reset':True,'runtime_checks_active_during_reset':True}
        physics=world.get_physics_context()
        cell.enable_ccd(world)
        physics.set_solver_type('PGS')
        world.reset()
        stop_contacts()
        arm=SingleArticulation(prim_path=C.ROBOT_PRIM,name='option_a_er4ia')
        arm.initialize()
        arm.set_solver_velocity_iteration_count(1)
        indices={n:i for i,n in enumerate(arm.dof_names)}
        ai=[indices['J'+str(i)] for i in range(1,7)]
        report['physics']={'solver':physics.get_solver_type(),'physics_dt':float(world.get_physics_dt()),
            'render_dt':float(world.get_rendering_dt()),'position_iterations':int(arm.get_solver_position_iteration_count()),
            'velocity_iterations':int(arm.get_solver_velocity_iteration_count())}
        if report['physics'] != {'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':1}:
            raise RuntimeError('Original physics readback differs')
        report['installed_full_body_mass_properties']={}
        for name,path in roots.items():
            if name=='nest':continue
            value=query_mass(stage,stage.GetPrimAtPath(path),report['part_masses_kg'][name],world)
            mp=design['mass_properties'][name];R=rotation(value['principal_axes_wxyz'])
            tensor=R@np.diag(value['diagonal_inertia_kg_m2'])@R.T
            report['installed_full_body_mass_properties'][name]=value
            if (abs(value['mass_kg']-mp['mass_kg'])>1e-8 or
                np.linalg.norm(np.array(value['centre_of_mass_local_m'])-mp['centre_of_mass_local_m'])>1e-7 or
                not np.allclose(tensor,mp['inertia_tensor_kg_m2'],rtol=1e-5,atol=1e-12)):
                raise RuntimeError('CAD mass/COM/inertia readback differs')
        for cam in cameras.values():cam.initialize()
        prims={n:SingleXFormPrim(path) for n,path in roots.items()}
        ik.set_tool_axis(C.TOOL_AXIS);ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
        tool=tuple(np.array(C.TOOL_AXIS)*cell.tip_offset())
        if not np.allclose(tool,p['tool_offset_m'],rtol=0,atol=1e-12):raise RuntimeError('Source TCP changed')
        joint_limits=np.deg2rad([C.JOINT_LIMITS_DEG['J'+str(i)] for i in range(1,7)])
        down=np.array([0.,0.,-1.]);spread=np.array([0.,1.,0.]);goal=np.array(p['home_robot_m'])
        q=np.deg2rad([C.HOME_POSE_DEG['J'+str(i)] for i in range(1,7)])
        q,okay=ik.solve(q,goal,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
        if not okay:raise RuntimeError('Home IK failed')
        jaw_state='open'
        issued={}
        def command(value):
            nonlocal issued
            opening=C.JAW_OPEN if value=='open' else C.JAW_CLOSED
            issued={n:float(v) for n,v in cell.jaw_targets(opening).items()}
            if issued != p['jaw_targets'][value]:raise RuntimeError('Binary jaw source mapping differs')
            full=arm.get_joint_positions().copy();full[ai]=q
            for n,v in issued.items():full[indices[n]]=v
            arm.apply_action(ArticulationAction(joint_positions=full))
        full=arm.get_joint_positions().copy();full[ai]=q
        for n,v in p['jaw_targets']['open'].items():full[indices[n]]=v
        arm.set_joint_positions(full)
        report['kinematic_binding']={'arm_joint_names':['J'+str(i) for i in range(1,7)],
            'arm_joint_indices':ai,'joint_names':list(arm.dof_names),'tool_offset_m':list(tool),
            'table_top_z_m':float(C.TABLE_TOP_Z),'joint_dtype':str(full.dtype)}
        partial=out/'pickup.partial.mp4'
        writer=imageio_ffmpeg.write_frames(str(partial),(1280,960),fps=30,codec='libx264',quality=7,
            output_params=['-preset','fast','-threads','2','-movflags','+faststart'])
        writer.send(None)
        def poses():
            result={}
            for name,prim in prims.items():
                pos,quat=prim.get_world_pose()
                dtype={'position':str(np.asarray(pos).dtype),'orientation':str(np.asarray(quat).dtype)}
                binding=report.setdefault('pose_readback_dtypes',{})
                counts=report.setdefault('pose_readback_dtype_capture_counts',{})
                counts[name]=counts.get(name,0)+1
                errors=report.setdefault('pose_readback_dtype_errors',[])
                if name not in binding:binding[name]=dtype
                elif binding[name]!=dtype:
                    errors.append({'body':name,'step':state['step'],'declared':binding[name],'observed':dtype})
                    raise RuntimeError('SDK pose dtype changed')
                pos=np.asarray(pos,float);quat=np.asarray(quat,float);R=rotation(quat)
                if not np.isfinite(pos).all():raise RuntimeError('Invalid product feedback')
                result[name]={'position_world_m':pos.tolist(),'orientation_wxyz':quat.tolist(),
                    'tilt_deg':math.degrees(math.acos(float(np.clip(R[2,2],-1,1)))),
                    'yaw_deg':math.degrees(math.atan2(R[1,0],R[0,0]))}
            return result
        def snapshot():
            paths=[C.ROBOT_PRIM+'/JawL',C.ROBOT_PRIM+'/JawR']+list(roots.values())
            return {'step':state['step'],'sim_time':float(world.current_time),
                'actor_world_matrices':{path:[list(row) for row in UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)] for path in paths},
                'joint_positions':arm.get_joint_positions().tolist(),'joint_velocities':arm.get_joint_velocities().tolist(),
                'commanded_arm_joints':q.tolist(),'issued_jaw_joint_targets_m':dict(issued)}
        held_reference=None
        held_local_reference=None
        def advance(count=1):
            for _ in range(count):
                state['step']+=1
                before=snapshot()
                report['diagnostic_trace'].append(dict(before,phase=state['phase']))
                t0=float(world.current_time);world.step(render=False);t1=float(world.current_time)
                if abs(t1-t0-1/60)>2e-6:raise RuntimeError('Physics dt changed')
                report.setdefault('step_timing_audit',{'samples':0,'maximum_step_error_s':0.,'maximum_render_advance_s':0.})
                timing=report['step_timing_audit'];timing['samples']+=1
                timing['maximum_step_error_s']=max(timing['maximum_step_error_s'],abs(t1-t0-1/60))
                render=state['step']%2==0
                if render:
                    world.render()
                    timing['maximum_render_advance_s']=max(timing['maximum_render_advance_s'],abs(float(world.current_time)-t1))
                    if abs(float(world.current_time)-t1)>1e-9:raise RuntimeError('Render advanced physics')
                actual=arm.get_joint_positions();speed=arm.get_joint_velocities();R,tip=fk.tool_pose(actual[ai],tool)
                row={'step':state['step'],'sim_time':t1,'phase':state['phase'],'poses':poses(),
                    'actual_q_rad':actual.tolist(),'actual_qdot_rad_s':speed.tolist(),'issued_arm_q_rad':q.tolist(),
                    'issued_jaw_targets_m':dict(issued),'tip_world_m':(np.asarray(tip)+[0,0,C.TABLE_TOP_Z]).tolist(),
                    'tool_rotation_world':R.tolist()}
                report['full_motion_observations'].append(row)
                report['full_motion_last_snapshot']=snapshot()
                payload=record_payload_then_check_contacts(report,
                    lambda:payload_score(row['poses']['carrier'],row['poses']['pcb'],.004),stop_contacts)
                row['payload_score']=payload
                if not payload['pass']:raise RuntimeError('Loose PCB guard failed')
                if held_reference is not None:
                    pose=row['poses'][a.part]
                    relative=np.array(pose['position_world_m'])-row['tip_world_m']
                    drift=float(np.linalg.norm(relative-held_reference)*1000)
                    score={'step':state['step'],'phase':state['phase'],'relative_slip_mm':drift,
                           'tilt_deg':pose['tilt_deg'],'yaw_deg':abs(pose['yaw_deg']),
                           'diagnostic_tool_local_slip_mm':float(np.linalg.norm(np.asarray(row['tool_rotation_world']).T@relative-held_local_reference)*1000)}
                    report.setdefault('held_trace',[]).append(score)
                    if drift>2 or score['tilt_deg']>5 or score['yaw_deg']>3:
                        raise RuntimeError('Held part slip/tilt/yaw limit exceeded')
                    if state['phase']==p['hold_phase'] and (pose['position_world_m'][2]-report['initial_poses'][a.part]['position_world_m'][2])*1000<35:
                        raise RuntimeError('Held part dropped below minimum lift')
                if render:
                    rgba=cameras['overview'].get_rgba()
                    if rgba is not None and np.asarray(rgba).size:
                        rgb=np.ascontiguousarray(np.asarray(rgba[:,:,:3],np.uint8).copy())
                        if rgb.shape!=(960,1280,3):raise RuntimeError('Recording dimensions differ')
                        cv2.rectangle(rgb,(0,0),(1280,64),(20,28,38),-1)
                        cv2.putText(rgb,'OPTION A | '+a.part+' | unchanged fingers | known fixture',(18,25),cv2.FONT_HERSHEY_SIMPLEX,.62,(245,245,245),1,cv2.LINE_AA)
                        cv2.putText(rgb,state['phase'],(18,51),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,210,110),1,cv2.LINE_AA)
                        writer.send(rgb);report['video']['frames']+=1
                        report['video']['frame_ledger'].append({'frame_index':report['video']['frames'],
                            'step':state['step'],'sim_time':float(world.current_time),
                            'phase':state['phase'],'frozen_tail':False})
                    elif report['video']['frames']:raise RuntimeError('Recording camera disappeared')
        def feedback():
            actual=arm.get_joint_positions()[ai];speed=arm.get_joint_velocities()[ai]
            R,tip=fk.tool_pose(actual,tool)
            if not np.isfinite([actual,speed]).all() or not np.isfinite(tip).all():raise RuntimeError('Nonfinite arm feedback')
            axis=math.degrees(math.acos(float(np.clip(np.dot(R@C.TOOL_AXIS,down),-1,1))))
            cross=math.degrees(math.acos(float(np.clip(np.dot(R@C.FINGER_SPREAD_AXIS,spread),-1,1))))
            return np.asarray(tip),float(np.max(np.abs(speed))),axis,cross
        def capture(label):
            trace=[];good=0
            for _ in range(180):
                command(jaw_state);advance(2);tip,speed,axis,cross=feedback()
                good=good+1 if speed<.02 else 0
                trace.append({'step':state['step'],'sim_time':float(world.current_time),
                              'maximum_raw_joint_speed_rad_s':speed,'tip_robot_m':tip.tolist()})
                if good>=6:break
            if good<6:raise RuntimeError('Six-sample raw stillness gate failed')
            shot={'phase':label,'step':state['step'],'sim_time':float(world.current_time),'poses':poses(),
                  'stillness_trace':trace[-6:],'tip_robot_m':tip.tolist(),'tool_axis_error_deg':axis,
                  'spread_axis_error_deg':cross,'issued_jaw_joint_targets_m':dict(issued),'images':{}}
            for key,cam in cameras.items():
                rgba=cam.get_rgba()
                if rgba is None or not np.asarray(rgba).size:raise RuntimeError('Missing camera: '+key)
                filename=label+'-'+key+'.png'
                if not cv2.imwrite(str(out/filename),cv2.cvtColor(np.asarray(rgba[:,:,:3],np.uint8),cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('Image save failed')
                shot['images'][key]={'path':filename,'sha256':sha(out/filename),'buffer_read_sim_time':float(world.current_time),
                                     'native_pixel_age_certified':False}
            report['checkpoints'].append(shot);persist();print('PICKUP_CHECKPOINT',a.part,label,flush=True)
            return shot
        command('open');advance(120)
        initial=capture('initial')
        report['initial_poses']=initial['poses']
        query=omni.physx.get_physx_scene_query_interface()
        report['finger_surface_audit']=audit_cooked_finger_surfaces(stage,C.ROBOT_PRIM,gripper_geometry.finger_profile(),query)
        if not report['finger_surface_audit']['source_surface_match']:raise RuntimeError('Finger source geometry probe failed')
        report['cooked_geometry_probes']=[]
        for test in json.loads((source/'geometry-probes.json').read_text()):
            name=test['part'];pose=initial['poses'][name];R=rotation(pose['orientation_wxyz'])
            origin=np.array(pose['position_world_m'])+R@np.array(test['start_m'])
            direction=R@np.array(test['direction']);hits=[]
            def callback(hit):
                if str(hit.collision).startswith(roots[name]+'/'):
                    hits.append({'distance_m':float(hit.distance),'collider':str(hit.collision)})
                return True
            query.raycast_all(tuple(origin),tuple(direction),test['maximum_distance_m'],callback)
            hit=min(hits,key=lambda x:x['distance_m']) if hits else None
            passed=bool(hit)==test['hit']
            if hit is not None and test['hit']:
                passed=passed and abs(hit['distance_m']*1000-test['expected_distance_mm'])<=.03
            report['cooked_geometry_probes'].append(dict(test,actual=hit,passed=passed))
        if not all(v['passed'] for v in report['cooked_geometry_probes']):raise RuntimeError('Cooked part geometry probe failed')
        report['cooked_geometry_probe_pass']=True
        persist()
        def endpoint(label):
            good=0;trace=[]
            for _ in range(240):
                command(jaw_state);advance();tip,speed,axis,cross=feedback()
                error=float(np.linalg.norm(tip-goal)*1000)
                good=good+1 if error<=2 and axis<=1 and cross<=1 else 0
                trace.append({'step':state['step'],'tip_error_mm':error,'tool_axis_error_deg':axis,
                              'spread_axis_error_deg':cross,'max_raw_joint_speed_rad_s':speed})
                if good>=6:break
            report['endpoint_traces'][label]=trace[-6:]
            if good<6:raise RuntimeError('Measured endpoint gate failed')
        for phase in p['phases']:
            state['phase']=phase['name'];state['active']=a.part
            state['allow_active_table']=True
            state['allow_grip']=phase['role'] in ('close','lift')
            target=np.array(phase['target_robot_m'])
            jaw_state=phase['jaw_state']
            start=goal.copy();times=[]
            for elapsed,s in motion_samples(phase['duration_s'],1/60):
                wanted=start+(target-start)*s
                if phase['role']=='lift' and elapsed/phase['duration_s']>.2:state['allow_active_table']=False
                q,okay=ik.solve(q,wanted,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
                if not okay:raise RuntimeError('IK failed in '+state['phase'])
                command(jaw_state);times.append(float(world.current_time))
                report['commands'].append({'next_step':state['step']+1,'phase':state['phase'],
                    'time':float(world.current_time),'desired_tip_robot_m':wanted.tolist(),
                    'issued_arm_q_rad':q.tolist(),'issued_jaw_targets_m':dict(issued)})
                advance()
            goal=target.copy()
            report['motion_sampling']['phases'].append({'phase':phase['name'],'requested_duration_s':phase['duration_s'],
                'actual_duration_s':float(world.current_time)-times[0], 'commands':len(times),
                'maximum_command_interval_s':max((b-a for a,b in zip(times,times[1:])),default=0.)})
            endpoint(phase['name'])
            shot=capture(phase['name'])
            if phase['role']=='close':
                reference=report['full_motion_observations'][-1]
                held_reference=np.array(reference['poses'][a.part]['position_world_m'])-reference['tip_world_m']
                held_local_reference=np.asarray(reference['tool_rotation_world']).T@held_reference
                report['lift_held_reference']=reference
            if phase['role']=='lift':
                score=pickup_score(initial['poses'][a.part],shot['poses'][a.part])
                if a.part=='carrier':score['pcb_lift_mm']=(shot['poses']['pcb']['position_world_m'][2]-initial['poses']['pcb']['position_world_m'][2])*1000
                report['pickup_score']=score
                if not score['pass'] or (a.part=='carrier' and score['pcb_lift_mm']<35):raise RuntimeError('Independent pickup/PCB lift gate failed')
        reference=report['full_motion_observations'][-1]
        held_reference=np.array(reference['poses'][a.part]['position_world_m'])-reference['tip_world_m']
        held_local_reference=np.asarray(reference['tool_rotation_world']).T@held_reference
        report['hold_reference']=reference
        state['phase']=p['hold_phase'];state['allow_active_table']=False
        hold_start=state['step'];time_start=float(world.current_time)
        for _ in range(120):command('closed');advance()
        report['pickup_hold']={'start_step':hold_start,'end_step':state['step'],'requested_steps':120,
            'requested_duration_s':2.,'start_sim_time':time_start,'end_sim_time':float(world.current_time),
            'actual_duration_s':float(world.current_time)-time_start}
        # Inspection adds separately labelled samples; the fixed hold remains exactly120steps.
        final=capture(a.part+'_hold_end')
        report['part_results']=[{'part':a.part,'pickup_verified':True,'hold_verified':True,
                                'placement_verified':False,'release_verified':False}]
        report['status']='DIAGNOSTIC_PASS'
        stop_contacts()
    except Exception as error:
        report['status']='FAILED';report['error']=str(error);traceback.print_exc()
        try:
            report['full_motion_last_snapshot']=snapshot()
            report['terminal_poses']=poses()
        except Exception as diagnostic_error:report['terminal_snapshot_error']=repr(diagnostic_error)
        try:
            if world is not None:
                world.render()
                for name,cam in cameras.items():
                    rgba=cam.get_rgba()
                    if rgba is not None and np.asarray(rgba).size:
                        cv2.imwrite(str(out/('FAILED-stop-'+name+'.png')),cv2.cvtColor(np.asarray(rgba[:,:,:3],np.uint8),cv2.COLOR_RGB2BGR))
                if writer is not None:
                    rgba=cameras['overview'].get_rgba()
                    if rgba is not None and np.asarray(rgba).size:
                        rgb=np.ascontiguousarray(np.asarray(rgba[:,:,:3],np.uint8).copy())
                        cv2.rectangle(rgb,(0,0),(1280,100),(80,15,15),-1)
                        cv2.putText(rgb,'FAILED - FROZEN TAIL / no extra physics',(18,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
                        cv2.putText(rgb,str(error)[:105],(18,60),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
                        for _ in range(30):
                            writer.send(rgb);report['video']['frames']+=1
                            report['video']['frame_ledger'].append({'frame_index':report['video']['frames'],
                                'step':state['step'],'sim_time':float(world.current_time),
                                'phase':state['phase'],'frozen_tail':True})
                        report['video']['frozen_tail_frames']=30
        except Exception as video_error:report['failure_video_error']=repr(video_error)
    finally:
        if writer is not None:
            try:
                writer.close()
                video=out/(('FAILED-' if report['status']=='FAILED' else 'RECORDED-')+a.part+'-pickup.mp4')
                partial.rename(video)
                report['video'].update(path=str(video),sha256=sha(video),bytes=video.stat().st_size,
                                       duration_s=report['video']['frames']/30)
            except Exception as finalization_error:
                report['video_finalization_error']=repr(finalization_error)
                report['status']='FAILED'
                report.setdefault('error','Video evidence finalization failed')
        report['contact_callback_count']=locals().get('callback_count',0)
        persist()
        contact_sub=None
        app.close()
    return 1 if report['status']=='FAILED' else 0


if __name__=='__main__':
    raise SystemExit(main())
