"""Original lid observed placement diagnostic. No learned or real control.

Simulator poses are independent scoring/stop evidence, never target corrections.
Video stays in the remote run directory; no download operation is implemented.
"""
import argparse,hashlib,json,math,sys,traceback
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from held_pose_multistart import estimate
from held_pose_bounded import placement_from_held,project,front_edges
from lid_release_schedule import lid_release_samples,lid_release_bounds
from passive_velocity import PassiveVelocity
from lid_contact_trace import ContactTrace
from payload_contact_trace import PayloadContactTrace
from cooked_payload_evidence import read_cooked_payload
from continuous_lid_command import held_foot_points,make_command_record,IK_POSITION_TOL_M,FOOT_COMMAND_RESIDUAL_MM
from incoming_lid_stability import make_sample as incoming_sample,qualify as qualify_incoming
from lid_pose import estimate as estimate_lid, placement_from_held as place_lid_from_held
from lid_diagnostic_contract import expected_phases, expected_checkpoints, validate_foot_budget, precontact_budget, orientation_for_lid_phase
from foot_audit import all_foot_gaps, predict_from_held, error_budget_at_feet, qualify_prediction, observed_tcp_tracking_foot_errors
from release_plan import square_release_plan
from motion_schedule import motion_samples
from payload_schedule import payload_carry_samples,profile_bounds
from assembly_checks import classify_contact,contact_violation,grasp_command_m
from fixture_contacts import guide_contact
from box_contract import bounds
from full_mass_readback import query_mass
from finger_cooking import configure_source_fidelity,audit_cooked_finger_surfaces
from payload_checks import payload_score,initial_supports,placement_supports,horizontal_seating_normals,horizontal_support_pairs,initial_supports_for_case

ORDER=('base',)
COLORS=((.12,.43,.70),(.08,.58,.30),(.86,.48,.07),(.49,.64,.79))


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    global ORDER
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('starter','mesh','design','out'):ap.add_argument('--'+name,required=True)
    ap.add_argument('--case',choices=['lid-placement-diagnostic'],required=True)
    ap.add_argument('--velocity-iterations',type=int,choices=[1],required=True)
    ap.add_argument('--geometry-variant',choices=['original'],required=True)
    ap.add_argument('--stage',choices=['precontact','seat','release'],required=True)
    ap.add_argument('--foot-budget',required=True,help='Evidence-backed bounds; unknown values cannot default to zero')
    a=ap.parse_args();starter=Path(a.starter).resolve();meshpath=Path(a.mesh).resolve();designpath=Path(a.design).resolve();out=Path(a.out).resolve()
    ORDER=('cover',)
    if a.case!='lid-placement-diagnostic' or a.stage!='release':raise ValueError('V58 original-lid observational placement only')
    binding={'original':('nest-mesh.json','nest-design.json','verified-seating-mass-reference.json'),'supported-saddle':('lid-saddle-mesh.json','lid-saddle-design.json','lid-saddle-mass-reference.json')}[a.geometry_variant]
    if (meshpath.name,designpath.name)!=binding[:2]:raise ValueError('Geometry variant file binding mismatch')
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
    report={'status':'STARTING','task':'isolated_lid_supported_saddle_diagnostic','case':a.case,'vision_guided':True,'physical_hardware_test':False,
        'design_sha256':digest(designpath),'design_revision':design['revision'],'mesh_sha256':digest(meshpath),'trial_script_sha256':digest(__file__),
        'part_order':list(ORDER),'physical_parts':physical_parts,'limits':limits,'payload_limits':geometry['payload_limits'],'layout_m':layout,'collision_geometry_audit':geometry['collision_audit'],'grip_command':grip,'grip_commands':commands,
        'checkpoints':[],'part_results':[],'unexpected_contacts':[],'contact_monitor_errors':[],'contact_summary':{},
        'assumptions':{'material_density_kg_m3':1240,'solid_density_not_actual_print_mass':True,'friction':'unchanged supplied generic 0.9/0.8 and estimated high-friction fingers','all_product_parts_dynamic':True,'static_nest_fixture':True,'fixture_or_kinematic_attachment':False,'grasps_or_poses_corrected_from_truth':False},
        'payload_initial_gap_mm':geometry['payload_initial_gap_mm'],'structural_subassembly_assumption':design.get('rigid_subassembly_assumption'),'scope':'Known-pose base, loaded tray, lid assembly. Base uses RGB held-pose correction and staged release from accepted286139. Tray/lid use existing CAD recipe targets. PCB separate loose dynamic body. No trained detector, random-pose, print or physical-hardware qualification.',
        'initial_preparation':{'product_parts_preassembled_together':False,'pcb_preloaded_in_carrier':True,'only_preplaced_fixture':(['base','nest'] if a.case=='loaded-carrier' else ['nest']),'robot_placement_order':list(ORDER)},
        'video':{'download_to_laptop':False,'camera':'angled overview camera; continuous actual simulation frames','fps':30,'resolution':[1280,960],'frames':0},'print_release':False,'motion_sampling':{'command_hz':60,'physics_steps_per_command':1,'path_and_phase_durations_unchanged':False,'original_phase_durations_retained':False,'only_duration_change':{'2_carrier_carry':{'before_s':2.,'after_s':4.},'3_cover_release':{'before_s':1.,'after_s':3.}},'changes':'Verified base RGB leveling/measured target/staged release; existing tray and lid CAD paths retained','phases':[]}}
    report['held_pose_estimator_sha256']=digest(Path(__file__).with_name('held_pose_multistart.py'))
    report['base_multistart_original_estimator_sha256']=digest(Path(__file__).with_name('held_pose_bounded.py'))
    report['base_multistart_scope']='Fixed25 original-objective image-only multistart with mandatory original zero-seed image gate; cost is not an accuracy certificate. All original repeatability/accuracy and motion gates remain. Simulation precontact only.'
    report['lid_pose_estimator_sha256']=digest(Path(__file__).with_name('lid_pose.py'))
    report['diagnostic_contract_sha256']=digest(Path(__file__).with_name('lid_diagnostic_contract.py'))
    report['foot_audit_sha256']=digest(Path(__file__).with_name('foot_audit.py'))
    report['diagnostic_stage']=a.stage
    report['full_assembly_accepted']=False
    report['lid_pose_foot_budget']=validate_foot_budget(json.loads(Path(a.foot_budget).read_text()),'precontact')
    report['lid_diagnostic_scope']='PRECONTACT ONLY: original base/tray prefix; actual-CAD RGB held-lid correction, leveling and1.10mm nominal gap. Final seat/release BLOCKED. Simulation only.'
    report['scope']='Known-pose base/tray prefix followed by measured RGB lid leveling/precontact; no physical, print or full assembly acceptance.'
    report['motion_sampling']['changes']='Base/tray prefix unchanged; measured lid leveling/precontact, lid-only full3D orientation preservation. No final seating/release.'
    report['motion_sampling']['only_duration_change']['3_cover_precontact']={'before_s':2.,'after_s':4.}
    report['motion_sampling']['changes']+=' V48 only: precontact duration2s to4s, same cubic normalized path and full orientation endpoints; no grasp-reference reset or threshold change.'
    report['expected_motion_phases']=expected_phases(a.stage)
    report['expected_checkpoint_phases']=expected_checkpoints(a.stage)
    report['diagnostic_trace']=[]
    report['continuous_lid_command_helper_sha256']=digest(Path(__file__).with_name('continuous_lid_command.py'))
    report['incoming_lid_stability_helper_sha256']=digest(Path(__file__).with_name('incoming_lid_stability.py'))
    report['body_velocity_telemetry']={'source':'finite_difference_existing_USD_pose_snapshots','helper_sha256':digest(Path(__file__).with_name('passive_velocity.py')),'linear_reference':'body origin, not center of mass','angular_reference':'world log(Rnow Rprev.T)/dt','instantaneous_velocity_claim':False,'used_for_targets_or_motion_gates':False,'physical_scoring_input':False}
    report['lid_contact_trace_provenance']={'scope':'READONLY TELEMETRY ONLY; original commands/gates unchanged; failure remains failure','helper_sha256':digest(Path(__file__).with_name('lid_contact_trace.py')),'geometry_binding_file':'telemetry-finger-geometry.json','geometry_binding_sha256':digest(Path(__file__).with_name('telemetry-finger-geometry.json')),'parent_runtime_sha256':digest(Path(__file__).with_name('multistart_lid_precontact_trial.py')),'control_or_truth_steering':False,'extra_physics_queries_or_registration':False,'runtime_logging_IO_per_step':False}
    report['payload_contact_trace_provenance']={'scope':'READONLY PCB/TRAY TELEMETRY; commands and gates unchanged','helper_sha256':digest(Path(__file__).with_name('payload_contact_trace.py')),'cooked_reader_sha256':digest(Path(__file__).with_name('cooked_payload_evidence.py')),'parent_runtime_sha256':digest(Path(__file__).with_name('contact_trace_lid_precontact_trial.py')),'control_or_truth_steering':False,'runtime_logging_IO_per_step':False,'extra_physics_queries_or_registration':False,'cooked_readback_timing':'finally after original terminal outcome; no further physics or commands'}
    report['payload_contact_pose_trace']=[]
    report['candidate_provenance']='Conditional passive observer replacement; eligibility depends on independent interpretation of comparison286764. No repaired/physical/full-assembly claim.'
    report['vision_guided']=False
    report['precontact_accepted']=False
    report['seating_accepted']=False
    report['release_accepted']=False
    report['unexercised_operations']=['base robot placement','tray robot placement','lid transport','RGB pose fitting','precontact approach','seating','release']
    report['motion_sampling']['only_duration_change']={}
    report['motion_sampling']['original_phase_durations_retained']=True
    report['scope']='ISOLATED LID GRIP ONLY: original approach/close/lift plus fixed incoming hold; no base/tray robot placement, transport, RGB fitting, seating or release'
    report['diagnostic_stage']='isolated-grip'
    report['lid_diagnostic_scope']=report['scope']
    report['velocity_iteration_experiment']={'requested':a.velocity_iterations,'original_control':1,'position_iterations':32,'numerical_settings_unchanged':True}
    report['grip_geometry_experiment']={'variant':a.geometry_variant,'extension_mm':0.,'only_mechanical_feature_changed':('none; original CAD' if a.geometry_variant=='original' else 'cover oldC053/C055/C056 supported pickup saddle; originalC054neck andbody exact'),'supported_saddle_proof_sha256':digest(Path(__file__).with_name('supported-saddle-proof.json')),'supported_saddle_clearance_sha256':digest(Path(__file__).with_name('supported-saddle-clearance.json')),'fingers_materials_density_gains_timestep_preload_limits_unchanged':True,'mass_COM_inertia_follow_declared_geometry':True,'isolation_does_not_qualify_assembly':True}
    report['mesh_source_filename']=binding[0]
    report['design_source_filename']=binding[1]
    report['mass_reference_source_filename']=binding[2]
    report['expected_motion_phases']=['3_cover_orient','3_cover_above','3_cover_approach','3_cover_side_entry','3_cover_close','3_cover_lift']
    report['expected_checkpoint_phases']=['initial']+report['expected_motion_phases']+['3_cover_isolated_hold']
    report['motion_sampling']['changes']='Paired original/supported-saddle lid geometry. Original PGS32:1, fingers, TCP, preload, timing, pickup phases and all gates in both cases. Declared lid mass/COM/inertia follows new CAD. No full V53 contact-history equivalence.'
    report['initial_preparation']['robot_placement_order']=['cover pickup only; no placement']
    report['initial_preparation']['scene_fixture_scope']='All product bodies start at the original known startup coordinates; none is claimed robot-placed here'
    report['candidate_provenance']='V57 one lid-only supported-saddle hypothesis; V55 repair failed, V56 mirrors blocked before motion; original fingers/controller/PGS32:1/gates remain; no demonstrated fix or full assembly acceptance'
    def persist():(out/'report.json').write_text(json.dumps(report,separators=(',',':'))+'\n')
    report['physical_nest']={'contract_sha256':digest(Path(__file__).with_name('nest-proof.json')),'contact_checker_sha256':digest(Path(__file__).with_name('fixture_contacts.py')),'base_dynamic_throughout':True,'product_attachments':False,'no_clamp_or_preload':True}
    from placement_policy import CONTRACT, PHASES, DURATIONS
    report['task']='original_lid_observed_placement_diagnostic'
    report['placement_policy']=CONTRACT
    report['placement_policy_sha256']=digest(Path(__file__).with_name('placement_policy.py'))
    report['candidate_provenance']='V58 explicit user request to observe actual placement despite old .020mm movement advisory; no mechanical or solver repair claimed, original lid/fingers only.'
    report['grip_geometry_experiment']={'variant':'original','extension_mm':0.,'only_mechanical_feature_changed':'none; original CAD','original_physical_limits_retained':True,'old_hold_criterion_advisory':True,'mass_COM_inertia_follow_declared_geometry':True,'isolation_does_not_qualify_assembly':True}
    report['scope']='Original lid picked, RGB measured and placed/released onto prepositioned dynamic base and loaded tray. Isolated placement diagnostic; NOT full robot assembly.'
    report['lid_diagnostic_scope']=report['scope']
    report['diagnostic_stage']='observed-lid-placement'
    report['vision_guided']=True
    report['predictive_clearance_certified']=False
    report['historical_foot_budget_qualified_for_this_run']=False
    report['initial_preparation']={'product_parts_preassembled_together':True,'lid_preassembled':False,'pcb_preloaded_in_carrier':True,'preplaced_dynamic_parts':['base','carrier','pcb'],'only_static_fixture':['nest'],'robot_placement_order':['cover'],'fixtures_credited_as_robot_placed':False}
    report['expected_motion_phases']=PHASES
    report['expected_motion_durations_s']=DURATIONS
    report['expected_checkpoint_phases']=['initial']+PHASES
    report['unexercised_operations']=['robot base placement','robot loaded-tray placement','random-pose perception','physical hardware']
    report['motion_sampling']['changes']='Original pickup/controller; existing RGB held-pose leveling and continuous-command transfer/lower. Old .020mm readiness and its derived clearance budgets are measured advisories by explicit user request. Actual contacts, tracking, inventory and placement/release limits retained.'
    report['old_criterion_advisories']=[]
    report['incoming_lid_holds']=[]
    report['placement_observations']=[]
    report['support_contact_records']=[]
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
            xy=layout['pickup_body_centres']['cover'] if name=='cover' else layout['assembly_body_centre']
            z=surface+(.0003 if name=='cover' else design['assembly']['part_bottom_z'][name]*.001)
            # Startup fixture only: no body pose setter or truth target later.
            report.setdefault('fixture_startup_recipe',{})[name]={'position_world_m':[float(xy[0]),float(xy[1]),float(z)],'robot_placed':False}
            UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(float(xy[0]),float(xy[1]),z))
            for label,data,visible in [('Visual',geometry['parts'][name]['visual'],True)]+[
                    (f'C_{i:03d}_{r["role"]}',r,False) for i,r in enumerate(geometry['parts'][name]['colliders'])]:
                v=np.asarray(data['vertices_mm'],float);f=np.asarray(data['faces'],int)
                primitive=not visible and ((name=='cover' and label in [f'C_{i:03d}_ear' for i in (4,5,6,7)]) or (name=='base' and label in [f'C_{i:03d}_floor' for i in (8,9,10,11)]))
                if primitive:
                    bb=bounds(data);m=UsdGeom.Cube.Define(stage,path+'/'+label);m.CreateSizeAttr(1.)
                    xf=UsdGeom.Xformable(m.GetPrim());xf.AddTranslateOp().Set(Gf.Vec3d(*bb['centre_m']));xf.AddScaleOp().Set(Gf.Vec3f(*bb['size_m']))
                    local=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*np.array(bb['size_m'])+np.array(bb['centre_m']);ref=np.asarray(data['vertices_mm'])*.001
                    dist=np.linalg.norm(local[:,None]-ref[None,:],axis=-1);err=float(max(dist.min(0).max(),dist.min(1).max()));assert err<=1e-7
                    report.setdefault('native_primitive_audit',[]).append({'part':name,'path':str(m.GetPath()),'source_role':data['role'],'geometry_identity_error_m':err,'full_volume_retained':True,'size_m':bb['size_m'],'centre_m':bb['centre_m']})
                else:
                    m=UsdGeom.Mesh.Define(stage,path+'/'+label)
                    if visible:v=v*.001
                    else:
                        v=v*.1;UsdGeom.Xformable(m.GetPrim()).AddScaleOp().Set(Gf.Vec3f(.01,.01,.01))
                    m.CreatePointsAttr([Gf.Vec3f(*q) for q in v]);m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel().tolist());m.CreateSubdivisionSchemeAttr('none')
                if visible:m.CreateDisplayColorAttr([Gf.Vec3f(*{'nest':(.4,.4,.4),'base':(.12,.43,.70),'carrier':(.58,.65,.68),'pcb':(.08,.58,.30),'frame':(.86,.48,.07),'cover':(.49,.64,.79)}[name])])
                else:
                    UsdGeom.Imageable(m).CreateVisibilityAttr().Set('invisible');UsdPhysics.CollisionAPI.Apply(m.GetPrim())
                    if not primitive:
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
        report['native_box_contact_migration']={'parts_and_gripper_unchanged':True,'source_seating_gpu':286456,'seating_report_sha256':'58204f061b9d1c6750c489912ff426c6793582f8e5ca0b61da3373a28ff0c6cd','only_native_box_colliders':{'cover':[4,5,6,7],'base':[8,9,10,11]},'all_other_colliders_use_original_convex_cooking':True}
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
        support_pairs=initial_supports(True)|placement_supports()['base']|placement_supports()['carrier'];placed=['nest','base','carrier','pcb'];state['fixture_entry_enabled']=True;contact_count=0
        lid_contact_trace=ContactTrace(roots['cover'],[C.ROBOT_PRIM+'/JawL',C.ROBOT_PRIM+'/JawR'])
        report['lid_contact_trace']=lid_contact_trace.value
        lid_contact_trace.value['scope']='Isolated lid close/lift/fixed hold through terminal outcome; complete selected records, not a full assembly trace'
        payload_contact_trace=PayloadContactTrace(roots['pcb'],[roots['carrier']])
        report['payload_contact_trace']=payload_contact_trace.value
        # Read-only before-step evidence: never feeds commands or pass/fail gates.
        diagnostic_roots=[C.ROBOT_PRIM+'/JawL',C.ROBOT_PRIM+'/JawR']+[roots[n] for n in ('base','carrier','cover')]+[roots['pcb']]
        contact_pose_trace=[]
        issued_jaw=None;issued_jaw_targets={};lid_orientation=None
        passive_velocity=PassiveVelocity({name:roots[name] for name in ('base','carrier','pcb','cover')})
        def contact_pose_snapshot():
            snapshot={'step':state['step'],'sim_time':float(world.current_time),
                'actor_world_matrices':{path:[list(row) for row in UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)] for path in diagnostic_roots},
                'joint_positions':arm.get_joint_positions().tolist(),
                'joint_velocities':arm.get_joint_velocities().tolist(),
                'commanded_arm_joints':q.tolist(),'commanded_jaw_per_side_m':float(issued_jaw),
                'issued_jaw_joint_targets_m':dict(issued_jaw_targets)}
            snapshot['body_velocities']=passive_velocity.sample(snapshot['sim_time'],snapshot['actor_world_matrices'])
            lid_contact_trace.note_existing_snapshot(snapshot)
            payload_contact_trace.note_existing_snapshot(snapshot)
            return snapshot
        def pair(a,b):return tuple(sorted([tuple(a),tuple(b)]))
        planned_supports=placement_supports()
        def fail_contacts():
            if report['contact_monitor_errors']:raise RuntimeError('Contact monitor error: '+report['contact_monitor_errors'][0])
            if report['unexpected_contacts']:raise RuntimeError('Unintended contact: '+json.dumps(report['unexpected_contacts'][0]))
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        def on_contacts(headers,data,*unused):
            nonlocal contact_count
            telemetry_callback=lid_contact_trace.begin_callback(state['step'],state['phase'],lambda:float(world.current_time))
            payload_callback=payload_contact_trace.begin_callback(state['step'],state['phase'],lambda:float(world.current_time))
            report['support_callback_count']=report.get('support_callback_count',0)+1
            for header_ordinal,h in enumerate(headers):
                contact_count+=1
                lid_contact_trace.observe_header(telemetry_callback,h,data,lambda raw:str(PhysicsSchemaTools.intToSdfPath(raw)),collider_roles)
                payload_contact_trace.observe_header(payload_callback,h,data,lambda raw:str(PhysicsSchemaTools.intToSdfPath(raw)),collider_roles)
                try:
                    record_actors=[str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))]
                    if set(record_actors)=={roots['cover'],roots['base']}:
                        from lid_contact_trace import ordered_identity,serialize_header,selected_pair
                        identity=ordered_identity(h,lambda raw:str(PhysicsSchemaTools.intToSdfPath(raw)),collider_roles)
                        record=serialize_header(h,data,identity,selected_pair(identity['actors'],identity['colliders'],roots['cover'],[roots['base']]),report['support_callback_count'],header_ordinal,state['step'],state['phase'],float(world.current_time))
                        report['support_contact_records'].append(record)
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
            lid_contact_trace.end_callback(telemetry_callback)
            payload_contact_trace.end_callback(payload_callback)
        report['payload_contact_events']=0
        contact_sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
        report['contact_monitor_setup']={'api_and_subscription_before_reset':True,'runtime_checks_active_during_reset':True}

        physics=world.get_physics_context();cell.enable_ccd(world);physics.set_solver_type('PGS');world.reset()
        report['contact_monitor_setup']['headers_seen_during_reset']=contact_count
        fail_contacts()
        if physics.get_solver_type()!='PGS':raise RuntimeError('PGS readback failed')
        reference=json.loads(Path(__file__).with_name(binding[2]).read_text());report['installed_full_body_mass_properties']={}
        from payload_checks import rotation as mass_rotation
        for name,path in roots.items():
            if name=='nest':continue
            value=query_mass(stage,stage.GetPrimAtPath(path),report['part_masses_kg'][name],world);expected=reference[name]
            A=mass_rotation(value['principal_axes_wxyz']);B=mass_rotation(expected['principal_axes_wxyz']);ia=A@np.diag(value['diagonal_inertia_kg_m2'])@A.T;ib=B@np.diag(expected['diagonal_inertia_kg_m2'])@B.T
            report.setdefault('setup_mass_readbacks',{})[name]={'actual':value,'expected':expected};persist()
            assert abs(value['mass_kg']-expected['mass_kg'])<=1e-8 and np.linalg.norm(np.array(value['centre_of_mass_local_m'])-expected['centre_of_mass_local_m'])<=1e-7 and np.allclose(ia,ib,rtol=1e-5,atol=1e-12),'Native contact patch changed mass distribution'
            report['installed_full_body_mass_properties'][name]=value
        arm=SingleArticulation(prim_path=C.ROBOT_PRIM,name='assembly_er4ia');arm.initialize()
        arm.set_solver_velocity_iteration_count(a.velocity_iterations)
        names=list(arm.dof_names);indices={n:i for i,n in enumerate(names)};ai=[indices['J'+str(i)] for i in range(1,7)]
        report['physics']={'solver':physics.get_solver_type(),'physics_dt':float(world.get_physics_dt()),'render_dt':float(world.get_rendering_dt()),
            'position_iterations':int(arm.get_solver_position_iteration_count()),'velocity_iterations':int(arm.get_solver_velocity_iteration_count())}
        if (report['physics']['position_iterations'],report['physics']['velocity_iterations'])!=(32,a.velocity_iterations):raise RuntimeError('Isolated articulation iteration readback failed')
        if not math.isclose(report['physics']['physics_dt'],1/60,abs_tol=1e-9) or not math.isclose(report['physics']['render_dt'],1/30,abs_tol=1e-9):raise RuntimeError('Physics/render timestep readback differs')
        report['step_timing_audit']={'samples':0,'maximum_step_error_s':0.,'maximum_render_advance_s':0.,'method':'one physics-only step; separate render every two steps'}
        for cam in cameras.values():cam.initialize()
        prims={name:SingleXFormPrim(path) for name,path in roots.items()}
        # Passive estimates reuse existing pose snapshots; no rigid observer registration.
        ik.set_tool_axis(C.TOOL_AXIS);ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
        tool=tuple(np.array(C.TOOL_AXIS)*cell.tip_offset());joint_limits=np.deg2rad([C.JOINT_LIMITS_DEG['J'+str(i)] for i in range(1,7)])
        down=np.array([0.,0.,-1.]);spread=np.array([0.,1.,0.]);goal=np.array([.30,-.10,C.SURFACE_DZ+layout['travel_tip_height_above_table']]);jaw=C.JAW_OPEN
        q=np.deg2rad([C.HOME_POSE_DEG['J'+str(i)] for i in range(1,7)])
        q,ok=ik.solve(q,goal,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
        if not ok:raise RuntimeError('Initial home IK failed')
        def command(j):
            nonlocal issued_jaw,issued_jaw_targets
            issued_jaw=float(j);issued_jaw_targets={n:float(v) for n,v in cell.jaw_targets(j).items()}
            full=arm.get_joint_positions().copy();full[ai]=q
            for name,value in cell.jaw_targets(j).items():full[indices[name]]=value
            arm.apply_action(ArticulationAction(joint_positions=full))
        full=arm.get_joint_positions().copy();full[ai]=q
        for name,value in cell.jaw_targets(jaw).items():full[indices[name]]=value
        report['kinematic_binding']={'arm_joint_names':['J'+str(i) for i in range(1,7)],'arm_joint_indices':list(ai),'tool_offset_m':list(tool),'table_top_z_m':float(C.TABLE_TOP_Z),'joint_dtype':str(full.dtype)}
        arm.set_joint_positions(full)
        partial=out/'angled-view-recording.partial.mp4'
        writer=imageio_ffmpeg.write_frames(str(partial),(1280,960),fps=30,codec='libx264',quality=7,
            output_params=['-preset','fast','-threads','2','-movflags','+faststart'])
        writer.send(None)
        def advance(n=1):
            for _ in range(n):
                state['step']+=1;render=state['step']%2==0
                snapshot=contact_pose_snapshot();contact_pose_trace.append(snapshot)
                del contact_pose_trace[:-6]
                if state['active']=='cover':report['diagnostic_trace'].append(dict(snapshot,phase=state['phase']))
                lid_contact_trace.begin_step(snapshot,len(report['diagnostic_trace'])-1,state['phase'],bool(state.get('incoming_lid_capture') or state.get('isolated_contact_capture')))
                if state['active']=='carrier' or payload_contact_trace.active:
                    report['payload_contact_pose_trace'].append(dict(snapshot,phase=state['phase']))
                    payload_contact_trace.begin_step(snapshot,len(report['payload_contact_pose_trace'])-1,state['phase'],True)
                before_step=float(world.current_time);world.step(render=False)
                after_step=float(world.current_time);lid_contact_trace.end_step(state['step'],after_step)
                payload_contact_trace.end_step(state['step'],after_step)
                step_error=abs(after_step-before_step-1/60)
                timing=report['step_timing_audit'];timing['samples']+=1;timing['maximum_step_error_s']=max(timing['maximum_step_error_s'],step_error)
                if step_error>2e-6:raise RuntimeError('Physics step duration differs from 1/60 s')
                if render:
                    world.render();render_advance=abs(float(world.current_time)-after_step)
                    timing['maximum_render_advance_s']=max(timing['maximum_render_advance_s'],render_advance)
                    if render_advance>1e-9:raise RuntimeError('Rendering advanced physics unexpectedly')
                if state['active']=='cover':
                    live_R,live_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                    report['placement_observations'].append({'step':state['step'],'sim_time':after_step,'phase':state['phase'],'poses':poses(),'actual_q_rad':arm.get_joint_positions().tolist(),'actual_qdot_rad_s':arm.get_joint_velocities().tolist(),'issued_arm_q_rad':q.tolist(),'issued_jaw_targets_m':dict(issued_jaw_targets),'tip_world_m':(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),'tool_rotation_world':live_R.tolist()})
                fail_contacts()
                score=payload_score(poses()['carrier'],poses()['pcb'],design['carrier']['floor_thickness']*.001)
                audit=report.setdefault('payload_motion_audit',{'samples':0,'maximum_xy_mm':0.,'maximum_z_mm':0.,'maximum_relative_tilt_deg':0.,'maximum_relative_yaw_deg':0.})
                audit['samples']+=1
                for key,resultkey in [('xy_mm','maximum_xy_mm'),('z_mm','maximum_z_mm'),('tilt_deg','maximum_relative_tilt_deg'),('yaw_deg','maximum_relative_yaw_deg')]:audit[resultkey]=max(audit[resultkey],score[key])
                if not score['pass']:
                    report['payload_failure']={'phase':state['phase'],'step':state['step'],'score':score}
                    raise RuntimeError('Loose PCB shifted/tilted outside carrier limits during '+state['phase'])
                if state.get('incoming_lid_capture'):
                    live_R,live_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                    actual_cover=poses()['cover']
                    sample=incoming_sample(state['step'],float(world.current_time),actual_cover['position_world_m'],mass_rotation(actual_cover['orientation_wxyz']),(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),live_R.tolist(),incoming_corners,q.tolist(),issued_jaw_targets)
                    report['incoming_lid_stability']['samples'].append(sample)
                if state.get('lid_clearance_ready'):
                    ref=state['lid_fk_reference'];live_R,live_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                    current=predict_from_held(ref['estimate'],ref['tip_world_m'],ref['tool_rotation'],(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),live_R.tolist())
                    base_recipe={'position_world_m':[*layout['assembly_body_centre'],surface+design['assembly']['part_bottom_z']['base']*.001],'rotation_world':np.eye(3).tolist()}
                    gaps=all_foot_gaps(geometry,current,base_recipe)
                    Q=live_R@np.asarray(ref['tool_rotation']).T;ideal=np.asarray(ref['estimate']['rotation_world']).T
                    radius=max(np.linalg.norm(v) for f in __import__('foot_audit').support_geometry(geometry) for v in f['corners_mm'])
                    fit_norm=report['lid_pose_foot_budget']['fit_position_error_mm']+2*radius*math.sin(math.radians(report['lid_pose_foot_budget']['fit_rotation_error_deg'])/2)
                    fit_extra=fit_norm*float(np.linalg.norm(Q[2]-ideal[2]))
                    budget=precontact_budget(geometry,report['lid_pose_foot_budget'],observed_fk=True,fit_extra_mm=fit_extra)
                    result=qualify_prediction(gaps,budget)
                    audit=report.setdefault('lid_observed_fk_clearance_guard',{'samples':0,'minimum_clearance_lower_bound_mm':float('inf'),'minimum_pad_edge_lower_bound_mm':float('inf'),'maximum_fit_axis_extra_mm':0.,'tracking_semantics':'current observed FK incorporated directly; .05mm command tracking is endpoint-only, no continuous constant lag assumption'})
                    audit['samples']+=1;audit['minimum_clearance_lower_bound_mm']=min(audit['minimum_clearance_lower_bound_mm'],result['positive_clearance_lower_bound_mm']);audit['minimum_pad_edge_lower_bound_mm']=min(audit['minimum_pad_edge_lower_bound_mm'],result['pad_edge_margin_lower_bound_mm']);audit['maximum_fit_axis_extra_mm']=max(audit['maximum_fit_axis_extra_mm'],fit_extra)
                    audit['advisory_only_by_user_request']=True
                    audit['old_budget_certified']=False
                if state.get('lid_drift_reference') is not None:
                    ref=state['lid_drift_reference'];live_R,live_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
                    expected=predict_from_held(ref['actual_pose'],ref['tip_world_m'],ref['tool_rotation'],(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),live_R.tolist())
                    actual_pose=poses()['cover'];drift=0.
                    for foot in __import__('foot_audit').support_geometry(geometry):
                        for point in foot['corners_mm']:
                            actual_corner=np.asarray(actual_pose['position_world_m'])+mass_rotation(actual_pose['orientation_wxyz'])@(np.asarray(point)*.001)
                            expected_corner=np.asarray(expected['position_world_m'])+np.asarray(expected['rotation_world'])@(np.asarray(point)*.001)
                            drift=max(drift,float(np.linalg.norm(actual_corner-expected_corner)*1000))
                    audit=report.setdefault('lid_grasp_corner_drift_veto',{'samples':0,'maximum_position_error_mm':0.,'truth_used_for_targets':False})
                    audit['samples']+=1;audit['maximum_position_error_mm']=max(audit['maximum_position_error_mm'],drift)
                    audit['advisory_only_by_user_request']=True
                    audit['old_limit_mm']=.020
                    audit['old_limit_exceeded']=audit['maximum_position_error_mm']>.020
                    if not score_placed('base',poses()['base'])['pass']:raise RuntimeError('Base left original support placement envelope')
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
                    cv2.putText(rgb,'SIMULATION | original lid placement diagnostic | current fingers',(18,25),cv2.FONT_HERSHEY_SIMPLEX,.62,(245,245,245),1,cv2.LINE_AA)
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
            shot['issued_jaw_per_side_m']=float(issued_jaw);shot['issued_jaw_joint_targets_m']=dict(issued_jaw_targets)
            shot['body_velocities']=contact_pose_snapshot()['body_velocities']
            report['checkpoints'].append(shot)
            for name,cam in cameras.items():
                rgba=cam.get_rgba()
                if rgba is None or not np.asarray(rgba).size:raise RuntimeError('Missing camera '+name)
                if not cv2.imwrite(str(out/(label+'-'+name+'.png')),cv2.cvtColor(np.asarray(rgba[:,:,:3],np.uint8),cv2.COLOR_RGB2BGR)):raise RuntimeError('Camera evidence write failed')
            persist();print('[assembly]',label,flush=True);return shot
        def observed_command_fk():
            actual_q=arm.get_joint_positions()[ai].copy();actual_R,actual_tip=fk.tool_pose(actual_q,tool)
            return {'step':state['step'],'sim_time':float(world.current_time),'joint_dtype':str(actual_q.dtype),'q_rad':actual_q.tolist(),'tip_robot_m':np.asarray(actual_tip).tolist(),'rotation_world':actual_R.tolist()}
        def move(label,newgoal,newjaw,duration,allow_table=False,new_support=False,new_spread=None,new_orientation=None):
            nonlocal q,jaw,goal,spread,down
            new_orientation=orientation_for_lid_phase(label,new_orientation,new_spread,lid_orientation)
            state['phase']=label;start=goal.copy();j0=jaw;samples=(lid_release_samples(duration,1/60) if label=='3_cover_release' else payload_carry_samples(duration,1/60) if label=='2_carrier_carry' else motion_samples(duration,1/60));state['allow_active_table']=allow_table
            phase_start=float(world.current_time);command_times=[]
            if label in ('3_cover_precontact','3_cover_lower'):
                command_start_q=q.copy();command_start_R,command_start_tip=fk.tool_pose(command_start_q,tool)
                actual_q=arm.get_joint_positions()[ai].copy();actual_start_R,actual_start_tip=fk.tool_pose(actual_q,tool)
                start=np.asarray(command_start_tip).copy()
                reference=state['lid_fk_reference']
                command_foot_points=held_foot_points(reference['estimate'],reference['tip_world_m'],reference['tool_rotation'],incoming_corners)
                report.setdefault('placement_command_traces',{})
                report['placement_command_traces'][label]={'phase':label,'start_step':state['step'],'start_sim_time':phase_start,'duration_s':duration,'commands':len(samples),'local_ik_position_tol_m':IK_POSITION_TOL_M,'foot_command_residual_limit_mm':FOOT_COMMAND_RESIDUAL_MM,'start_source':'FK of last issued arm command q; never object truth','last_issued_q_rad':command_start_q.tolist(),'start_tip_robot_m':start.tolist(),'start_rotation_world':command_start_R.tolist(),'actual_start_q_rad':actual_q.tolist(),'actual_start_joint_dtype':str(actual_q.dtype),'actual_start_tip_robot_m':np.asarray(actual_start_tip).tolist(),'actual_start_rotation_world':actual_start_R.tolist(),'tool_offset_m':list(tool),'unchanged_target_tip_robot_m':np.asarray(newgoal).tolist(),'unchanged_target_rotation_world':np.asarray(new_orientation).tolist(),'held_RGB_observation':reference['estimate'],'held_actual_FK_tip_world_m':reference['tip_world_m'],'held_actual_FK_rotation_world':reference['tool_rotation'],'estimated_foot_points_tool_m':command_foot_points.tolist(),'foot_geometry_mesh_sha256':report['mesh_sha256'],'truth_used_for_command_targets_or_residual':False,'records':[]}
                persist()
            start_angle=math.atan2(spread[1],spread[0]);end=spread if new_spread is None else np.asarray(new_spread)
            delta_angle=(math.atan2(end[1],end[0])-start_angle+math.pi)%(2*math.pi)-math.pi
            active=state['active']
            orientation_path=None
            if new_orientation is not None:
                start_R,_=fk.tool_pose(command_start_q if label in ('3_cover_precontact','3_cover_lower') else arm.get_joint_positions()[ai],tool)
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
                if label in ('3_cover_precontact','3_cover_lower'):
                    record={'index':len(report['placement_command_traces'][label]['records'])+1,'next_step':state['step']+1,'sim_time_before_issue':float(world.current_time),'seed_q_rad':q.tolist(),'desired_tip_robot_m':target.tolist(),'desired_rotation_world':desired_R.tolist(),'solved_q_rad':None,'solved_tip_robot_m':None,'solved_rotation_world':None,'jaw_joint_targets_m':dict(cell.jaw_targets(j)),'status':'SOLVING','issued':False,'actual_before':observed_command_fk()}
                    record['actual_after']=dict(record['actual_before']);record['physics_advanced']=False
                    report['placement_command_traces'][label]['records'].append(record)
                    proposed_q,ok=ik.solve(q,target,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500,tol=IK_POSITION_TOL_M)
                    solved_R,solved_tip=fk.tool_pose(proposed_q,tool)
                    record.update(make_command_record(record['index'],state['step']+1,float(world.current_time),target,desired_R,proposed_q,solved_tip,solved_R,cell.jaw_targets(j),command_foot_points,ok));record['status']='SOLVED'
                    if not record['prospective_geometry_eligible']:
                        record['status']='REJECTED_BEFORE_ISSUE';persist();raise RuntimeError('Lid prospective command SE3 foot-residual veto before issue')
                    q=proposed_q
                else:q,ok=ik.solve(q,target,down,joint_limits,tool_offset=tool,spread_dir=spread,iters=500)
                if not ok:raise RuntimeError('IK failed at '+label)
                command_times.append(float(world.current_time));command(j)
                if label in ('3_cover_precontact','3_cover_lower'):record['issued']=True;record['status']='ISSUED'
                if label in ('3_cover_precontact','3_cover_lower'):
                    try:advance(1)
                    finally:record['actual_after']=observed_command_fk();record['physics_advanced']=record['actual_after']['sim_time']>record['actual_before']['sim_time']
                else:advance(1)
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
        incoming_corners=[v for foot in __import__('foot_audit').support_geometry(geometry) for v in foot['corners_mm']]
        def qualify_incoming_hold(boundary):
            audit=report['incoming_lid_stability']
            result=qualify_incoming(audit['samples'],audit['expected_arm_joints'],audit['expected_jaw_joint_targets_m'],report['lid_pose_foot_budget']['hold_drift_position_error_mm'],incoming_corners)
            audit[boundary]=result;persist()
            report['old_criterion_advisories'].append({'boundary':boundary,'step':state['step'],'measured_mm':result['maximum_pairwise_corner_diameter_mm'],'old_limit_mm':.020,'exceeded':not result['pass'],'terminates_this_diagnostic':False})
        def fixed_incoming_hold():
            if state.get('lid_drift_reference') is not None:raise RuntimeError('Incoming hold cannot reset an existing grasp reference')
            audit={'scope':'Ideal-simulation incoming readiness veto only; no physical sensor claim','source':'actual CAD foot corners transformed through independent simulation pose and measured FK; truth only veto, never targets or hold duration','expected_arm_joints':q.tolist(),'expected_jaw_joint_targets_m':dict(issued_jaw_targets),'start_step':state['step'],'start_sim_time':float(world.current_time),'requested_duration_s':2.,'requested_commands':120,'fixed_window_start_index':95,'fixed_window_samples':25,'adaptive_wait':False,'retry':False,'truth_used_for_targets':False,'samples':[]}
            report['incoming_lid_holds'].append(audit)
            report['incoming_lid_stability']=audit;state['incoming_lid_capture']=True
            for _ in range(120):command(jaw);advance(1)
            audit['end_step']=state['step'];audit['end_sim_time']=float(world.current_time);audit['actual_duration_s']=audit['end_sim_time']-audit['start_sim_time'];audit['issued_commands']=120
            if abs(audit['actual_duration_s']-2.)>2e-6:raise RuntimeError('Incoming fixed hold timing mismatch')
            qualify_incoming_hold('fixed_hold_result')
        def measure_lid(label):
            estimates=[]
            for sample in ('a','b'):
                shot=save(label+'_'+sample)
                tip_world=np.asarray(shot['tip_robot_m'])+np.array([0,0,C.TABLE_TOP_Z])
                nominal=tip_world-np.array([*np.asarray(design['cover']['handle_centre_xy'])*.001,design['cover']['tcp_above_bottom_mm']*.001])
                images={};models={}
                for key in ('inspection_x','inspection_y'):
                    cam=cameras[key];rgba=cam.get_rgba();images[key]=np.ascontiguousarray(rgba[:,:,:3],dtype=np.uint8)
                    gc=UsdGeom.Camera(stage.GetPrimAtPath('/World/'+key));xf=UsdGeom.Xformable(gc.GetPrim()).ComputeLocalToWorldTransform(0)
                    models[key]={'resolution':[1920,1440],'focal_length_mm':float(gc.GetFocalLengthAttr().Get()),'horizontal_aperture_mm':float(gc.GetHorizontalApertureAttr().Get()),'vertical_aperture_mm':float(gc.GetVerticalApertureAttr().Get()),'camera_to_world':[list(row) for row in xf]}
                value=estimate_lid(images,models,geometry['parts']['cover']['visual'],nominal)
                value.update(label=label+'_'+sample,step=state['step'],tip_world_m=tip_world.tolist(),camera_models=models)
                estimates.append(value);report.setdefault('lid_pose_observations',[]).append(value);persist()
            p=np.asarray(estimates[-1]['position_world_m']);R=np.asarray(estimates[-1]['rotation_world'])
            delta=float(np.linalg.norm(np.array(estimates[0]['position_world_m'])-p)*1000)
            angle=float(Rotation.from_matrix(np.array(estimates[0]['rotation_world']).T@R).magnitude()*180/np.pi)
            repeat={'label':label,'position_difference_mm':delta,'orientation_difference_deg':angle,'pass':delta<=.1 and angle<=.15}
            report.setdefault('lid_pose_repeatability',[]).append(repeat)
            if not repeat['pass']:raise RuntimeError('Lid RGB two-frame agreement failed')
            # Independent truth checks only veto; they never construct a target.
            actual=poses()['cover'];actual_R=mass_rotation(actual['orientation_wxyz'])
            corrected_actual={'position_world_m':(R.T@(np.asarray(actual['position_world_m'])-p)).tolist(),'rotation_world':(R.T@actual_R).tolist()}
            corrected_fitted={'position_world_m':[0,0,0],'rotation_world':np.eye(3).tolist()}
            actual_feet=all_foot_gaps(geometry,corrected_actual,{'position_world_m':[0,0,0],'rotation_world':np.eye(3).tolist()})
            fitted_feet=all_foot_gaps(geometry,corrected_fitted,{'position_world_m':[0,0,0],'rotation_world':np.eye(3).tolist()})
            foot_error=max(abs(a-b) for x,y in zip(actual_feet['feet'],fitted_feet['feet']) for a,b in zip(x['corner_gaps_mm'],y['corner_gaps_mm']))
            score={'label':label,'position_error_mm':float(np.linalg.norm(np.array(actual['position_world_m'])-p)*1000),'orientation_error_deg':float(Rotation.from_matrix(R.T@actual_R).magnitude()*180/np.pi),'maximum_foot_height_error_mm':foot_error,'foot_height_error_frame':'estimated lid leveling correction frame R_est.T; measurement only, no precision stop','purpose':'independent simulation precision measurement only; no target correction'}
            report.setdefault('lid_pose_independent_scores',[]).append(score);persist()
            score['historical_precision_advisory']={'position_limit_mm':.15,'orientation_limit_deg':.25,'foot_height_limit_mm':.05,'position_pass':score['position_error_mm']<=.15,'orientation_pass':score['orientation_error_deg']<=.25,'foot_height_pass':foot_error<=.05,'terminates_this_diagnostic':False,'truth_used_for_target_correction':False}
            live_R,live_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
            if label=='3_cover_leveled_inspection':
                if report['incoming_lid_stability'].get('reference_freeze_count',0)!=0:raise RuntimeError('Final lid grasp reference already frozen')
                qualify_incoming_hold('final_measurement_result')
                state['incoming_lid_capture']=False
                report['incoming_lid_stability']['reference_freeze_step']=state['step']
                report['incoming_lid_stability']['reference_freeze_sim_time']=float(world.current_time)
                report['incoming_lid_stability']['reference_freeze_count']=1
            state['lid_drift_reference']=({'actual_pose':actual,'tip_world_m':(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),'tool_rotation':live_R.tolist()} if label=='3_cover_leveled_inspection' else None)
            state['lid_fk_reference']={'estimate':estimates[-1],'tip_world_m':(np.asarray(live_tip)+[0,0,C.TABLE_TOP_Z]).tolist(),'tool_rotation':live_R.tolist()}
            state['lid_clearance_ready']=label=='3_cover_leveled_inspection'
            return estimates[-1]
        def lid_clearance_guard(observation,current_tip,current_R,target_tip,target_R,label):
            b=report['lid_pose_foot_budget']
            if max(b['fit_foot_height_bounds_mm'])>.05:raise RuntimeError('Lid direct empirical fit-foot bound exceeds .05mm')
            prediction=predict_from_held(observation,np.asarray(current_tip).tolist(),np.asarray(current_R).tolist(),np.asarray(target_tip).tolist(),np.asarray(target_R).tolist())
            base_recipe={'position_world_m':[*layout['assembly_body_centre'],surface+design['assembly']['part_bottom_z']['base']*.001],'rotation_world':np.eye(3).tolist()}
            gaps=all_foot_gaps(geometry,prediction,base_recipe)
            Q=np.asarray(target_R)@np.asarray(current_R).T;ideal=np.asarray(observation['rotation_world']).T
            radius=max(np.linalg.norm(v) for f in __import__('foot_audit').support_geometry(geometry) for v in f['corners_mm'])
            fit_norm=b['fit_position_error_mm']+2*radius*math.sin(math.radians(b['fit_rotation_error_deg'])/2)
            fit_extra=fit_norm*float(np.linalg.norm(Q[2]-ideal[2]))
            budget=precontact_budget(geometry,b,fit_extra_mm=fit_extra)
            result={'label':label,'prediction':prediction,'base_recipe':base_recipe,'fit_axis_extra_mm':fit_extra,'gaps':gaps,'budget':budget,'qualification':qualify_prediction(gaps,budget),'source':'RGB estimate + measured FK + static CAD recipe + explicit evidence-backed uncertainty; no truth target'}
            report.setdefault('lid_clearance_guards',[]).append(result);persist()
            result['advisory_only_by_user_request']=True
            result['predictive_certification_claim']=False
            return result
        def finish_diagnostic(label):
            nonlocal writer
            check_inventory(report['checkpoints'][-1]);fail_contacts()
            report['diagnostic_stage_accepted']=label;report['full_assembly_accepted']=False;report['isolated_lid_accepted']=False
            report['final_part_scores']={name:score_placed(name,poses()[name]) for name in placed}
            writer.close();writer=None;video=out/('DIAGNOSTIC-'+label+'-PASS.mp4');partial.rename(video)
            report['video'].update(path=str(video),duration_s=report['video']['frames']/30,bytes=video.stat().st_size,sha256=digest(video))
            report['contact_headers_seen']=contact_count;report['status']='DIAGNOSTIC_PASS';persist()
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
            report['preplaced_fixture_parts']=['nest','base','carrier','pcb'];check_inventory(initial)
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
        for name in ORDER:
            number=3
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
            state['isolated_contact_capture']=True
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
            state['phase']='3_cover_incoming_hold'
            fixed_incoming_hold()
            save('3_cover_incoming_hold')
            qualify_incoming_hold('post_camera_hold_result')
            state['incoming_lid_capture']=False
            travel=lift.copy();travel[2]=C.SURFACE_DZ+layout['travel_tip_height_above_table']
            move(prefix+'_raise_for_transfer',travel,jaw,1.5)
            above_dest=np.array([*dest_xy,travel[2]])
            move(prefix+'_carry',above_dest,jaw,2.)
            observed=measure_lid('3_cover_held_inspection')
            actual_R,actual_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
            support_origin=np.array([*layout['assembly_body_centre'],surface+design['assembly']['part_bottom_z']['cover']*.001])
            planned=place_lid_from_held(observed['position_world_m'],observed['rotation_world'],np.asarray(actual_tip)+[0,0,C.TABLE_TOP_Z],actual_R,support_origin,geometry['parts']['cover']['visual']['vertices_mm'],gap_mm=.10)
            level_R=np.asarray(planned['tool_rotation_world']);local=np.asarray(planned['part_origin_in_tool_m']);lid_orientation=level_R
            level_tip=np.asarray(observed['position_world_m'])-level_R@local-np.array([0,0,C.TABLE_TOP_Z])
            move(prefix+'_level_held',level_tip,jaw,2.,new_orientation=level_R)
            state['phase']='3_cover_leveled_hold';fixed_incoming_hold()
            observed=measure_lid('3_cover_leveled_inspection')
            actual_R,actual_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
            measured_tip=np.asarray(actual_tip)+[0,0,C.TABLE_TOP_Z]
            planned=place_lid_from_held(observed['position_world_m'],observed['rotation_world'],measured_tip,actual_R,support_origin,geometry['parts']['cover']['visual']['vertices_mm'],gap_mm=.10)
            report['lid_measured_placement_plan']=planned
            report['placement_input_contract']='RGB estimates + camera calibration + measured robot FK + static known CAD/fixture recipe; no actual object pose used for targets'
            place=np.asarray(planned['tip_world_m'])-np.array([0,0,C.TABLE_TOP_Z]);target_R=np.asarray(planned['tool_rotation_world']);lid_orientation=target_R
            precontact=place+np.array([0,0,.001])
            lid_clearance_guard(observed,measured_tip,actual_R,precontact+[0,0,C.TABLE_TOP_Z],target_R,'precontact_dispatch_advisory')
            move(prefix+'_precontact',precontact,jaw,4.,new_orientation=target_R)
            pre_R,pre_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
            lid_clearance_guard(observed,measured_tip,actual_R,np.asarray(pre_tip)+[0,0,C.TABLE_TOP_Z],pre_R,'precontact_actual_fk_advisory')
            lid_clearance_guard(observed,measured_tip,actual_R,place+[0,0,C.TABLE_TOP_Z],target_R,'final_dispatch_advisory')
            lower=move(prefix+'_lower',place,jaw,2.,new_support=True,new_orientation=target_R)
            current_R,current_tip=fk.tool_pose(arm.get_joint_positions()[ai],tool)
            lid_clearance_guard(observed,measured_tip,actual_R,np.asarray(current_tip)+[0,0,C.TABLE_TOP_Z],current_R,'pre_release_geometry_advisory')
            # A finite valid RGB target and actual tracking/contact limits remain
            # mandatory. Predictive clearance eligibility is explicitly advisory.
            report['preopening_actual_placement_score']=score_placed('cover',lower['poses']['cover'])
            if not report['preopening_actual_placement_score']['pass']:raise RuntimeError('Actual lid pre-opening alignment outside original placement tolerance')
            report['held_to_release_transition']={'step':state['step'],'sim_time':float(world.current_time),'reference_world_m':lower['poses']['cover']['position_world_m'],'phase':'before_first_open_command','reference_not_used_for_targets':True,'actual_q':arm.get_joint_positions().tolist(),'issued_arm_q':q.tolist(),'issued_jaw_targets_m':dict(issued_jaw_targets)}
            state['held_reference']=None
            state['incoming_lid_capture']=False
            state['lid_drift_reference']=None
            state['lid_clearance_ready']=False
            state['released_reference']=lower['poses']['cover']['position_world_m']
            report['lid_release_profile']=lid_release_bounds(abs(jaw-C.JAW_OPEN));persist()
            move(prefix+'_release',place,C.JAW_OPEN,3.,new_orientation=target_R)
            exit_goal=place+np.array([*entry,0.])
            move(prefix+'_side_exit',exit_goal,C.JAW_OPEN,1.5,new_orientation=target_R)
            state['allow_grip']=False
            retreat=exit_goal.copy();retreat[2]=C.SURFACE_DZ+layout['travel_tip_height_above_table']
            final=move(prefix+'_retreat',retreat,C.JAW_OPEN,2.,new_orientation=target_R)
            pos=np.asarray(final['poses']['cover']['position_world_m']);released=np.asarray(lower['poses']['cover']['position_world_m'])
            follow=float(np.linalg.norm(pos-released)*1000);placement=score_placed('cover',final['poses']['cover'])
            result.update(placement=placement,placement_verified=placement['pass'],release_follow_mm=follow,release_verified=follow<=limits['release_follow_mm'] and all(abs(v)<=.00025 for v in final['jaw_joints'].values()))
            if not result['placement_verified'] or not result['release_verified']:raise RuntimeError('Observed lid placement/release outside original tolerance')
            placed.append('cover');persist()
        state['phase']='lid_final_inspection'
        report['final_inspection_start_step']=state['step']
        for _ in range(120):command(jaw);advance(1)
        final=save('lid_final_inspection');check_inventory(final);fail_contacts()
        report['final_part_scores']={n:score_placed(n,final['poses'][n]) for n in placed}
        if len(report['part_results'])!=1 or not all(s['pass'] for s in report['final_part_scores'].values()):raise RuntimeError('Final actual inventory/placement failed')
        writer.close();writer=None
        video=out/'OBSERVED-LID-PLACEMENT-DIAGNOSTIC.mp4';partial.rename(video)
        report['video'].update(path=str(video),duration_s=report['video']['frames']/30,bytes=video.stat().st_size,sha256=digest(video))
        report['contact_headers_seen']=contact_count;report['full_assembly_accepted']=False
        report['status']='DIAGNOSTIC_PASS';report['diagnostic_stage_accepted']='observed-lid-placement'
        report['runtime_placement_completed']=True
        report['independent_support_and_outcome_audit_required']=True
        print('ORIGINAL_LID_PLACEMENT_MOTION_COMPLETED_INDEPENDENT_AUDIT_REQUIRED_NOT_FULL_ASSEMBLY',flush=True)
    except Exception as e:
        report['status']='FAILED';report['error']=str(e);traceback.print_exc()
        try:
            report['failed_stop_robot_feedback']={'tip_speed_axis':feedback(),'q_actual':arm.get_joint_positions().tolist(),'q_velocity':arm.get_joint_velocities().tolist(),'q_command_arm':q.tolist(),'jaw_command_per_side_m':issued_jaw,'issued_jaw_joint_targets_m':dict(issued_jaw_targets),'goal_robot_m':goal.tolist(),'part_poses':poses()}
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
        if 'lid_contact_trace' in locals():lid_contact_trace.finalize(report['status'],report.get('error'))
        if 'payload_contact_trace' in locals():payload_contact_trace.finalize(report['status'],report.get('error'))
        if 'stage' in locals() and 'world' in locals():
            report['payload_cooked_geometry']=read_cooked_payload(stage,world,roots,report['status'])
        report['contact_headers_seen']=locals().get('contact_count',0)
        if writer is not None:
            try:writer.close()
            except Exception as e:report['video']['encoder_close_error']=repr(e)
        if report['status'] not in ('SIMULATION_PASS','DIAGNOSTIC_PASS'):
            partial=out/'angled-view-recording.partial.mp4'
            if partial.exists():
                failed=out/('FAILED-'+a.case+'-angled-view.mp4');partial.rename(failed);report['video']['failed_recording_path']=str(failed)
        persist();app.close()


if __name__=='__main__':main()
