"""CPU audit tests; synthetic fixtures never claim simulator/assembly success."""
import ast,copy,hashlib,json,sys,unittest,tempfile,shutil
from pathlib import Path
from types import SimpleNamespace
import numpy as np
HERE=Path(__file__).resolve().parent
SOURCE=HERE if (HERE/'isolated_lid_grip_trial.py').is_file() else Path('/private/tmp/team-v-lid-stem-v55-20261022/snapshot')
sys.path.insert(0,str(SOURCE));sys.path.insert(0,str(HERE))
import check_paired_lid_grip as C
from incoming_lid_stability import qualify,make_sample
from foot_audit import support_geometry
from continuous_lid_command import independent_fk
from passive_velocity import PassiveVelocity
from lid_contact_trace import ContactTrace
from paired_trace_integrity import validate_trace
G=json.loads((SOURCE/'nest-mesh.json').read_text());DESIGN=json.loads((SOURCE/'nest-design.json').read_text())
FIXTURE=SOURCE/'isolation-fixtures/v53-readiness.json'
if not FIXTURE.exists():FIXTURE=HERE/'isolation-fixtures/v53-readiness.json'
D=json.loads(FIXTURE.read_text())
CORNERS=[v for f in support_geometry(G) for v in f['corners_mm']]

def matrix(p=(0,0,0),R=np.eye(3)):
 m=np.eye(4);m[:3,:3]=np.asarray(R).T;m[3,:3]=p;return m.tolist()

def incoming_fixture(count=132,late_drift=False):
 start=1000;t=20.;q=D['post_step']['joint_positions'];F,p=independent_fk(q[:6],[.293,0,0],'float32');tip=(np.asarray(p)+[0,0,.9]).tolist()
 expected=D['incoming']['expected_arm_joints'];jaws=D['incoming']['expected_jaw_joint_targets_m'];basepose=[.4,-.16,.9];cover=[.43,.04,.94]
 samples=[];snapshots=[]
 for i in range(count):
  pos=np.array(cover);pos[0]+=.00004 if late_drift and i==count-1 else 0
  sample=make_sample(start+i+1,t+(i+1)/60,pos,np.eye(3),tip,F,CORNERS,expected,jaws);samples.append(sample)
  mats={'/World/Assembly_'+n:matrix(basepose) for n in ['base','carrier']};mats['/World/Assembly_pcb']=matrix(np.array(basepose)+[0,0,.002]);mats['/World/Assembly_cover']=matrix(pos)
  mats.update({'/World/ER4iA/'+n:matrix() for n in ['JawL','JawR']})
  snapshots.append({'step':start+i+2,'sim_time':sample['sim_time'],'actor_world_matrices':mats,'joint_positions':list(q),'phase':'3_cover_isolated_hold'})
 a={'requested_duration_s':2.,'requested_commands':120,'issued_commands':120,'start_step':start,'end_step':start+120,'start_sim_time':t,'end_sim_time':t+2.,'actual_duration_s':2.,'fixed_window_start_index':95,'fixed_window_samples':25,'adaptive_wait':False,'retry':False,'truth_used_for_targets':False,'expected_arm_joints':expected,'expected_jaw_joint_targets_m':jaws,'samples':samples}
 a['fixed_hold_result']=qualify(samples[:120],expected,jaws,.02,CORNERS)
 if count>120:a['post_camera_hold_result']=qualify(samples,expected,jaws,.02,CORNERS)
 return {'incoming_lid_stability':a,'diagnostic_trace':snapshots,'lid_contact_trace':{'last_existing_snapshot':snapshots[-1]}}

def raw_fixture():
 root='/World/Assembly_cover';jaw='/World/ER4iA/JawL';tr=ContactTrace(root,[jaw,'/World/ER4iA/JawR'])
 mats={p:matrix() for p in [root,jaw,'/World/ER4iA/JawR']}
 pre={'step':1,'sim_time':0.,'phase':'3_cover_close','actor_world_matrices':mats};post=dict(copy.deepcopy(pre),sim_time=1/60)
 tr.begin_step(pre,0,'3_cover_close',True);token=tr.begin_callback(1,'3_cover_close',1/60)
 ids={1:root,2:jaw,3:root+'/C_054_grip_stem',4:jaw+'/Part02'}
 h=SimpleNamespace(actor0=1,actor1=2,collider0=3,collider1=4,num_contact_data=1,contact_data_offset=0,type=2)
 p=SimpleNamespace(position=[0,0,0],normal=[0,1,0],impulse=[0,.001,0],separation=-.00005)
 tr.observe_header(token,h,[p],ids.__getitem__,{ids[3]:('cover','grip_stem')})
 # Include an ordered zero-point LOST event; it must survive accounting.
 h.num_contact_data=0;h.type=1;tr.observe_header(token,h,[],ids.__getitem__,{ids[3]:('cover','grip_stem')})
 tr.end_callback(token);tr.end_step(1,1/60);tr.note_existing_snapshot(post);tr.finalize('FAILED','synthetic unrelated gate failure')
 return {'diagnostic_stage':'isolated-grip','full_assembly_accepted':False,'physical_hardware_test':False,'print_release':False,'status':'FAILED','error':'synthetic unrelated gate failure','lid_contact_trace_provenance':{'control_or_truth_steering':False,'extra_physics_queries_or_registration':False,'runtime_logging_IO_per_step':False},'lid_contact_trace':tr.value,'diagnostic_trace':[pre],'contact_timing_diagnostics':{'post_step':post}}

class ReadinessTests(unittest.TestCase):
 def test_actual_archived_failure_remains_failure(self):
  r={'incoming_lid_stability':D['incoming'],'diagnostic_trace':D['pose_trace_level_and_hold'],'lid_contact_trace':{'last_existing_snapshot':D['post_step']}}
  got=C.check_incoming(r,G,SOURCE);self.assertAlmostEqual(got['fixed_maximum_mm'],.2131325447876711,places=8);self.assertTrue(all(x>.02 for x in got['actual_jaw_diameters_mm'].values()))
 def test_synthetic_stable_is_local_only(self):
  got=C.check_incoming(incoming_fixture(),G,SOURCE);self.assertLess(got['through_terminal_maximum_mm'],1e-8)
 def test_late_outlier_does_not_roll_away(self):
  r=incoming_fixture(late_drift=True);got=C.check_incoming(r,G,SOURCE);self.assertLess(got['fixed_maximum_mm'],.02);self.assertGreater(got['through_terminal_maximum_mm'],.02)
 def test_forged_late_pass_rejected(self):
  r=incoming_fixture(late_drift=True);r['incoming_lid_stability']['post_camera_hold_result']['pass']=True
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)
 def test_corner_corruption_rejected(self):
  r=incoming_fixture();r['incoming_lid_stability']['samples'][110]['corners_tool_mm'][0][0]+=.1
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)
 def test_missing_sample_rejected(self):
  r=incoming_fixture();del r['incoming_lid_stability']['samples'][100]
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)
 def test_changed_command_rejected(self):
  r=incoming_fixture();r['incoming_lid_stability']['samples'][110]['commanded_arm_joints']=list(r['incoming_lid_stability']['expected_arm_joints']);r['incoming_lid_stability']['samples'][110]['commanded_arm_joints'][0]+=.01
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)
 def test_reset_window_rejected(self):
  r=incoming_fixture();r['incoming_lid_stability']['fixed_window_start_index']=107
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)
 def test_body_snapshot_missing_rejected(self):
  r=incoming_fixture();del r['diagnostic_trace'][110]['actor_world_matrices']['/World/Assembly_carrier']
  with self.assertRaises((C.AuditError,KeyError)):C.check_incoming(r,G,SOURCE)
 def test_fk_sample_forgery_rejected(self):
  r=incoming_fixture();s=r['incoming_lid_stability']['samples'][110];s['tip_world_m'][0]+=.001
  s['corners_tool_mm']=(((np.array(CORNERS)*.001)@np.array(s['part_rotation_world']).T+np.array(s['part_position_world_m'])-s['tip_world_m'])@np.array(s['tool_rotation_world'])*1000).tolist()
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)
 def test_pcb_shift_rejected(self):
  r=incoming_fixture();r['diagnostic_trace'][110]['actor_world_matrices']['/World/Assembly_pcb'][3][0]+=.001
  with self.assertRaises(C.AuditError):C.check_incoming(r,G,SOURCE)

class TraceTests(unittest.TestCase):
 def test_failed_trace_valid_never_promoted(self):
  r=raw_fixture();v=validate_trace(r);self.assertTrue(v['telemetry_integrity_valid']);self.assertEqual(v['original_outcome'],'FAILED');self.assertFalse(v['mechanical_success_checked_here']);self.assertEqual(v['selected_headers'],2);self.assertEqual(v['selected_points'],1)
 def test_missing_zero_point_event_rejected(self):
  r=raw_fixture();r['lid_contact_trace']['rows'][0]['headers'].pop()
  with self.assertRaises(AssertionError):validate_trace(r)
 def test_contact_point_corruption_rejected(self):
  r=raw_fixture();r['lid_contact_trace']['rows'][0]['headers'][0]['points'][0]['separation_mm']=-.1
  with self.assertRaises(AssertionError):validate_trace(r)
 def test_callback_sequence_corruption_rejected(self):
  r=raw_fixture();r['lid_contact_trace']['rows'][0]['callbacks'][0]['sequence']=3
  with self.assertRaises(AssertionError):validate_trace(r)
 def test_unclosed_bracket_rejected(self):
  r=raw_fixture();r['lid_contact_trace']['rows'][0]['closed']=False
  with self.assertRaises(AssertionError):validate_trace(r)
 def test_wrong_pose_link_rejected(self):
  r=raw_fixture();r['lid_contact_trace']['rows'][0]['pre_step_snapshot_link']['step']=2
  with self.assertRaises(AssertionError):validate_trace(r)

def motion_fixture():
 def poses(lift):
  out={n:{'position_world_m':[0.,0.,0.], 'orientation_wxyz':[1.,0.,0.,0.]} for n in ['base','carrier','pcb','cover','nest']}
  out['pcb']['position_world_m'][2]=DESIGN['carrier']['floor_thickness']*.001;out['cover']['position_world_m'][2]=.04 if lift else 0.
  return out
 shots=[]
 for i,name in enumerate(C.CHECKPOINTS):
  shots.append({'phase':name,'poses':poses(i>=6),'stillness_trace':[{'step':i*20+j+1,'sim_time':(i*20+j+1)/60,'maximum_raw_joint_speed_rad_s':.001} for j in range(6)]})
 return {'motion_sampling':{'phases':[{'phase':n,'requested_duration_s':t,'actual_duration_s':t,'commands':round(t*60),'maximum_command_interval_s':1/60} for n,t in zip(C.PHASES,C.DURATIONS)],'command_hz':60,'physics_steps_per_command':1},'endpoint_traces':{n:[{'tip_error_mm':0.,'tool_axis_error_deg':0.,'spread_axis_error_deg':0.} for _ in range(6)] for n in C.PHASES},'checkpoints':shots,'part_results':[{'part':'cover','placement_verified':False,'release_verified':False,'pickup':{'pass':True,'lift_mm':40.,'xy_slip_mm':0.,'tilt_deg':0.,'yaw_deg':0.}}],'held_motion_audits':{'cover':{'samples':132,'maximum_relative_slip_mm':0.,'maximum_tilt_deg':0.,'maximum_yaw_deg':0.}}}

def identity_fixture():
 geometry=json.loads((SOURCE/'telemetry-finger-geometry.json').read_text());starter=dict(geometry['SDK_source_sha256']);starter.update({'fk.py':'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed','ik.py':'6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795'})
 r={'trial_script_sha256':C.digest(SOURCE/'isolated_lid_grip_trial.py'),'limits':copy.deepcopy(G['limits']),'lid_pose_foot_budget':{'hold_drift_position_error_mm':.02},'payload_limits':G['payload_limits'],'layout_m':G['layout_m'],'physics':{'solver':'PGS','physics_dt':1/60,'render_dt':1/30,'position_iterations':32,'velocity_iterations':1},'velocity_iteration_experiment':{'requested':1,'numerical_settings_unchanged':True},'grip_geometry_experiment':{'variant':'original','extension_mm':0.,'fingers_materials_density_gains_timestep_preload_limits_unchanged':True,'mass_COM_inertia_follow_declared_geometry':True,'isolation_does_not_qualify_assembly':True},'mesh_source_filename':'nest-mesh.json','design_source_filename':'nest-design.json','mass_reference_source_filename':'verified-seating-mass-reference.json','status':'DIAGNOSTIC_PASS','task':'isolated_lid_grip_geometry_diagnostic','case':'isolated-lid-grip','diagnostic_stage':'isolated-grip','diagnostic_stage_accepted':'isolated-grip','full_assembly_accepted':False,'isolated_lid_accepted':False,'precontact_accepted':False,'seating_accepted':False,'release_accepted':False,'vision_guided':False,'physical_hardware_test':False,'print_release':False,'starter_hashes':starter,'kinematic_binding':{'arm_joint_names':['J'+str(i) for i in range(1,7)],'arm_joint_indices':list(range(6)),'tool_offset_m':[.293,0.,0.],'table_top_z_m':.9,'joint_dtype':'float32'},'part_order':['cover'],'physical_parts':['base','carrier','pcb','cover','nest'],'expected_motion_phases':C.PHASES,'expected_checkpoint_phases':C.CHECKPOINTS}
 for k,n in [('mesh_sha256','nest-mesh.json'),('design_sha256','nest-design.json'),('incoming_lid_stability_helper_sha256','incoming_lid_stability.py'),('foot_audit_sha256','foot_audit.py'),('finger_cooking_script_sha256','finger_cooking.py')]:r[k]=C.digest(SOURCE/n)
 return r

def physics_fixture():
 reference=json.loads((SOURCE/'verified-seating-mass-reference.json').read_text())
 r={'payload_initial_gap_mm':0.,'part_attachment_joints':[],'separate_body_audit':{},'physical_collider_counts':{},'installed_full_body_mass_properties':copy.deepcopy(reference),'grip_commands':G['grip_commands'],'contact_monitor_setup':{'api_and_subscription_before_reset':True,'runtime_checks_active_during_reset':True},'contact_headers_seen':1,'unexpected_contacts':[],'contact_monitor_errors':[],'contact_summary':{'synthetic grip':{'minimum_separation_mm':-.05,'maximum_impulse_Ns':.001,'allowed_pair':True}},'finger_cooking':{'mode':'source_fidelity_cooking','collider_count':138,'source_profile_dimensions_unchanged':True,'min_thickness_m':0.,'hull_vertex_limit':255,'maximum_world_source_vertex_change_m':0.},'finger_surface_audit':{'source_surface_match':True,'probes':[{'same_jaw_hit':True,'inward_error_mm':0.} for _ in range(12)],'maximum_allowed_absolute_error_mm':.03},'cooked_geometry_probe_pass':True,'cooked_geometry_probes':[],'step_timing_audit':{'samples':10,'maximum_step_error_s':0.,'maximum_render_advance_s':0.},'payload_motion_audit':{'samples':10,'maximum_xy_mm':0.,'maximum_z_mm':0.,'maximum_relative_tilt_deg':0.,'maximum_relative_yaw_deg':0.}}
 for name in C.BODY_NAMES+['nest']:
  r['separate_body_audit'][name]={'root':'/World/Assembly_'+name,'dynamic':name!='nest','fixture_static':name=='nest','kinematic':False,'gravity_disabled':False,'mass_kg':G['parts'][name]['volume_mm3']*1e-9*1240};r['physical_collider_counts'][name]=len(G['parts'][name]['colliders'])
 for v in r['installed_full_body_mass_properties'].values():v.update(validated=True,physics_time_advanced_s=0.)
 for p in G['geometry_probes']:
  q={'part':p['part'],'label':p['label'],'expected_part_hit':p['hit'],'actual_part_hit':p['hit'],'distance_pass':True}
  if p['hit']:q.update(maximum_distance_error_mm=.03,expected_distance_mm=p['expected_distance_mm'],actual_distance_mm=p['expected_distance_mm'])
  r['cooked_geometry_probes'].append(q)
 return r

class FullGateComponentTests(unittest.TestCase):
 def test_complete_motion_fixture(self):self.assertEqual(C.check_motion(motion_fixture(),DESIGN,True),C.CHECKPOINTS)
 def test_forged_pickup_summary_rejected(self):
  r=motion_fixture();r['part_results'][0]['pickup']['lift_mm']=41.
  with self.assertRaises(C.AuditError):C.check_motion(r,DESIGN,True)
 def test_camera_not_still_rejected(self):
  r=motion_fixture();r['checkpoints'][-1]['stillness_trace'][0]['maximum_raw_joint_speed_rad_s']=.03
  with self.assertRaises(C.AuditError):C.check_motion(r,DESIGN,True)
 def test_missing_endpoint_rejected(self):
  r=motion_fixture();r['endpoint_traces']['3_cover_lift'].pop()
  with self.assertRaises(C.AuditError):C.check_motion(r,DESIGN,True)
 def test_correct_identity(self):C.check_identity(identity_fixture(),G,SOURCE,1)
 def test_changed_fk_source_rejected(self):
  r=identity_fixture();r['starter_hashes']['fk.py']='0'*64
  with self.assertRaises(C.AuditError):C.check_identity(r,G,SOURCE,1)
 def test_wrong_joint_index_binding_rejected(self):
  r=identity_fixture();r['kinematic_binding']['arm_joint_indices']=[0,1,2,3,4,6]
  with self.assertRaises(C.AuditError):C.check_identity(r,G,SOURCE,1)
 def test_precontact_promotion_rejected(self):
  r=identity_fixture();r['precontact_accepted']=True
  with self.assertRaises(C.AuditError):C.check_identity(r,G,SOURCE,1)
 def test_relaxed_limit_rejected(self):
  r=identity_fixture();r['limits']['maximum_intended_penetration_mm']=.3
  with self.assertRaises(C.AuditError):C.check_identity(r,G,SOURCE,1)
 def test_complete_physics_fixture(self):C.check_original_physics(physics_fixture(),G,DESIGN,True,SOURCE)
 def test_excess_contact_depth_rejected(self):
  r=physics_fixture();r['contact_summary']['synthetic grip']['minimum_separation_mm']=-.250001
  with self.assertRaises(C.AuditError):C.check_original_physics(r,G,DESIGN,True,SOURCE)
 def test_pcb_kinematic_rejected(self):
  r=physics_fixture();r['separate_body_audit']['pcb']['kinematic']=True
  with self.assertRaises(C.AuditError):C.check_original_physics(r,G,DESIGN,True,SOURCE)
 def test_com_changed_rejected(self):
  r=physics_fixture();r['installed_full_body_mass_properties']['cover']['centre_of_mass_local_m'][0]+=.001
  with self.assertRaises(C.AuditError):C.check_original_physics(r,G,DESIGN,True,SOURCE)

class PhysicalTests(unittest.TestCase):
 def test_pcb_score_recomputed(self):
  carrier={'position_world_m':[0,0,0],'orientation_wxyz':[1,0,0,0]};pcb={'position_world_m':[0,0,.002],'orientation_wxyz':[1,0,0,0]};self.assertTrue(C.payload_ok(carrier,pcb));pcb['position_world_m'][0]=.0008;self.assertFalse(C.payload_ok(carrier,pcb))
 def test_displacement_pairs_keep_corner_identity(self):
  pts=np.zeros((2,16,3));pts[:,1,0]=1000.;self.assertEqual(C.diameter(pts)[0],0.);pts[1,3,0]=.021;self.assertEqual(C.diameter(pts)[0],.021)
 def test_missing_original_roots_reproduces_and_candidate_repairs(self):
  roots={n:'/World/Assembly_'+n for n in C.BODY_NAMES};broken={p:matrix() for p in [roots['cover'],roots['pcb']]}
  with self.assertRaises(KeyError):PassiveVelocity(roots).sample(0.,broken)
  text=(SOURCE/'isolated_lid_grip_trial.py').read_text();tree=ast.parse(text);statement=[a for a in ast.walk(tree) if isinstance(a,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='diagnostic_roots' for t in a.targets)][0]
  expr=ast.get_source_segment(text,statement.value)
  # Candidate-owned known expression, restricted names, no simulator import.
  paths=eval(compile(ast.Expression(statement.value),'candidate-diagnostic-roots','eval'),{'__builtins__':{}},{'C':SimpleNamespace(ROBOT_PRIM='/World/ER4iA'),'roots':roots,'ORDER':('cover',)})
  self.assertEqual(set(paths),set(roots.values())|{'/World/ER4iA/JawL','/World/ER4iA/JawR'})
  v=PassiveVelocity(roots).sample(0.,{p:matrix() for p in paths});self.assertEqual(set(v),set(C.BODY_NAMES))
 def test_unreviewed_runtime_rejected(self):
  with self.assertRaises(C.AuditError):C.verify_source(SOURCE,'0'*64)
 def test_source_bundle_reverses_and_hashes(self):
  # Root is still assembling dispatch files; use a separately frozen CPU fixture.
  with tempfile.TemporaryDirectory(prefix='v54-checker-source-') as temp:
   target=Path(temp)/'snapshot';shutil.copytree(SOURCE,target,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
   manifest={p.name:C.digest(p) for p in target.iterdir() if p.is_file() and p.name!='snapshot-sha256.json'};(target/'snapshot-sha256.json').write_text(json.dumps(manifest))
   g,_=C.verify_source(target,C.digest(target/'isolated_lid_grip_trial.py'));self.assertEqual(g['limits']['maximum_intended_penetration_mm'],.25)
   (target/'assembly_checks.py').write_text('# corrupted CPU fixture only')
   with self.assertRaises(C.AuditError):C.verify_source(target,C.digest(target/'isolated_lid_grip_trial.py'))

class GeometryVariantTests(unittest.TestCase):
 def new_identity(self):
  r=identity_fixture();g=json.loads((SOURCE/'lid-stem-mesh.json').read_text());r['grip_geometry_experiment'].update(variant='stem-plus3',extension_mm=3.);r.update(mesh_source_filename='lid-stem-mesh.json',design_source_filename='lid-stem-design.json',mass_reference_source_filename='lid-stem-mass-reference.json',mesh_sha256=C.digest(SOURCE/'lid-stem-mesh.json'),design_sha256=C.digest(SOURCE/'lid-stem-design.json'));return r,g
 def new_physics(self):
  r=physics_fixture();g=json.loads((SOURCE/'lid-stem-mesh.json').read_text());r['separate_body_audit']['cover']['mass_kg']=g['parts']['cover']['volume_mm3']*1e-9*1240;r['installed_full_body_mass_properties']=json.loads((SOURCE/'lid-stem-mass-reference.json').read_text());r['cooked_geometry_probes']=[]
  for v in r['installed_full_body_mass_properties'].values():v.update(validated=True,physics_time_advanced_s=0.)
  for p in g['geometry_probes']:
   q={'part':p['part'],'label':p['label'],'expected_part_hit':p['hit'],'actual_part_hit':p['hit'],'distance_pass':True}
   if p['hit']:q.update(maximum_distance_error_mm=.03,expected_distance_mm=p['expected_distance_mm'],actual_distance_mm=p['expected_distance_mm'])
   r['cooked_geometry_probes'].append(q)
  return r,g
 def test_new_identity_explicit(self):
  r,g=self.new_identity();C.check_identity(r,g,SOURCE,1,'stem-plus3')
 def test_new_prospective_mass_and_probes(self):
  r,g=self.new_physics();C.check_original_physics(r,g,DESIGN,True,SOURCE,'stem-plus3')
 def test_new_mesh_with_old_hash_rejected(self):
  r,g=self.new_identity();r['mesh_sha256']=C.digest(SOURCE/'nest-mesh.json')
  with self.assertRaises(C.AuditError):C.check_identity(r,g,SOURCE,1,'stem-plus3')
 def test_new_mesh_with_old_reference_binding_rejected(self):
  r,g=self.new_identity();r['mass_reference_source_filename']='verified-seating-mass-reference.json'
  with self.assertRaises(C.AuditError):C.check_identity(r,g,SOURCE,1,'stem-plus3')
 def test_new_mesh_with_old_mass_readback_rejected(self):
  r,g=self.new_physics();r['installed_full_body_mass_properties']['cover']=physics_fixture()['installed_full_body_mass_properties']['cover']
  with self.assertRaises(C.AuditError):C.check_original_physics(r,g,DESIGN,True,SOURCE,'stem-plus3')
 def test_stale_self_consistent_top_probe_rejected(self):
  r,g=self.new_physics();probe=next(p for p in r['cooked_geometry_probes'] if p['label']=='lid_flat_grip_top');probe['expected_distance_mm']=probe['actual_distance_mm']=69.6
  with self.assertRaises(C.AuditError):C.check_original_physics(r,g,DESIGN,True,SOURCE,'stem-plus3')
 def test_new_empty_probe_forged_hit_rejected(self):
  r,g=self.new_physics();probe=next(p for p in r['cooked_geometry_probes'] if p['label']=='v55_above_guide_open_side');probe.update(expected_part_hit=True,actual_part_hit=True,expected_distance_mm=1.,actual_distance_mm=1.,maximum_distance_error_mm=.03)
  with self.assertRaises(C.AuditError):C.check_original_physics(r,g,DESIGN,True,SOURCE,'stem-plus3')
 def test_old_probe_corruption_rejected_too(self):
  r=physics_fixture();probe=next(p for p in r['cooked_geometry_probes'] if p['label']=='lid_flat_grip_top');probe['expected_distance_mm']=probe['actual_distance_mm']=66.6
  with self.assertRaises(C.AuditError):C.check_original_physics(r,G,DESIGN,True,SOURCE)
 def test_new_variant_false_numerical_provenance_rejected(self):
  r,g=self.new_identity();r['velocity_iteration_experiment']['numerical_settings_unchanged']=False
  with self.assertRaises(C.AuditError):C.check_identity(r,g,SOURCE,1,'stem-plus3')
 def audits(self,control='NEGATIVE_CONTROL_REPRODUCED',candidate='ISOLATED_DIAGNOSTIC_PASS'):
  common={'audit_integrity_valid':True,'velocity_iterations':1,'runtime_sha256':'sealed-same-runtime','source_manifest_sha256':'sealed-same-manifest','command_recipe_sha256':'same-recipe','fixed_hold_arm_targets':[0]*6,'fixed_hold_jaw_targets':{'jaw_l':.004,'jaw_r':.004}}
  return [dict(common,geometry_variant='original',classification=control),dict(common,geometry_variant='stem-plus3',classification=candidate)]
 def test_paired_geometry_pass_only_local(self):self.assertEqual(C.classify_comparison(self.audits()),'LOCAL_GEOMETRY_CHANGE_SUPPORTED_FULL_SEQUENCE_REQUIRED')
 def test_control_all_pass_nondiagnostic(self):self.assertEqual(C.classify_comparison(self.audits(control='ISOLATED_DIAGNOSTIC_PASS')),'NONDIAGNOSTIC_CONTROL_DID_NOT_REPRODUCE')
 def test_candidate_failure_remains_failed(self):self.assertEqual(C.classify_comparison(self.audits(candidate='NONDIAGNOSTIC')),'NO_LOCAL_REPAIR_DEMONSTRATED')
 def test_separate_bundle_not_matched_control(self):
  a=self.audits();a[1]['source_manifest_sha256']='other'
  with self.assertRaises(C.AuditError):C.classify_comparison(a)
 def test_changed_hold_commands_rejected(self):
  a=self.audits();a[1]['fixed_hold_arm_targets']=[0,0,0,0,0,.001]
  with self.assertRaises(C.AuditError):C.classify_comparison(a)
 def test_changed_recipe_nondiagnostic(self):
  a=self.audits();a[1]['command_recipe_sha256']='different'
  self.assertEqual(C.classify_comparison(a),'NONDIAGNOSTIC_UNMATCHED_COMMAND_RECIPE')

if __name__=='__main__':unittest.main()
