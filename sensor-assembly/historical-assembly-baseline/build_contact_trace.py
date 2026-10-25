"""Reversible telemetry-only runtime builder; never runs simulation."""
import json
from pathlib import Path
p=Path(__file__).parent
s=(p/'multistart_lid_precontact_trial.py').read_text();patches=[]
def patch(old,new):
    global s
    assert s.count(old)==1,(old[:80],s.count(old));s=s.replace(old,new);patches.append([old,new])
patch('from passive_velocity import PassiveVelocity','from passive_velocity import PassiveVelocity\nfrom lid_contact_trace import ContactTrace')
patch("    report['candidate_provenance']=", "    report['lid_contact_trace_provenance']={'scope':'READONLY TELEMETRY ONLY; original commands/gates unchanged; failure remains failure','helper_sha256':digest(Path(__file__).with_name('lid_contact_trace.py')),'geometry_binding_file':'telemetry-finger-geometry.json','geometry_binding_sha256':digest(Path(__file__).with_name('telemetry-finger-geometry.json')),'parent_runtime_sha256':digest(Path(__file__).with_name('multistart_lid_precontact_trial.py')),'control_or_truth_steering':False,'extra_physics_queries_or_registration':False,'runtime_logging_IO_per_step':False}\n    report['candidate_provenance']=")
patch("        support_pairs=initial_supports_for_case(a.case);placed=['nest'];contact_count=0", "        support_pairs=initial_supports_for_case(a.case);placed=['nest'];contact_count=0\n        lid_contact_trace=ContactTrace(roots['cover'],[C.ROBOT_PRIM+'/JawL',C.ROBOT_PRIM+'/JawR'])\n        report['lid_contact_trace']=lid_contact_trace.value")
patch("            snapshot['body_velocities']=passive_velocity.sample(snapshot['sim_time'],snapshot['actor_world_matrices'])\n            return snapshot", "            snapshot['body_velocities']=passive_velocity.sample(snapshot['sim_time'],snapshot['actor_world_matrices'])\n            lid_contact_trace.note_existing_snapshot(snapshot)\n            return snapshot")
patch('        def on_contacts(headers,data,*unused):\n            nonlocal contact_count', "        def on_contacts(headers,data,*unused):\n            nonlocal contact_count\n            telemetry_callback=lid_contact_trace.begin_callback(state['step'],state['phase'],lambda:float(world.current_time))")
patch('                contact_count+=1\n                try:', "                contact_count+=1\n                lid_contact_trace.observe_header(telemetry_callback,h,data,lambda raw:str(PhysicsSchemaTools.intToSdfPath(raw)),collider_roles)\n                try:")
patch("        report['payload_contact_events']=0", "            lid_contact_trace.end_callback(telemetry_callback)\n        report['payload_contact_events']=0")
patch("                if state['active']=='cover':report['diagnostic_trace'].append(dict(snapshot,phase=state['phase']))", "                if state['active']=='cover':report['diagnostic_trace'].append(dict(snapshot,phase=state['phase']))\n                lid_contact_trace.begin_step(snapshot,len(report['diagnostic_trace'])-1,state['phase'],bool(state.get('incoming_lid_capture')))")
patch('                after_step=float(world.current_time);step_error=abs(after_step-before_step-1/60)', "                after_step=float(world.current_time);lid_contact_trace.end_step(state['step'],after_step)\n                step_error=abs(after_step-before_step-1/60)")
patch("        report['contact_headers_seen']=locals().get('contact_count',0)", "        if 'lid_contact_trace' in locals():lid_contact_trace.finalize(report['status'],report.get('error'))\n        report['contact_headers_seen']=locals().get('contact_count',0)")
(p/'contact_trace_lid_precontact_trial.py').write_text(s)
(p/'contact-trace-runtime-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
