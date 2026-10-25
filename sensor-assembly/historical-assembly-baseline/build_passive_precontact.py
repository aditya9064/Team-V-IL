"""Construct the conditional passive-telemetry candidate; no simulation/job calls."""
from pathlib import Path
import json
p=Path(__file__).parent;s=(p/'lid_pose_trial.py').read_text();runtime=[]
def patch(a,b):
 global s
 assert s.count(a)==1,(a[:100],s.count(a));s=s.replace(a,b);runtime.append([a,b])
patch('from lid_release_schedule import lid_release_samples,lid_release_bounds','from lid_release_schedule import lid_release_samples,lid_release_bounds\nfrom passive_velocity import PassiveVelocity')
patch('        from isaacsim.core.prims import SingleArticulation,SingleXFormPrim,SingleRigidPrim','        from isaacsim.core.prims import SingleArticulation,SingleXFormPrim')
patch("        issued_jaw=None;issued_jaw_targets={};velocity_readers={};lid_orientation=None", "        issued_jaw=None;issued_jaw_targets={};lid_orientation=None\n        passive_velocity=PassiveVelocity({name:roots[name] for name in ('base','carrier','pcb','cover')})")
patch("            return {'step':state['step'],'sim_time':float(world.current_time),", "            snapshot={'step':state['step'],'sim_time':float(world.current_time),")
patch("                'issued_jaw_joint_targets_m':dict(issued_jaw_targets),\n                'body_velocities':{name:{'linear_world_m_s':v.get_linear_velocity().tolist(),'angular_world_rad_s':v.get_angular_velocity().tolist()} for name,v in velocity_readers.items()}}", "                'issued_jaw_joint_targets_m':dict(issued_jaw_targets)}\n            snapshot['body_velocities']=passive_velocity.sample(snapshot['sim_time'],snapshot['actor_world_matrices'])\n            return snapshot")
patch("        # Existing dynamic bodies only; no mass/material/pose/velocity setters.\n        velocity_readers={name:SingleRigidPrim(path,name='velocity_'+name,reset_xform_properties=False) for name,path in roots.items() if name!='nest'}\n        for reader in velocity_readers.values():reader.initialize()", "        # Passive estimates reuse existing pose snapshots; no rigid observer registration.")
patch("    report['diagnostic_trace']=[]", "    report['diagnostic_trace']=[]\n    report['body_velocity_telemetry']={'source':'finite_difference_existing_USD_pose_snapshots','helper_sha256':digest(Path(__file__).with_name('passive_velocity.py')),'linear_reference':'body origin, not center of mass','angular_reference':'world log(Rnow Rprev.T)/dt','instantaneous_velocity_claim':False,'used_for_targets_or_motion_gates':False,'physical_scoring_input':False}\n    report['candidate_provenance']='Conditional passive observer replacement; eligibility depends on independent interpretation of comparison286764. No repaired/physical/full-assembly claim.'")
(p/'passive_lid_precontact_trial.py').write_text(s);(p/'passive-runtime-patches.json').write_text(json.dumps(runtime,indent=2)+'\n')
s=(p/'check_lid_pose.py').read_text();checker=[]
def cp(a,b):
 global s
 assert s.count(a)==1,(a[:100],s.count(a));s=s.replace(a,b);checker.append([a,b])
cp("def main():", """def validate_passive_velocity(value):
    assert value['available'] is True or value['available'] is False
    assert value['source']=='finite_difference_existing_USD_pose_snapshots'
    assert value['linear_reference']=='body origin, not center of mass'
    assert value['angular_reference']=='world log(Rnow Rprev.T)/dt'
    if value['available']:
        assert isinstance(value['interval_s'],(int,float)) and not isinstance(value['interval_s'],bool) and math.isfinite(value['interval_s']) and value['interval_s']>0
        for name in ('linear_world_m_s','angular_world_rad_s'):
            vector=value[name];assert isinstance(vector,list) and len(vector)==3
            assert all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in vector)
    else:
        assert value['linear_world_m_s'] is None and value['angular_world_rad_s'] is None
        assert (value.get('reason')=='first_sample' and value['interval_s'] is None) or (value.get('reason')=='same_simulation_time' and value['interval_s']==0)


def main():""")
cp("[('lid_pose_trial.py','trial_script_sha256')", "[('passive_lid_precontact_trial.py','trial_script_sha256')")
cp("        for body in v['body_velocities'].values():assert all(math.isfinite(x) for x in body['linear_world_m_s']+body['angular_world_rad_s'])", "        for body in v['body_velocities'].values():validate_passive_velocity(body)")
cp("    assert r['diagnostic_trace']", "    assert r['body_velocity_telemetry']=={'source':'finite_difference_existing_USD_pose_snapshots','helper_sha256':digest(source/'passive_velocity.py'),'linear_reference':'body origin, not center of mass','angular_reference':'world log(Rnow Rprev.T)/dt','instantaneous_velocity_claim':False,'used_for_targets_or_motion_gates':False,'physical_scoring_input':False}\n    for shot in r['checkpoints']:\n        assert set(shot['body_velocities'])=={'base','carrier','pcb','cover'}\n        for body in shot['body_velocities'].values():validate_passive_velocity(body)\n    assert r['diagnostic_trace']")
cp("'placements':len(order)", "'placements':sum(bool(v['placement_verified']) for v in r['part_results']),'diagnostic_stage':stage,'full_assembly_accepted':r['full_assembly_accepted']")
(p/'check_passive_lid_precontact.py').write_text(s);(p/'passive-checker-patches.json').write_text(json.dumps(checker,indent=2)+'\n')
