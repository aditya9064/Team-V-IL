"""Build duration-only V48 candidate from frozen V47; never starts simulation."""
import json
from pathlib import Path
p=Path(__file__).parent
s=(p/'passive_lid_precontact_trial.py').read_text();patches=[]
def patch(a,b):
 global s
 assert s.count(a)==1,(a[:100],s.count(a));s=s.replace(a,b);patches.append([a,b])
patch("    report['expected_motion_phases']=expected_phases(a.stage)","    report['motion_sampling']['only_duration_change']['3_cover_precontact']={'before_s':2.,'after_s':4.}\n    report['motion_sampling']['changes']+=' V48 only: precontact duration2s to4s, same cubic normalized path and full orientation endpoints; no grasp-reference reset or threshold change.'\n    report['expected_motion_phases']=expected_phases(a.stage)")
patch("                move(prefix+'_precontact',precontact,jaw,2.,new_orientation=target_R)","""                path_length=float(np.linalg.norm(precontact-goal))
                report['slow_precontact_profile']={'phase':'3_cover_precontact','law':'s(u)=3u^2-2u^3; original motion_samples','commanded_start_robot_m':goal.tolist(),'commanded_end_robot_m':precontact.tolist(),'path_length_m':path_length,'before_duration_s':2.,'duration_s':4.,'duration_ratio':2.,'before_peak_speed_m_s':1.5*path_length/2.,'peak_speed_m_s':1.5*path_length/4.,'speed_ratio':.5,'before_peak_acceleration_m_s2':6*path_length/2.**2,'peak_acceleration_m_s2':6*path_length/4.**2,'acceleration_ratio':.25,'before_interior_jerk_m_s3':12*path_length/2.**3,'interior_jerk_m_s3':12*path_length/4.**3,'interior_jerk_ratio':.125,'endpoint_acceleration_discontinuity_retained':True,'orientation_endpoints_unchanged':True,'grasp_reference_unchanged':True,'thresholds_unchanged':True,'bounds_are_commanded_analytic_not_measured':True}
                persist()
                move(prefix+'_precontact',precontact,jaw,4.,new_orientation=target_R)""")
(p/'slow_lid_precontact_trial.py').write_text(s);(p/'slow-runtime-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
s=(p/'check_passive_lid_precontact.py').read_text();patches=[]
patch("[('passive_lid_precontact_trial.py','trial_script_sha256')","[('slow_lid_precontact_trial.py','trial_script_sha256')")
patch("'3_cover_release':{'before_s':1.,'after_s':3.}}","'3_cover_release':{'before_s':1.,'after_s':3.},'3_cover_precontact':{'before_s':2.,'after_s':4.}}")
patch("def main():",'''ORIGINAL_PHASE_DURATIONS={'1_base_orient':1.,'1_base_above':2.,'1_base_approach':2.,'1_base_side_entry':1.5,'1_base_close':3.,'1_base_lift':1.5,'1_base_raise_for_transfer':1.5,'1_base_carry':2.,'1_base_level_held':2.,'1_base_lower':2.,'1_base_relax_clamp':2.,'1_base_square_fingers':2.,'1_base_release':1.,'1_base_side_exit':1.5,'1_base_retreat':2.,'2_carrier_orient':1.,'2_carrier_above':2.,'2_carrier_approach':2.,'2_carrier_side_entry':1.5,'2_carrier_close':3.,'2_carrier_lift':1.5,'2_carrier_raise_for_transfer':1.5,'2_carrier_carry':4.,'2_carrier_lower':2.,'2_carrier_release':1.,'2_carrier_side_exit':1.5,'2_carrier_retreat':2.,'3_cover_orient':1.,'3_cover_above':2.,'3_cover_approach':2.,'3_cover_side_entry':1.5,'3_cover_close':3.,'3_cover_lift':1.5,'3_cover_raise_for_transfer':1.5,'3_cover_carry':2.,'3_cover_level_held':2.,'3_cover_precontact':2.}

def validate_slow_precontact(r):
    rows=r['motion_sampling']['phases'];assert [x['phase'] for x in rows]==list(ORIGINAL_PHASE_DURATIONS)
    for x in rows:
        duration=4. if x['phase']=='3_cover_precontact' else ORIGINAL_PHASE_DURATIONS[x['phase']]
        assert x['requested_duration_s']==duration and x['commands']==round(duration*60)
    v=r['slow_precontact_profile'];assert v['phase']=='3_cover_precontact' and v['law']=='s(u)=3u^2-2u^3; original motion_samples'
    a=v['commanded_start_robot_m'];b=v['commanded_end_robot_m'];assert len(a)==len(b)==3
    assert all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in a+b)
    distance=math.sqrt(sum((y-x)**2 for x,y in zip(a,b)));assert distance>0
    expected={'path_length_m':distance,'before_duration_s':2.,'duration_s':4.,'duration_ratio':2.,'before_peak_speed_m_s':1.5*distance/2.,'peak_speed_m_s':1.5*distance/4.,'speed_ratio':.5,'before_peak_acceleration_m_s2':6*distance/4.,'peak_acceleration_m_s2':6*distance/16.,'acceleration_ratio':.25,'before_interior_jerk_m_s3':12*distance/8.,'interior_jerk_m_s3':12*distance/64.,'interior_jerk_ratio':.125}
    for k,value in expected.items():assert isinstance(v[k],(int,float)) and not isinstance(v[k],bool) and math.isfinite(v[k]) and math.isclose(v[k],value,rel_tol=1e-10,abs_tol=1e-12),k
    for k in ('endpoint_acceleration_discontinuity_retained','orientation_endpoints_unchanged','grasp_reference_unchanged','thresholds_unchanged','bounds_are_commanded_analytic_not_measured'):assert v[k] is True


def main():''')
patch("    transfer=next(p for p in sampling['phases'] if p['phase']=='2_carrier_carry');", "    validate_slow_precontact(r)\n    transfer=next(p for p in sampling['phases'] if p['phase']=='2_carrier_carry');")
(p/'check_slow_lid_precontact.py').write_text(s);(p/'slow-checker-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
