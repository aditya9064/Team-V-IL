"""Case-specific expectations; an intentional HOLD must have the exact cause."""
import argparse,json,math
from pathlib import Path

SCENARIOS=('occupied_destination','missing_view','late_arrival','occluded_view','placed_moved','nominal-six')
def check(r,case):
    states=[e['state'] for e in r['events']];attempts=r['attempts']
    injections=r.get('simulator_score',{}).get('injections',[])
    kinds=[i['kind'] for i in injections]
    assert r['truth_used_for_runtime'] is False
    assert r.get('simulator_score',{}).get('evaluation_only') is True,'Independent score missing'
    assert all(a['trajectory_ik_failures']==0 for a in attempts)
    assert r['detector_backend']=='geometric','These trials validate the baseline; no trained model supplied'
    if case in ('late_arrival','nominal-six'):
        count=1 if case=='late_arrival' else 6
        assert r['status']=='COMPLETE' and r['task_complete']
        assert len(attempts)==r['verified_pickups']==r['verified_placements']==count
        assert all(s['camera_completed'] and s['truth_correct'] for s in r['simulator_score']['slots'])
        if case=='late_arrival':
            assert 'parts_parked_before_physics_reset' in kinds
            assert 'part_arrived_after_wait' in kinds
            assert states.index('WAIT_FOR_PART')<states.index('DISPATCH_PICK')
    else:
        reasons={'occupied_destination':'unconfirmed_destination_occupied','missing_view':'camera_image_missing:side',
                 'occluded_view':'pickup_camera_evidence_uncertain','placed_moved':'previous_placement_lost_or_changed'}
        assert r['status']=='HOLD_FOR_OPERATOR' and r.get('reason')==reasons[case],(r['status'],r.get('reason'))
        assert not r['task_complete']
        if case in ('occupied_destination','missing_view'):
            assert attempts==[] and 'DISPATCH_PICK' not in states
            assert ('destination_occupied' if case=='occupied_destination' else 'rgb_stream_missing') in kinds
        if case=='missing_view':assert r.get('held_capture_error')=='camera_image_missing:side'
        if case=='occluded_view':
            assert 'verification_view_obstructed' in kinds
            assert len(attempts)==1 and r['verified_pickups']==r['verified_placements']==0
            evidence=attempts[0]['pickup_evidence']
            assert len(evidence)==2 and all(e['verdict']=='UNCERTAIN' and not e['source_present'] for e in evidence)
            assert 'DISPATCH_PLACE' not in states and 'RETRY' not in states
        if case=='placed_moved':
            assert 'completed_placement_displaced' in kinds
            assert len(attempts)==r['verified_pickups']==r['verified_placements']==1
            assert r['completed_slots']==['slot-0'] and len(r['slots'])==2
            assert not r['simulator_score']['slots'][0]['truth_correct']
    return True

def audit_frames(run):
    peak=0.;count=0;previous=None
    for f in sorted(run.glob('observations/*/observation.json')):
        d=json.loads(f.read_text());samples=d['stillness']['accepted_trace']
        assert d['robot_idle'] and d['robot_joint_velocity_max']<.02 and len(samples)==6
        assert d['stillness']['raw_velocity_limit_rad_s']==.02 and d['stillness']['required_consecutive_samples']==6
        for sample in samples:
            assert len(sample['joint_velocities'])==6
            assert all(math.isfinite(v) and abs(v)<.02 for v in sample['joint_velocities'])
            peak=max(peak,*map(abs,sample['joint_velocities']))
        assert all(samples[i+1]['sim_time']>samples[i]['sim_time'] for i in range(5))
        if previous:
            assert all(d['views'][v]['sensor_frame']>previous['views'][v]['sensor_frame'] and d['views'][v]['sensor_time']>previous['views'][v]['sensor_time'] for v in d['views'])
        assert max(v['sensor_time'] for v in d['views'].values())-min(v['sensor_time'] for v in d['views'].values())<=.067
        assert all(.85<=x['silhouette_iou']<=1 for x in d['detections'])
        previous=d;count+=1
    assert count>0
    return dict(accepted_captures=count,peak_raw_joint_speed_rad_s=peak)

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--case',choices=SCENARIOS,required=True)
    a=p.parse_args();r=json.loads((a.run/'execution_report.json').read_text());check(r,a.case)
    audit=audit_frames(a.run)
    motion=[m for attempt in r['attempts'] for m in attempt.get('motion',[])]
    assert all(m['endpoint_stable'] and math.isfinite(m['tip_error_mm']) and m['tip_error_mm']<=2 for m in motion)
    audit.update(case=a.case,status=r['status'],case_acceptance_passed=True,max_settled_tcp_error_mm=max((m['tip_error_mm'] for m in motion),default=None))
    (a.run/'acceptance-audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print('ADDITIONAL_CASE_PASS',a.case,r['status'],audit,flush=True)

if __name__=='__main__':main()
