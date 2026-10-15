"""Report-based scenario acceptance; process exit status alone is insufficient."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--expect',required=True)
a=p.parse_args();r=json.loads((a.run/'execution_report.json').read_text())
assert r['status']==a.expect,(r['status'],r.get('reason'))
assert r['truth_used_for_runtime'] is False
assert all(t['trajectory_ik_failures']==0 for t in r['attempts'])
if a.expect=='COMPLETE':
    assert r['task_complete'] and r['verified_placements']==len(r['slots'])
    assert all(s['camera_completed'] and s['truth_correct'] for s in r['simulator_score']['slots'])
    assert all(t['pickup_verified'] for t in r['attempts'] if t['placement_verified'])
    if r['scenario']=='moved':assert any(e['state']=='REPLAN' for e in r['events'])
    if r['scenario']=='miss_once':
        assert len(r['attempts'])==2 and any(e['state']=='RETRY' for e in r['events'])
        assert r['attempts'][0]['pickup_verified'] is False
elif a.expect=='WAIT_FOR_PART':
    assert not r['task_complete'] and not r['attempts']
elif a.expect=='HOLD_FOR_OPERATOR':
    assert not r['task_complete']
    if r['scenario']=='stale':assert r['reason']=='stale_observation' and not r['attempts']
    if r['scenario']=='miss_twice':
        assert r['reason']=='empty_grasp_retry_exhausted' and len(r['attempts'])==2
        assert not any(t['pickup_verified'] or t['placement_verified'] for t in r['attempts'])
print('TEAM_V_DYNAMIC_SCENARIO_PASS',r['scenario'],r['status'],flush=True)
