"""Summarize saved reports without converting an expected HOLD into kit success."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import subprocess
import sys

EXPECTED={'empty':'WAIT_FOR_PART','missing':'WAIT_FOR_PART','stale':'HOLD_FOR_OPERATOR',
          'moved':'COMPLETE','miss_once':'COMPLETE','miss_twice':'HOLD_FOR_OPERATOR',
          'nominal-six':'COMPLETE'}
p=argparse.ArgumentParser()
p.add_argument('base',type=Path)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--prior-base',type=Path,action='append',default=[],
               help='Reuse unchanged gate-only cases from a prior suite; newest report wins')
a=p.parse_args();rows=[]
for scenario,expected in EXPECTED.items():
    path=next((base/scenario/'execution_report.json' for base in [a.base,*a.prior_base]
               if (base/scenario/'execution_report.json').exists()),None)
    if path is None:
        rows.append(dict(scenario=scenario,expected=expected,result='NOT_RUN'))
        continue
    data=path.read_bytes();r=json.loads(data)
    verdict=subprocess.run([sys.executable,str(Path(__file__).with_name('check_trial.py')),
                            str(path.parent),'--expect',expected],capture_output=True,text=True)
    motion=[m for attempt in r['attempts'] for m in attempt.get('motion',[])]
    row=dict(scenario=scenario,expected=expected,status=r['status'],reason=r.get('reason'),
             source_suite=path.parent.parent.name,
             result='PASS' if verdict.returncode==0 else 'FAIL',
             task_complete=r['task_complete'],attempts=len(r['attempts']),
             verified_pickups=r.get('verified_pickups',0),verified_placements=r.get('verified_placements',0),
             states=dict(collections.Counter(e['state'] for e in r['events'])),
             report_sha256=hashlib.sha256(data).hexdigest(),runtime_sha256=r.get('runtime_sha256'),
             max_settled_tip_error_mm=max((m['tip_error_mm'] for m in motion),default=None))
    if verdict.returncode:row['acceptance_error']=(verdict.stderr or verdict.stdout).strip()[-1500:]
    rows.append(row)
summary=dict(suites=[base.name for base in [a.base,*a.prior_base]],physical_robot_tested=False,
             all_scenarios_passed=all(row['result']=='PASS' for row in rows),scenarios=rows)
a.out.parent.mkdir(parents=True,exist_ok=True)
a.out.write_text(json.dumps(summary,indent=2)+'\n')
for row in rows:
    print(row['scenario'],row['result'],row.get('status',''),
          'attempts',row.get('attempts',0),'verified placements',row.get('verified_placements',0))
print('ALL_SCENARIOS_PASSED',summary['all_scenarios_passed'])
