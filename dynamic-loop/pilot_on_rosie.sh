#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --job-name=team-v-dynamic
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4
starter="$HOME/team-v-starter/fanuc_er4ia_sim"
code="$HOME/team-v-tests/dynamic-loop"
out="$HOME/team-v-runs/dynamic-pilot-$SLURM_JOB_ID"
nvidia-smi --query-gpu=name,uuid,driver_version --format=csv
"$HOME/team-v-isaac/bin/python" - <<'PY'
import torch
assert torch.cuda.is_available(), 'Allocated CUDA GPU unavailable; simulation not started'
x=torch.ones(1,device='cuda')
torch.cuda.synchronize()
print('Allocated CUDA GPU:',torch.cuda.get_device_name(0),flush=True)
PY
"$HOME/team-v-isaac/bin/python" -u "$code/runner.py" \
 --starter "$starter" --out "$out" --scenario nominal --colours "${1:-blue}"
"$HOME/team-v-isaac/bin/python" - "$out" <<'PY'
import json,sys
from pathlib import Path
r=json.loads((Path(sys.argv[1])/'execution_report.json').read_text())
print('Runtime outcome:',r['status'],r.get('reason'),flush=True)
assert r['status']!='ERROR',r
if r['task_complete']:
    assert r['verified_pickups']==r['verified_placements']==1
    assert all(s['truth_correct'] for s in r['simulator_score']['slots'])
    print('TEAM_V_DYNAMIC_ONE_CUBE_PASS',flush=True)
else:
    print('CAMERA/PREFLIGHT GATE HELD: inspect evidence; not an execution pass',flush=True)
PY
