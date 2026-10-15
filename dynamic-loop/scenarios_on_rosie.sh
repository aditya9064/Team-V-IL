#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=01:00:00
#SBATCH --job-name=team-v-looptests
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4
export FANUC_STARTER="$HOME/team-v-starter/fanuc_er4ia_sim"
code="$HOME/team-v-tests/dynamic-loop"
base="$HOME/team-v-runs/dynamic-scenarios-$SLURM_JOB_ID"
mkdir -p "$base"
nvidia-smi --query-gpu=name,uuid,driver_version --format=csv
"$HOME/team-v-isaac/bin/python" - <<'PY'
import torch
assert torch.cuda.is_available(), 'Allocated CUDA GPU unavailable'
x=torch.ones(1,device='cuda');torch.cuda.synchronize()
print('Allocated CUDA GPU:',torch.cuda.get_device_name(0),flush=True)
PY
cd "$code"
"$HOME/team-v-isaac/bin/python" -m unittest test_supervisor test_vision_geometry
cases=(empty missing stale moved miss_once miss_twice nominal-six)
if (( $# )); then cases=("$@"); fi
for scenario in "${cases[@]}"; do
  colour=blue
  expect=COMPLETE
  runtime_scenario="$scenario"
  colour_args=(--colours "$colour")
  case "$scenario" in
    empty) expect=WAIT_FOR_PART ;;
    missing) colour_args=(--colours red); expect=WAIT_FOR_PART ;;
    stale|miss_twice) expect=HOLD_FOR_OPERATOR ;;
    nominal-six) runtime_scenario=nominal; colour_args=() ;;
    moved|miss_once) ;;
    *) echo "Unknown scenario: $scenario" >&2; exit 2 ;;
  esac
  echo "SCENARIO_START $scenario expected=$expect"
  "$HOME/team-v-isaac/bin/python" -u "$code/runner.py" \
    --starter "$HOME/team-v-starter/fanuc_er4ia_sim" --out "$base/$scenario" \
    --scenario "$runtime_scenario" "${colour_args[@]}" > "$base/$scenario.log" 2>&1
  "$HOME/team-v-isaac/bin/python" "$code/check_trial.py" "$base/$scenario" --expect "$expect"
done
echo TEAM_V_SELECTED_CASES_PASS "${cases[@]}"
