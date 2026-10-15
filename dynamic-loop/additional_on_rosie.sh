#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=01:00:00
#SBATCH --job-name=team-v-plugin
#SBATCH --exclude=dh-node11
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4
export FANUC_STARTER="$HOME/team-v-starter/fanuc_er4ia_sim"
code="${TEAM_V_DYNAMIC_CODE:-$HOME/team-v-tests/plugin-failures-v2/dynamic-loop}"
base="$HOME/team-v-runs/plugin-failures-$SLURM_JOB_ID"
mkdir -p "$base"
cd "$code"
nvidia-smi --query-gpu=name,uuid,driver_version --format=csv
"$HOME/team-v-isaac/bin/python" - <<'PY'
import torch
assert torch.cuda.is_available(), 'Allocated GPU unavailable'
x=torch.ones(1,device='cuda');torch.cuda.synchronize()
print('Allocated GPU:',torch.cuda.get_device_name(0),flush=True)
PY
"$HOME/team-v-isaac/bin/python" -m unittest test_detector_contract test_detector_backend test_supervisor test_vision_geometry test_arrival_fixture
"$HOME/team-v-isaac/bin/python" "$code/backend_smoke.py" --starter "$FANUC_STARTER" --out "$base/backend-smoke.json"
failures=()
scenarios=("$@")
if (( ${#scenarios[@]} == 0 )); then
 scenarios=(occupied_destination missing_view late_arrival occluded_view placed_moved nominal-six)
fi
for scenario in "${scenarios[@]}"; do
 colour_args=(--colours blue)
 runtime="$scenario"
 if [[ "$scenario" == placed_moved ]]; then colour_args=(--colours blue blue); fi
 if [[ "$scenario" == nominal-six ]]; then runtime=nominal; colour_args=(); fi
 echo "ADDITIONAL_CASE_START $scenario"
 if ! "$HOME/team-v-isaac/bin/python" -u "$code/runner.py" --starter "$FANUC_STARTER" \
   --out "$base/$scenario" --scenario "$runtime" --backend geometric "${colour_args[@]}" > "$base/$scenario.log" 2>&1; then
   echo "CASE_PROCESS_NONZERO $scenario"
 fi
 if ! "$HOME/team-v-isaac/bin/python" "$code/check_additional.py" "$base/$scenario" --case "$scenario"; then
   failures+=("$scenario")
 fi
done
if (( ${#failures[@]} )); then echo "ADDITIONAL_CASES_FAILED ${failures[*]}"; exit 1; fi
echo TEAM_V_PLUGIN_FAILURE_SUITE_PASS
