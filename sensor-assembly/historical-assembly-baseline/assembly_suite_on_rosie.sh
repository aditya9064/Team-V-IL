#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:59:00
#SBATCH --exclude=dh-node11
#SBATCH --job-name=team-v-full-v60c-01
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
TASK_CODE_DIR="${1:?Pass source directory}"
TASK_STARTER="$HOME/team-v-starter/fanuc_er4ia_sim"
TASK_RUN="$HOME/team-v-runs/full-observed-assembly-v60c-$SLURM_JOB_ID"
cd "$TASK_CODE_DIR"
"$HOME/team-v-isaac/bin/python" - <<'VERIFY'
import hashlib,json
from pathlib import Path
for n,h in json.loads(Path('snapshot-sha256.json').read_text()).items():assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==h,n
print('IMMUTABLE_FULL_ASSEMBLY_SOURCE_VERIFIED',flush=True)
VERIFY
"$HOME/team-v-isaac/bin/python" -m unittest test_assembly_checks test_payload_checks test_compact_fixture test_motion_schedule test_seating_support test_contact_setup test_full_sequence test_sensor_product test_supported_grip test_base_grip test_release_clearance test_nest_clearance test_carrier_guides test_lid_seating test_proven_controller test_extended_flat test_centred_base test_representation test_isolation test_held_pose test_release_plan test_full_held_migration test_payload_schedule test_lid_clearance test_flat_lid test_physical_nest test_native_assembly test_held_budget test_gentle_lid test_lid_pose test_foot_audit test_lid_pose_integration test_lid_review test_passive_precontact test_slow_precontact test_incoming_lid_stability test_continuous_lid test_base_multistart test_lid_contact_trace test_contact_trace_review test_payload_contact_trace test_isolated_lid_grip test_paired_lid_grip test_supported_saddle_grip test_independent_supported_saddle test_observed_lid_placement test_independent_oracle_advisory test_json_token_equivalence test_compact_runtime test_execution_timing test_full_motion_dispatch test_full_sleep_binding test_passive_sleep_state test_unavailable_sleep_state test_full_observed_assembly test_full_pose_readback test_full_runtime_migration
cd "$TASK_STARTER"
"$HOME/team-v-isaac/bin/python" -m unittest test_sort_planning test_cube_perception test_cube_pose test_runtime_config
mkdir -p "$TASK_RUN"
set +e
timeout --signal=TERM --kill-after=30s 3390 "$HOME/team-v-isaac/bin/python" -u "$TASK_CODE_DIR/execution_timing.py" --events "$TASK_RUN/runtime-timing.jsonl" -- "$HOME/team-v-isaac/bin/python" -u "$TASK_CODE_DIR/full_observed_assembly_trial.py" --starter "$TASK_STARTER" --mesh "$TASK_CODE_DIR/nest-mesh.json" --design "$TASK_CODE_DIR/nest-design.json" --case full-observed-assembly --stage release --foot-budget "$TASK_CODE_DIR/precontact-budget.json" --velocity-iterations 1 --geometry-variant original --out "$TASK_RUN/full-assembly" > "$TASK_RUN/full-runtime.log" 2>&1
TASK_EXIT=$?
set -e
echo "FULL_ASSEMBLY_RUNTIME_RECORDED runtime_exit=$TASK_EXIT; CPU review job audits independently"
exit "$TASK_EXIT"
