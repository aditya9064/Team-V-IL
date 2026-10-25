#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:15:00
#SBATCH --exclude=dh-node11
#SBATCH --job-name=team-v-option-a-pickup-01
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
TASK_CODE_DIR="${1:?Source directory}"
TASK_MANIFEST_SHA="${2:?Externally reviewed manifest SHA}"
TASK_STARTER="$HOME/team-v-starter/fanuc_er4ia_sim"
TASK_PYTHON="$HOME/team-v-isaac/bin/python"
TASK_RUN="$HOME/team-v-runs/option-a-pickup-$SLURM_JOB_ID"
cd "$TASK_CODE_DIR"
"$TASK_PYTHON" - "$TASK_MANIFEST_SHA" <<'VERIFY'
import hashlib,json,sys
from pathlib import Path
p=Path('snapshot-sha256.json')
assert hashlib.sha256(p.read_bytes()).hexdigest()==sys.argv[1]
manifest=json.loads(p.read_text())
actual={str(p.relative_to(Path.cwd())) for p in Path.cwd().rglob('*') if p.is_file()}
assert actual==set(manifest)|{'snapshot-sha256.json'}
assert not any(p.is_symlink() for p in Path.cwd().rglob('*'))
for name,h in manifest.items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==h,name
print('IMMUTABLE_OPTION_A_SOURCE_VERIFIED',flush=True)
VERIFY
"$TASK_PYTHON" -W error -m unittest test_option_a test_admission test_pickup_integration test_option_a_pickup_trace_checker
cd "$TASK_STARTER"
"$TASK_PYTHON" -m unittest test_sort_planning test_cube_perception test_cube_pose test_runtime_config
mkdir -p "$TASK_RUN"
TASK_FAILED=0
for TASK_PART in base carrier cover; do
  # Each declared independent case runs once. No tuning/retry between cases.
  set +e
  timeout --signal=TERM --kill-after=20s 250 "$TASK_PYTHON" -u "$TASK_CODE_DIR/option_a_pickup.py" --starter "$TASK_STARTER" --part "$TASK_PART" --out "$TASK_RUN/$TASK_PART" > "$TASK_RUN/$TASK_PART-runtime.log" 2>&1
  TASK_EXIT=$?
  set -e
  printf '%s\n' "CASE_RECORDED part=$TASK_PART runtime_exit=$TASK_EXIT; independent audit required"
  printf '%s\n' "$TASK_EXIT" > "$TASK_RUN/$TASK_PART-runtime-exit.txt"
  if [ -f "$TASK_RUN/$TASK_PART/report.json" ]; then
    set +e
    timeout --signal=TERM --kill-after=2s 15 "$TASK_PYTHON" "$TASK_CODE_DIR/check_option_a_pickup.py" --source "$TASK_CODE_DIR" --report "$TASK_RUN/$TASK_PART/report.json" --manifest-sha256 "$TASK_MANIFEST_SHA" --part "$TASK_PART" > "$TASK_RUN/$TASK_PART-audit.log" 2>&1
    TASK_AUDIT_EXIT=$?
    set -e
    printf '%s\n' "$TASK_AUDIT_EXIT" > "$TASK_RUN/$TASK_PART-audit-exit.txt"
    if [ "$TASK_AUDIT_EXIT" -ne 0 ]; then TASK_FAILED=1; fi
  else
    TASK_FAILED=1
  fi
  # A missing/timed-out audit stays incomplete. It may be read and checked on
  # CPU later without another simulator allocation; never infer pass from0.
  if [ "$TASK_EXIT" -ne 0 ]; then TASK_FAILED=1; fi
done
exit "$TASK_FAILED"
