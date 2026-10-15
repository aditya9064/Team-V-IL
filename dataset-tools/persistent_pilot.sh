#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:20:00
#SBATCH --job-name=team-v-persist
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
: "${SLURM_JOB_ID:?Submit through sbatch}"
PILOT_COUNT="${PILOT_COUNT:-3}"
case "$PILOT_COUNT" in 1|2|3|4|5|6|7|8|9|10) ;; *) exit 2 ;; esac
PILOT_OUT="$HOME/team-v-samples/persistent-pilot-$SLURM_JOB_ID"
"$HOME/team-v-isaac/bin/python" -u "$HOME/team-v-tests/persistent_capture.py" \
  --starter "$HOME/team-v-starter/fanuc_er4ia_sim" \
  --out "$PILOT_OUT" --count "$PILOT_COUNT" --start-seed 424201
"$HOME/team-v-isaac/bin/python" - "$PILOT_OUT" "$PILOT_COUNT" <<'PY'
import json,sys
from pathlib import Path
out=Path(sys.argv[1]); count=int(sys.argv[2])
report=json.loads((out/'batch_report.json').read_text())
assert report['complete'] and report['valid_samples']==count, report
for s in report['samples']:
    assert s['valid'],s
    assert (out/f"seed-{s['seed']}"/'world_camera.png').is_file()
print('TEAM_V_PERSISTENT_PILOT_PASS',count,'scenes; startup seconds:',report['startup_s'],
      'sample seconds:',[round(s['elapsed_s'],3) for s in report['samples']],flush=True)
PY
