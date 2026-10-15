#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:20:00
#SBATCH --job-name=team-v-variety
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
: "${SLURM_JOB_ID:?Submit through sbatch}"
VARIED_COUNT="${VARIED_COUNT:-6}"
VARIED_SEED="${VARIED_SEED:-525101}"
VARIED_OUT="$HOME/team-v-samples/diversity-v2-$SLURM_JOB_ID"
"$HOME/team-v-isaac/bin/python" -u "$HOME/team-v-tests/varied_capture.py" \
  --starter "$HOME/team-v-starter/fanuc_er4ia_sim" --out "$VARIED_OUT" \
  --count "$VARIED_COUNT" --start-seed "$VARIED_SEED"
"$HOME/team-v-isaac/bin/python" - "$VARIED_OUT" "$VARIED_COUNT" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);r=json.loads((root/'batch_report.json').read_text())
assert r['complete'] and len(r['samples'])==int(sys.argv[2]) and all(s['valid'] for s in r['samples']),r
print('TEAM_V_VARIETY_LABEL_QA_PASS',len(r['samples']),'scenes',flush=True)
PY
