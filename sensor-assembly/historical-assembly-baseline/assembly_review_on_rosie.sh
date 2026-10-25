#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=00:15:00
#SBATCH --job-name=team-v-full-v60c-review
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
TASK_CODE_DIR="${1:?Source directory}"
TASK_GPU_JOB="${2:?GPU job}"
"$HOME/team-v-isaac/bin/python" "$TASK_CODE_DIR/build_full_observed_review.py" --run "$HOME/team-v-runs/full-observed-assembly-v60c-$TASK_GPU_JOB"
