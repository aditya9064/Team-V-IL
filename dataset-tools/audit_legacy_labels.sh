#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:15:00
#SBATCH --job-name=team-v-labelqa
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$HOME/team-v-isaac/bin/python" -u "$HOME/team-v-tests/audit_legacy_labels.py" \
 --samples "$HOME/team-v-samples" --out "$HOME/team-v-datasets/synthetic-v1-originfix"
