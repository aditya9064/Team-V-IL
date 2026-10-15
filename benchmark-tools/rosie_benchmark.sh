#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:20:00
#SBATCH --job-name=team-v-bench
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
: "${SLURM_JOB_ID:?Run with sbatch}"
"$HOME/team-v-isaac/bin/python" -u "$HOME/team-v-benchmark/benchmark_capture.py" \
 --python "$HOME/team-v-isaac/bin/python" \
 --starter "$HOME/team-v-starter/fanuc_er4ia_sim" \
 --out "$HOME/team-v-benchmarks/rosie-$SLURM_JOB_ID" --host-label rosie-t4
