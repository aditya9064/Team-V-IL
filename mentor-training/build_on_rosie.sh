#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:15:00
#SBATCH --job-name=team-v-crops
set -euo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$HOME/team-v-isaac/bin/python" -u "$HOME/team-v-tests/mentor-training/prepare_bundle.py" \
 --samples "$HOME/team-v-samples" --legacy-dataset "$HOME/team-v-datasets/synthetic-v1-originfix" \
 --diverse "$HOME/team-v-samples/diversity-v2-283831" "$HOME/team-v-samples/diversity-v2-283838" \
 --tools "$HOME/team-v-tests" --out "$HOME/team-v-datasets/mentor-detector-pilot-v2"
