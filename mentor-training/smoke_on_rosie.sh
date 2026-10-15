#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --gres=gpu:t4:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=00:10:00
#SBATCH --job-name=team-v-modelcheck
set -euo pipefail
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$HOME/team-v-tests/mentor-eval-deps${PYTHONPATH:+:$PYTHONPATH}"
cd "$HOME/team-v-datasets/mentor-detector-pilot-v2/training"
"$HOME/team-v-isaac/bin/python" -m unittest -v test_data
"$HOME/team-v-isaac/bin/python" -u smoke.py \
 --data "$HOME/team-v-datasets/mentor-detector-pilot-v2" \
 --out "$HOME/team-v-datasets/mentor-detector-pilot-v2/model_smoke.json"
"$HOME/team-v-isaac/bin/python" -m pip freeze > "$HOME/team-v-datasets/mentor-detector-pilot-v2/rosie_smoke_environment.txt"
