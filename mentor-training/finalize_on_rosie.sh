#!/bin/bash
#SBATCH --account=students
#SBATCH --qos=interactive
#SBATCH --partition=teaching
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:10:00
#SBATCH --job-name=team-v-package
set -euo pipefail
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$HOME/team-v-tests/mentor-eval-deps${PYTHONPATH:+:$PYTHONPATH}"
release="$HOME/team-v-datasets/mentor-detector-pilot-v2"
cp "$HOME/team-v-tests/mentor-training/test_data.py" "$release/training/"
cp "$HOME/team-v-tests/mentor-training/requirements.txt" "$release/training/"
cp "$HOME/team-v-tests/mentor-training/finalize_bundle.py" "$release/provenance/"
cp "$HOME/team-v-logs/mentor-model-smoke-${TEAM_V_SMOKE_JOB:?}.log" "$release/verification/model-smoke.log"
cd "$release/training"
"$HOME/team-v-isaac/bin/python" -m unittest -v test_data > "$release/verification/adapter-tests.log" 2>&1
"$HOME/team-v-isaac/bin/python" -u verify_release.py --data "$release" > "$release/verification/release-verify.log" 2>&1
"$HOME/team-v-isaac/bin/python" -u "$HOME/team-v-tests/mentor-training/finalize_bundle.py" --release "$release"
