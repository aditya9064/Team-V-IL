#!/bin/bash
# After the v2 trainings: push their best.pt to the GB10, score them on the fixed
# baseline set next to Cosmos3-Nano, and bring the table back to $DATA_ROOT/BASELINE.txt.
#   sbatch --dependency=afterany:<train ids> rosie/jobs/gb10_job.sh [run names...]
# Default run names: cubes_v2 cubes_v1v2 (pass e.g. just cubes_v2 to score one model early).
# The GB10 side (envs/cosmos, baseline_set, run_baseline.py) was set up by hand once.
#SBATCH --job-name=av-gb10
#SBATCH --cpus-per-task=1
#SBATCH --mem=2G
set -euo pipefail
source "$PWD/rosie/common.sh"
echo "[gb10] $(hostname)  job ${SLURM_JOB_ID:-local}"
SSH=(-i "$GB10_KEY" -o BatchMode=yes -o ConnectTimeout=30 -o ServerAliveInterval=60)

latest() {  # newest best.pt for a run-name prefix
    ls -td "$RUNS_DIR"/detect/"$1"_[0-9]*/weights/best.pt 2>/dev/null | head -1
}

models=("$@")
[ ${#models[@]} -gt 0 ] || models=(cubes_v2 cubes_v1v2)
names=()
for name in "${models[@]}"; do
    w=$(latest "$name")
    if [ -z "$w" ]; then echo "[gb10] no $name model, skipping"; continue; fi
    echo "[gb10] $name <- $w"
    scp "${SSH[@]}" "$w" "$GB10_HOST:$GB10_DIR/models/$name.pt"
    names+=("$name")
done
[ ${#names[@]} -gt 0 ] || die "no trained v2 models to score"

# On the GB10: let a running Cosmos baseline finish first (one GPU job at a time on
# the shared box), score each model (resumable), then rebuild the summary table.
ssh "${SSH[@]}" "$GB10_HOST" bash -s -- "$GB10_DIR" "${names[@]}" <<'REMOTE'
set -euo pipefail
cd "$HOME/$1"; shift
export PYTHONNOUSERSITE=1 HF_HOME="$PWD/hf" YOLO_CONFIG_DIR="$PWD/ultralytics"
while pgrep -f "run_baseline.py.*--detector cosmos" >/dev/null; do
    echo "[gb10] Cosmos baseline still running, waiting"; sleep 300
done
for name in "$@"; do
    envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set \
        --detector yolo --weights "models/$name.pt" --name "yolo_${name#cubes_}" | tail -1
done
envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set --summary
REMOTE

scp "${SSH[@]}" "$GB10_HOST:$GB10_DIR/baseline_set/BASELINE.txt" "$DATA_ROOT/BASELINE.txt"
scp "${SSH[@]}" "$GB10_HOST:$GB10_DIR/baseline_set/results.jsonl" "$DATA_ROOT/baseline_results.jsonl"
echo "[gb10] done -> $DATA_ROOT/BASELINE.txt"
