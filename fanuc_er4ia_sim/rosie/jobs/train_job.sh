#!/bin/bash
# Step 7 GPU job. Submitted by rosie/av.sh:  train_job.sh <data.yaml> <run-name> [extra args]
#SBATCH --job-name=av-train
#SBATCH --cpus-per-task=16
#SBATCH --mem=128G
set -euo pipefail
source "$PWD/rosie/common.sh"

data="${1:?usage: train_job.sh <data.yaml> <run-name>}"
name="${2:?usage: train_job.sh <data.yaml> <run-name>}_$(date +%m%d_%H%M)"
shift 2
echo "[job] $(hostname)  job ${SLURM_JOB_ID:-local}"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
# Pretrained weights were cached by `av.sh setup`; compute nodes may be offline.
export YOLO_CONFIG_DIR="$DATA_ROOT/ultralytics"
cd "$DATA_ROOT/weights"
# RAM caching decodes every image once (~4.1 MB each at 1664x896): fine for v1 (7k = 30 GB)
# but 17-24k v2 images exceed the job's memory (OOM 2026-10-16). Large sets read from disk;
# 16 workers decode far faster than the ~30 images/s the H100 consumes.
train_images=$(find "$(dirname "$data")/images/train" -name '*.png' | wc -l)
cache=ram
[ "$train_images" -gt 10000 ] && cache=none
echo "[job] $train_images training images -> cache=$cache"
"$TRAIN_ENV/bin/python" "$PROJECT/synth/train_detector.py" \
    --data "$data" --name "$name" --project "$RUNS_DIR/detect" \
    --workers "${SLURM_CPUS_PER_TASK:-8}" --cache "$cache" "$@"
