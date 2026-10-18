#!/bin/bash
# One-off smoke test of the training settings (1 epoch, 200 frames). Not part of RESULTS.
set -euo pipefail
source "$PWD/rosie/common.sh"
export YOLO_CONFIG_DIR="$DATA_ROOT/ultralytics"
cd "$DATA_ROOT/weights"
"$TRAIN_ENV/bin/python" "$PROJECT/synth/train_detector.py" --data "$DATASETS_DIR/smoke_200/data.yaml" \
    --name smoke --project "$DATA_ROOT/smoke_runs" --epochs 1 --workers "${SLURM_CPUS_PER_TASK:-16}"
