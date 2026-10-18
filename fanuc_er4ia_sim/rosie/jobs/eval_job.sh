#!/bin/bash
# V2 robustness check on an H100: every finished model on v1 (easy) and v2 (hard) val sets.
#SBATCH --job-name=av-eval
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
set -euo pipefail
source "$PWD/rosie/common.sh"
export YOLO_CONFIG_DIR="$DATA_ROOT/ultralytics"

latest() {  # newest best.pt for a run-name prefix
    ls -td "$RUNS_DIR"/detect/"$1"_[0-9]*/weights/best.pt 2>/dev/null | head -1
}
models=()
for name in cubes_all cubes_v2 cubes_v1v2; do
    w=$(latest "$name")
    [ -n "$w" ] && models+=(--model "${name}=$w") || echo "[eval] no finished $name model"
done
[ ${#models[@]} -gt 0 ] || die "no trained models to evaluate"
"$TRAIN_ENV/bin/python" "$PROJECT/synth/eval_detector.py" "${models[@]}" \
    --data "v1_easy=$DATASETS_DIR/cubes_all/data.yaml" \
    --data "v2_hard=$DATASETS_DIR/cubes_v2/data.yaml" \
    --out "$RUNS_DIR/eval_v2"
