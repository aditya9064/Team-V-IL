#!/bin/bash
# C3 baseline: full starter task (cube tower) in the digital twin with YOLO as the eyes.
# One episode per cube-layout seed: capture -> perceive_yolo -> plan_sort -> sort_scene.
#   sbatch ... rosie/jobs/sim_job.sh <weights | run name> <first seed> <count> [--render]
# Results: $RUNS_DIR/sim/<model>/seed_<n>/ and $DATA_ROOT/SIM_RESULTS.txt
# --render episodes write ~1,650 full-HD frames per camera and then encode two videos:
# on a T4 that is ~2 h render + ~25 min encode PER episode, so submit them with --time >= 04:00:00
# (a 2 h limit killed the first one mid-encode; frames survived and were re-encoded).
#SBATCH --job-name=av-sim
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
set -euo pipefail
source "$PWD/rosie/common.sh"
export YOLO_CONFIG_DIR="$DATA_ROOT/ultralytics"
export AV_RENDER_DENOISE=median3  # T4 has no DLSS: same speckle filter as the training renders
WEIGHTS="$1" FIRST="$2" COUNT="$3" RENDER="${4:-}"
if [ ! -f "$WEIGHTS" ]; then  # a run name (e.g. cubes_v2): newest finished checkpoint at start time
    WEIGHTS=$(ls -td "$RUNS_DIR"/detect/"$WEIGHTS"_[0-9]*/weights/best.pt 2>/dev/null | head -1)
    [ -n "$WEIGHTS" ] || die "no trained model for $1"
fi
MODEL="$(basename "$(dirname "$(dirname "$WEIGHTS")")")"
OUT="$RUNS_DIR/sim/$MODEL"
SORT_FLAGS=(--physics-only)
if [ "$RENDER" = "--render" ]; then  # video episodes live apart from the measured ones
    OUT="$RUNS_DIR/sim_video/$MODEL"
    SORT_FLAGS=()
fi
mkdir -p "$OUT"
echo "[sim] $(hostname) job ${SLURM_JOB_ID:-local} model $MODEL seeds $FIRST..$((FIRST + COUNT - 1))"

episode() {  # episode <seed>: one full capture -> execute run; keeps going on failure
    local seed=$1 run="$OUT/seed_$1" t0
    [ -f "$run/execution_report.json" ] && { echo "[sim] seed $seed done"; return 0; }
    mkdir -p "$run"
    export AV_LAYOUT_SEED="$seed"
    t0=$(date +%s)
    # Isaac can exit 0 after an exception: require a camera image written by THIS capture.
    # The first Isaac launch on a cold node can return no camera image (shader warm-up);
    # a second launch on the same node works, so allow one retry.
    for attempt in 1 2; do
        isaac_exec "$PROJECT/capture_scene.py" --out "$run" || true
        if [ -f "$run/world_camera.png" ] && [ "$(stat -c %Y "$run/world_camera.png")" -ge "$t0" ]; then
            break
        fi
        echo "[sim] seed $seed capture attempt $attempt gave no fresh image"
        [ "$attempt" = 2 ] && { echo "[sim] seed $seed capture failed"; return 0; }
    done
    (cd "$PROJECT" && "$TRAIN_ENV/bin/python" perceive_yolo.py --run "$run" --weights "$WEIGHTS") \
        || { echo "[sim] seed $seed perceive failed"; return 0; }
    (cd "$PROJECT" && "$HOST_ENV/bin/python" plan_sort.py --run "$run") \
        || { echo "[sim] seed $seed plan failed"; return 0; }
    isaac_exec "$PROJECT/sort_scene.py" --run "$run" ${SORT_FLAGS[@]+"${SORT_FLAGS[@]}"} \
        || echo "[sim] seed $seed execution failed"
    for view in frames side_frames; do  # rendered episodes: one mp4 per camera
        if [ -d "$run/$view" ] && ls "$run/$view"/*.png >/dev/null 2>&1; then
            (cd "$PROJECT" && "$HOST_ENV/bin/python" encode_video.py --frames "$run/$view" \
                --out "$run/${view%frames}video.mp4" --fps 30) || echo "[sim] seed $seed $view encode failed"
        fi
    done
    echo "$(( $(date +%s) - t0 ))" > "$run/wall_seconds.txt"
}

for seed in $(seq "$FIRST" $((FIRST + COUNT - 1))); do
    episode "$seed" 2>&1 | sed "s/^/[seed $seed] /"
done
(cd "$PROJECT" && "$HOST_ENV/bin/python" -m synth.sim_summary "$RUNS_DIR/sim") | tee "$DATA_ROOT/SIM_RESULTS.txt"
