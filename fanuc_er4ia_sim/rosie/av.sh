#!/bin/bash
# AdaptiveVision on ROSIE: run single stages by hand (the full automatic chain is rosie/run_all.sh).
#
#   ./rosie/av.sh check           what Rosie offers (partitions, GPUs, storage, containers)
#   ./rosie/av.sh setup           Python envs, textures, host tests, YOLO weights
#   ./rosie/av.sh pull-isaac      Isaac Sim container (needs NGC_API_KEY in the environment)
#   ./rosie/av.sh twin            step 3  render the twin once            (GPU job)
#   ./rosie/av.sh gen-test        step 4  20 frames, measures speed        (GPU job)
#   ./rosie/av.sh preview         step 5  draw labels on the test frames
#   ./rosie/av.sh gen             step 4  full dataset as a job array      (GPU jobs)
#   ./rosie/av.sh split [frames]  step 6  train/val split, optional subset size
#   ./rosie/av.sh train <dataset-name> [extra train args]   step 7       (GPU job)
#   ./rosie/av.sh status          queue, frame counts, disk use
# Prefix any job command with AFTER=<jobid> to start it when that job succeeds.
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

submit() {
    # submit <partition> <gres> <constraint> <time> <script> [args...]
    local partition="$1" gres="$2" constraint="$3" time="$4"
    shift 4
    mkdir -p "$LOG_DIR" || die "Cannot write $LOG_DIR; fix DATA_ROOT in rosie/config.env"
    local opts=() o
    while IFS= read -r o; do opts+=("$o"); done < <(slurm_opts "$partition" "$gres" "$constraint")
    # AFTER=<jobid> ./rosie/av.sh <cmd> waits for that job to finish successfully.
    [ -n "${AFTER:-}" ] && opts+=(--dependency="afterok:$AFTER")
    sbatch "${opts[@]}" --time="$time" --output="$LOG_DIR/%x-%A_%a.out" "$@"
}

cmd_check() {
    echo "== OS / user";        uname -a; echo "user $USER  home $HOME"
    echo "== partitions and GPUs (ISAAC_* / TRAIN_* in config.env)"
    sinfo -o "%P %a %l %D %G %f" 2>/dev/null || echo "sinfo unavailable"
    echo "== containers";       container_cmd || true
    echo "== modules";          (module avail 2>&1 | grep -iE "apptainer|singularity|cuda|conda|python" | head -20) || true
    echo "== conda";            command -v conda || echo "conda not on PATH"
    echo "== storage (DATA_ROOT=$DATA_ROOT)"
    df -h "$HOME" 2>/dev/null || true
    (quota -s 2>/dev/null || true)
    echo "== candidate large-storage dirs (writable ones marked)"
    for d in /data /scratch /work /projects /mnt/* "/data/$USER" "/scratch/$USER"; do
        [ -d "$d" ] || continue
        if [ -w "$d" ]; then w="WRITABLE"; else w="read-only"; fi
        echo "$w  $(df -h --output=size,avail "$d" 2>/dev/null | tail -1)  $d"
    done
    echo "== GPU driver: run './rosie/av.sh twin' or check a job log; login nodes may have no GPU"
}

ensure_env() {
    # ensure_env <prefix> <import check> <pip args...>: create, install, verify, repair.
    local env="$1" check="$2"
    shift 2
    [ -x "$env/bin/python" ] || conda create -y -p "$env" "python=$PYTHON_VERSION"
    "$env/bin/python" -m pip install "$@"
    if ! "$env/bin/python" -c "$check"; then
        echo "[setup] $env broken (e.g. deps resolved from ~/.local); force-reinstalling"
        "$env/bin/python" -m pip install --force-reinstall "$@"
        "$env/bin/python" -c "$check" || die "$env still fails: $check"
    fi
}

cmd_setup() {
    command -v conda >/dev/null || die "conda not found; try 'module load anaconda' and rerun"
    mkdir -p "$DATA_ROOT" || die "Cannot create $DATA_ROOT; pick a writable dir from 'av.sh check'"
    mkdir -p "$DATA_ROOT"/{envs,weights,ultralytics} "$SYNTH_DIR" "$DATASETS_DIR" "$RUNS_DIR"
    ensure_env "$HOST_ENV" "import numpy, scipy, cv2" -r "$PROJECT/requirements.txt"
    "$HOST_ENV/bin/python" "$PROJECT/make_textures.py"
    (cd "$PROJECT" && "$HOST_ENV/bin/python" -m unittest test_sort_planning test_cube_perception \
        test_cube_pose test_runtime_config test_perceive_yolo test_real_photo_test synth.test_synth)
    ensure_env "$TRAIN_ENV" "import torch, torchvision, ultralytics" \
        torch torchvision -r "$PROJECT/synth/requirements-train.txt"
    # Cache weights and the plot font now; compute nodes may have no internet.
    (cd "$DATA_ROOT/weights" && YOLO_CONFIG_DIR="$DATA_ROOT/ultralytics" "$TRAIN_ENV/bin/python" -c \
        "from ultralytics import YOLO; from ultralytics.utils.checks import check_font; YOLO('yolo11m.pt'); YOLO('yolo11n.pt'); check_font('Arial.ttf')")
    echo "setup done: host env $HOST_ENV, train env $TRAIN_ENV"
}

cmd_pull_isaac() {
    : "${NGC_API_KEY:?export NGC_API_KEY=<key from ngc.nvidia.com> first (never commit it)}"
    # Runs as a CPU job: survives a dropped browser shell. sbatch exports the
    # current environment (including the key) to the job; nothing is written to disk.
    submit "$ISAAC_PARTITION" "" "" "03:00:00" --export=ALL "$PROJECT"/rosie/jobs/pull_job.sh
    echo "Follow progress: tail -f $LOG_DIR/av-pull-*.out   (Ctrl+C stops watching, not the job)"
}

require_container() {
    # Fail at submit time, not after queueing; a pending pull (AFTER=) builds it later.
    [ -f "$ISAAC_SIF" ] || [ -n "${AFTER:-}" ] || \
        die "Missing $ISAAC_SIF. Wait for pull-isaac, or run: AFTER=<pull-job-id> $0 $1"
}

cmd_preview() {
    local chunk="$SYNTH_DIR/chunk_0000"
    "$HOST_ENV/bin/python" "$PROJECT/synth/preview_labels.py" "$chunk" --limit 20
    echo "Open $chunk/preview in the ROSIE Files app and check every box sits on its object."
    grep -h "labels," "$LOG_DIR"/av-isaac-*.out 2>/dev/null | tail -5 || true
}

cmd_split() {
    local frames="${1:-}" name="cubes_all" extra=()
    if [ -n "$frames" ]; then name="cubes_${frames}"; extra=(--max-frames "$frames"); fi
    "$HOST_ENV/bin/python" "$PROJECT/synth/split_dataset.py" "$SYNTH_DIR"/chunk_* \
        --out "$DATASETS_DIR/$name" ${extra[@]+"${extra[@]}"}
}

cmd_status() {
    squeue -u "$USER" || true
    for c in "$SYNTH_DIR"/chunk_*; do
        [ -d "$c/images" ] && echo "$(basename "$c"): $(find "$c/images" -name '*.png' | wc -l) frames"
    done
    du -sh "$SYNTH_DIR" "$DATASETS_DIR" "$RUNS_DIR" 2>/dev/null || true
}

case "${1:-}" in
    check) cmd_check ;;
    setup) cmd_setup ;;
    pull-isaac) cmd_pull_isaac ;;
    twin) require_container "twin"; submit "$ISAAC_PARTITION" "$ISAAC_GRES" "$ISAAC_CONSTRAINT" "00:45:00" "$PROJECT"/rosie/jobs/isaac_job.sh twin ;;
    gen-test) require_container "gen-test"; submit "$ISAAC_PARTITION" "$ISAAC_GRES" "$ISAAC_CONSTRAINT" "01:00:00" "$PROJECT"/rosie/jobs/isaac_job.sh gen 20 ;;
    preview) cmd_preview ;;
    gen) require_container "gen"; submit "$ISAAC_PARTITION" "$ISAAC_GRES" "$ISAAC_CONSTRAINT" "$GEN_TIME" \
        --array="0-$((NUM_CHUNKS - 1))%$MAX_PARALLEL" "$PROJECT"/rosie/jobs/isaac_job.sh gen "$CHUNK_SIZE" ;;
    split) cmd_split "${2:-}" ;;
    train)
        name="${2:?usage: av.sh train <dataset-name> [extra args]}"
        submit "$TRAIN_PARTITION" "$TRAIN_GRES" "$TRAIN_CONSTRAINT" "$TRAIN_TIME" "$PROJECT"/rosie/jobs/train_job.sh \
            "$DATASETS_DIR/$name/data.yaml" "$name" "${@:3}" ;;
    status) cmd_status ;;
    *) sed -n '2,14p' "$0"; exit 1 ;;
esac
