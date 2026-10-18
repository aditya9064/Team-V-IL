#!/bin/bash
# Isaac Sim GPU job. Submitted by rosie/av.sh, which sets partition/GPU/logs.
#   isaac_job.sh twin               step 3: render every camera once
#   isaac_job.sh gen <count>        steps 4-5: one chunk (array task = chunk id)
#   isaac_job.sh test2              v2: V2_TEST_COUNT frames per camera for the label check
#   isaac_job.sh gen2               v2: one chunk; camera from the chunk index
#   isaac_job.sh gen2m              v2: V2_GPUS_PER_JOB chunks in parallel, one per GPU
#   isaac_job.sh pintest            v2: 3 frames on each GPU of the job (GPU pinning check)
#SBATCH --job-name=av-isaac
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
set -euo pipefail
# Slurm runs a spooled copy of this file; --chdir puts us at the project root.
source "$PWD/rosie/common.sh"

echo "[job] $(hostname)  job ${SLURM_JOB_ID:-local}  task ${SLURM_ARRAY_TASK_ID:-0}"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv

count_files() {
    # count_files <dir> <ext>: 0 for a missing dir (find would fail the pipeline under pipefail)
    [ -d "$1" ] || { echo 0; return 0; }
    find "$1" -name "*.$2" | wc -l
}

v2_chunk() {
    # v2_chunk <chunk> <gpu>: render one v2 chunk on one GPU of this job (skips if done).
    local chunk=$1 gpu=$2 cam out have n
    cam="$(v2_camera_of "$chunk")"
    grep -qx "$cam" "$V2_DIR/passed_cameras" 2>/dev/null || { echo "$cam failed the label check; skip"; return 0; }
    out="$V2_DIR/$(printf 'chunk_%04d' "$chunk")"
    have=$(count_files "$out/labels" txt)
    [ "$have" -ge "$V2_CHUNK_SIZE" ] && { echo "$out complete"; return 0; }
    isaac_exec "$PROJECT/synth/generate_v2.py" --out "$out" --camera "$cam" --gpu "$gpu" \
        --start "$((chunk * V2_CHUNK_SIZE))" --count "$V2_CHUNK_SIZE" --seed "$V2_SEED"
    n=$(count_files "$out/images" png)
    echo "[job] chunk $chunk ($cam, gpu $gpu): $n images"
    [ "$n" -ge "$V2_CHUNK_SIZE" ]
}

case "${1:-}" in
    twin)
        out="$RUNS_DIR/twin_${SLURM_JOB_ID:-local}"
        isaac_exec "$PROJECT/build_scene.py" --headless --shoot "$out"
        ls -R "$out" | head -20
        # Isaac can exit 0 after an exception; require real images.
        find "$out" -name '*.png' | grep -q . || die "twin rendered no images; see the log above"
        ;;
    gen)
        count="${2:?usage: isaac_job.sh gen <count>}"
        chunk="${SLURM_ARRAY_TASK_ID:-0}"
        out="$SYNTH_DIR/$(printf 'chunk_%04d' "$chunk")"
        have=$(count_files "$out/labels" txt)
        if [ "$have" -ge "$count" ]; then
            echo "[job] $out already has $have/$count frames; nothing to do"; exit 0
        fi
        isaac_exec "$PROJECT/synth/generate_synthetic.py" \
            --out "$out" --start "$((chunk * CHUNK_SIZE))" --count "$count" --seed "$SEED"
        n=$(count_files "$out/images" png)
        echo "[job] images in $out: $n"
        [ "$n" -ge "$count" ] || die "expected $count images, found $n"
        ;;
    test2)
        for cam in table side wrist; do
            out="$V2_DIR/test_$cam"
            isaac_exec "$PROJECT/synth/generate_v2.py" --out "$out" --camera "$cam" \
                --start 0 --count "$V2_TEST_COUNT" --seed "$((V2_SEED + 7))" --prefix v2t_ || true
            echo "[job] $cam test frames: $(count_files "$out/images" png)"
        done
        ;;
    gen2)
        chunk="${SLURM_ARRAY_TASK_ID:-0}"
        cam="$(v2_camera_of "$chunk")"
        if ! grep -qx "$cam" "$V2_DIR/passed_cameras" 2>/dev/null; then
            echo "[job] $cam failed the v2 label check; chunk $chunk skipped"; exit 0
        fi
        out="$V2_DIR/$(printf 'chunk_%04d' "$chunk")"
        have=$(count_files "$out/labels" txt)
        [ "$have" -ge "$V2_CHUNK_SIZE" ] && { echo "[job] $out complete"; exit 0; }
        isaac_exec "$PROJECT/synth/generate_v2.py" --out "$out" --camera "$cam" \
            --start "$((chunk * V2_CHUNK_SIZE))" --count "$V2_CHUNK_SIZE" --seed "$V2_SEED"
        n=$(count_files "$out/images" png)
        echo "[job] $cam images in $out: $n"
        [ "$n" -ge "$V2_CHUNK_SIZE" ] || die "expected $V2_CHUNK_SIZE images, found $n"
        ;;
    gen2m)
        group="${SLURM_ARRAY_TASK_ID:-0}" pids=() status=0
        for k in $(seq 0 $((V2_GPUS_PER_JOB - 1))); do
            chunk=$((group * V2_GPUS_PER_JOB + k))
            [ "$chunk" -lt "$V2_CHUNKS" ] || continue
            v2_chunk "$chunk" "$k" > "$LOG_DIR/av-v2gen-chunk$(printf %04d "$chunk").log" 2>&1 &
            pids+=($!)
        done
        for pid in "${pids[@]}"; do wait "$pid" || status=1; done
        exit "$status"
        ;;
    pintest)
        for k in $(seq 0 $((V2_GPUS_PER_JOB - 1))); do
            isaac_exec "$PROJECT/synth/generate_v2.py" --out "$V2_DIR/pintest_gpu$k" --camera table \
                --start 0 --count 3 --seed "$((V2_SEED + 11))" --prefix v2p_ --gpu "$k" \
                > "$LOG_DIR/av-pintest-gpu$k.log" 2>&1 &
        done
        sleep 240; nvidia-smi --query-compute-apps=gpu_bus_id,pid,used_memory --format=csv
        wait
        ;;
    *)
        die "usage: isaac_job.sh twin | gen <count> | test2 | gen2 | gen2m | pintest"
        ;;
esac
