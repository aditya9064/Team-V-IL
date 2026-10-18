#!/bin/bash
# CPU pipeline stages (no GPU). Submitted by rosie/run_all.sh.
#   cpu_job.sh setup            Python envs, textures, tests, weights (self-repairing)
#   cpu_job.sh check            label-alignment gate on the test chunk
#   cpu_job.sh split            datasets: cubes_1000, cubes_5000, cubes_all
#   cpu_job.sh report           collect every training summary into RESULTS.txt
#   cpu_job.sh check2           v2 label check per camera -> synth_v2/passed_cameras
#   cpu_job.sh split2           datasets: cubes_v2 and cubes_v1v2 (v1 + v2 combined)
#   cpu_job.sh report2          v1-vs-v2 robustness table -> RESULTS_V2.txt
#SBATCH --job-name=av-cpu
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
set -euo pipefail
source "$PWD/rosie/common.sh"
echo "[job] $(hostname)  job ${SLURM_JOB_ID:-local}  stage ${1:-}"

split_all() {
    local frames
    frames=$(find "$SYNTH_DIR" -path '*/labels/*.txt' | wc -l)
    echo "[split] $frames labelled frames"
    [ "$frames" -ge "$MIN_FRAMES" ] || die "Only $frames frames (< MIN_FRAMES=$MIN_FRAMES); rerun run_all.sh to resume generation"
    for n in 1000 5000; do
        [ "$frames" -gt "$n" ] && "$PROJECT/rosie/av.sh" split "$n"
    done
    "$PROJECT/rosie/av.sh" split
}

check_v2() {
    : > "$V2_DIR/passed_cameras"
    for cam in table side wrist; do
        local dir="$V2_DIR/test_$cam" min=20
        [ "$cam" = table ] || min=8  # side/wrist frames show fewer clearly visible cubes
        "$HOST_ENV/bin/python" "$PROJECT/synth/preview_labels.py" "$dir" --limit 20 || true
        if "$HOST_ENV/bin/python" "$PROJECT/synth/check_labels.py" "$dir" --min-cubes "$min"; then
            echo "$cam" >> "$V2_DIR/passed_cameras"
        else
            echo "[check2] $cam FAILED: its chunks will be skipped"
        fi
    done
    grep -qx table "$V2_DIR/passed_cameras" || die "table camera failed the v2 label check"
}

split_v2() {
    local frames
    frames=$(find "$V2_DIR" -path '*/chunk_*/labels/*.txt' | wc -l)
    echo "[split2] $frames v2 frames"
    [ "$frames" -ge "$MIN_FRAMES" ] || die "Only $frames v2 frames (< MIN_FRAMES=$MIN_FRAMES)"
    "$HOST_ENV/bin/python" "$PROJECT/synth/split_dataset.py" "$V2_DIR"/chunk_* --out "$DATASETS_DIR/cubes_v2"
    "$HOST_ENV/bin/python" "$PROJECT/synth/split_dataset.py" "$SYNTH_DIR"/chunk_* "$V2_DIR"/chunk_* \
        --out "$DATASETS_DIR/cubes_v1v2"
}

case "${1:-}" in
    setup) "$PROJECT/rosie/av.sh" setup ;;
    check)
        "$HOST_ENV/bin/python" "$PROJECT/synth/preview_labels.py" "$SYNTH_DIR/chunk_0000" --limit 20
        "$HOST_ENV/bin/python" "$PROJECT/synth/check_labels.py" "$SYNTH_DIR/chunk_0000"
        ;;
    split) split_all ;;
    report)
        # RESULTS.txt marks the pipeline as finished; write it only for real results.
        "$HOST_ENV/bin/python" "$PROJECT/synth/report_runs.py" "$RUNS_DIR/detect" > "$DATA_ROOT/RESULTS.tmp"
        mv "$DATA_ROOT/RESULTS.tmp" "$DATA_ROOT/RESULTS.txt"
        cat "$DATA_ROOT/RESULTS.txt"
        ;;
    check2) check_v2 ;;
    split2) split_v2 ;;
    report2)
        { "$HOST_ENV/bin/python" "$PROJECT/synth/report_runs.py" "$RUNS_DIR/detect"; echo;
          cat "$RUNS_DIR/eval_v2.txt" 2>/dev/null; } > "$DATA_ROOT/RESULTS_V2.tmp"
        mv "$DATA_ROOT/RESULTS_V2.tmp" "$DATA_ROOT/RESULTS_V2.txt"
        cat "$DATA_ROOT/RESULTS_V2.txt"
        ;;
    *) die "usage: cpu_job.sh setup | check | split | report | check2 | split2 | report2" ;;
esac
