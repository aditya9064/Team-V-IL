#!/bin/bash
# One-screen live status of everything running for AdaptiveVision (Rosie + GB10).
#   bash ~/fanuc_er4ia_sim/rosie/status_all.sh        (or just: avstatus)
# Read-only: it only lists jobs and reads logs.
source "$(cd "$(dirname "$0")" && pwd)/common.sh"
SSH=(-i "$GB10_KEY" -o BatchMode=yes -o ConnectTimeout=15)

last_epoch() {  # "<epoch>/<total>  mAP50-95 <x>" for the newest run of a name
    local d
    d=$(ls -td "$RUNS_DIR"/detect/"$1"_[0-9]* 2>/dev/null | head -1)
    [ -f "$d/results.csv" ] || { echo "not started"; return; }
    tail -1 "$d/results.csv" | awk -F, '{printf "epoch %d  mAP50-95 %.3f", $1, $9}'
}

echo "=== AdaptiveVision status  $(date '+%a %H:%M') ==="
echo
echo "--- Rosie jobs (yours) ---"
squeue -u "$USER" -o "%.8i %.10j %.9T %.10M %R" 2>/dev/null
echo
echo "--- YOLO training ---"
echo "cubes_v2   : $(last_epoch cubes_v2)"
echo "cubes_v1v2 : $(last_epoch cubes_v1v2)"
echo
echo "--- Results files ---"
for f in RESULTS.txt RESULTS_V2.txt BASELINE.txt PERCEPTION.txt; do
    if [ -f "$DATA_ROOT/$f" ]; then echo "READY   $DATA_ROOT/$f"; else echo "waiting $f"; fi
done
echo
echo "--- GB10 (spark1) ---"
ssh "${SSH[@]}" "$GB10_HOST" '
    B=~/adaptivevision
    echo "GPU use: $(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader)"
    echo "Cosmos vs YOLO baseline: $(grep -c . $B/baseline_set/results.jsonl 2>/dev/null) scored rows"
    echo "Extended baseline (300 imgs): cosmos $(grep -c "\"detector\": \"cosmos\"" $B/baseline_set_ext/results.jsonl 2>/dev/null)/300"
    L=$B/transfer/run.log
    if [ -f $L ]; then
        echo "Cosmos Transfer sweep: $(grep -E "^STAGE" $L | tail -1)"
        for d in $B/transfer/sweep_out/*/; do
            [ -d "$d" ] && [ "$(basename $d)" != specs ] && \
                echo "  $(basename $d): $(ls $d*.jpg 2>/dev/null | grep -vc control) images"
        done
        grep -E "FAILED|Traceback" $L | tail -2
    fi' 2>/dev/null || echo "(GB10 not reachable right now)"
echo
echo "Task list: cat $DATA_ROOT/TASKS.md"
