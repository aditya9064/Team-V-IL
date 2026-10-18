#!/bin/bash
# Submit the whole pipeline (steps 2-7) as one chain of Slurm jobs, then log out.
#
#   ./rosie/run_all.sh
#
#   setup (CPU) ─┐
#   pull  (CPU) ─┴─> twin (T4) -> gen-test (T4) -> check (CPU) -> gen x N (T4)
#                -> split (CPU) -> train all / 1k / 5k in parallel (3 H100s) -> report (CPU)
#
# Each stage starts only when the one before it succeeds. If the label check
# fails, nothing after it runs and no GPU hours are wasted. Safe to rerun:
# finished images are kept and generation resumes where it stopped.
# A watchdog job (av-watch) resumes the chain after transient failures.
# Results: ~/adaptivevision/RESULTS.txt   Logs: ~/adaptivevision/logs/
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"
mkdir -p "$LOG_DIR"
JOBS="$PROJECT/rosie/jobs"

clear_old_jobs() {
    # Pending stages from earlier manual submits would duplicate this chain.
    local running pending
    running=$(squeue -u "$USER" -h -t R -o "%i %j" | grep " av-" | grep -v -E " av-(pull|watch)$" || true)
    [ -z "$running" ] || die "Pipeline jobs already running:
$running
Wait for them, or cancel with scancel <id>, then rerun."
    pending=$(squeue -u "$USER" -h -t PD -o "%i %j" | grep " av-" | grep -v -E " av-(pull|watch)$" || true)
    if [ -n "$pending" ]; then
        echo "Cancelling earlier pending pipeline jobs:"; echo "$pending"
        # Array jobs show as 123_[0-19%4]; scancel needs the base id.
        echo "$pending" | awk '{sub(/_.*/, "", $1); print $1}' | sort -u | xargs scancel
    fi
}

pull_dependency() {
    # Prints "afterok:<id>" when the container still has to be built, else nothing.
    local id
    [ -f "$ISAAC_SIF" ] && return 0
    id=$(squeue -u "$USER" -h -n av-pull -o %i | head -1)
    if [ -z "$id" ]; then
        [ -n "${NGC_API_KEY:-}" ] || die "No Isaac container and no pull running. First: export NGC_API_KEY=<key>"
        id=$(sub av-pull "$ISAAC_PARTITION" "" 03:00:00 "" --export=ALL "$JOBS/pull_job.sh")
    fi
    echo "afterok:$id"
}

main() {
    clear_old_jobs
    local pull setup twin test check gen refill split t_all t_1k t_5k report
    pull=$(pull_dependency)
    setup=$(sub av-setup "$ISAAC_PARTITION" "" 02:00:00 "" "$JOBS/cpu_job.sh" setup)
    twin=$(sub av-twin "$ISAAC_PARTITION" "$ISAAC_GRES" 00:45:00 \
        "afterok:$setup${pull:+:${pull#afterok:}}" "$JOBS/isaac_job.sh" twin)
    test=$(sub av-gentest "$ISAAC_PARTITION" "$ISAAC_GRES" 01:00:00 "afterok:$twin" "$JOBS/isaac_job.sh" gen 20)
    check=$(sub av-check "$ISAAC_PARTITION" "" 00:20:00 "afterok:$test" "$JOBS/cpu_job.sh" check)
    gen=$(sub av-gen "$ISAAC_PARTITION" "$ISAAC_GRES" "$GEN_TIME" "afterok:$check" \
        --array="0-$((NUM_CHUNKS - 1))%$MAX_PARALLEL" "$JOBS/isaac_job.sh" gen "$CHUNK_SIZE")
    # Rosie's job-end cleanup can SIGKILL GPU processes of other jobs on the same node,
    # so a second pass re-runs only incomplete chunks (complete ones exit in seconds).
    # Two passes: Rosie kills every GPU process on a node when any job there ends,
    # including refill tasks. Each pass only renders what is still missing.
    refill=$(sub av-gen "$ISAAC_PARTITION" "$ISAAC_GRES" "$GEN_TIME" "afterany:$gen" \
        --array="0-$((NUM_CHUNKS - 1))%$MAX_PARALLEL" "$JOBS/isaac_job.sh" gen "$CHUNK_SIZE")
    refill=$(sub av-gen "$ISAAC_PARTITION" "$ISAAC_GRES" "$GEN_TIME" "afterany:$refill" \
        --array="0-$((NUM_CHUNKS - 1))%$MAX_PARALLEL" "$JOBS/isaac_job.sh" gen "$CHUNK_SIZE")
    # afterany: a chunk that timed out still leaves usable frames; split enforces MIN_FRAMES.
    # Comma = AND: the check must have passed, and every refill task must have ended.
    split=$(sub av-split "$ISAAC_PARTITION" "" 01:00:00 "afterok:$check,afterany:$refill" "$JOBS/cpu_job.sh" split)
    t_all=$(sub av-train "$TRAIN_PARTITION" "$TRAIN_GRES" "$TRAIN_TIME" "afterok:$split" \
        "$JOBS/train_job.sh" "$DATASETS_DIR/cubes_all/data.yaml" cubes_all)
    t_1k=$(sub av-train "$TRAIN_PARTITION" "$TRAIN_GRES" "$TRAIN_TIME" "afterok:$split" \
        "$JOBS/train_job.sh" "$DATASETS_DIR/cubes_1000/data.yaml" cubes_1000)
    t_5k=$(sub av-train "$TRAIN_PARTITION" "$TRAIN_GRES" "$TRAIN_TIME" "afterok:$split" \
        "$JOBS/train_job.sh" "$DATASETS_DIR/cubes_5000/data.yaml" cubes_5000)
    report=$(sub av-report "$ISAAC_PARTITION" "" 00:10:00 "afterok:$split,afterany:$t_all:$t_1k:$t_5k" "$JOBS/cpu_job.sh" report)
    echo "FIRST_JOB=$setup" > "$DATA_ROOT/pipeline.env"
    local watch="already running"
    if [ -z "${WATCHDOG_CHILD:-}" ]; then
        rm -f "$DATA_ROOT/watchdog_retries" "$DATA_ROOT/WATCHDOG.txt"
        squeue -u "$USER" -h -n av-watch -o %i | xargs -r scancel
        watch=$(submit_watchdog)
    fi
    cat <<EOF

Pipeline submitted. You can log out now.
  container : ${pull:-ready}
  setup $setup -> twin $twin -> test $test -> check $check -> gen $gen ($NUM_CHUNKS x $CHUNK_SIZE) -> refill $refill
  split $split -> train all $t_all -> 1k $t_1k -> 5k $t_5k -> report $report
  watchdog  : $watch (resumes after timeouts/node failures; emails + writes WATCHDOG.txt on real errors)

Progress : squeue -u \$USER       (PD with "Dependency" = waiting its turn)
Results  : cat $DATA_ROOT/RESULTS.txt
If a stage fails, later stages are cancelled automatically. Find it with:
  sacct -u \$USER -S today --format=JobID,JobName%12,State,Elapsed
  tail -40 $LOG_DIR/<jobname>-<jobid>_*.out
EOF
}

main
