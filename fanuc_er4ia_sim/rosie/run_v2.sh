#!/bin/bash
# Submit the v2 hard-case dataset pipeline as one chain of Slurm jobs, then log out.
#
#   ./rosie/run_v2.sh
#
#   test2 (T4: 20 frames x table/side/wrist) -> check2 (CPU: cameras that fail are skipped)
#   -> gen2 x 40 chunks (T4) -> refill (T4) -> split2 (CPU: cubes_v2, cubes_v1v2)
#   -> train cubes_v2 + cubes_v1v2 in parallel (H100) -> eval v1/v2 models on easy + hard (H100)
#   -> report2 (CPU) -> ~/adaptivevision/RESULTS_V2.txt
#   + after training: GB10 Cosmos-vs-YOLO baseline -> ~/adaptivevision/BASELINE.txt
#
# Runs alongside v1: separate folders, v2_ file prefix, its own seed.
# Safe to rerun: finished frames and chunks are skipped.
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"
mkdir -p "$LOG_DIR" "$V2_DIR"
JOBS="$PROJECT/rosie/jobs"
[ -f "$ISAAC_SIF" ] || die "Missing $ISAAC_SIF; build the container first (rosie/av.sh pull-isaac)"

main() {
    # Each array task renders V2_GPUS_PER_JOB chunks at once (one per GPU): Rosie caps
    # running jobs, not GPUs. Pinning verified: both T4s at 100%, ~3.5 s/frame each.
    local groups=$(( (V2_CHUNKS + V2_GPUS_PER_JOB - 1) / V2_GPUS_PER_JOB ))
    local array="0-$((groups - 1))%$MAX_PARALLEL" test check gen refill split t_v2 t_mix ev rep gb10 percep
    local multi=(--gres="gpu:t4:$V2_GPUS_PER_JOB" --cpus-per-task=$((8 * V2_GPUS_PER_JOB)) --mem=$((48 * V2_GPUS_PER_JOB))G)
    test=$(sub av-v2test "$ISAAC_PARTITION" "$ISAAC_GRES" 02:00:00 "" "$JOBS/isaac_job.sh" test2)
    check=$(sub av-v2check "$ISAAC_PARTITION" "" 00:30:00 "afterok:$test" "$JOBS/cpu_job.sh" check2)
    gen=$(sub av-v2gen "$ISAAC_PARTITION" "" "$GEN_TIME" "afterok:$check" \
        "${multi[@]}" --array="$array" "$JOBS/isaac_job.sh" gen2m)
    # Rosie kills GPU processes node-wide when any job ends; the refill fills gaps.
    refill=$(sub av-v2gen "$ISAAC_PARTITION" "" "$GEN_TIME" "afterany:$gen" \
        "${multi[@]}" --array="$array" "$JOBS/isaac_job.sh" gen2m)
    split=$(sub av-v2split "$ISAAC_PARTITION" "" 02:00:00 "afterok:$check,afterany:$refill" \
        "$JOBS/cpu_job.sh" split2)
    t_v2=$(sub av-train "$TRAIN_PARTITION" "$TRAIN_GRES" "$TRAIN_TIME" "afterok:$split" \
        "$JOBS/train_job.sh" "$DATASETS_DIR/cubes_v2/data.yaml" cubes_v2)
    t_mix=$(sub av-train "$TRAIN_PARTITION" "$TRAIN_GRES" "$TRAIN_TIME" "afterok:$split" \
        "$JOBS/train_job.sh" "$DATASETS_DIR/cubes_v1v2/data.yaml" cubes_v1v2)
    ev=$(sub av-eval "$TRAIN_PARTITION" "$TRAIN_GRES" 03:00:00 "afterany:$t_v2:$t_mix" "$JOBS/eval_job.sh")
    rep=$(sub av-report "$ISAAC_PARTITION" "" 00:10:00 "afterany:$ev" "$JOBS/cpu_job.sh" report2)
    # Cosmos-vs-YOLO baseline on the GB10 (needs its one-time setup; see gb10_job.sh).
    gb10=$(sub av-gb10 "$ISAAC_PARTITION" "" 04:00:00 "afterany:$t_v2:$t_mix" "$JOBS/gb10_job.sh")
    # B1-B3: YOLO -> table position + cube yaw vs. true poses on held-out frames.
    percep=$(sub av-percep "$ISAAC_PARTITION" "$ISAAC_GRES" 08:00:00 "afterany:$t_v2:$t_mix" \
        "$JOBS/perception_job.sh")
    cat <<EOF

V2 pipeline submitted ($((V2_CHUNKS * V2_CHUNK_SIZE)) frames). You can log out now.
  test $test -> check $check -> gen $gen -> refill $refill -> split $split
  train v2 $t_v2 + v1v2 $t_mix (parallel) -> eval $ev -> report $rep
  GB10 Cosmos-vs-YOLO baseline $gb10 -> $DATA_ROOT/BASELINE.txt
  perception (B1-B3) $percep -> $DATA_ROOT/PERCEPTION.txt
Progress : squeue -u \$USER
Results  : cat $DATA_ROOT/RESULTS_V2.txt
Test previews (look before the full run lands): $V2_DIR/test_{table,side,wrist}/preview
EOF
}

main
