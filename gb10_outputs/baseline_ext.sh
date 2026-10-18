#!/bin/bash
# Extended Cosmos-vs-YOLO baseline (300 disjoint held-out images). Polite on the shared GB10:
# waits while the GPU is busy before starting; results append to baseline_set_ext/results.jsonl.
cd ~/adaptivevision
export PYTHONNOUSERSITE=1 HF_HOME=$PWD/hf YOLO_CONFIG_DIR=$PWD/ultralytics
until [ "$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits | tr -d " ")" -lt 30 ]; do
    echo "GPU busy, waiting $(date +%H:%M)"; sleep 300
done
envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set_ext --detector yolo --weights models/cubes_all.pt --name yolo_v1 | tail -1
envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set_ext --detector cosmos
envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set_ext --summary
echo EXT_DONE
