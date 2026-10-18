#!/bin/bash
# Overnight Transfer batch #2 on the shared GB10: waits politely for a free GPU, then runs.
cd ~/adaptivevision/transfer
until [ "$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits | tr -d " ")" -lt 30 ]; do
    echo "GPU busy, waiting $(date +%H:%M)"; sleep 300
done
SCRIPT=gb10_batch.sh IN=/trial/batch2_in OUT=/trial/batch2_out CHUNK=100 bash gb10_run.sh
echo BATCH2_DONE
