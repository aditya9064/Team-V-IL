#!/bin/bash
# Mac side: wait for the Rosie shard + the Brev instance, upload, start batch_run.sh.
#   launch_batch.sh <instance> <shard> <shutdown HH:MM UTC>
# The HF token goes straight from ~/.cache/huggingface/token to the instance (never printed).
set -euo pipefail
INST="$1" SHARD="$2" SHUTDOWN_UTC="$3"
HERE="$(cd "$(dirname "$0")" && pwd)"
STAGE_DIR="/tmp/av_${SHARD}_$$"
SSH=(-o BatchMode=yes -o ConnectTimeout=30 -o StrictHostKeyChecking=accept-new)

until ssh "${SSH[@]}" rosie '! squeue -h -n av-tbatch -u "$USER" | grep -q . && test -d ~/adaptivevision/transfer_batch/'"$SHARD"'/train'; do
    sleep 30
done
echo "[launch] $SHARD ready on Rosie $(date +%H:%M:%S)"
until brev ls 2>/dev/null | grep -E "^ *$INST " | grep -q "RUNNING.*COMPLETED.* READY"; do sleep 20; done
brev refresh >/dev/null 2>&1 || true
echo "[launch] $INST ready $(date +%H:%M:%S)"

mkdir -p "$STAGE_DIR"
ssh "${SSH[@]}" rosie "tar cf - -C ~/adaptivevision/transfer_batch/$SHARD ." | tar xf - -C "$STAGE_DIR"
cp "$HERE"/{batch_run.sh,sweep_specs.py,make_specs.py,transfer_qc.py,prompts_neutral.json} "$STAGE_DIR"/
tar czf - -C "$STAGE_DIR" . | ssh "${SSH[@]}" "$INST" 'mkdir -p ~/batch && tar xzf - -C ~/batch'
rm -rf "$STAGE_DIR"
ssh "${SSH[@]}" "$INST" 'mkdir -p ~/.cache/huggingface && chmod 700 ~/.cache/huggingface && umask 077 && cat > ~/.cache/huggingface/token' \
    < "$HOME/.cache/huggingface/token"
ssh "${SSH[@]}" "$INST" "cd ~/batch && (nohup bash batch_run.sh $SHUTDOWN_UTC > run.log 2>&1 < /dev/null &); sleep 15; head -3 run.log; ls train/images | wc -l; ls heldout/images | wc -l"
echo "[launch] $INST started $(date +%H:%M:%S)"
