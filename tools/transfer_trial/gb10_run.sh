#!/bin/bash
# Host side on the GB10: run gb10_sweep.sh in the Cosmos Transfer container on the GPU.
#   nohup bash ~/adaptivevision/transfer/gb10_run.sh > ~/adaptivevision/transfer/run.log 2>&1 &
# Env passed through: IN, OUT, CONFIGS, PROMPTS. The HF token is read from
# ~/adaptivevision/transfer/.hf_token (chmod 600) and only handed to the container.
set -euo pipefail
BASE="$HOME/adaptivevision"
TOKEN_FILE="$BASE/transfer/.hf_token"
[ -s "$TOKEN_FILE" ] || { echo "missing $TOKEN_FILE"; exit 1; }
mkdir -p "$BASE/hf_transfer"
docker run --rm --gpus all --ipc=host \
    -v "$BASE/cosmos-transfer2.5:/workspace" \
    -v "$BASE/transfer:/trial" \
    -v "$BASE/hf_transfer:/hf" \
    -e HF_HOME=/hf -e HF_TOKEN="$(cat "$TOKEN_FILE")" \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -e IN -e OUT -e CONFIGS -e PROMPTS -e CHUNK \
    av-transfer:nightly bash "/trial/${SCRIPT:-gb10_sweep.sh}"
