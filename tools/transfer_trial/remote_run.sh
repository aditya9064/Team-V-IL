#!/bin/bash
# Cosmos Transfer 2.5 trial on a Brev A100 80 GB: install, then photo-real versions
# of ~/trial/in/images/*.png with edge control. Runs unattended under nohup:
#   nohup bash ~/trial/remote_run.sh > ~/trial/run.log 2>&1 &
# Progress markers: STAGE <name>. Output: ~/trial/out/*.png
set -euo pipefail
T="$HOME/trial"
REPO="$HOME/cosmos-transfer2.5"
stage() { echo "STAGE $1 $(date +%H:%M:%S)"; }

# Budget backstop: power off at a fixed UTC time (set per trial) even if nobody stops it.
sudo shutdown -h 21:15 || echo "WARN: could not schedule shutdown"  # 16:15 CDT, ~$5.70 since 13:28

stage system
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq git-lfs ffmpeg libx11-dev python3-pip >/dev/null
python3 -m pip install -q --user uv 2>/dev/null || python3 -m pip install -q --user --break-system-packages uv
export PATH="$HOME/.local/bin:$PATH"

stage clone
git lfs install --skip-repo
[ -d "$REPO" ] || git clone -q --depth 1 https://github.com/nvidia-cosmos/cosmos-transfer2.5.git "$REPO"
cd "$REPO" && git lfs pull

stage env
# The repo pins 3.13, but its cu128 flash-attn wheel is published for cp310 only.
uv python pin 3.10
uv sync --extra=cu128
source .venv/bin/activate

stage specs
python "$T/make_specs.py" "$T/in/images" "$T/specs" "$T/prompts.json"

stage infer
python examples/inference.py -i "$T"/specs/*.json -o "$T/raw_out"

stage collect
python "$T/collect.py" "$T/raw_out" "$T/out"
stage done
