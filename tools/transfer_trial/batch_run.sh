#!/bin/bash
# Cosmos Transfer batch on a Brev A100 (unattended, resumable):
#   nohup bash ~/batch/batch_run.sh <shutdown HH:MM UTC> > ~/batch/run.log 2>&1 &
# Input : ~/batch/{heldout,train}/{images,labels} (white-balanced JPEGs, <stem>__<camera>.jpg)
# Output: ~/batch/out_<tag>/<stem>__<camera>.jpg and ~/batch/qc_<tag>.json (colour gate)
# Recipe (trial 2 winner): edge 1.0 + vis 1.0 very_low, sigma_max 80, neutral prompts.
set -euo pipefail
B="$HOME/batch"
REPO="$HOME/cosmos-transfer2.5"
CHUNK=250  # specs per model load; a crash loses at most one chunk
stage() { echo "STAGE $1 $(date +%H:%M:%S)"; }

sudo shutdown -h "$1" || echo "WARN: could not schedule shutdown"  # budget backstop

stage system
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq git-lfs ffmpeg libx11-dev python3-pip >/dev/null
python3 -m pip install -q --user uv 2>/dev/null || python3 -m pip install -q --user --break-system-packages uv
export PATH="$HOME/.local/bin:$PATH"

stage env
git lfs install --skip-repo
[ -d "$REPO" ] || GIT_LFS_SKIP_SMUDGE=1 git clone -q --depth 1 https://github.com/nvidia-cosmos/cosmos-transfer2.5.git "$REPO"
cd "$REPO"
uv python pin 3.10  # the cu128 flash-attn wheel is published for cp310 only
uv sync --extra=cu128
source .venv/bin/activate

for tag in heldout train; do
    [ -d "$B/$tag/images" ] || continue
    stage "specs_$tag"
    python "$B/sweep_specs.py" "$B/$tag/images" "$B/specs_$tag" "$B/prompts_neutral.json" --only edgevis_s80
    mapfile -t todo < <(for s in "$B/specs_$tag"/edgevis_s80/*.json; do
        n=$(basename "$s" .json); [ -f "$B/out_$tag/$n.jpg" ] || echo "$s"; done)
    stage "infer_$tag (${#todo[@]} to do)"
    for ((i = 0; i < ${#todo[@]}; i += CHUNK)); do
        python examples/inference.py -i "${todo[@]:i:CHUNK}" -o "$B/out_$tag" \
            || echo "FAILED chunk $i of $tag"
        echo "PROGRESS $tag $(ls "$B/out_$tag"/*.jpg 2>/dev/null | grep -vc control)/$(ls "$B/$tag/images" | wc -l)"
    done
    stage "qc_$tag"
    python "$B/transfer_qc.py" "$B/$tag" "$B/out_$tag" --json "$B/qc_$tag.json"
done
stage done
