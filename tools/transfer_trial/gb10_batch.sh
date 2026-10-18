#!/bin/bash
# Runs INSIDE the av-transfer:nightly container on the GB10 (via SCRIPT=gb10_batch.sh gb10_run.sh).
# Resumable chunked Transfer batch with the colour-safe recipe, then the colour gate:
#   $IN/train/images/*.jpg  ->  $OUT/out_train/*.jpg  and  $OUT/qc_train.json
set -euo pipefail
cd /workspace
IN="${IN:?}" OUT="${OUT:?}" CHUNK="${CHUNK:-100}"
stage() { echo "STAGE $1 $(date +%H:%M:%S)"; }

for tag in train heldout; do
    [ -d "$IN/$tag/images" ] || continue
    stage "specs_$tag"
    python /trial/sweep_specs.py "$IN/$tag/images" "$OUT/specs_$tag" /trial/prompts_neutral.json --only edgevis_s80
    mapfile -t todo < <(for s in "$OUT/specs_$tag"/edgevis_s80/*.json; do
        n=$(basename "$s" .json); [ -f "$OUT/out_$tag/$n.jpg" ] || echo "$s"; done)
    stage "infer_$tag (${#todo[@]} to do)"
    for ((i = 0; i < ${#todo[@]}; i += CHUNK)); do
        python examples/inference.py -i "${todo[@]:i:CHUNK}" -o "$OUT/out_$tag" || echo "FAILED chunk $i of $tag"
        echo "PROGRESS $tag $(ls "$OUT/out_$tag"/*.jpg 2>/dev/null | grep -vc control)/$(ls "$IN/$tag/images" | wc -l)"
    done
    stage "qc_$tag"
    python /trial/transfer_qc.py "$IN/$tag" "$OUT/out_$tag" --json "$OUT/qc_$tag.json"
done
chown -R "${HOST_UID:-1000}:${HOST_GID:-1000}" /trial "$HF_HOME" 2>/dev/null || true
stage done
