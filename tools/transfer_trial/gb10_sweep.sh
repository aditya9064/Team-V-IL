#!/bin/bash
# Runs INSIDE the av-transfer:nightly container on the GB10 (see gb10_run.sh).
#   /trial/sweep_in/images/*.png  ->  /trial/sweep_out/<config>/*.jpg
# Configs come from sweep_specs.py (or $CONFIGS, comma-separated). Model loads once per config.
set -euo pipefail
cd /workspace
stage() { echo "STAGE $1 $(date +%H:%M:%S)"; }
IN="${IN:-/trial/sweep_in}"
OUT="${OUT:-/trial/sweep_out}"

stage specs
python /trial/sweep_specs.py "$IN/images" "$OUT/specs" "/trial/${PROMPTS:-prompts_neutral.json}" \
    ${CONFIGS:+--only "$CONFIGS"}
for dir in "$OUT"/specs/*/; do
    name=$(basename "$dir")
    [ "$name" = videos ] && continue
    if ls "$OUT/$name"/*.jpg >/dev/null 2>&1; then echo "skip $name (done)"; continue; fi
    stage "infer_$name"
    python examples/inference.py -i "$dir"*.json -o "$OUT/$name" || echo "FAILED $name"
done
chown -R "${HOST_UID:-1000}:${HOST_GID:-1000}" /trial "$HF_HOME" 2>/dev/null || true
stage done
