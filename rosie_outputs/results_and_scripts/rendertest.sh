#!/bin/bash
# One-off: compare real-time vs path-traced synthetic frames (3 each) on a T4.
set -uo pipefail
source "$PWD/rosie/common.sh"
T="$DATA_ROOT/rendertest"; rm -rf "$T"
for mode in rt30 pt32; do
    case $mode in rt30) a=(--settle 30) ;; pt32) a=(--pathtrace --spp 32 --settle 2) ;; esac
    echo "== $mode"
    isaac_exec "$PROJECT/synth/generate_synthetic.py" --out "$T/$mode" --count 3 --seed 20261015 "${a[@]}" 2>&1 \
        | grep -E "\[synth\]|Traceback|Error:"
done
