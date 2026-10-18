#!/bin/bash
# One-off: is each Isaac copy really on its own GPU? Sample per-GPU load while both render.
set -uo pipefail
source "$PWD/rosie/common.sh"
for k in 0 1; do
    isaac_exec "$PROJECT/synth/generate_v2.py" --out "$V2_DIR/pintest2_gpu$k" --camera table \
        --start 0 --count 25 --seed "$((V2_SEED + 13))" --prefix v2p_ --gpu "$k" > "$LOG_DIR/av-pintest2-gpu$k.log" 2>&1 &
done
for i in $(seq 1 40); do
    sleep 10
    echo "t=$((i*10))s $(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader | tr "\n" " ")"
done
wait
