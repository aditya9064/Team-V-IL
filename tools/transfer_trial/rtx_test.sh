#!/bin/bash
# RTX re-render test on a Brev L40S: same v2 scenes as Rosie chunk_0000, rendered with
# (a) real-time RTX + the GPU's own denoising/DLSS and (b) OptiX-denoised path tracing,
# both WITHOUT the median3 patch, for a side-by-side with the T4 renders.
#   nohup bash ~/rtx/rtx_test.sh <shutdown HH:MM UTC> > ~/rtx/run.log 2>&1 &
set -euo pipefail
R="$HOME/rtx"
IMAGE="nvcr.io/nvidia/isaac-sim:6.0.0"  # same version as Rosie's container
COUNT="${COUNT:-25}"
stage() { echo "STAGE $1 $(date +%H:%M:%S)"; }

sudo shutdown -h "$1" || echo "WARN: could not schedule shutdown"  # budget backstop
stage pull
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
docker pull -q "$IMAGE"

mkdir -p "$R/cache"/{kit,home_cache,home_nv,logs,data} "$R/out"
# The Isaac image runs as uid 1234 (isaac-sim, home /isaac-sim): let it write outputs and caches.
sudo chmod -R a+rwX "$R/cache" "$R/out"
isaac() {  # isaac <script> [args...]: run with Isaac's Python on the GPU, project at /work
    docker run --rm --gpus all --network host \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
        -v "$R/fanuc_er4ia_sim:/work" -v "$R/out:/out" \
        -v "$R/cache/kit:/isaac-sim/kit/cache" \
        -v "$R/cache/home_cache:/isaac-sim/.cache" -v "$R/cache/home_nv:/isaac-sim/.nv" \
        -v "$R/cache/data:/isaac-sim/kit/data" -v "$R/cache/logs:/isaac-sim/kit/logs" \
        -w /work --entrypoint /isaac-sim/python.sh "$IMAGE" "$@"
}

for mode in realtime pathtracing; do
    stage "render_$mode"
    isaac synth/generate_v2.py --out "/out/$mode" --camera table --start 0 --count "$COUNT" \
        --seed 20261016 --renderer "$mode" --denoise none || echo "FAILED $mode"
    echo "$mode images: $(ls "$R/out/$mode/images" 2>/dev/null | wc -l)"
done
sudo chown -R "$(id -u):$(id -g)" "$R/out"
stage done
