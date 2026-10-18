# Shared settings for the ROSIE scripts. Sourced, not executed.
# shellcheck shell=bash

ROSIE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$ROSIE_DIR/.." && pwd)"
# shellcheck source=config.env
source "$ROSIE_DIR/config.env"

# Ignore ~/.local site-packages: old user installs (e.g. a cu121 torch) would
# shadow the project envs and the Isaac container's Python.
export PYTHONNOUSERSITE=1

HOST_ENV="$DATA_ROOT/envs/host"
TRAIN_ENV="$DATA_ROOT/envs/train"
SYNTH_DIR="$DATA_ROOT/synth"
DATASETS_DIR="$DATA_ROOT/datasets"
RUNS_DIR="$DATA_ROOT/runs"
LOG_DIR="$DATA_ROOT/logs"
V2_DIR="$DATA_ROOT/synth_v2"
V2_CHUNKS=$((V2_TABLE_CHUNKS + V2_SIDE_CHUNKS + V2_WRIST_CHUNKS))

die() { echo "ERROR: $*" >&2; exit 1; }

container_cmd() {
    # Some clusters expose Apptainer/Singularity only after `module load`.
    if ! command -v apptainer >/dev/null && ! command -v singularity >/dev/null; then
        module load apptainer 2>/dev/null || module load singularity 2>/dev/null || true
    fi
    command -v apptainer || command -v singularity || die "No apptainer/singularity found"
}

slurm_opts() {
    # $1 = partition, $2 = gres, $3 = constraint; prints common sbatch options
    local opts=(--partition="$1" --chdir="$PROJECT")
    [ -n "$2" ] && opts+=(--gres="$2")
    [ -n "$3" ] && opts+=(--constraint="$3")
    [ -n "$ACCOUNT" ] && opts+=(--account="$ACCOUNT")
    printf '%s\n' "${opts[@]}"
}

nv_extra_binds() {
    # Singularity 3.10's --nv list predates libnvidia-gpucomp (needed for Vulkan
    # since driver 535) and libnvidia-allocator. Without them vkCreateInstance
    # reports ERROR_INCOMPATIBLE_DRIVER. Mount this node's copies (versions vary
    # per node) into /opt/nvextra, which isaac_run puts first on LD_LIBRARY_PATH.
    local f
    for f in /usr/lib/x86_64-linux-gnu/libnvidia-{gpucomp,allocator}.so* \
             /usr/lib64/libnvidia-{gpucomp,allocator}.so*; do
        [ -e "$f" ] && printf '%s\n' --bind "$(readlink -f "$f"):/opt/nvextra/$(basename "$f")"
    done
    # Docker's NVIDIA runtime injects the Vulkan/EGL vendor files; --nv does not.
    for f in /usr/share/vulkan/icd.d/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json \
             /usr/share/glvnd/egl_vendor.d/10_nvidia.json; do
        [ -f "$f" ] && printf '%s\n' --bind "$f:$f"
    done
    return 0
}

isaac_run() {
    # Run any command inside the Isaac container with GPU graphics: isaac_run <cmd> [args...]
    local cmd cache="$DATA_ROOT/isaac-cache" extra=() line
    cmd="$(container_cmd)"
    [ -f "$ISAAC_SIF" ] || die "Missing $ISAAC_SIF; run ./rosie/av.sh pull-isaac"
    mkdir -p "$cache"/{kit,ov,pip,glcache,computecache,logs,data,documents}
    # Bind targets under $HOME must exist; the large caches live in $DATA_ROOT.
    mkdir -p "$HOME/.cache/ov" "$HOME/.cache/nvidia/GLCache" "$HOME/.nv/ComputeCache" "$HOME/Documents"
    while IFS= read -r line; do extra+=("$line"); done < <(nv_extra_binds)
    # Ray-tracing shaders take 20+ min to compile per driver version on a T4;
    # keep the driver's shader cache on shared storage, big enough to never evict.
    "$cmd" exec --nv ${extra[@]+"${extra[@]}"} \
        --env "ACCEPT_EULA=Y,PRIVACY_CONSENT=Y,OMNI_KIT_ACCEPT_EULA=YES" \
        --env "FANUC_ROBOT_USD=$FANUC_ROBOT_USD,XDG_CACHE_HOME=$cache" \
        --env "LD_LIBRARY_PATH=/opt/nvextra:/.singularity.d/libs" \
        --env "__GL_SHADER_DISK_CACHE=1,__GL_SHADER_DISK_CACHE_PATH=$cache/glcache" \
        --env "__GL_SHADER_DISK_CACHE_SIZE=10737418240,__GL_SHADER_DISK_CACHE_SKIP_CLEANUP=1" \
        --bind "$PROJECT:$PROJECT" --bind "$DATA_ROOT:$DATA_ROOT" \
        --bind "$cache/kit:/isaac-sim/kit/cache" \
        --bind "$cache/data:/isaac-sim/kit/data" \
        --bind "$cache/logs:/isaac-sim/kit/logs" \
        --bind "$cache/ov:$HOME/.cache/ov" \
        --bind "$cache/glcache:$HOME/.cache/nvidia/GLCache" \
        --bind "$cache/computecache:$HOME/.nv/ComputeCache" \
        --bind "$cache/documents:$HOME/Documents" \
        "$ISAAC_SIF" "$@"
}

isaac_exec() {
    # Run an Isaac Python script inside the container: isaac_exec script.py [args...]
    isaac_run /isaac-sim/python.sh "$@"
}

submit_watchdog() {
    # One watchdog job; it resubmits itself before Rosie's 1-day job limit.
    local opts=(--parsable --job-name=av-watch --partition="$ISAAC_PARTITION" --chdir="$PROJECT"
                --time="$WATCH_TIME" --output="$LOG_DIR/%x-%A_%a.out")
    [ -n "$ACCOUNT" ] && opts+=(--account="$ACCOUNT")
    [ -n "$MAIL_USER" ] && opts+=(--mail-type=FAIL --mail-user="$MAIL_USER")
    sbatch "${opts[@]}" "$PROJECT/rosie/jobs/watchdog.sh"
}

v2_camera_of() {
    # Chunk index -> camera: table chunks first, then side, then wrist.
    local chunk=$1
    if [ "$chunk" -lt "$V2_TABLE_CHUNKS" ]; then echo table
    elif [ "$chunk" -lt $((V2_TABLE_CHUNKS + V2_SIDE_CHUNKS)) ]; then echo side
    else echo wrist; fi
}

sub() {
    # Used by run_all.sh and run_v2.sh.
    # sub <name> <partition> <gres> <time> <dependency> <script> [args...] -> prints job id
    local opts=(--parsable --job-name="$1" --partition="$2" --chdir="$PROJECT" --time="$4"
                --output="$LOG_DIR/%x-%A_%a.out")
    [ -n "$3" ] && opts+=(--gres="$3")
    [ -n "$5" ] && opts+=(--dependency="$5" --kill-on-invalid-dep=yes)
    [ -n "$ACCOUNT" ] && opts+=(--account="$ACCOUNT")
    if [ -n "$MAIL_USER" ]; then
        local mail=FAIL
        [ "$1" = av-report ] && mail=END,FAIL  # one "results ready" email at the end
        opts+=(--mail-type="$mail" --mail-user="$MAIL_USER")
    fi
    shift 5
    sbatch "${opts[@]}" "$@" | cut -d';' -f1
}
