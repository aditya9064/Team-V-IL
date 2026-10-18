#!/bin/bash
# Build the Isaac Sim .sif on a compute node so a dropped browser shell can't kill it.
# Submitted by `rosie/av.sh pull-isaac`; NGC_API_KEY arrives via the job environment.
#SBATCH --job-name=av-pull
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
set -euo pipefail
source "$PWD/rosie/common.sh"

: "${NGC_API_KEY:?NGC_API_KEY missing from the job environment}"
echo "[job] $(hostname)  job ${SLURM_JOB_ID:-local}  pulling $ISAAC_IMAGE"
cmd="$(container_cmd)"
mkdir -p "$(dirname "$ISAAC_SIF")" "$DATA_ROOT/tmp"
# Layers already in the default cache (~/.singularity/cache) are reused, so a retry resumes quickly.
export APPTAINER_TMPDIR="$DATA_ROOT/tmp" SINGULARITY_TMPDIR="$DATA_ROOT/tmp"
export APPTAINER_DOCKER_USERNAME='$oauthtoken' SINGULARITY_DOCKER_USERNAME='$oauthtoken'
export APPTAINER_DOCKER_PASSWORD="$NGC_API_KEY" SINGULARITY_DOCKER_PASSWORD="$NGC_API_KEY"
# Rootless extraction prints one harmless EPERM warning per file; keep the log readable.
"$cmd" pull --force "$ISAAC_SIF.partial" "$ISAAC_IMAGE" 2>&1 | grep -v "harmless EPERM" || true
[ -s "$ISAAC_SIF.partial" ] || die "Pull failed; see the lines above (network or key?)"
mv "$ISAAC_SIF.partial" "$ISAAC_SIF"
ls -lh "$ISAAC_SIF"
echo "[job] Isaac container ready"
