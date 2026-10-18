#!/bin/bash
# Wait for the image build, then run the sweep (all output in run.log).
while pgrep -f "docker build -f docker/nightly.Dockerfile" >/dev/null; do sleep 30; done
docker image inspect av-transfer:nightly >/dev/null 2>&1 || { echo "BUILD FAILED - see transfer_build.log"; exit 1; }
echo "build ok $(date +%H:%M:%S)"
exec bash "$HOME/adaptivevision/transfer/gb10_run.sh"
