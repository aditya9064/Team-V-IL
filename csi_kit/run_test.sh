#!/bin/bash
# CSI real-photo test. Put photos from the cell camera in csi_kit/photos/, then run:
#   bash ~/Desktop/AdaptiveVision/csi_kit/run_test.sh            (table camera, v2 model)
#   bash ~/Desktop/AdaptiveVision/csi_kit/run_test.sh side       (side camera photos)
# Works offline (no Rosie, no VPN). Results go to csi_kit/results/<time>/.
set -euo pipefail
KIT="$(cd "$(dirname "$0")" && pwd)"
AV="$(dirname "$KIT")"
CAMERA="${1:-table}"
OUT="$KIT/results/$(date +%H%M%S)_$CAMERA"
shopt -s nullglob nocaseglob
PHOTOS=("$KIT"/photos/*.{jpg,jpeg,png})
[ ${#PHOTOS[@]} -gt 0 ] || { echo "No photos in $KIT/photos/ - copy the camera frames there first."; exit 1; }
cd "$AV/fanuc_er4ia_sim"
"$AV/.venv-yolo/bin/python" real_photo_test.py "${PHOTOS[@]}" --weights "$AV/models/cubes_v2.pt" \
    --camera "$CAMERA" --out "$OUT"
[ "${NO_OPEN:-}" ] || open "$OUT"
