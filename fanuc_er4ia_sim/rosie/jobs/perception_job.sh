#!/bin/bash
# B1-B3 check at scale: newest cubes_v2 / cubes_v1v2 / cubes_all models on held-out v2 frames
# (YOLO -> plan_sort geometry vs. true poses). Writes $DATA_ROOT/PERCEPTION.txt.
#   sbatch --dependency=afterany:<train ids> rosie/jobs/perception_job.sh [run names...]
# Default: cubes_all cubes_v2 cubes_v1v2. Per-model blocks in runs/perception/<name>.txt are merged.
#SBATCH --job-name=av-percep
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
set -euo pipefail
source "$PWD/rosie/common.sh"
export YOLO_CONFIG_DIR="$DATA_ROOT/ultralytics"
OUT="$RUNS_DIR/perception"
FRAMES="$OUT/frames"
mkdir -p "$FRAMES"

# Seeded held-out sample as <stem>__<camera>.{png,json} symlinks (image + generator meta).
"$TRAIN_ENV/bin/python" - "$DATASETS_DIR/cubes_v2/images/val" "$FRAMES" <<'PY'
import json, random, sys
from pathlib import Path
val, out = Path(sys.argv[1]), Path(sys.argv[2])
by = {}
for p in sorted(val.glob("*.png")):
    chunk = p.resolve().parent.parent
    by.setdefault(json.loads((chunk / "dataset.json").read_text())["camera"], []).append(p)
rng = random.Random(16)
for camera, n in (("table", 400), ("side", 100), ("wrist", 100)):
    for p in rng.sample(by.get(camera, []), min(n, len(by.get(camera, [])))):
        r = p.resolve()
        for src, ext in ((r, ".png"), (r.parent.parent / "meta" / f"{r.stem}.json", ".json")):
            link = out / f"{r.stem}__{camera}{ext}"
            if not link.exists():
                link.symlink_to(src)
PY

latest() {
    ls -td "$RUNS_DIR"/detect/"$1"_[0-9]*/weights/best.pt 2>/dev/null | head -1
}
models=("$@")
[ ${#models[@]} -gt 0 ] || models=(cubes_all cubes_v2 cubes_v1v2)
for name in "${models[@]}"; do
    w=$(latest "$name")
    [ -n "$w" ] || { echo "[percep] no $name model"; continue; }
    (cd "$PROJECT" && "$TRAIN_ENV/bin/python" -m synth.eval_perception "$FRAMES" \
        --weights "$w" --out "$OUT/$name.json" > "$OUT/$name.log" 2>&1)
    { echo "=== $name ($w)"; "$TRAIN_ENV/bin/python" -c "
import json, sys
s = json.load(open(sys.argv[1]))['summary']
keys = ['frames', 'cube_recall', 'cube_precision', 'colour_accuracy', 'stacked_cubes_detected', 'pos_mm_median', 'pos_mm_p90',
        'pos_within_2mm', 'yaw_deg_median', 'yaw_deg_p90', 'yaw_within_5deg', 'pose_ready_share']
print(f\"{'metric':20}\" + ''.join(f'{c:>9}' for c in s))
for k in keys:
    print(f'{k:20}' + ''.join(f'{str(s[c][k]):>9}' for c in s))
" "$OUT/$name.json"; echo; } > "$OUT/$name.txt"
done
# One combined report from every model evaluated so far (early and late runs).
cat "$OUT"/cubes_*.txt > "$DATA_ROOT/PERCEPTION.txt"
echo "[percep] done -> $DATA_ROOT/PERCEPTION.txt"
