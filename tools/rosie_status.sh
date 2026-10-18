#!/bin/bash
# Pipeline status from the Mac: job states, plus the log tail of every failed stage.
# Usage: tools/rosie_status.sh            (needs the "rosie" entry in ~/.ssh/config)
set -euo pipefail
ssh -o BatchMode=yes -o ConnectTimeout=20 rosie 'bash -s' <<'REMOTE'
echo "== queue"; squeue -u "$USER" -o "%.12i %.10j %.3t %.10M %R" | grep -E "JOBID| av-" || true
echo "== today"; sacct -u "$USER" -S today -X -n -o JobID%16,JobName%12,State%14,Elapsed | grep " av-" || true
echo "== container"; ls -lh ~/adaptivevision/containers 2>/dev/null | tail -n +2
echo "== frames"; for c in ~/adaptivevision/synth/chunk_*; do [ -d "$c/images" ] && printf "%s %s\n" "$(basename "$c")" "$(ls "$c/images" | wc -l)"; done 2>/dev/null || true
echo "== results"; cat ~/adaptivevision/RESULTS.txt 2>/dev/null || echo "(none yet)"
failed=$(sacct -u "$USER" -S today -X -n -o JobID%16,JobName%12,State%14 | awk '$2 ~ /^av-/ && $2 != "av-isaac" && $3 ~ /FAILED|TIMEOUT|OUT_OF_ME|NODE_FAIL/ {print $1"|"$2}')
for f in $failed; do
    id=${f%%|*}; name=${f##*|}; base=${id%%_*}
    log=$(ls -t ~/adaptivevision/logs/"$name"-"$base"_*.out 2>/dev/null | head -1)
    echo "== FAILED $name $id  ($log)"; [ -n "$log" ] && tail -30 "$log"
done
REMOTE
