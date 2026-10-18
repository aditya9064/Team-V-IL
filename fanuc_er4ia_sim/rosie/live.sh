#!/bin/bash
# Live pipeline dashboard on Rosie; refreshes every 15 s. Ctrl+C to exit.
#   ./rosie/live.sh          dashboard
#   ./rosie/live.sh log      follow the log of the newest running pipeline job
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

if [ "${1:-}" = log ]; then
    id=$(squeue -u "$USER" -h -t R -o "%i %j" | awk '$2 ~ /^av-/ && $2 != "av-watch" {print $1}' | sort -n | tail -1)
    [ -n "$id" ] || die "No pipeline job running right now"
    name=$(squeue -h -j "${id%%_*}" -o %j | head -1)
    log=$(ls -t "$LOG_DIR/$name-${id%%_*}"_*.out | head -1)
    echo "Following $log (Ctrl+C stops watching, not the job)"
    exec tail -f "$log"
fi

exec watch -n 15 -t "
echo 'AdaptiveVision pipeline   $(hostname)   refreshed:' \$(date +%H:%M:%S); echo
echo 'STAGES (R = running, PD = waiting its turn)'
squeue -u \$USER -o '%.14i %.11j %.3t %.10M  %R' | grep -E 'JOBID| av-'
echo; echo 'FINISHED TODAY'
sacct -u \$USER -S today -X -n -o JobID%14,JobName%12,State%12,Elapsed | grep ' av-' | grep -v -E 'RUNNING|PENDING' | tail -12
echo; echo 'SYNTHETIC FRAMES'
total=0; for c in $SYNTH_DIR/chunk_*; do [ -d \$c/images ] || continue; n=\$(ls \$c/images | wc -l); total=\$((total+n)); printf '  %s %5s\n' \$(basename \$c) \$n; done; echo \"  total \$total\"
echo; echo 'WATCHDOG'; cat $DATA_ROOT/WATCHDOG.txt 2>/dev/null | head -3 || true; [ -f $DATA_ROOT/WATCHDOG.txt ] || echo '  no problems reported'
echo; echo 'RESULTS'; head -8 $DATA_ROOT/RESULTS.txt 2>/dev/null || echo '  not yet'
"
