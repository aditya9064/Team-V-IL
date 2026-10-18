#!/bin/bash
# Email a plain-English AdaptiveVision status report to MAIL_USER (sent via Rosie's mail).
#   tools/status_mail.sh            send once
#   tools/status_mail.sh --loop 20  send every 20 minutes until stopped
#   tools/status_mail.sh --file     only write the report to $TMPDIR/av_status_report.txt
# Rosie's login node has no mail transport (no postfix config), so sending fails there;
# the report file is always written.
set -uo pipefail
SSH=(-o BatchMode=yes -o ConnectTimeout=30)
BATCH="$HOME/Desktop/AdaptiveVision/transfer_trial/batch"

section() { printf '\n==== %s ====\n' "$1"; }

body() {
    echo "AdaptiveVision status - $(date '+%a %b %d, %I:%M %p')"
    section "1. WHAT'S HAPPENING NOW"
    ssh "${SSH[@]}" rosie 'bash ~/fanuc_er4ia_sim/rosie/status_all.sh 2>/dev/null | sed -n "/Rosie jobs/,/Task list/p" | grep -v "Task list"' \
        || echo "(Rosie not reachable from the Mac right now)"
    echo
    echo "--- Brev (paid cloud GPUs) ---"
    brev ls 2>/dev/null | grep -E "^ *av-" || echo "no Brev machines running (nothing billing)"
    local b0 b1
    b0=$(ls "$BATCH"/b0/out_*/*.jpg 2>/dev/null | grep -vc control)
    b1=$(ls "$BATCH"/b1/out_*/*.jpg 2>/dev/null | grep -vc control)
    echo "Photo-real Transfer images copied to Mac: $((b0 + b1)) of 2,200"

    section "2. SUCCESSES SO FAR"
    ssh "${SSH[@]}" rosie 'cd ~/adaptivevision
        for f in SIM_RESULTS.txt BASELINE.txt PERCEPTION.txt RESULTS_V2.txt; do
            [ -s "$f" ] && { echo "--- $f"; head -40 "$f"; echo; }
        done' 2>/dev/null
    echo "- Robot in simulator (first test): tower complete 2/2, 12/12 picks + placements, 0.18 mm, 1.4 deg."
    echo "- RTX path-traced renders: photo-like, labels exactly aligned (Brev L40S, done, deleted)."
    echo "- Transfer colour-safe recipe: 99% of cube colours kept on the batch so far."

    section "3. WHAT IT LOOKS LIKE (files on your Mac)"
    ls ~/Desktop/AdaptiveVision/videos/*.mp4 2>/dev/null | sed 's/^/  video: /' || true
    echo "  RTX comparison: ~/Desktop/AdaptiveVision/rtx_test/cmp3_v2_000004.jpg"
    echo "  Labels on path-traced render: ~/Desktop/AdaptiveVision/rtx_test/pt_check/labels_v2_000008.jpg"
    echo "  Transfer before/after: ~/Desktop/AdaptiveVision/transfer_trial/trial2/compare_wb_edgevis_s80/"
    echo "  Robot camera + detections: ~/Desktop/AdaptiveVision/b_eval/sim_seed1/detections_overlay.png"

    section "4. STEPS REMAINING"
    ssh "${SSH[@]}" rosie 'sed -n "/^## Next/,\$p" ~/adaptivevision/TASKS.md' 2>/dev/null
    echo
    echo "Live view anytime: type 'avstatus' in the Rosie terminal."
}

send() {
    local subject="AdaptiveVision status $(date '+%I:%M %p')"
    body | tee "${TMPDIR:-/tmp}/av_status_report.txt" | ssh "${SSH[@]}" rosie 'source ~/fanuc_er4ia_sim/rosie/config.env 2>/dev/null; mail -s "'"$subject"'" "$MAIL_USER"' \
        && echo "[mail] sent $(date +%H:%M)" || echo "[mail] FAILED $(date +%H:%M)"
}

if [ "${1:-}" = "--file" ]; then
    body > "${TMPDIR:-/tmp}/av_status_report.txt" && echo "[report] written $(date +%H:%M)"
elif [ "${1:-}" = "--loop" ]; then
    while true; do send; sleep "$(( ${2:-20} * 60 ))"; done
else
    send
fi
