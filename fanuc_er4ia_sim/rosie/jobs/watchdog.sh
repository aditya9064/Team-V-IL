#!/bin/bash
# Pipeline watchdog: runs on Rosie for days, so nobody has to keep a laptop awake.
# Submitted by rosie/run_all.sh. Every 10 minutes:
#   - results exist                  -> done, exit 0
#   - pipeline still queued/running  -> wait
#   - pipeline stopped early:
#       TIMEOUT / NODE_FAIL / PREEMPTED -> transient: rerun run_all.sh (resumes), max 3 times
#       anything else (FAILED, OOM...)  -> needs a code fix: write WATCHDOG.txt, exit 1
#                                          (exit 1 triggers the FAIL email when MAIL_USER is set)
# Rosie caps jobs at 1 day, so before its time runs out the watchdog submits its successor.
#SBATCH --job-name=av-watch
#SBATCH --cpus-per-task=1
#SBATCH --mem=1G
set -euo pipefail
source "$PWD/rosie/common.sh"
STATE="$DATA_ROOT/pipeline.env"     # FIRST_JOB=<id> written by run_all.sh
REPORT="$DATA_ROOT/WATCHDOG.txt"
RETRIES_FILE="$DATA_ROOT/watchdog_retries"
MAX_RETRIES=3

pipeline_active() {
    squeue -u "$USER" -h -o "%j" | grep -v '^av-watch$' | grep -q '^av-'
}

first_problem() {
    # "<jobid> <name> <state>" of the earliest non-successful stage since FIRST_JOB
    source "$STATE"
    sacct -u "$USER" -S now-7days -X -n -P -o JobID,JobName,State |
        awk -F'|' -v first="$FIRST_JOB" '$2 ~ /^av-/ && $2 != "av-watch" {
            split($1, id, "_"); if (id[1] + 0 < first + 0) next
            if ($3 !~ /^(COMPLETED|RUNNING|PENDING|CANCELLED)/) { print $1, $2, $3; exit } }'
}

handle_stop() {
    local problem retries
    problem=$(first_problem)
    retries=$(cat "$RETRIES_FILE" 2>/dev/null || echo 0)
    if [[ "$problem" =~ (TIMEOUT|NODE_FAIL|PREEMPTED) ]] && [ "$retries" -lt "$MAX_RETRIES" ]; then
        echo "$((retries + 1))" > "$RETRIES_FILE"
        echo "[watch] $(date) transient stop ($problem); resume attempt $((retries + 1))"
        WATCHDOG_CHILD=1 "$PROJECT/rosie/run_all.sh"
        return 0
    fi
    {
        echo "Pipeline stopped at $(date) and needs a fix."
        echo "First problem: ${problem:-unknown (check sacct)}   transient retries used: $retries"
        local name=${problem#* }; name=${name%% *}; local id=${problem%% *}
        tail -40 "$LOG_DIR/$name-${id%%_*}"_*.out 2>/dev/null || true
    } > "$REPORT"
    cat "$REPORT"
    exit 1
}

echo "[watch] started $(date) on $(hostname)"
renew_at=$(( $(date +%s) + WATCH_RENEW_AFTER ))
while true; do
    sleep 600
    if [ "$(date +%s)" -ge "$renew_at" ]; then
        echo "[watch] handing over to watchdog job $(submit_watchdog)"; exit 0
    fi
    if [ -s "$DATA_ROOT/RESULTS.txt" ]; then
        echo "[watch] results ready $(date)"; exit 0
    fi
    pipeline_active || handle_stop
done
