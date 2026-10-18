#!/bin/bash
# When each Brev Transfer machine is done (batch_run.sh printed "STAGE done") or has
# powered itself off at its budget shutdown: copy all outputs to the Mac, wipe the
# Hugging Face token on the machine, then delete the machine.
#   bash brev_finish.sh av-transfer-b0:b0 av-transfer-b1:b1
set -uo pipefail
D="$HOME/Desktop/AdaptiveVision/transfer_trial/batch"
SSH=(-o BatchMode=yes -o ConnectTimeout=30)

status_of() { brev ls 2>/dev/null | awk -v n="$1" '$1 == n {print $2}'; }

is_finished() {  # finished = batch done, or machine no longer running (budget shutdown)
    local inst=$1 st
    st=$(status_of "$inst")
    [ -z "$st" ] && return 0
    [ "$st" != "RUNNING" ] && return 0
    ssh "${SSH[@]}" "$inst" 'grep -q "^STAGE done" ~/batch/run.log' 2>/dev/null
}

finish() {  # finish <instance> <local dir>
    local inst=$1 dir="$D/$2" st
    st=$(status_of "$inst")
    if [ -n "$st" ] && [ "$st" != "RUNNING" ]; then  # powered off: start it to read its disk
        echo "$(date +%H:%M) $inst is $st; starting it to copy results"
        brev start "$inst" >/dev/null 2>&1
        until [ "$(status_of "$inst")" = "RUNNING" ]; do sleep 20; done
        sleep 60
    fi
    rsync -a --timeout=300 -e "ssh ${SSH[*]}" --include="out_*/" --include="out_*/*.jpg" \
        --include="qc_*.json" --include="run.log" --exclude="*" "$inst":batch/ "$dir"/
    ssh "${SSH[@]}" "$inst" 'shred -u ~/.cache/huggingface/token 2>/dev/null; test ! -e ~/.cache/huggingface/token' \
        && echo "$inst token wiped" || echo "$inst token wipe NOT confirmed"
    echo "$(date +%H:%M) $inst final: $(ls "$dir"/out_*/*.jpg 2>/dev/null | grep -vc control) images on the Mac"
    brev delete "$inst" 2>&1 | tail -1
}

pending=("$@")
while [ ${#pending[@]} -gt 0 ]; do
    still=()
    for pair in "${pending[@]}"; do
        inst=${pair%%:*}
        if is_finished "$inst"; then finish "$inst" "${pair#*:}"; else still+=("$pair"); fi
    done
    pending=(${still[@]+"${still[@]}"})
    [ ${#pending[@]} -gt 0 ] && sleep 120
done
brev ls 2>&1 | grep -E "^ *av-" || echo "$(date +%H:%M) no Brev machines left (nothing billing)"
