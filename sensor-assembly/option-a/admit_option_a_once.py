"""One approved finite Option A suite. Refuse overlap or uncertain resubmission."""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

NAME='option-a-pickup-2x-20261025-01'
JOB_NAME='team-v-option-a-pickup-01'
TERMINAL={'BOOT_FAIL','CANCELLED','COMPLETED','DEADLINE','FAILED','NODE_FAIL','OUT_OF_MEMORY','TIMEOUT'}


def fields(raw):
    return dict(re.findall(r'(\w+)=(\S+)',raw))


def require_owner(raw,job,launcher,uid):
    value=fields(raw)
    owner=re.fullmatch(r'.+\((\d+)\)',value.get('UserId',''))
    if (value.get('JobId')!=str(job) or value.get('JobName')!=JOB_NAME or
        value.get('Command')!=str(launcher) or not owner or int(owner[1])!=uid):
        raise RuntimeError('Recovered identity/source/owner mismatch; do not resubmit')


def recover(run,launcher,uid):
    queue=run(['squeue','--all','--states=all','-r','-h','-u',str(uid),'-o','%i|%j'])
    live=[line.split('|')[0] for line in queue.splitlines() if line and line.split('|')[1]==JOB_NAME]
    history=run(['sacct','-u',str(uid),'-n','-X','--name='+JOB_NAME,
                 '--starttime=2026-10-24','--format=JobIDRaw','-P'])
    ids=set(live+[line.split('|')[0].strip() for line in history.splitlines() if line.strip()])
    if not all(v.isdigit() for v in ids) or len(ids)>1:
        raise RuntimeError('Ambiguous Slurm history; inspect, do not resubmit')
    if not ids:return None
    job=next(iter(ids))
    require_owner(run(['scontrol','-o','show','job',job]),job,launcher,uid)
    return job


def queue_guard(run,uid):
    raw=run(['squeue','--all','--states=all','-r','-h','-u',str(uid),'-o','%i|%j|%T'])
    retained=[]
    for line in raw.splitlines():
        if not line.strip():continue
        items=line.split('|')
        if len(items)!=3 or not items[0].isdigit():raise RuntimeError('Unknown user queue identity/state')
        job,name,status=items
        if status not in TERMINAL:raise RuntimeError('Other jobs active/queued; no overlap or cancellation permitted')
        control=run(['scontrol','-o','show','job',job]);value=fields(control)
        owner=re.fullmatch(r'.+\((\d+)\)',value.get('UserId',''))
        if (value.get('JobId')!=job or value.get('JobName')!=name or
            value.get('JobState')!=status or not owner or int(owner[1])!=uid):
            raise RuntimeError('Retained terminal job identity/state is unconfirmed')
        retained.append({'job':job,'name':name,'terminal_state':status,'raw_scontrol':control})
    return {'all_state_numeric_uid_queue':raw,'numeric_uid':uid,'retained_terminal_jobs':retained,'active_or_queued_jobs':0}


def admit(code,expected_manifest,home,run):
    code=Path(code).resolve();home=Path(home).resolve()
    if code!=home/'team-v-tests'/NAME:raise ValueError('Unexpected immutable snapshot path')
    if not re.fullmatch(r'[0-9a-f]{64}',expected_manifest):raise ValueError('Invalid external source hash')
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    mf=code/'snapshot-sha256.json'
    if digest(mf)!=expected_manifest:raise RuntimeError('Manifest binding differs')
    manifest=json.loads(mf.read_text())
    actual=set()
    for path in code.rglob('*'):
        if path.is_symlink():raise RuntimeError('Symlink in source')
        if path.is_file():actual.add(str(path.relative_to(code)))
    if actual!=set(manifest)|{'snapshot-sha256.json'}:raise RuntimeError('Missing or extra source files')
    for name,value in manifest.items():
        file=code/name
        if not file.resolve().is_relative_to(code) or digest(file)!=value:raise RuntimeError('Source mismatch: '+name)
    receipts=home/'team-v-tests/submission-receipts';receipts.mkdir(mode=0o700,parents=True,exist_ok=True)
    launcher=code/'option_a_on_rosie.sh'
    with (receipts/(NAME+'.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        receipt=receipts/(NAME+'.json')
        state=json.loads(receipt.read_text()) if receipt.exists() else {'snapshot':NAME,'manifest_sha256':expected_manifest}
        if state['manifest_sha256']!=expected_manifest:raise RuntimeError('Receipt binding mismatch')
        def save():
            tmp=receipt.with_suffix('.json.tmp')
            with tmp.open('w') as f:
                json.dump(state,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
            os.replace(tmp,receipt)
        if state.get('gpu'):
            require_owner(run(['scontrol','-o','show','job',state['gpu']]),state['gpu'],launcher,os.getuid())
            return state
        previous=recover(run,launcher,os.getuid())
        if previous:
            state.update(gpu=previous,recovered_from_slurm=True);save();return state
        if state.get('submission_intent'):
            raise RuntimeError('Unresolved previous intent; inspect live/accounting evidence, never resubmit blindly')
        guard=queue_guard(run,os.getuid())
        now=datetime.datetime.now(datetime.timezone.utc).isoformat()
        state.update(admission_at_UTC=now,queue_guard=guard,uid=os.getuid(),
            allocation_cap_seconds=900,cases=['base','carrier','cover'],automatic_retries=0,
            authorization_message='user-approved prototype pickup',
            submission_intent={'at_UTC':now,'script':str(launcher),'manifest_sha256':expected_manifest})
        save()
        (home/'team-v-logs').mkdir(exist_ok=True)
        raw=run(['sbatch','--parsable','--output='+str(home/'team-v-logs/option-a-pickup-%j.log'),
                 str(launcher),str(code),expected_manifest])
        if not raw.isdigit():raise RuntimeError('Uncertain sbatch response; intent retained, inspect before retry')
        state.update(gpu=raw,status='SUBMITTED');save();return state


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--expected-manifest',required=True)
    args=ap.parse_args()
    def run(argv):return subprocess.check_output(argv,text=True,timeout=45).strip()
    print(json.dumps(admit(Path.home()/'team-v-tests'/NAME,args.expected_manifest,Path.home(),run)))


if __name__=='__main__':main()
