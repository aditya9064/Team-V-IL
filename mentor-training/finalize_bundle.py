"""Record completed visual/model checks and create a verified portable archive."""
import argparse
import datetime
import hashlib
import json
import zipfile
from pathlib import Path

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--release',type=Path,required=True);a=p.parse_args()
    root=a.release
    smoke=json.loads((root/'model_smoke.json').read_text())
    assert smoke['passed'] and smoke['empty_target_backward_passed'] and smoke['optimizer_steps']==0
    qa=sorted((root/'qa').glob('*.png'))
    assert len(qa)==15,len(qa)
    record=dict(passed=True,reviewer='Codex visual inspection',
                scope='All 78 cropped-scene overlays, all 432 enlarged cube patches, and three empty scenes',
                reviewed_qa_sheets={p.name:sha(p) for p in qa},
                findings='Cube colours and visible boundaries agree; no crop clipping or labels on caps/vials; empty scenes have no cube boxes.',
                timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    (root/'visual_review.json').write_text(json.dumps(record,indent=2)+'\n')
    validation=json.loads((root/'data_validation.json').read_text())
    validation.update(visual_crop_review_complete=True,visual_review_scope=record['scope'],
                      model_adapter_smoke_passed=True,neural_training_started=False,
                      mentor_pilot_handoff_ready=True,real_hardware_readiness=False)
    (root/'data_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    # Exclude interpreter caches and any unexpected training output/credentials.
    files=sorted(p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    for path in files:
        if path.suffix in ('.pt','.pth') or path.name in ('id_rsa','rosie_key') or path.name.startswith('.env'):
            raise ValueError('Unexpected checkpoint/credential in handoff')
    hashes={str(p.relative_to(root)):sha(p) for p in files if p.name!='SHA256SUMS.json'}
    (root/'SHA256SUMS.json').write_text(json.dumps(hashes,indent=2)+'\n')
    archive=root.parent/'Team-V-Detector-Pilot-v2.zip'
    if archive.exists():raise ValueError('Archive already exists; preserve the earlier release')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for name in sorted(hashes):z.write(root/name,root.name+'/'+name)
        z.write(root/'SHA256SUMS.json',root.name+'/SHA256SUMS.json')
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name,wanted in hashes.items():
            assert hashlib.sha256(z.read(root.name+'/'+name)).hexdigest()==wanted,name
        assert len([n for n in z.namelist() if '/images/' in n and n.endswith('.png')])==78
        assert len([n for n in z.namelist() if '/labels/' in n and n.endswith('.txt')])==78
    receipt=dict(archive=archive.name,sha256=sha(archive),size_bytes=archive.stat().st_size,
                 files=len(hashes)+1,images=78,labels=432,archive_hashes_verified=True)
    archive.with_suffix('.receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('TEAM_V_MENTOR_PACKAGE_PASS',json.dumps(receipt),flush=True)

if __name__=='__main__':main()
