"""Prepare a local experiment from the separately obtained official SDK.

Copies only five hash-pinned Python dependency files. Does not launch a job,
connect to a robot, download data or change the supplied starter directory.
"""
import sys
sys.dont_write_bytecode = True
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from pickup_protocol import SDK_PINS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--starter',required=True,type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    destination=root/'pinned-sdk'
    destination.mkdir(exist_ok=True)
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    for name,expected in SDK_PINS.items():
        source=args.starter/name
        if digest(source)!=expected:raise ValueError('Supplied starter version differs: '+name)
        target=destination/name
        if target.exists() and digest(target)!=expected:raise ValueError('Existing dependency differs: '+name)
        if not target.exists():shutil.copy2(source,target)
    hashes={}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():raise ValueError('Symlinks are not part of this experiment')
        if not path.is_file() or path.name=='snapshot-sha256.json':continue
        if '__pycache__' in path.parts:raise ValueError('Remove local Python caches before sealing')
        hashes[str(path.relative_to(root))]=digest(path)
    manifest=root/'snapshot-sha256.json'
    manifest.write_text(json.dumps(hashes,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'source_files':len(hashes),'manifest_sha256':digest(manifest),
        'jobs_submitted':0,'source_review_required_before_execution':True},indent=2))


if __name__=='__main__':main()
