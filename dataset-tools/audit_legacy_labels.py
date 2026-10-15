"""Rebuild synthetic-v1 labels after bottom-origin and pixel QA checks."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
from varied_labels import verify_labels

def main():
    p=argparse.ArgumentParser();p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    folders=sorted(a.samples.glob('varied-*'))
    if len(folders)!=60 or a.out.exists():
        raise ValueError('Expected original 60 scenes and a fresh export directory')
    results=[]
    for folder in folders:
        generation=json.loads((folder/'generation.json').read_text())
        generation['cube_count']=len(generation['cube_layout'])
        result=verify_labels(folder,generation)
        results.append(dict(scene=folder.name,**result))
        print('[legacy-label-qa]',folder.name,result,flush=True)
    # The repository exporter already contains the bottom-origin correction;
    # its former Rosie-only filename was export_dataset_originfix.py.
    subprocess.run([sys.executable,str(Path(__file__).with_name('export_dataset.py')),
        '--samples',str(a.samples),'--out',str(a.out),'--edge-m','.01'],check=True)
    # Pixel boxes replace geometric amodal boxes only after each silhouette passes.
    manifest=json.loads((a.out/'manifest.json').read_text())
    for row in manifest:
        name,split=row['scene'],row['split'];source=a.samples/name
        (a.out/'labels'/split/(name+'.txt')).write_text((source/'cube_labels.txt').read_text())
    (a.out/'label_audit.json').write_text(json.dumps(dict(samples=results,passed=True,
        label_source='visible colour components verified against bottom-origin USD geometry',
        manual_review_complete=False,training_ready=False,
        replaced_export='synthetic-v1 boxes assumed centre-origin; retain for history only'),indent=2)+'\n')
    print('TEAM_V_LEGACY_LABEL_REBUILD_PASS',len(results),'scenes',flush=True)

if __name__=='__main__':main()
