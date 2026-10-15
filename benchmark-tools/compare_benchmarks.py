import argparse, json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('rosie',type=Path)
p.add_argument('brev',type=Path)
a=p.parse_args()
r,b=[json.loads(x.read_text()) for x in (a.rosie,a.brev)]
if r['source_sha256'] != b['source_sha256']:
    raise SystemExit('Sources differ: rerun with identical files before comparing.')
if not r['complete'] or not b['complete']:
    raise SystemExit('A benchmark is incomplete; compare failure logs, not speed.')
print('Both completed 3 valid images. Full-resolution visual QA still required.')
for s in (r,b):
    print(s['host_label'], 'total seconds:', round(s['total_elapsed_s'],2),
          'scene seconds:', [round(x['elapsed_s'],2) for x in s['samples']],
          'valid images/hour:', round(s['valid_images_per_hour_including_script_startups'],2),
          'script-only estimated cost:', round(s['estimated_script_compute_cost'],3))
print('Brev speedup over Rosie:',round(r['total_elapsed_s']/b['total_elapsed_s'],2))
print('Do not extrapolate persistent capture speed from this process-per-scene benchmark.')
