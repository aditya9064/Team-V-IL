"""Explicit parent-tray/current-full provenance; videos remain remote."""
import argparse,json,html,os,shutil,subprocess,sys
from pathlib import Path
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args();root=Path(a.run);root.mkdir(exist_ok=True,parents=True);here=Path(__file__).parent
 parent=Path.home()/'team-v-runs/pcb-holder-suite-285573/loaded-carrier';old=Path.home()/'team-v-tests/supported-tray-v13d-20261019-01'
 cards=[];results={}
 for label,run,code,prefix in [('Loaded tray — passed parent job285573',parent,old,'parent-285573-loaded-carrier'),('Full assembly — current job',root/'full-assembly',here,'full-assembly')]:
  subprocess.run([sys.executable,str(code/'build_assembly_review.py'),'--run',str(run)],check=True)
  s=json.loads((run/'review-summary.json').read_text());results[prefix]=dict(source_run=str(run),source_code=str(code),summary=s)
  if (run/'report.json').exists():shutil.copy2(run/'report.json',root/(prefix+'-report.json'))
  esc=lambda v:html.escape(str(v));player='<p>No motion recording available.</p>'
  if s.get('video_file'):
   source=run/Path(s['video_file']).name;target=root/(prefix+'.mp4')
   if not target.exists():os.link(source,target)
   player=f'<video controls playsinline preload="metadata" controlsList="nodownload" src="{esc(target.name)}"></video>'
  color='#176838' if s['review_status']=='SIMULATION_PASS' else '#a13a18'
  cards.append(f'<section><h2>{esc(label)}</h2><p style="color:{color};font-weight:bold">{esc(s["review_status"])}</p>{player}<p>Last checkpoint: {esc(s.get("last_checkpoint"))}</p><pre>{esc(s.get("error") or "Independent report and video check passed.")}</pre><a href="{prefix}-report.json">Report with original run provenance</a></section>')
 (root/'suite-summary.json').write_text(json.dumps(dict(cases=results,geometry_same_between_cases=True,only_cover_probe_expectation_corrected=True,loaded_and_full_are_different_jobs=True,videos_downloaded_to_laptop=False,print_release=False),indent=2)+'\n')
 body='''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Repaired tray and full assembly</title><style>body{font:17px system-ui;max-width:1100px;margin:24px auto;padding:0 18px;background:#f4f7fa;color:#173044}section{background:white;border-radius:12px;padding:20px;margin:20px 0}video{width:100%;max-height:75vh;background:#111}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style></head><body><h1>Repaired tray — loaded and full-assembly verification</h1><p>Current SMC gripper and fingers unchanged. Printed dummy PCB stays separate and intact. Known starting positions; no trained perception or real robot. The loaded-tray pass is from parent job285573, independently rechecked before the current full trial. Product geometry is identical; only the full-case lid probe expectation was corrected. Videos remain on Rosie. No print release.</p>'''+''.join(cards)+'</body></html>'
 (root/'review.html').write_text(body);print('COMBINED_PARENT_AND_FULL_REVIEW_SAVED',str(root/'review.html'),flush=True)
if __name__=='__main__':main()
