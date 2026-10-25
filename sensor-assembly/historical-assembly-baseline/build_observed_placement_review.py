"""Truthful remote-only actual recording; no invented animation or full pass."""
import argparse,hashlib,html,json,subprocess,sys
from pathlib import Path

def build(root):
 root=Path(root);source=Path(__file__).parent;p=root/'original/report.json'
 if not p.exists():
  root.mkdir(exist_ok=True);(root/'review.html').write_text('<h1>Original lid placement diagnostic</h1><p>No report exists. Outcome unknown; no success claim.</p>');return
 r=json.loads(p.read_text());meta=json.loads((source/'PLACEMENT-SOURCE.json').read_text())
 result=subprocess.run([sys.executable,str(source/'check_observed_lid_placement.py'),str(p),'--source',str(source),'--runtime-sha256',meta['runtime_sha256']],capture_output=True,text=True)
 (root/'independent-checker.log').write_text(result.stdout+'\n'+result.stderr)
 a=root/'original/independent-placement-audit.json';audit=json.loads(a.read_text()) if a.exists() else {'classification':'INCONCLUSIVE_NO_AUDIT','isolated_lid_placement_accepted':False}
 summary={'schema':'ORIGINAL_LID_PLACEMENT_DIAGNOSTIC_REVIEW','runtime_status':r['status'],'runtime_error':r.get('error'),'audit':audit,'checker_exit':result.returncode,'full_assembly_accepted':False,'actual_video_visually_reviewed':False,'media_stays_remote':True}
 v=r.get('video',{});path=v.get('path') or v.get('failed_recording_path');media='<p>No finalized recording. The report identifies the stopping point.</p>'
 if path and Path(path).is_file():
  actual=Path(path).resolve();assert root.resolve() in actual.parents
  alias=root/'original.mp4'
  if not alias.exists():alias.hardlink_to(actual)
  else:assert not alias.is_symlink() and alias.stat().st_ino==actual.stat().st_ino
  import imageio_ffmpeg
  frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(actual));summary['video_verified']={'sha256':hashlib.sha256(actual.read_bytes()).hexdigest(),'frames':frames,'seconds':seconds,'remote_path':str(actual)}
  media='<video controls preload="metadata" src="original.mp4"></video>'
 (root/'review-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
 title='Original lid — actual placement diagnostic'
 page='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Original lid placement diagnostic</title><style>body{font:16px system-ui;background:#141414;color:#eee;max-width:1100px;margin:32px auto;padding:0 20px}video{width:100%;max-height:720px;background:#000}a{color:#ff9b45}.note{border-left:4px solid #f77c24;padding:14px;background:#242424}p{line-height:1.5}</style>'''
 page+=f'<h1>{title}</h1><p class="note">Actual Isaac Sim recording. Original lid and supplied fingers. Base and loaded tray start prepositioned as dynamic test fixtures. The old 0.020 mm holding criterion is recorded as an advisory in this separately authorized experiment. This is not a complete robot-built assembly or physical qualification.</p>'
 page+=f'<p>Runtime: {html.escape(r["status"])}. {html.escape(r.get("error") or "Motion sequence completed; inspect independent support/outcome audit.")}</p><p>Independent outcome: <strong>{html.escape(audit["classification"])}</strong></p>'+media+'<p><a href="review-summary.json">Outcome, historical movement measurements, and evidence</a></p></html>'
 (root/'review.html').write_text(page);print('ORIGINAL_LID_PLACEMENT_REVIEW_READY',root)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);build(ap.parse_args().run)
