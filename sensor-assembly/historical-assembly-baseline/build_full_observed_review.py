"""Review the actual full-run recording, including unsuccessful/inconclusive runs."""
import argparse,hashlib,html,json,subprocess,sys
from pathlib import Path
def digest(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def build(root):
 root=Path(root);root.mkdir(parents=True,exist_ok=True);source=Path(__file__).resolve().parent;path=root/'full-assembly/report.json';summary={'schema':'FULL_OBSERVED_ASSEMBLY_REVIEW_V60','full_assembly_accepted':False,'media_stays_remote':True,'actual_video_visually_reviewed':False}
 if not path.exists():
  summary.update(runtime_status='NO_REPORT',audit={'classification':'INCONCLUSIVE_NO_REPORT'})
  media='<p>No report or finalized movie is available. Outcome is unknown.</p>'
 else:
  r=json.loads(path.read_text());meta=json.loads((source/'FULL-SOURCE.json').read_text());manifest=digest(source/'snapshot-sha256.json')
  if digest(source/meta['runtime_file'])!=meta['runtime_sha256'] or digest(source/'check_full_observed_assembly.py')!=meta['checker_sha256']:raise ValueError('Full review source differs from sealed identity')
  result=subprocess.run([sys.executable,str(source/'check_full_observed_assembly.py'),str(path),'--source',str(source),'--runtime-file',meta['runtime_file'],'--runtime-sha256',meta['runtime_sha256'],'--manifest-sha256',manifest],capture_output=True,text=True)
  (root/'independent-checker.log').write_text(result.stdout+'\n'+result.stderr)
  audit_path=path.with_name('independent-full-observed-audit.json');audit=json.loads(audit_path.read_text()) if audit_path.exists() else {'classification':'INCONCLUSIVE_NO_AUDIT','full_assembly_accepted':False}
  summary.update(runtime_status=r.get('status'),runtime_error=r.get('error'),audit=audit,checker_exit=result.returncode,part_results=r.get('part_results',[]),raw_report_sha256=digest(path),runtime_sha256=meta['runtime_sha256'],source_manifest_sha256=manifest,full_assembly_accepted=audit.get('full_assembly_accepted') is True)
  v=r.get('video',{});movie=v.get('path') or v.get('failed_recording_path');media='<p>No finalized recording. The report identifies the stopping point.</p>'
  if movie and Path(movie).is_file():
   actual=Path(movie).resolve()
   if root.resolve() not in actual.parents:raise ValueError('Recording outside this run')
   alias=root/'assembly.mp4'
   if alias.exists():
    if alias.is_symlink() or alias.stat().st_ino!=actual.stat().st_ino:raise ValueError('Movie alias identity mismatch')
   else:alias.hardlink_to(actual)
   try:
    import imageio_ffmpeg
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(actual));summary['video_verified']={'sha256':digest(actual),'frames':frames,'seconds':seconds,'remote_path':str(actual)}
    media='<video controls preload="metadata" src="assembly.mp4"></video>'
   except Exception as e:summary['video_error']=type(e).__name__+': '+str(e)
 classification=summary['audit']['classification'];status=summary['runtime_status'];error=summary.get('runtime_error')
 if summary['full_assembly_accepted']:title='Full assembly — independent simulation checks passed'
 elif status=='MOTION_COMPLETE':title='Full sequence completed — inspect support evidence'
 elif status=='FAILED':title='Full assembly — stopped at a check'
 else:title='Full assembly — outcome not yet verified'
 rows=''.join('<tr><td>'+html.escape(str(r['part']))+'</td><td>'+('Yes' if r.get('pickup',{}).get('pass') else 'Unverified')+'</td><td>'+('Yes' if r.get('placement_verified') else 'Unverified')+'</td><td>'+('Yes' if r.get('release_verified') else 'Unverified')+'</td></tr>' for r in summary.get('part_results',[]))
 page='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Team V — full assembly trial</title><style>body{font:16px system-ui;background:#151515;color:#f4f4f4;max-width:1120px;margin:28px auto;padding:0 20px}h1{font-size:28px}video{width:100%;max-height:760px;background:#000}.note{border-left:4px solid #f77c24;background:#242424;padding:14px}p{line-height:1.5}a{color:#ff9b45}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:10px;border-bottom:1px solid #444}</style>'''
 page+=f'<h1>{html.escape(title)}</h1><p class="note">Actual Isaac Sim recording: base → loaded tray → original lid. The PCB starts inside its tray as a separate loose body. Only the nest is prepositioned. Supplied SMC gripper and fingers are unchanged. Known pickup positions; no claim of random-pose, physical robot or printing readiness.</p><p>Runtime: {html.escape(str(status))}. {html.escape(error or "")}</p><p>Independent outcome: <strong>{html.escape(classification)}</strong>. Motion completion and the strict support-force check are reported separately. Missing force reports remain inconclusive.</p>'+media
 if rows:page+='<table><tr><th>Part</th><th>Pickup</th><th>Placement</th><th>Release</th></tr>'+rows+'</table>'
 page+='<p><a href="review-summary.json">Checked results and provenance</a></p></html>'
 (root/'review-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');(root/'review.html').write_text(page)
 print('FULL_OBSERVED_ASSEMBLY_REVIEW_READY',root,classification,flush=True)
 return summary
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);build(ap.parse_args().run)
