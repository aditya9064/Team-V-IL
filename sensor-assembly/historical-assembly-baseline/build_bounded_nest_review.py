"""Remote-only evidence viewer for bounded base seating/release; failures retained."""
import argparse,json,html,hashlib,os,subprocess,sys
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args();root=Path(a.run);root.mkdir(exist_ok=True,parents=True);run=root/'full-assembly';rp=run/'report.json';code=Path(__file__).parent
r=json.loads(rp.read_text()) if rp.exists() else {'status':'NO_REPORT','error':'No report; inspect Slurm log'}
check=subprocess.run([sys.executable,str(code/'check_bounded_nest_full.py'),str(rp)],capture_output=True,text=True) if rp.exists() else None
summary={'runtime_status':r['status'],'independent_full_acceptance':bool(check and check.returncode==0),'full_assembly_accepted':bool(check and check.returncode==0),'report_sha256':hashlib.sha256(rp.read_bytes()).hexdigest() if rp.exists() else None,'error':r.get('error'),'checker_stdout':check.stdout if check else '', 'checker_stderr':check.stderr if check else '', 'video_stays_remote':True,'part_results':r.get('part_results'), 'held_pose_independent_scores':r.get('held_pose_independent_scores'), 'held_pose_repeatability':r.get('held_pose_repeatability')}
v=r.get('video',{});video=v.get('path') or v.get('failed_recording_path');player='<p>No recording available.</p>'
if video and Path(video).is_file():
    target=root/'full-assembly-result.mp4'
    if not target.exists():os.link(video,target)
    import imageio_ffmpeg
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(target));summary['video']={'frames':frames,'seconds':seconds,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()};player='<video controls playsinline preload="metadata" controlsList="nodownload" src="full-assembly-result.mp4"></video>'
(root/'review-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
e=lambda x:html.escape(str(x));
for q in [rp,*sorted(run.glob('*-fit.png'))]:
 if q.exists():
  name='full-assembly-report.json' if q.name=='report.json' else q.name
  if not (root/name).exists():os.link(q,root/name)
fitlinks=''.join('<li><a href="'+e(p.name)+'">'+e(p.name)+'</a></li>' for p in sorted(run.glob('*-fit.png')))
body='''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Full assembly with physical three-pad nest</title><style>body{font:17px system-ui;max-width:1100px;margin:24px auto;padding:0 18px;background:#f4f7fa;color:#173044}section{background:white;padding:20px;border-radius:12px;margin:20px 0}video{width:100%;max-height:75vh;background:#111}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style></head><body><h1>Base → loaded tray → lid: strict assembly test</h1><p>Current SMC gripper and fingers unchanged. The physical nest has three support pads and six low tapered guides. The exact rectangular lid feet and base pads now use native box collision shapes after seating-only286456 passed; product geometry, mass and gripper remain unchanged. The base stays fully dynamic under gravity with no lock, joint or clamp. Product parts, gripper and thresholds are unchanged. The density and strict limits are unchanged; mass follows actual CAD volume. Two fixed calibrated inspection cameras measure the held base from RGB; simulation object coordinates are independent scoring/stop evidence only. Known pickup and fixture positions. Only the base uses the verified RGB held-pose correction; tray and lid retain their existing CAD recipe motions. Base, loaded tray and lid in order; PCB separate and loose. No printing or real robot qualification. Video stays on Rosie.</p>'''
body+='<section><h2>'+e('FULL ASSEMBLY ACCEPTED' if summary['independent_full_acceptance'] else 'NOT ACCEPTED — '+r['status'])+'</h2><p>'+e(('Stopped at '+next(iter(r.get('unexpected_contacts',[])),{}).get('phase','a strict gate')+'. Original failure and complete details retained below.') if r.get('error') else 'Independent full assembly checks passed.')+'</p>'+player+'</section><section><h2>Measurements and independent checks</h2><details><summary>Technical evidence</summary><pre>'+e(json.dumps(summary,indent=2))+'</pre></details></section><section><h2>Camera fits</h2><ul>'+fitlinks+'</ul><a href="full-assembly-report.json">Complete original report</a></section></body></html>'
(root/'review.html').write_text(body);print('FULL_ASSEMBLY_REVIEW_READY',json.dumps(summary),flush=True)
