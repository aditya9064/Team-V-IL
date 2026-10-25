"""Create a remote HTML/video review even when the GPU trial fails."""
import argparse,html,json,subprocess,sys
from pathlib import Path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args();run=Path(a.run)
    run.mkdir(parents=True,exist_ok=True);path=run/'report.json'
    r=json.loads(path.read_text()) if path.exists() else {'status':'NO_REPORT','error':'The trial ended before a report was saved.','part_results':[]}
    status=r.get('status','UNKNOWN');verification=None
    if status=='SIMULATION_PASS':
        proc=subprocess.run([sys.executable,str(Path(__file__).with_name('check_holder_assembly.py')),str(path)],capture_output=True,text=True,timeout=120)
        verification={'exit_code':proc.returncode,'output':proc.stdout[-4000:],'error':proc.stderr[-4000:]}
        if proc.returncode:status='INDEPENDENT_REVIEW_FAILED'
    if status in ('STARTING','RUNNING'):status='INCOMPLETE_NO_VERIFIED_RESULT'
    candidates=[run/Path(name).name for name in [r.get('video',{}).get('path',''),r.get('video',{}).get('failed_recording_path','')] if name]
    video=next((p for p in candidates if p.is_file()),None);playback=None
    if video is not None:
        try:
            import imageio_ffmpeg
            frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(video));playback={'frames':frames,'seconds':seconds,'bytes':video.stat().st_size}
        except Exception as e:playback={'error':str(e)}
    summary={'design_revision':r.get('design_revision'),'review_status':status,'runtime_status':r.get('status'),'error':r.get('error'),'part_results':r.get('part_results',[]),'last_checkpoint':r.get('checkpoints',[{}])[-1].get('phase') if r.get('checkpoints') else None,'contact_violations':r.get('unexpected_contacts',[]),'verification':verification,'video_file':video.name if video else None,'video_playback':playback,'video_downloaded_to_laptop':False,'print_release':False}
    (run/'review-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    esc=lambda s:html.escape(str(s));rows=[]
    results={p['part']:p for p in summary['part_results']}
    for name in r.get('part_order',['base']):
        p=results.get(name)
        if p is None:rows.append(f'<tr><td>{esc(name)}</td><td colspan="4">Not reached / no accepted result</td></tr>');continue
        rows.append('<tr>'+''.join('<td>'+esc(v)+'</td>' for v in [name,p.get('pickup',{}).get('pass',False),p.get('placement_verified',False),p.get('release_verified',False),p.get('placement',{}).get('xy_mm','—')])+'</tr>')
    if video and playback and 'error' not in playback:
        player=f'<video controls playsinline preload="metadata" controlsList="nodownload" src="{esc(video.name)}"></video><p>{playback["frames"]} frames · {playback["seconds"]:.1f} seconds. The MP4 stays in this Rosie folder.</p>'
    else:player='<p>No complete playable recording was produced. Check the report and job log; a startup or termination failure is not an assembly result.</p>'
    image=next((run/name for name in ['FAILED-stop-overview.png','final_assembly_inspection-overview.png','FAILED-stop-side.png','FAILED-stop-assembly.png','final_assembly_inspection-assembly.png'] if (run/name).exists()),None)
    picture=f'<img src="{esc(image.name)}" alt="Recorded simulation inspection view">' if image else ''
    body=f'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Team V assembly review</title><style>body{{font:17px system-ui;max-width:1100px;margin:30px auto;padding:0 18px;color:#173044;background:#f4f7fa}}video,img{{width:100%;max-height:78vh;background:#111}}section{{background:white;padding:22px;margin:18px 0;border-radius:10px}}table{{border-collapse:collapse;width:100%}}td,th{{padding:10px;text-align:left;border-bottom:1px solid #ddd}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}.badge{{font-weight:bold;color:{'#176838' if status=='SIMULATION_PASS' else '#a13a18'}}}</style></head><body><h1>PCB holder — base pickup pilot</h1><p class="badge">{esc(status)}</p><p>Known starting positions; existing modeled gripper. This tests the base only. It does not test the PCB, loaded carrier, retainer or cover. No real hardware or trained perception. Print release remains pending.</p><section>{player}</section><section><h2>What happened</h2><p>Last saved stage: {esc(summary['last_checkpoint'])}</p><details><summary>Technical stop details</summary><pre>{esc(summary['error'] or '')}</pre></details><table><tr><th>Part</th><th>Pickup</th><th>Placement</th><th>Release</th><th>XY error (mm)</th></tr>{''.join(rows)}</table></section><section>{picture}<p>Raw evidence and numerical measurements: <a href="report.json">report.json</a> · <a href="review-summary.json">review-summary.json</a></p></section></body></html>'''
    body=body.replace('PCB holder — base pickup pilot','PCB holder — '+esc(r.get('case','unknown case'))).replace('This tests the base only. It does not test the PCB, loaded carrier, retainer or cover.','Printed machine-monitor prototype: base, dummy-board tray and lid. No electronics was purchased. Three separate product pieces start at different pickup locations: base, loaded tray and lid. The robot must place them in that order. Only the nest is prepositioned. The intact dummy PCB is preloaded in its tray and remains a separate gravity-enabled body; this test does not load the bare PCB or fasten the lid.')
    if r.get('case')=='loaded-carrier':body=body.replace('The robot must place them in that order. Only the nest is prepositioned.','This preliminary trial moves only the loaded dummy-board tray; base and nest are prepositioned. Full assembly is attempted only after this trial passes.')
    (run/'review.html').write_text(body)
    print('ASSEMBLY_REVIEW_SAVED',status,str(run/'review.html'),flush=True)


if __name__=='__main__':main()
