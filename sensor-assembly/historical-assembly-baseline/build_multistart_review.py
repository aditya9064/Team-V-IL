"""Generate a truthful remote-only review of a staged lid-pose trial."""
import argparse
import hashlib
import html
import json
import os
from pathlib import Path
import subprocess
import sys


def acceptance(report, checker_ok):
    stage_ok = bool(checker_ok and report.get('diagnostic_stage_accepted') == report.get('diagnostic_stage'))
    full_ok = bool(stage_ok and report.get('diagnostic_stage') == 'release'
                   and report.get('full_assembly_accepted') is True)
    return stage_ok, full_ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    args = parser.parse_args()
    root = Path(args.run)
    root.mkdir(parents=True, exist_ok=True)
    rp = root / 'full-assembly' / 'report.json'
    report = json.loads(rp.read_text()) if rp.is_file() else {'status': 'NO_REPORT', 'error': 'No report; inspect the Slurm log.'}
    check = subprocess.run([sys.executable, str(Path(__file__).with_name('check_multistart_lid_precontact.py')), str(rp)],
                           capture_output=True, text=True) if rp.is_file() else None
    stage_ok, full_ok = acceptance(report, bool(check and check.returncode == 0))
    summary = {
        'runtime_status': report['status'],
        'diagnostic_stage': report.get('diagnostic_stage'),
        'independent_stage_acceptance': stage_ok,
        'independent_full_acceptance': full_ok,
        'full_assembly_accepted': full_ok,
        'report_sha256': hashlib.sha256(rp.read_bytes()).hexdigest() if rp.is_file() else None,
        'error': report.get('error'),
        'checker_stdout': check.stdout if check else '',
        'checker_stderr': check.stderr if check else '',
        'part_results': report.get('part_results', []),
        'video_stays_remote': True,
    }
    video = report.get('video', {})
    path = Path(video.get('path') or video.get('failed_recording_path') or '/nonexistent')
    player = '<p>No recording is available; inspect the report and log.</p>'
    if path.is_file():
        alias = root / 'full-assembly-result.mp4'
        if not alias.exists():
            os.link(path, alias)
        import imageio_ffmpeg
        frames, seconds = imageio_ffmpeg.count_frames_and_secs(str(alias))
        summary['video'] = {'frames': frames, 'seconds': seconds,
                            'sha256': hashlib.sha256(alias.read_bytes()).hexdigest()}
        player = '<video controls playsinline preload="metadata" controlsList="nodownload" src="full-assembly-result.mp4"></video>'
    if rp.is_file() and not (root / 'full-assembly-report.json').exists():
        os.link(rp, root / 'full-assembly-report.json')
    for png in rp.parent.glob('*-fit.png'):
        if not (root / png.name).exists():
            os.link(png, root / png.name)
    (root / 'review-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    e = lambda v: html.escape(str(v))
    title = ('Full simulated assembly accepted' if full_ok else
             'Diagnostic stage accepted — full assembly unproven' if stage_ok else
             'Stopped — not accepted')
    rows = ''.join('<tr>' + ''.join('<td>' + e(v) + '</td>' for v in (
        {'base': 'Base', 'carrier': 'Tray + separate PCB', 'cover': 'Lid'}.get(p['part'], p['part']),
        'Passed' if p['pickup']['pass'] else 'Failed',
        'Passed' if p.get('placement_verified') else 'Unverified',
        'Passed' if p.get('release_verified') else 'Unverified')) + '</tr>'
        for p in report.get('part_results', []))
    fits = ''.join('<li><a href="' + e(p.name) + '">' + e(p.name) + '</a></li>'
                   for p in sorted(rp.parent.glob('*-fit.png')))
    page = '''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Team V — measured lid placement</title><style>*{box-sizing:border-box}body{margin:0;background:#f5f5f5;color:#171717;font:16px/1.5 system-ui,Arial}main{max-width:1120px;margin:26px auto;padding:0 18px}h1{font-size:28px}section{background:white;border:1px solid #ddd;padding:18px;margin:18px 0}.status{border-left:5px solid #df6a24}video{width:100%;max-height:72vh;background:#111}table{width:100%;border-collapse:collapse}th,td{text-align:left;border-bottom:1px solid #ddd;padding:8px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}a{color:#aa4914}</style></head><body><main>
<h1>Base → loaded tray → lid</h1><p>Actual Rosie simulation • current grippers unchanged</p>'''
    page += '<section class="status"><strong>' + e(title) + '</strong><p>Stage: ' + e(report.get('diagnostic_stage', 'not started')) + '.</p><p>' + e(report.get('error', 'The staged independent checker accepted only the scope stated above.')) + '</p></section>'
    page += '<section>' + player + '<p>The video stays on Rosie and streams privately. A precontact or seating diagnostic is not a completed assembly.</p></section>'
    page += '<section><table><tr><th>Part</th><th>Pickup</th><th>Placement</th><th>Release</th></tr>' + rows + '</table></section>'
    page += '<p>Known starting positions. Intact, separate, loose PCB. No printing or real-arm qualification.</p><details><summary>Original evidence and measurements</summary><a href="full-assembly-report.json">Complete report</a><ul>' + fits + '</ul><pre>' + e(json.dumps(summary, indent=2)) + '</pre></details></main></body></html>'
    (root / 'review.html').write_text(page)
    print('LID_POSE_REVIEW_READY', json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
