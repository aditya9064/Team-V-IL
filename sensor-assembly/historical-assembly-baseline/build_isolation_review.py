"""Remote-only A-B-A evidence comparison, no automatic design changes."""
from pathlib import Path
import argparse,json,html,os,hashlib
from isolation_tools import comparable,classify_terminal
from check_seating_diagnostic import check
ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args();root=Path(a.run);root.mkdir(exist_ok=True);reports={};rows=[];esc=lambda x:html.escape(str(x))
for label in ['present-a','removed','present-b']:
    path=root/label/'report.json'
    if not path.exists():rows.append(dict(case=label,outcome='NOT_ATTEMPTED',assembly_accepted=False));continue
    r=json.loads(path.read_text());reports[label]=r
    row=dict(case=label,status=r.get('status'),error=r.get('error'),outcome=classify_terminal(r),assembly_accepted=False)
    try:row['evidence_validation']=check(path);row['evidence_valid']=True
    except Exception as e:row.update(evidence_valid=False,validation_error=str(e))
    row['free_held_prediction']=r.get('diagnostic',{}).get('free_held_prediction')
    contacts=r.get('diagnostic_contact_trace',[])
    def is_support(v):return any(x and x[0]=='nest' for x in v['roles'])
    def is_grip(v):return any(x and x[0]=='base' and x[1].startswith('grip_') for x in v['roles'])
    row['first_support_report_step']=next((v['step'] for v in contacts if is_support(v)),None)
    row['first_support_penetrating_contact_step']=next((v['step'] for v in contacts if is_support(v) and v['minimum_separation_mm']<=0),None)
    row['first_grip_violation_step']=next((v['step'] for v in contacts if is_grip(v) and v['minimum_separation_mm']<-.25),None)
    row['maximum_support_depth_mm']=max([max(0,-v['minimum_separation_mm']) for v in contacts if is_support(v)]+[0])
    row['maximum_grip_depth_mm']=max([max(0,-v['minimum_separation_mm']) for v in contacts if is_grip(v)]+[0])
    vid=r.get('video',{});p=Path(vid.get('path') or vid.get('failed_recording_path') or '/NONEXISTENT')
    if p.is_file():
        assert p.resolve().parent==path.resolve().parent
        dest=root/(label+'.mp4')
        if not dest.exists():os.link(p,dest)
        row['video']=dest.name;row['video_sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    dest=root/(label+'-report.json');dest.write_bytes(path.read_bytes());row['report']=dest.name
    rows.append(row)
comparisons={}
for label in ['removed','present-b']:
    if 'present-a' in reports and label in reports:
        try:comparisons[label]=comparable(reports['present-a'],reports[label])
        except Exception as e:comparisons[label]={'comparable':False,'reason':str(e)}
all_valid=len(rows)==3 and all(r.get('evidence_valid') for r in rows) and len(comparisons)==2 and all(c['comparable'] for c in comparisons.values())
outcomes=[r['outcome'] for r in rows]
interpretation='INCONCLUSIVE: evidence or pre-intervention states do not meet comparison conditions.'
if all_valid:
    if outcomes==['STRICT_STOP_DURING_LOWERING','LOWERING_COMPLETED_WITHOUT_STRICT_STOP','STRICT_STOP_DURING_LOWERING']:
        interpretation='Receiving support contributes to failure under this fixed grip and path. This does not identify the exact force/geometry cause, validate release, or qualify assembly.'
    elif outcomes[1]=='STRICT_STOP_DURING_LOWERING':interpretation='Removing the receiving support did not eliminate the strict lowering failure. Support contact alone is insufficient to explain it.'
    else:interpretation='Recorded outcomes did not reproduce the predicted A-B-A pattern; do not infer a fix or a unique cause.'
summary=dict(task='isolated_base_seating_A_B_A',rows=rows,pre_intervention_comparisons=comparisons,comparison_valid=all_valid,interpretation=interpretation,assembly_accepted=False,design_changed=False,hardware_changed=False,strict_numeric_gates_changed=False,video_download_to_laptop=False)
(root/'isolation-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
body='''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Base seating — isolated evidence</title><style>body{font:17px system-ui;color:#193348;background:#f3f6f9;max-width:1100px;margin:24px auto;padding:0 18px}section{background:white;padding:18px;margin:18px 0;border-radius:12px}video{width:100%;max-height:70vh;background:#111}pre{white-space:pre-wrap;overflow-wrap:anywhere}td,th{padding:8px;text-align:left;border-bottom:1px solid #ccc}table{width:100%;border-collapse:collapse}summary{cursor:pointer}</style>'''
body+='<h1>Why does the base bind during seating?</h1><p>Controlled diagnostic: support present → moved away after carry → present again. Same parts, original grippers, commands and strict limits. No assembly or release success is claimed. All recordings stay on Rosie.</p><p>'+esc(interpretation)+'</p>'
for r in rows:
    label=r['case'];body+='<section><h2>'+esc(label)+' — '+esc(r['outcome'])+'</h2>'
    if r.get('video'):body+='<video controls playsinline preload="metadata" controlsList="nodownload" src="'+r['video']+'"></video>'
    body+='<details><summary>Recorded evidence</summary><pre>'+esc(json.dumps(r,indent=2))+'</pre></details>'
    if r.get('report'):body+='<p><a href="'+r['report']+'">Raw report</a></p>'
    body+='</section>'
body+='<details><summary>Were the held poses comparable before the support moved?</summary><pre>'+esc(json.dumps(comparisons,indent=2))+'</pre></details><p>No design changes made. No threshold relaxation. Printing remains on hold.</p>'
(root/'review.html').write_text(body);print(json.dumps({'outcomes':outcomes,'comparison_valid':all_valid,'interpretation':interpretation,'assembly_accepted':False}),flush=True)
