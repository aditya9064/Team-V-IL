"""Append acquisition integrity separately; never promote mechanical acceptance."""
import argparse,json,html,subprocess,sys
from pathlib import Path
import build_multistart_review as parent

def append_acquisition(summary,returncode,stdout,stderr):
    result=dict(summary)
    result['contact_telemetry_acquisition']={'checker_returncode':returncode,'acquisition_integrity_checked':returncode==0,'stdout':stdout,'stderr':stderr,'mechanical_success_checked_here':False,'not_stage_or_full_assembly_acceptance':True}
    return result

def main():
    parent.main()
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args();root=Path(a.run)
    rp=root/'full-assembly/report.json'
    c=subprocess.run([sys.executable,str(Path(__file__).with_name('check_contact_trace_lid_precontact.py')),str(rp)],capture_output=True,text=True)
    sp=root/'review-summary.json';summary=append_acquisition(json.loads(sp.read_text()),c.returncode,c.stdout,c.stderr);sp.write_text(json.dumps(summary,indent=2)+'\n')
    evidence=summary['contact_telemetry_acquisition'];(root/'contact-telemetry-acquisition.json').write_text(json.dumps(evidence,indent=2)+'\n')
    page=root/'review.html';extra='<section><strong>Finger-contact recording</strong><p>'+('Recording-integrity checks completed.' if c.returncode==0 else 'Recording integrity is incomplete or unverified.')+' This is an evidence check, not an assembly or motion pass.</p><a href="contact-telemetry-acquisition.json">Contact-recording audit</a></section>'
    page.write_text(page.read_text().replace('</main>',extra+'</main>'))
    print('CONTACT_TELEMETRY_REVIEW_READY',json.dumps(evidence),flush=True)
if __name__=='__main__':main()
