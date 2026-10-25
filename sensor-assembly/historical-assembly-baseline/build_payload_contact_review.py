"""Keep mechanical rejection separate from payload/lid acquisition integrity."""
import argparse,json,subprocess,sys,hashlib
from pathlib import Path
import build_contact_trace_review as parent

def main():
 parent.main()
 ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args();root=Path(a.run);rp=root/'full-assembly/report.json'
 c=subprocess.run([sys.executable,str(Path(__file__).with_name('check_payload_contact_trace.py')),str(rp)],capture_output=True,text=True)
 p=root/'review-summary.json';summary=json.loads(p.read_text());summary['payload_contact_acquisition']={'checker_returncode':c.returncode,'acquisition_integrity_checked':c.returncode==0,'stdout':c.stdout,'stderr':c.stderr,'mechanical_success_checked_here':False,'not_stage_or_full_assembly_acceptance':True}
 p.write_text(json.dumps(summary,indent=2)+'\n');(root/'payload-contact-acquisition.json').write_text(json.dumps(summary['payload_contact_acquisition'],indent=2)+'\n')
 page=root/'review.html';page.write_text(page.read_text().replace('</main>','<section><strong>PCB/tray contact diagnostic</strong><p>'+('Recording integrity checked.' if c.returncode==0 else 'Recording incomplete or unverified.')+' All original motion outcomes remain separate. Cooked data is a post-trial cooking-service representation, not direct runtime PxShape readback.</p><a href="payload-contact-acquisition.json">Payload acquisition audit</a></section></main>'))
 print('PAYLOAD_CONTACT_REVIEW_READY',json.dumps(summary['payload_contact_acquisition']),flush=True)
if __name__=='__main__':main()
