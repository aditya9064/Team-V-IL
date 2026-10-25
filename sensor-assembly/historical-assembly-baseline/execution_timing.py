"""External stdout timing only. No simulator imports or robot/control API."""
import argparse,datetime,json,subprocess,sys,time
from pathlib import Path

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--events',required=True);ap.add_argument('command',nargs=argparse.REMAINDER);a=ap.parse_args();command=a.command
 if command and command[0]=='--':command=command[1:]
 if not command:ap.error('runtime command required')
 start=time.monotonic();p=Path(a.events);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x',buffering=1) as events:
  def note(kind,**fields):events.write(json.dumps({'kind':kind,'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'elapsed_wall_s':time.monotonic()-start,**fields})+'\n')
  note('process_start')
  child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',bufsize=1)
  assert child.stdout is not None
  for line in child.stdout:
   sys.stdout.write(line);sys.stdout.flush()
   if line.startswith('[assembly]') or 'Startup Complete' in line or 'Traceback' in line or 'MOTION_COMPLETED' in line:note('runtime_marker',text=line.rstrip('\n'))
  code=child.wait();note('process_exit',returncode=code)
 return code
if __name__=='__main__':raise SystemExit(main())
