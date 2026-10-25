"""Read-only, loopback-only, expiring remote viewer. Stream; never copy MP4 locally."""
import argparse,json,mimetypes,os,re,secrets,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote,urlsplit


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--lifetime',type=int,default=3600);a=ap.parse_args()
    root=Path(a.run).resolve();token=secrets.token_urlsafe(24);prefix='/'+token+'/'
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_HEAD(self):self.respond(False)
        def do_GET(self):self.respond(True)
        def respond(self,body):
            path=unquote(urlsplit(self.path).path)
            if not path.startswith(prefix):self.send_error(404);return
            name=path[len(prefix):] or 'review.html'
            if '/' in name or name.startswith('.') or Path(name).suffix not in ('.html','.json','.png','.mp4'):self.send_error(404);return
            f=(root/name).resolve()
            if f.parent!=root or not f.is_file():self.send_error(404);return
            size=f.stat().st_size;start=0;end=size-1;partial=False
            if self.headers.get('Range'):
                m=re.fullmatch(r'bytes=(\d+)-(\d*)',self.headers['Range'])
                if not m:self.send_error(416);return
                start=int(m[1]);end=min(size-1,int(m[2])) if m[2] else size-1;partial=True
                if start>end or start>=size:self.send_error(416);return
            self.send_response(206 if partial else 200)
            self.send_header('Content-Type',mimetypes.guess_type(name)[0] or 'application/octet-stream')
            self.send_header('Content-Length',str(end-start+1));self.send_header('Accept-Ranges','bytes')
            self.send_header('Cache-Control','no-store, max-age=0');self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Disposition','inline')
            if partial:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
            self.end_headers()
            if body:
                try:
                    with f.open('rb') as stream:
                        stream.seek(start);remaining=end-start+1
                        while remaining:
                            chunk=stream.read(min(256*1024,remaining))
                            if not chunk:break
                            self.wfile.write(chunk);remaining-=len(chunk)
                except (BrokenPipeError,ConnectionResetError):pass
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    info={'pid':os.getpid(),'remote_port':server.server_port,'prefix':prefix,'page':'review.html','loopback_only':True,'lifetime_seconds':a.lifetime}
    fd=os.open(str(root/'review-viewer.json'),os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
    with os.fdopen(fd,'w') as stream:json.dump(info,stream)
    timer=threading.Timer(a.lifetime,server.shutdown);timer.daemon=True;timer.start()
    try:server.serve_forever(poll_interval=1)
    finally:server.server_close()


if __name__=='__main__':main()
