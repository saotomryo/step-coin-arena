"""Loopback-only presentation/control service. Commands are queued to physics.

No server thread calls Genesis. A per-process token prevents cross-origin writes.
"""
import json,secrets,hmac,queue
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
from threading import Thread

class ControlServer:
    def __init__(self,panel_path,port=0):
        self.token=secrets.token_urlsafe(32);self.commands=queue.Queue(maxsize=16);self.state={'control_status':'preparing'}
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def send(self,status,value,content_type='application/json'):
                body=value.encode() if isinstance(value,str) else json.dumps(value).encode()
                self.send_response(status);self.send_header('Content-Type',content_type+'; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def valid_host(self):return self.headers.get('Host')==f'127.0.0.1:{owner.server.server_port}'
            def do_GET(self):
                if not self.valid_host():return self.send(403,{'error':'invalid host'})
                if self.path=='/':return self.send(200,Path(panel_path).read_text().replace('__TOKEN__',owner.token),'text/html')
                if self.path=='/state':return self.send(200,owner.state)
                self.send(404,{'error':'not found'})
            def do_POST(self):
                if not self.valid_host():return self.send(403,{'error':'invalid host'})
                origin=self.headers.get('Origin')
                if origin and origin!=owner.url:return self.send(403,{'error':'invalid origin'})
                if not hmac.compare_digest(self.headers.get('X-Control-Token','').encode(),owner.token.encode()):return self.send(403,{'error':'invalid token'})
                if self.path!='/command':return self.send(404,{'error':'not found'})
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=4096:raise ValueError('invalid length')
                    command=json.loads(self.rfile.read(length))
                    action=command['action']
                    if action not in ('start','pause','resume','stop','reset','shutdown'):raise ValueError('unknown action')
                    owner.commands.put_nowait(action)
                except queue.Full:return self.send(429,{'error':'command queue full'})
                except (ValueError,KeyError,TypeError):return self.send(400,{'error':'invalid command'})
                self.send(202,{'queued':action})
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler);self.url=f'http://127.0.0.1:{self.server.server_port}'
        self.thread=Thread(target=self.server.serve_forever,daemon=True)
    def start(self):self.thread.start()
    def close(self):self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)
