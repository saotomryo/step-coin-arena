"""Local control boundary: reads are public; writes require token and origin."""
import unittest,json,urllib.request,urllib.error
from pathlib import Path
from xrobocon.control_server import ControlServer

class ControlServerTests(unittest.TestCase):
    def setUp(self):
        self.server=ControlServer(Path(__file__).resolve().parents[1]/'xrobocon/assets/competition_panel.html')
        self.server.start()
    def tearDown(self):self.server.close()
    def post(self,body,token=None,extra=None):
        headers={'Content-Type':'application/json','X-Control-Token':self.server.token if token is None else token}
        headers.update(extra or {})
        req=urllib.request.Request(self.server.url+'/command',data=body,headers=headers)
        try:return urllib.request.urlopen(req).status
        except urllib.error.HTTPError as error:return error.code
    def test_bad_requests_cannot_queue_commands(self):
        self.assertEqual(self.post(b'{"action":"start"}',token=''),403)
        self.assertEqual(self.post(b'{"action":"start"}',extra={'Origin':'http://example.invalid'}),403)
        self.assertEqual(self.post(b'{"action":"start"}',extra={'Host':'example.invalid'}),403)
        self.assertEqual(self.post(b'not-json'),400)
        self.assertEqual(self.post(b'{"action":"unknown"}'),400)
        self.assertTrue(self.server.commands.empty())
    def test_valid_control_and_public_state(self):
        self.server.state={'control_status':'ready','remaining':180}
        state=json.loads(urllib.request.urlopen(self.server.url+'/state').read())
        self.assertEqual(state['remaining'],180)
        self.assertEqual(self.post(b'{"action":"start"}'),202)
        self.assertEqual(self.server.commands.get_nowait(),'start')
        for _ in range(16):self.server.commands.put_nowait('pause')
        self.assertEqual(self.post(b'{"action":"pause"}'),429)

if __name__=='__main__':unittest.main()
