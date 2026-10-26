"""Fixed desktop destinations; local activation reuses open windows across processes."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
from PySide6.QtCore import QTimer
from PySide6.QtNetwork import QLocalServer,QLocalSocket
ROOT=Path(__file__).resolve().parents[1]


def default_library():
    from xrobocon.storage import resource_root
    return resource_root()/'robot_workshop'


class DesktopLinks:
    def __init__(self,surface,library=None,catalog=None,out_root=None):
        self.surface=surface;self.library=Path(library).resolve() if library else default_library()
        self.catalog=Path(catalog).resolve() if catalog else ROOT/'configs/match_catalog.json'
        self.out_root=Path(out_root).resolve() if out_root else None
        self.server=None;self.sockets=[];self.children={}
    def key(self,target):
        return 'xrobocon-'+hashlib.sha256((str(ROOT)+'|'+str(self.library)+'|'+target).encode()).hexdigest()[:24]
    def activate(self,target):
        socket=QLocalSocket();socket.connectToServer(self.key(target))
        if not socket.waitForConnected(150):socket.abort();return False
        socket.write(b'activate\n');socket.waitForBytesWritten(200);socket.disconnectFromServer();return True
    def claim(self,window,on_activate=None):
        if self.activate(self.surface):return False
        self.server=QLocalServer(window)
        self.server.setSocketOptions(QLocalServer.SocketOption.UserAccessOption)
        if not self.server.listen(self.key(self.surface)):
            if self.activate(self.surface):return False
            QLocalServer.removeServer(self.key(self.surface))
            if not self.server.listen(self.key(self.surface)):raise RuntimeError('画面の起動を登録できません: '+self.server.errorString())
        def accept():
            while self.server.hasPendingConnections():
                socket=self.server.nextPendingConnection();self.sockets.append(socket)
                def activate_window(socket=socket):
                    if b'activate' not in bytes(socket.readAll()):return
                    window.showNormal();window.raise_();window.activateWindow()
                    if on_activate:on_activate()
                    socket.disconnectFromServer()
                socket.readyRead.connect(activate_window)
                socket.disconnected.connect(socket.deleteLater)
                if socket.bytesAvailable():activate_window()
        self.server.newConnection.connect(accept)
        return True
    def open(self,target):
        if target not in ('setup','workshop'):raise ValueError('unknown desktop destination')
        if self.activate(target):return 'activated'
        child=self.children.get(target)
        if child is not None and child.poll() is None:return 'starting'
        from xrobocon.workshop.design import ensure_storage
        folder=ensure_storage(self.library)/'jobs';folder.mkdir(parents=True,exist_ok=True)
        script='launch_competition.py' if target=='setup' else 'robot_workshop.py'
        catalog=self.library/'catalog.json'
        command=[sys.executable,str(ROOT/'scripts'/script),'--library',str(self.library),'--catalog',str(catalog if catalog.is_file() else self.catalog)]
        if target=='setup' and self.out_root:command+=['--out-root',str(self.out_root)]
        with (folder/f'ui-{target}-{time.time_ns()}.log').open('w') as log:
            self.children[target]=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return 'opened'
