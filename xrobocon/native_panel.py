"""Python operator window; separate GUI process, queue-only simulation access."""
import multiprocessing
import queue

LABELS={'ready':'待機中','running':'競技中','paused':'一時停止','finished':'競技終了','error':'エラー停止'}
ACTIONS=[('start','競技開始'),('pause','一時停止'),('resume','再開'),('stop','競技終了'),('reset','初期状態へ戻す'),('shutdown','環境を終了')]

def allowed(action,status):
    return action in ('reset','shutdown') or action=='start' and status in ('ready','finished') or action=='pause' and status=='running' or action=='resume' and status=='paused' or action=='stop' and status in ('running','paused')

def decision_status(state):
    parts=[]
    for name,agent in state.get('agent_states',{}).items():
        color={'self':'青','opponent':'赤'}.get(name,name)
        if agent.get('decision_error'):parts.append(f'{color}:AIエラー（ログ参照）')
        elif agent.get('decision_pending'):parts.append(f'{color}:Codex推論待ち')
        elif agent.get('remaining_coin_ids'):parts.append(f"{color}:計画 "+'→'.join(map(str,agent['remaining_coin_ids'][:5])))
        elif agent.get('target_id') is not None:parts.append(f"{color}:目標コイン{agent['target_id']}")
    return ' / '.join(parts)

def _window(states,commands,ready,screenshot,navigation=None):
    from xrobocon.workshop.qt_runtime import prepare_plugins
    prepare_plugins()
    from PySide6.QtWidgets import QApplication
    from PySide6.QtGui import QFont
    from xrobocon.desktop_navigation import DesktopLinks
    from xrobocon.operator_window import OperatorWindow
    app=QApplication([]);app.setFont(QFont('Hiragino Sans',12))
    links=DesktopLinks('operator',**(navigation or {}))
    window=OperatorWindow(states,commands,links,screenshot);window.show()
    ready.set();app.exec()

class NativePanel:
    def __init__(self,screenshot=None,navigation=None):
        ctx=multiprocessing.get_context('spawn')
        self.states=ctx.Queue(maxsize=2);self.commands=ctx.Queue();self.ready=ctx.Event()
        self.process=ctx.Process(target=_window,args=(self.states,self.commands,self.ready,screenshot,navigation),daemon=True)
        self._state={}
    @property
    def state(self):return self._state
    @state.setter
    def state(self,value):
        self._state=value
        try:self.states.put_nowait(value)
        except queue.Full:
            try:self.states.get_nowait()
            except queue.Empty:pass
            try:self.states.put_nowait(value)
            except queue.Full:pass
    def start(self):
        self.process.start()
        if not self.ready.wait(20):raise RuntimeError('Python操作ウィンドウの起動に失敗しました')
    def close(self):
        if self.process.is_alive():self.process.terminate()
        if self.process.pid is not None:self.process.join(timeout=3)
