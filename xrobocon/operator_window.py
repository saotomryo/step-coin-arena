"""Qt operator panel consumes snapshots and emits commands through queues only."""
import queue
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QGridLayout,QPlainTextEdit
from xrobocon.native_panel import LABELS,ACTIONS,allowed,decision_status

class OperatorWindow(QWidget):
    def __init__(self,states,commands,links,screenshot=None):
        super().__init__();self.states=states;self.commands=commands;self.links=links;self.state={};self.screenshot=screenshot;self.captured=False
        self.setWindowTitle('ステップコイン・アリーナ — 競技操作・得点');self.resize(800,700)
        layout=QVBoxLayout(self);title=QLabel('ステップコイン・アリーナ 競技操作');title.setStyleSheet('font-size:24px;font-weight:bold');layout.addWidget(title)
        row=QHBoxLayout();layout.addLayout(row)
        self.setup_button=QPushButton('対戦設定へ');self.workshop_button=QPushButton('ロボット作成・制御調整へ')
        row.addWidget(self.setup_button);row.addWidget(self.workshop_button)
        self.setup_button.clicked.connect(lambda:self.navigate('setup'));self.workshop_button.clicked.connect(lambda:self.navigate('workshop'))
        self.status=QLabel('初期化中');self.clock=QLabel('残り競技時間: --');self.clock.setStyleSheet('font-size:25px');layout.addWidget(self.status);layout.addWidget(self.clock)
        row=QHBoxLayout();layout.addLayout(row);self.scores={}
        for name,label,color in [('self','青','#1485cf'),('opponent','赤','#d74751')]:
            item=QLabel(label+': 0点');item.setStyleSheet(f'font-size:23px;color:{color}');self.scores[name]=item;row.addWidget(item)
        self.counts=QLabel();layout.addWidget(self.counts)
        grid=QGridLayout();layout.addLayout(grid);self.buttons={}
        for i,(action,text) in enumerate(ACTIONS):
            button=QPushButton(text);button.setMinimumHeight(40);grid.addWidget(button,i//3,i%3);self.buttons[action]=button
            button.clicked.connect(lambda checked=False,action=action:self.command(action))
        layout.addWidget(QLabel('競技イベント（最新8件）'))
        self.events=QPlainTextEdit();self.events.setReadOnly(True);layout.addWidget(self.events,1)
        self.error=QLabel();self.error.setWordWrap(True);layout.addWidget(self.error)
        self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100);self.refresh()
    def navigate(self,target):
        try:self.links.open(target)
        except Exception as error:self.error.setText(str(error))
    def command(self,action):
        if allowed(action,self.state.get('control_status','ready')):self.commands.put(action)
    def refresh(self):
        updated=False
        while True:
            try:self.state=self.states.get_nowait();updated=True
            except queue.Empty:break
        state=self.state;status=state.get('control_status','ready')
        self.status.setText(f"{LABELS.get(status,status)} 第{state.get('round_id',0)}試合")
        self.clock.setText(f"残り競技時間: {state.get('remaining',0):.1f}秒")
        for name,title in [('self','青'),('opponent','赤')]:self.scores[name].setText(f"{title}: {state.get('scores',{}).get(name,0)}点 / {state.get('collection_counts',{}).get(name,0)}枚")
        coins=state.get('coins',[])
        self.counts.setText(f"出現中 {sum(c['status']=='available' for c in coins)} / 取得済み {sum(c['status']=='collected' for c in coins)}　時計はシミュレーション時間")
        for action,button in self.buttons.items():button.setEnabled(allowed(action,status))
        names={'competition_started':'競技開始','competition_finished':'競技終了','coin_spawned':'出現','coin_collected':'取得','coin_contested':'同時取得候補','robot_recovered':'スタート復帰'}
        lines=[]
        for event in list(reversed(state.get('events',[])))[:8]:
            kind=event.get('type',event.get('event',''))
            line=f"{event.get('time',0):.2f}秒 {names.get(kind,kind)} {dict(self='青',opponent='赤').get(event.get('participant'),event.get('participant',''))}"
            if 'coin_id' in event:line+=f" コイン{event['coin_id']}"
            lines.append(line)
        text='\n'.join(lines)
        if text!=self.events.toPlainText():self.events.setPlainText(text)
        self.error.setText(str(state.get('error') or decision_status(state) or f"スタート復帰 青:{state.get('recovery_counts',{}).get('self',0)} 赤:{state.get('recovery_counts',{}).get('opponent',0)}。規則は暫定設定。"))
        if self.screenshot and updated and not self.captured:
            self.captured=True;QTimer.singleShot(0,lambda:self.grab().save(self.screenshot))
    def closeEvent(self,event):
        self.timer.stop();self.commands.put('shutdown');event.accept()
