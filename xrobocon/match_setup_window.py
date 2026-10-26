"""Qt match selection; Genesis remains owned by a separate child process."""
from copy import deepcopy
from pathlib import Path
from PySide6.QtCore import QTimer,QEvent
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QPushButton,QGroupBox,QFormLayout
from xrobocon.workshop.qt_runtime import prepare_plugins
from xrobocon.match_setup import MatchCatalog
prepare_plugins()

class MatchSetupWindow(QWidget):
    def __init__(self,catalog,launcher,library,links):
        super().__init__();self.catalog=catalog;self.launcher=launcher;self.library=Path(library);self.links=links
        self.choices=catalog.defaults();self.selectors={};self.preview_buttons=[];self.closing=False;self.last_result=None
        self.setWindowTitle('ステップコイン・アリーナ — 機体・戦略選択');self.resize(1040,650)
        layout=QVBoxLayout(self);header=QHBoxLayout();layout.addLayout(header)
        title=QLabel('ステップコイン・アリーナ 対戦設定');title.setStyleSheet('font-size:24px;font-weight:bold');header.addWidget(title,1)
        self.workshop_button=QPushButton('ロボット作成・制御調整へ');header.addWidget(self.workshop_button)
        self.workshop_button.clicked.connect(lambda:self.invoke(lambda:self.links.open('workshop')))
        reload_button=QPushButton('機体一覧を更新');header.addWidget(reload_button);reload_button.clicked.connect(lambda:self.invoke(self.reload_catalog))
        self.reload_button=reload_button
        layout.addWidget(QLabel('青・赤それぞれの機体、制御、経路戦略を選んで対戦します。'))
        columns=QHBoxLayout();layout.addLayout(columns)
        for name,title,color in [('self','青','#1485cf'),('opponent','赤','#d74751')]:
            box=QGroupBox(title);box.setStyleSheet(f'QGroupBox {{color:{color};font-weight:bold;}}');columns.addWidget(box)
            form=QFormLayout(box);preview=QPushButton('3Dプレビュー');form.addRow(preview);self.preview_buttons.append(preview)
            preview.clicked.connect(lambda checked=False,name=name:self.invoke(lambda:self.launcher.preview(self.choices[name]['robot'])))
            for field,label in [('robot','機体'),('control','動作制御'),('strategy','経路戦略')]:
                combo=QComboBox();combo.setMinimumContentsLength(15);combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
                self.selectors[name,field]=combo;form.addRow(label,combo)
                combo.currentIndexChanged.connect(lambda index,name=name,field=field:self.change(name,field))
        self.populate()
        layout.addWidget(QLabel('Codex戦略は推論中も競技時計が進みます。推論結果は対戦ログに保存されます。'))
        panel_row=QHBoxLayout();layout.addLayout(panel_row);panel_row.addWidget(QLabel('競技操作画面'))
        self.panel=QComboBox();self.panel.addItem('PySide6 別ウィンドウ','python');self.panel.addItem('Genesis内オーバーレイ（従来表示）','overlay');panel_row.addWidget(self.panel,1)
        self.status=QLabel('選択して「対戦開始」を押してください。');self.status.setWordWrap(True);layout.addWidget(self.status)
        self.result_label=QLabel();self.result_label.setStyleSheet('font-size:20px');layout.addWidget(self.result_label)
        row=QHBoxLayout();layout.addLayout(row);self.start_button=QPushButton('対戦開始');self.stop_button=QPushButton('対戦を中止');row.addWidget(self.start_button);row.addWidget(self.stop_button)
        self.start_button.clicked.connect(lambda:self.invoke(self.start));self.stop_button.clicked.connect(self.launcher.stop)
        layout.addWidget(QLabel('3D描画は独立したGenesisウィンドウです。対戦終了後はここで再戦できます。'))
        self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100)
        self.refresh()
    def invoke(self,action):
        try:action()
        except Exception as error:self.status.setText(str(error))
    def populate(self):
        for name in ('self','opponent'):
            choice=self.choices[name]
            robot=self.catalog.options('robots')[choice['robot']]
            for field,category in [('robot','robots'),('control','controls'),('strategy','strategies')]:
                combo=self.selectors[name,field];combo.blockSignals(True);combo.clear()
                for key,entry in self.catalog.options(category).items():
                    if field!='robot' and key not in robot[category]:continue
                    label=robot.get('control_labels',{}).get(key,entry['label']) if field=='control' else entry['label']
                    combo.addItem(label,key)
                combo.setCurrentIndex(combo.findData(choice[field]));combo.blockSignals(False)
    def change(self,name,field):
        value=self.selectors[name,field].currentData()
        if value is None or self.launcher.running:return
        self.choices[name][field]=value
        if field=='robot':
            robot=self.catalog.options('robots')[value]
            self.choices[name]['control']=robot.get('recommended_control',robot['controls'][0])
            if self.choices[name]['strategy'] not in robot['strategies']:self.choices[name]['strategy']=robot['strategies'][0]
            self.populate()
    def reload_catalog(self):
        if self.launcher.running:return
        custom=self.library/'catalog.json';source=custom if custom.is_file() else self.catalog.path
        catalog=MatchCatalog.load(source);choices=deepcopy(self.choices)
        for name in choices:
            if choices[name]['robot'] not in catalog.options('robots'):choices[name]=catalog.defaults()[name]
            robot=catalog.options('robots')[choices[name]['robot']]
            for field,category in [('control','controls'),('strategy','strategies')]:
                if choices[name][field] not in robot[category]:choices[name][field]=robot[category][0]
        self.catalog=catalog;self.launcher.catalog=catalog;self.choices=choices;self.populate()
        self.status.setText(f'機体一覧を更新しました（{len(catalog.options("robots"))}件）。')
    def start(self):
        self.reload_catalog();self.launcher.library=self.library
        self.launcher.launch(deepcopy(self.choices),panel=self.panel.currentData());self.last_result=None;self.result_label.clear()
    def refresh(self):
        result=self.launcher.poll();running=self.launcher.running
        if self.closing and not running:self.close();return
        for widget in [*self.selectors.values(),*self.preview_buttons,self.start_button,self.reload_button,self.panel]:widget.setEnabled(not running)
        self.stop_button.setEnabled(running)
        preview=self.launcher.preview_process
        if preview is not None and preview.poll() is not None and preview.returncode:
            self.status.setText('3Dプレビューの起動に失敗しました。preview.logを確認してください。');self.launcher.stop_preview()
        if running:self.status.setText('競技環境を実行中です。試合終了後に結果を表示します。')
        elif result is not None and result is not self.last_result:
            self.last_result=result;self.status.setText(result['error'] or '試合終了。選び直して再戦できます。')
            scores=result['scores'];counts=result['collection_counts']
            self.result_label.setText(f"青 {scores.get('self',0)}点 / {counts.get('self',0)}枚　赤 {scores.get('opponent',0)}点 / {counts.get('opponent',0)}枚")
            self.start_button.setText('再戦する')
    def changeEvent(self,event):
        super().changeEvent(event)
        if event.type()==QEvent.Type.ActivationChange and self.isActiveWindow() and hasattr(self,'status'):self.invoke(self.reload_catalog)
    def closeEvent(self,event):
        self.launcher.stop_preview()
        if self.launcher.running:self.closing=True;self.launcher.stop();self.status.setText('競技環境の終了を待っています…');event.ignore();return
        self.timer.stop();event.accept()


def show_setup(catalog,launcher,screenshot=None,smoke=False,library=None):
    from xrobocon.desktop_navigation import DesktopLinks
    app=QApplication.instance() or QApplication([]);app.setFont(QFont('Hiragino Sans',12))
    links=DesktopLinks('setup',library,catalog.path,launcher.output_root)
    window=MatchSetupWindow(catalog,launcher,links.library,links)
    if not smoke and not links.claim(window,window.reload_catalog):window.close();return
    window.show();errors=[]
    if smoke:
        def check():
            try:
                window.selectors['self','robot'].setCurrentIndex(1)
                assert window.choices['self']['control'] in catalog.options('robots')[window.choices['self']['robot']]['controls']
                if screenshot:assert window.grab().save(str(screenshot))
            except Exception as error:errors.append(error)
            window.close()
        QTimer.singleShot(400,check)
    elif screenshot:QTimer.singleShot(400,lambda:window.grab().save(str(screenshot)))
    app.exec()
    if errors:raise errors[0]
