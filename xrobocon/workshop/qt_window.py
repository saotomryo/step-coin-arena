"""Qt Widgets frontend; editing stays native and independent of child renderers."""
import json
import time
from pathlib import Path
from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QPlainTextEdit, QComboBox, QInputDialog, QDialog, QDialogButtonBox, QSpinBox, QCheckBox)
from xrobocon.workshop.ui_updates import ProposalCache
from xrobocon.workshop.qt_runtime import prepare_plugins
prepare_plugins()


class WorkshopWindow(QWidget):
    def __init__(self, controller, preview_smoke=False, links=None):
        super().__init__()
        self.links=links
        self.controller = controller
        self.preview_smoke = preview_smoke
        self.cache = ProposalCache()
        self.last_output = None
        self.actions = []
        self.setWindowTitle('ステップコイン・アリーナ — ロボット・制御作成')
        self.resize(1100, 830)
        layout = QVBoxLayout(self)
        title = QLabel('ロボット・制御ワークショップ')
        title.setStyleSheet('font-size:24px; font-weight:bold')
        layout.addWidget(title)
        layout.addWidget(QLabel('変更案は適用前に3D確認できます。診断・保存は適用後。関節追加には「機構を追加・再設計」を使います。'))
        row = QHBoxLayout()
        layout.addLayout(row)
        self.match_button=self.button(row, '対戦設定へ',self.open_match_setup,idle=False)
        self.preview_button = self.button(row, '3Dプレビュー', lambda: controller.preview(smoke=preview_smoke))
        self.sources = QComboBox()
        self.sources.setPlaceholderText('元機体・登録機体（新規作成後は独立した機体）')
        for robot_id in controller.catalog.options('robots'):
            self.sources.addItem(controller.catalog.data['robots'][robot_id]['label'], robot_id)
        row.addWidget(self.sources, 1)
        self.versions = QComboBox()
        row.addWidget(self.versions, 1)
        self.button(row, '保存版を再読込', self.reload_versions)
        self.sources.setCurrentIndex(0)
        controller.choose_robot(self.sources.currentData())
        self.sources.activated.connect(lambda _: self.invoke(lambda: controller.choose_robot(self.sources.currentData())))
        self.versions.activated.connect(lambda _: self.invoke(self.load_version))
        self.reload_versions()
        row = QHBoxLayout()
        layout.addLayout(row)
        self.button(row, '新規機体を生成', lambda: self.propose('new_robot'))
        self.button(row, '機構を追加・再設計', lambda: self.propose('mechanism'))
        self.button(row, '制御コードを生成・修正', lambda: self.propose('program'))
        self.current_robot = QLabel()
        self.current_robot.setWordWrap(True)
        layout.addWidget(self.current_robot)
        self.display_state=QLabel();self.display_state.setWordWrap(True);layout.addWidget(self.display_state)
        self.performance_label=QLabel();self.performance_label.setWordWrap(True);layout.addWidget(self.performance_label)
        score_row=QHBoxLayout();layout.addLayout(score_row)
        self.button(score_row,'制御性能スコアの詳細',self.show_performance,idle=False)
        row=QHBoxLayout();layout.addLayout(row)
        row.addWidget(QLabel('モデルID'))
        self.model=QComboBox();self.model.setEditable(True)
        self.model.addItems(['gpt-6-luna','gpt-6.1-sol','gpt-6-sol','gpt-6-astra'])
        self.model.setCurrentText(controller.inference_settings['model']);row.addWidget(self.model,1)
        row.addWidget(QLabel('推論'))
        self.effort=QComboBox();self.effort.addItems(['low','medium','high','xhigh','max'])
        self.effort.setCurrentText(controller.inference_settings['reasoning_effort']);row.addWidget(self.effort)
        row.addWidget(QLabel('推論制限秒'))
        self.inference_timeout=QSpinBox();self.inference_timeout.setRange(1,300)
        self.inference_timeout.setValue(int(controller.inference_settings['timeout']));row.addWidget(self.inference_timeout)
        self.button(row,'推論設定を保存',self.configure_inference)
        layout.addWidget(QLabel('プロンプト（モデルIDは直接入力も可能。利用できないモデルはエラーで停止）'))
        self.prompt = QPlainTextEdit()
        self.prompt.setObjectName('prompt')
        self.prompt.setPlaceholderText('例：車体を1cm下げてください。制御設定は維持してください。')
        self.prompt.setMinimumHeight(150)
        layout.addWidget(self.prompt, 2)
        row = QHBoxLayout()
        layout.addLayout(row)
        for label, mode in [('機体のみ生成・編集', 'robot_only'), ('機体＋制御の変更案を生成', 'robot'), ('制御設定だけの変更案を生成', 'control')]:
            self.button(row, label, lambda mode=mode: self.propose(mode))
        self.cancel_button = self.button(row, '生成・診断を取り消す', controller.cancel, idle=False)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        layout.addWidget(self.output, 3)
        row = QHBoxLayout()
        layout.addLayout(row)
        self.button(row, '1. 適用・検証', controller.apply)
        self.button(row, '2. 別の版として保存', self.save)
        self.button(row, '新しいロボットとして保存', self.save_as_new_robot)
        self.button(row, '3. 対戦へ登録', self.register)
        row = QHBoxLayout()
        layout.addLayout(row)
        self.button(row, '平地診断（直進・旋回・経路）', lambda: controller.diagnostic('flat'))
        self.button(row, '三段登坂の診断', lambda: controller.diagnostic('steps'))
        self.button(row, '関節・機構の動作確認', lambda: controller.diagnostic('joints'))
        row = QHBoxLayout()
        layout.addLayout(row)
        self.button(row, '検証ログを表示', self.show_logs, idle=False)
        self.button(row, 'ログから制御を改善', self.improve_from_logs)
        row=QHBoxLayout();layout.addLayout(row)
        self.allow_robot_changes=QCheckBox('自動改善で機体・機構も変更する（元の作成条件を維持）')
        self.allow_robot_changes.setChecked(True);row.addWidget(self.allow_robot_changes)
        self.creation_button=QPushButton('元の作成プロンプトを表示');self.creation_button.clicked.connect(self.show_creation_prompt);row.addWidget(self.creation_button)
        row=QHBoxLayout();layout.addLayout(row)
        row.addWidget(QLabel('自動改善の上限'))
        self.iterations=QSpinBox();self.iterations.setRange(1,30);self.iterations.setValue(3);row.addWidget(self.iterations)
        self.suite=QComboBox()
        for label,key in [('平地','flat'),('三段登坂','steps'),('平地＋登坂','all'),('関節スモーク','joints')]:self.suite.addItem(label,key)
        row.addWidget(self.suite)
        self.stop_on_pass=QCheckBox('性能試験に合格したら終了');self.stop_on_pass.setChecked(True);row.addWidget(self.stop_on_pass)
        row.addWidget(QLabel('工程制限秒'))
        self.stage_timeout=QSpinBox();self.stage_timeout.setRange(10,3600);self.stage_timeout.setValue(600);row.addWidget(self.stage_timeout)
        self.button(row,'自動改善を開始',self.start_automation)
        self.button(row,'改善プロンプト履歴',self.show_prompt_history,idle=False)
        self.loop_status=QLabel('最大N回の改善＋最後の再検証。機体変更の有無を選択できます。各改訂を保存し、取消で停止。')
        self.loop_status.setWordWrap(True);layout.addWidget(self.loop_status)
        self.last_saved_revision=None
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addWidget(QLabel('新規機体は関節・制御コードをセットで生成。適用前に変更案を確認し、動作診断してから対戦へ登録してください。'))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()
        self.prompt.setFocus()

    def button(self, row, label, action, idle=True):
        button = QPushButton(label)
        button.clicked.connect(lambda _: self.invoke(action))
        row.addWidget(button)
        if idle:
            self.actions.append(button)
        return button

    def invoke(self, action):
        try:
            action()
        except Exception as error:
            self.controller.message = str(error)
        self.refresh()

    def open_match_setup(self):
        if self.links is None:
            from xrobocon.desktop_navigation import DesktopLinks
            self.links=DesktopLinks('workshop',self.controller.library.root,self.controller.catalog.path)
        self.links.open('setup')

    def configure_inference(self):
        self.controller.configure_inference(self.model.currentText().strip(),self.effort.currentText(),self.inference_timeout.value())

    def propose(self,mode):
        self.configure_inference()
        self.controller.propose(self.prompt.toPlainText(),mode)

    def improve_from_logs(self):
        self.configure_inference()
        self.controller.improve_from_logs(self.prompt.toPlainText())

    def start_automation(self):
        self.configure_inference()
        self.controller.start_automation(self.iterations.value(),self.suite.currentData(),self.prompt.toPlainText(),self.stop_on_pass.isChecked(),self.stage_timeout.value(),allow_robot_changes=self.allow_robot_changes.isChecked())

    def load_version(self):
        directory = self.versions.currentData()
        if directory is not None:
            self.controller.load_revision(directory)

    def save(self):
        self.controller.save()
        self.reload_versions()

    def save_as_new_robot(self):
        name, accepted = QInputDialog.getText(self,'新しいロボットとして保存','新しいロボット名',text=(self.controller.current or {}).get('robot_name','新しいロボット'))
        if accepted:
            self.controller.save_as_new_robot(name)
            self.reload_versions()

    def register(self):
        self.controller.register()
        self.reload_sources()

    def reload_sources(self):
        self.sources.clear()
        selected=-1
        identity=(self.controller.current or {}).get('robot_id')
        for robot_id,entry in self.controller.catalog.options('robots').items():
            self.sources.addItem(entry['label'],robot_id)
            if identity and entry.get('workshop_robot_id')==identity:selected=self.sources.count()-1
        if selected>=0:self.sources.setCurrentIndex(selected)

    def show_creation_prompt(self):
        current=self.controller.current or {}
        prompt=current.get('creation_prompt')
        dialog=QDialog(self);dialog.setWindowTitle('元の作成プロンプト');dialog.resize(850,500)
        layout=QVBoxLayout(dialog);output=QPlainTextEdit();output.setReadOnly(True)
        output.setPlainText(prompt or '元の作成プロンプトは保存されていません。推測で補完せず、現在の機体と改善指示をもとに検討します。')
        layout.addWidget(output);close=QDialogButtonBox(QDialogButtonBox.StandardButton.Close);close.rejected.connect(dialog.close);layout.addWidget(close)
        dialog.show();self.creation_dialog=dialog

    def show_prompt_history(self):
        usage=(self.controller.current or {}).get('improvement_prompt_usage')
        loop=self.controller.automation
        state=loop.state if loop and (loop.active or usage and Path(usage['loop'])==loop.directory) else None
        if state is None and usage:
            path=Path(usage['loop'])/'loop.json'
            if path.is_file():state=json.loads(path.read_text())
        record=dict(saved_version_usage=usage,prompt_history=(state or {}).get('prompt_history',[]),
            uses=[e for e in (state or {}).get('history',[]) if e['kind'] in ('prompt_used','revision','selection')])
        dialog=QDialog(self);dialog.setWindowTitle('改善プロンプト履歴（時刻はUTC）');dialog.resize(950,650)
        layout=QVBoxLayout(dialog);output=QPlainTextEdit();output.setReadOnly(True)
        output.setPlainText(json.dumps(record,ensure_ascii=False,indent=2) if state or usage else '改善プロンプトの履歴はありません。')
        layout.addWidget(output);close=QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        close.rejected.connect(dialog.close);layout.addWidget(close);dialog.show();self.prompt_history_dialog=dialog

    def show_performance(self):
        dialog=QDialog(self);dialog.setWindowTitle('制御性能スコア（探索方針の評価ではありません）');dialog.resize(950,650)
        layout=QVBoxLayout(dialog);text=QPlainTextEdit();text.setReadOnly(True)
        text.setPlainText(json.dumps(self.controller.performance_view(),ensure_ascii=False,indent=2));layout.addWidget(text)
        close=QDialogButtonBox(QDialogButtonBox.StandardButton.Close);close.rejected.connect(dialog.close);layout.addWidget(close)
        dialog.show();self.performance_dialog=dialog

    def show_logs(self):
        dialog=QDialog(self)
        dialog.setWindowTitle('検証ログ・入力の対応と改善用の抜粋')
        dialog.resize(1000,700)
        layout=QVBoxLayout(dialog)
        output=QPlainTextEdit();output.setReadOnly(True)
        output.setPlainText(self.controller.log_context_text())
        layout.addWidget(output)
        close=QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        close.rejected.connect(dialog.close)
        layout.addWidget(close)
        dialog.show()
        self.log_dialog=dialog

    def reload_versions(self):
        self.versions.clear()
        self.versions.addItem('保存版を選択…', None)
        for manifest in self.controller.library.list():
            try:
                info=json.loads(manifest.read_text())
                label=f"{info.get('robot_name',info['title'])} / {info['title']} / {manifest.parent.name}"
            except (ValueError,OSError,KeyError):
                label=f'読込不可: {manifest.parent.name}'
            self.versions.addItem(label, str(manifest.parent))

    def refresh(self):
        from xrobocon.workshop.window import proposal_text
        try:
            self.controller.poll()
        except Exception as error:
            self.controller.message = f'処理結果の読込に失敗: {error}'
        current=self.controller.current or {}
        identity=current.get('robot_id')
        if identity:
            selected=next((i for i in range(self.sources.count()) if self.controller.catalog.options('robots')[self.sources.itemData(i)].get('workshop_robot_id')==identity),-1)
            if self.sources.currentIndex()!=selected:self.sources.setCurrentIndex(selected)
        self.current_robot.setText('適用済みのロボット: '+current.get('robot_name',current.get('entry',{}).get('label','未選択'))+' / 機体ID: '+current.get('robot_id','既存カタログ機体')+' / 版: '+current.get('revision_id','未保存'))
        view=self.controller.presentation()
        if getattr(self,'_display_view',None) is not view:
            self._display_view=view
            snapshot=view['snapshot']
            details=view['status']
            if snapshot:
                import xml.etree.ElementTree as ET
                details+=' / '+snapshot.get('proposal',{}).get('title',snapshot['entry']['label'])
                root=ET.fromstring(snapshot['xml']);joints=[j for j in root.iter('joint') if j.get('type','hinge')!='free']
                axes={j.get('axis','0 0 1') for j in joints}
                summary=snapshot.get('summary',{})
                details+=f" / 関節数: {len(joints)} / 回転・移動軸: {', '.join(sorted(axes))}"
                if summary.get('mass') is not None:details+=f" / 質量: {summary['mass']:.2f} kg"
                control=snapshot.get('controller',{})
                details+=' / 制御: '+('生成コード' if control else '既存の制御設定')
                if view['pending']:details+='\nスコアは適用済み機体の結果です。変更案はまだ未評価です。'
            elif view['error']:details+=': '+view['error']
            self.display_state.setText('3D・変更内容の表示対象: '+details)
        performance=self.controller.performance_view();score=performance['current']
        scope={'flat':'平地3項目','steps':'段差','all':'全4項目'}.get(score['scope'],'')
        value='未評価' if score['score'] is None else f"{score['score']:.1f}/100（{scope}）"
        cases=' / '.join(f"{c['label']}: {c['score'] if c['score'] is not None else '未評価'}" for c in score['cases'].values())
        delta=performance['comparison']['delta'];comparison=f' / 親版比 {delta:+.1f}' if delta is not None else ''
        self.performance_label.setText('制御性能スコア: '+value+comparison+'\n'+cases+'\n状態把握用。探索価値や改善可能性を示す点数ではありません。')
        loop=self.controller.automation
        if loop:
            state=loop.state
            self.loop_status.setText(f"自動改善 {state['improvement_attempts']}/{state['limit']} / 指示v{state.get('prompt_version',1)} / {state['phase']} / {state.get('reason',state['status'])} / {loop.directory}")
        revision=current.get('revision_id')
        if revision!=self.last_saved_revision:
            self.reload_versions();self.last_saved_revision=revision
        self.status.setText(self.controller.message)
        text = self.cache.render(self.controller, proposal_text)
        if text != self.last_output:
            self.output.setPlainText(text)
            self.last_output = text
        busy = bool(self.controller.busy)
        for button in self.actions:
            button.setEnabled(not busy)
        self.sources.setEnabled(not busy)
        self.versions.setEnabled(not busy)
        for field in (self.model,self.effort,self.inference_timeout,self.iterations,self.suite,self.stop_on_pass,self.stage_timeout,self.allow_robot_changes):field.setEnabled(not busy)
        # Never change focus, text, selection or editability on refresh.

    def closeEvent(self, event):
        self.timer.stop()
        self.controller.close()
        event.accept()


def run_workshop(controller, screenshot=None, smoke=False, metrics_path=None):
    app = QApplication.instance() or QApplication([])
    app.setFont(QFont('Hiragino Sans', 12))
    from xrobocon.desktop_navigation import DesktopLinks
    links=DesktopLinks('workshop',controller.library.root,controller.catalog.path)
    window = WorkshopWindow(controller, smoke,links)
    if not smoke and not links.claim(window):
        window.close();return dict(activated_existing=True)
    window.show()
    errors = []
    metrics = {'toolkit': 'PySide6', 'synthetic_input_only': True}
    if smoke:
        started = time.monotonic()
        phase = [0]
        def tick():
            try:
                if time.monotonic() - started > 40:
                    raise TimeoutError('Qt workshop smoke timed out')
                if phase[0] == 0:
                    assert controller.directory is None
                    window.preview_button.click()
                    assert controller.process is not None, controller.message
                    window.prompt.insertPlainText('プレビュー中の日本語入力')
                    assert not window.prompt.isReadOnly()
                    metrics['preview_typing'] = True
                    phase[0] = 1
                elif phase[0] == 1 and not controller.busy:
                    window.prompt.setPlainText('車体を1cm下げてください。制御設定は維持してください。')
                    controller.propose(window.prompt.toPlainText(), 'robot_only')
                    submitted = controller.prompt
                    window.prompt.insertPlainText('次の入力')
                    assert controller.prompt == submitted
                    metrics['inference_typing'] = True
                    phase[0] = 2
                elif phase[0] == 2 and not controller.busy:
                    assert controller.proposal is not None, controller.message
                    controller.apply()
                    window.save()
                    controller.register()
                    window.refresh()
                    assert (controller.library.root / 'catalog.json').exists()
                    if screenshot:
                        assert window.grab().save(str(screenshot))
                    metrics['saved'] = str(controller.directory)
                    window.close()
            except Exception as error:
                errors.append(error)
                window.close()
        smoke_timer = QTimer(window)
        smoke_timer.timeout.connect(tick)
        smoke_timer.start(100)
    try:
        app.exec()
    finally:
        controller.close()
    if metrics_path:
        Path(metrics_path).write_text(json.dumps(metrics, ensure_ascii=False, indent=2))
    if errors:
        raise errors[0]
    return metrics
