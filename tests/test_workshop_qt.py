"""Regression checks for native Qt editing, without claiming OS IME coverage."""
import tempfile
import unittest
from pathlib import Path
from PySide6.QtCore import Qt
from PySide6.QtGui import QInputMethodEvent, QKeySequence
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication
from xrobocon.match_setup import MatchCatalog
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.qt_window import WorkshopWindow

ROOT = Path(__file__).resolve().parents[1]

class QtWorkshopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.controller = WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'), self.temp.name)
        self.window = WorkshopWindow(self.controller)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()
        self.temp.cleanup()

    def test_ime_commit_clipboard_undo_and_refresh_preserve_selection(self):
        editor = self.window.prompt
        event = QInputMethodEvent()
        event.setCommitString('日本語の機体設計')
        self.app.sendEvent(editor, event)
        self.assertEqual(editor.toPlainText(), '日本語の機体設計')
        editor.selectAll()
        editor.copy()
        self.assertEqual(self.app.clipboard().text(), '日本語の機体設計')
        cursor_before = editor.textCursor()
        for _ in range(10):
            self.window.refresh()
        self.assertEqual(editor.textCursor().selectedText(), cursor_before.selectedText())
        editor.moveCursor(editor.textCursor().MoveOperation.End)
        self.app.clipboard().setText('\n重心を下げる')
        editor.paste()
        self.assertEqual(editor.toPlainText(), '日本語の機体設計\n重心を下げる')
        editor.undo()
        self.assertEqual(editor.toPlainText(), '日本語の機体設計')
        editor.redo()
        self.assertTrue(editor.toPlainText().endswith('重心を下げる'))

    def test_keyboard_and_busy_refresh_keep_editor_enabled(self):
        from concurrent.futures import Future
        editor = self.window.prompt
        QTest.keyClicks(editor, 'robot')
        self.controller.future = Future()
        self.window.refresh()
        self.assertFalse(self.window.preview_button.isEnabled())
        self.assertTrue(editor.isEnabled())
        QTest.keyClicks(editor, ' next')
        self.window.refresh()
        self.assertEqual(editor.toPlainText(), 'robot next')
        self.controller.future = None

    def test_error_stays_visible_and_output_refresh_is_cached(self):
        self.window.invoke(self.controller.apply)
        self.assertIn('先に変更案', self.window.status.text())
        count = self.window.cache.format_count
        for _ in range(10):
            self.window.refresh()
        self.assertEqual(self.window.cache.format_count, count)
        self.window.load_version()  # placeholder never attempts to load None

    def test_new_robot_buttons_generate_review_apply_save_register(self):
        from xrobocon.workshop.examples import lifting_robot
        from PySide6.QtWidgets import QPushButton
        class Generator:
            model='test';reasoning_effort='low'
            def propose(inner,snapshot,prompt,mode,cancelled):
                self.assertIsNone(snapshot)
                self.assertEqual(mode,'new_robot')
                return lifting_robot()
        self.controller.new_generator=Generator()
        self.window.prompt.setPlainText('持ち上げ機構付きの新規機体')
        buttons={button.text():button for button in self.window.findChildren(QPushButton)}
        buttons['新規機体を生成'].click()
        for _ in range(100):
            self.window.refresh()
            if not self.controller.busy:break
            QTest.qWait(5)
        self.assertIsNotNone(self.controller.proposal,self.controller.message)
        self.assertIn('def control(s)',self.window.output.toPlainText())
        for name in ['1. 適用・検証','2. 別の版として保存','3. 対戦へ登録']:
            buttons[name].click()
        self.assertTrue(self.controller.saved,self.controller.message)
        self.assertTrue((self.controller.library.root/'catalog.json').exists())

    def test_robot_identity_and_saved_names_are_visible(self):
        from xrobocon.workshop.examples import lifting_robot
        from unittest.mock import patch
        self.controller.mode='new_robot';self.controller.proposal=lifting_robot();self.controller.apply()
        self.window.save();self.window.refresh()
        self.assertIn(self.controller.current['robot_name'],self.window.current_robot.text())
        self.assertIn(self.controller.current['robot_name'],self.window.versions.itemText(1))
        self.assertEqual(self.window.sources.currentIndex(),-1)
        self.window.register();self.window.refresh()
        self.assertGreaterEqual(self.window.sources.currentIndex(),0)
        first=self.controller.current['robot_id']
        with patch('xrobocon.workshop.qt_window.QInputDialog.getText',return_value=('別ロボット',True)):
            self.window.save_as_new_robot()
        self.window.refresh()
        self.assertNotEqual(first,self.controller.current['robot_id'])
        self.assertIn('別ロボット',self.window.current_robot.text())

    def test_model_and_iteration_controls_reach_controller(self):
        from unittest.mock import patch
        self.window.model.setCurrentText('gpt-6.1-sol');self.window.effort.setCurrentText('high')
        self.window.inference_timeout.setValue(300);self.window.iterations.setValue(4)
        self.window.suite.setCurrentIndex(1);self.window.stop_on_pass.setChecked(False)
        with patch.object(self.controller,'start_automation') as start:
            self.window.start_automation()
        self.assertEqual(self.controller.new_generator.model,'gpt-6.1-sol')
        self.assertEqual(self.controller.new_generator.reasoning_effort,'high')
        self.assertEqual(start.call_args.args[:2],(4,'steps'))
        self.assertFalse(start.call_args.args[3])

    def test_robot_change_scope_and_creation_prompt_display(self):
        from unittest.mock import patch
        self.assertTrue(self.window.allow_robot_changes.isChecked())
        self.window.allow_robot_changes.setChecked(False)
        with patch.object(self.controller,'start_automation') as start:
            self.window.start_automation()
        self.assertFalse(start.call_args.kwargs['allow_robot_changes'])
        self.controller.current['creation_prompt']='四輪を維持する'
        self.window.show_creation_prompt()
        from PySide6.QtWidgets import QPlainTextEdit
        self.assertIn('四輪を維持する',self.window.creation_dialog.findChild(QPlainTextEdit).toPlainText())
        self.window.creation_dialog.close()

    def test_pending_shape_and_control_information_follow_apply(self):
        from tests.test_robot_workshop import proposal
        from xrobocon.workshop.examples import lifting_robot
        self.controller.proposal=proposal();self.window.refresh()
        self.assertIn('未適用',self.window.display_state.text())
        self.assertIn('0.13',self.window.output.toPlainText())
        first=self.controller.presentation()
        for _ in range(5):self.window.refresh()
        self.assertIs(first,self.controller.presentation())
        self.controller.apply();self.window.refresh()
        self.assertIn('適用済み',self.window.display_state.text())
        self.controller.mode='mechanism';self.controller.proposal=lifting_robot();self.window.refresh()
        self.assertIn('関節数: 5',self.window.display_state.text())
        self.assertIn('生成コード',self.window.display_state.text())
        self.assertIn('lift_joint',self.window.output.toPlainText())
