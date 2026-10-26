"""Native match/operator UI actions without launching a physics process."""
import queue,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock
from PySide6.QtWidgets import QApplication
from xrobocon.workshop.qt_runtime import prepare_plugins
prepare_plugins()
from xrobocon.match_setup import MatchCatalog
from xrobocon.match_setup_window import MatchSetupWindow
from xrobocon.operator_window import OperatorWindow
ROOT=Path(__file__).resolve().parents[1]

class CompetitionQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QApplication.instance() or QApplication([])
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.links=Mock()
        self.catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json')
        self.launcher=Mock(catalog=self.catalog,output_root=Path(self.temp.name),running=False,preview_process=None)
        self.launcher.poll.return_value=None
        self.window=MatchSetupWindow(self.catalog,self.launcher,self.temp.name,self.links)
    def tearDown(self):self.window.close();self.temp.cleanup()
    def test_selection_and_navigation(self):
        self.window.selectors['self','robot'].setCurrentIndex(1)
        robot=self.catalog.options('robots')[self.window.choices['self']['robot']]
        self.assertIn(self.window.choices['self']['control'],robot['controls'])
        self.window.workshop_button.click();self.links.open.assert_called_with('workshop')
        self.window.start_button.click();self.launcher.launch.assert_called_once()
        self.assertEqual(self.launcher.launch.call_args.kwargs['panel'],'python')
    def test_running_locks_selection_and_stop_works(self):
        self.launcher.running=True;self.window.refresh()
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertFalse(self.window.selectors['self','robot'].isEnabled())
        self.window.stop_button.click();self.launcher.stop.assert_called_once()
        self.launcher.running=False
    def test_newly_registered_robot_appears_without_losing_choices(self):
        import json
        from copy import deepcopy
        from xrobocon.workshop.controller import absolute_catalog
        before=deepcopy(self.window.choices)
        data=absolute_catalog(self.catalog)
        robot=deepcopy(next(iter(data['robots'].values())));robot['label']='新規登録した機体'
        data['robots']['new_workshop_robot']=robot
        (Path(self.temp.name)/'catalog.json').write_text(json.dumps(data))
        self.window.reload_catalog()
        self.assertEqual(self.window.choices,before)
        self.assertGreaterEqual(self.window.selectors['self','robot'].findData('new_workshop_robot'),0)
        self.assertIs(self.launcher.catalog,self.window.catalog)

    def test_operator_actions_and_state(self):
        states=queue.Queue();commands=queue.Queue()
        panel=OperatorWindow(states,commands,self.links)
        try:
            states.put(dict(control_status='running',round_id=2,remaining=45.,scores={'self':3,'opponent':5},collection_counts={'self':2,'opponent':3},coins=[],events=[]))
            panel.refresh()
            self.assertIn('45.0',panel.clock.text())
            self.assertIn('3点',panel.scores['self'].text())
            self.assertFalse(panel.buttons['start'].isEnabled())
            panel.buttons['pause'].click();self.assertEqual(commands.get_nowait(),'pause')
            panel.setup_button.click();self.links.open.assert_called_with('setup')
            panel.workshop_button.click();self.links.open.assert_called_with('workshop')
        finally:panel.close()
        self.assertEqual(commands.get_nowait(),'shutdown')
