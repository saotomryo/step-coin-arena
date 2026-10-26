"""Native operator action availability matches the competition lifecycle."""
import unittest
from xrobocon.native_panel import allowed,ACTIONS
class NativePanelTests(unittest.TestCase):
    def test_state_actions(self):
        expected={'ready':{'start','reset','shutdown'},'running':{'pause','stop','reset','shutdown'},'paused':{'resume','stop','reset','shutdown'},'finished':{'start','reset','shutdown'},'error':{'reset','shutdown'}}
        for state,actions in expected.items():
            with self.subTest(state=state):self.assertEqual({a for a,_ in ACTIONS if allowed(a,state)},actions)
