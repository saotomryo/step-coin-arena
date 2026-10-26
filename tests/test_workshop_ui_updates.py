import unittest
from types import SimpleNamespace
from xrobocon.workshop.ui_updates import DrawGate,ProposalCache,set_if_changed
class LightweightUITests(unittest.TestCase):
    def test_idle_frames_are_not_rendered(self):
        gate=DrawGate();calls=[]
        for _ in range(600):gate.draw(calls.append,.01)
        self.assertEqual(len(calls),1)
        gate.invalidate();gate.draw(calls.append,.01);self.assertEqual(len(calls),2)
    def test_reentrant_invalidation_is_preserved(self):
        gate=DrawGate();gate.draw(lambda dt:gate.invalidate(),0)
        self.assertTrue(gate.dirty)
    def test_unchanged_labels_are_not_assigned(self):
        class Label:
            def __init__(self):self.assignments=0;self._text='same'
            @property
            def text(self):return self._text
            @text.setter
            def text(self,value):self.assignments+=1;self._text=value
        label=Label();gate=DrawGate()
        for _ in range(100):set_if_changed(label,'text','same',gate.invalidate)
        self.assertEqual(label.assignments,0)
        set_if_changed(label,'text','new',gate.invalidate);self.assertEqual(label.assignments,1)
    def test_format_only_when_model_or_results_change(self):
        c=SimpleNamespace(current={},proposal=None,last_result=None);cache=ProposalCache();calls=[]
        def format(c):calls.append(1);return str(c.current)
        for _ in range(100):cache.render(c,format)
        self.assertEqual(len(calls),1)
        c.last_result={};cache.render(c,format);self.assertEqual(len(calls),2)
