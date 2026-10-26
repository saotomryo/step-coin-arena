import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from xrobocon.workshop.text_input import focus_prompt
class TextInputTests(unittest.TestCase):
    def test_mac_focuses_text_view_instead_of_gl_view(self):
        text_view=object();native=Mock();native.makeFirstResponder_.return_value=True
        native.firstResponder.return_value=text_view
        window=SimpleNamespace(_nswindow=native,_nsview=SimpleNamespace(_textview=text_view));caret=Mock()
        self.assertEqual(focus_prompt(window,caret)['backend'],'NSTextView')
        native.makeFirstResponder_.assert_called_once_with(text_view);caret.on_activate.assert_called_once()
    def test_missing_or_rejected_native_focus_reports_error(self):
        native=Mock();native.makeFirstResponder_.return_value=False
        with self.assertRaises(RuntimeError):focus_prompt(SimpleNamespace(_nswindow=native,_nsview=SimpleNamespace(_textview=object())),Mock())
        with self.assertRaises(RuntimeError):focus_prompt(SimpleNamespace(_nswindow=native,_nsview=SimpleNamespace()),Mock())
    def test_other_platforms_keep_pyglet_input(self):
        self.assertEqual(focus_prompt(SimpleNamespace(),Mock()),dict(backend='pyglet',focused=True))
    def test_native_responder_must_actually_switch(self):
        native=Mock();native.makeFirstResponder_.return_value=True
        native.firstResponder.return_value=object()
        with self.assertRaises(RuntimeError):
            focus_prompt(SimpleNamespace(_nswindow=native,_nsview=SimpleNamespace(_textview=object())),Mock())
