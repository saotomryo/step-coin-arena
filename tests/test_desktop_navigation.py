"""Activation reuse and fixed destination arguments, without spawning an app."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,patch
from xrobocon.desktop_navigation import DesktopLinks
class NavigationTests(unittest.TestCase):
    def test_existing_window_is_reused_without_launch(self):
        with tempfile.TemporaryDirectory() as d:
            links=DesktopLinks('workshop',d)
            with patch.object(links,'activate',return_value=True),patch('xrobocon.desktop_navigation.subprocess.Popen') as run:
                self.assertEqual(links.open('setup'),'activated');run.assert_not_called()
    def test_spawn_carries_library_and_does_not_duplicate_pending_start(self):
        with tempfile.TemporaryDirectory() as d:
            links=DesktopLinks('setup',d)
            process=Mock();process.poll.return_value=None
            with patch.object(links,'activate',return_value=False),patch('xrobocon.desktop_navigation.subprocess.Popen',return_value=process) as run:
                self.assertEqual(links.open('workshop'),'opened')
                self.assertEqual(links.open('workshop'),'starting')
                self.assertEqual(run.call_count,1)
                args=run.call_args.args[0];self.assertEqual(args[args.index('--library')+1],str(Path(d).resolve()))
                self.assertTrue(args[1].endswith('robot_workshop.py'))
            with self.assertRaises(ValueError):links.open('arbitrary.py')
