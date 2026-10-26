import os
import unittest
from unittest.mock import patch
import genesis as gs
from xrobocon.common import setup_genesis

class BackendTests(unittest.TestCase):
    def test_cpu_from_argument_and_environment(self):
        with patch.object(gs, '_initialized', False), patch.object(gs, 'init') as initialize:
            setup_genesis('cpu', seed=42)
            initialize.assert_called_once_with(backend=gs.cpu, seed=42)
        with patch.object(gs, '_initialized', False), patch.object(gs, 'init') as initialize:
            with patch.dict(os.environ, {'XROBOCON_BACKEND': 'cpu'}):
                setup_genesis()
            initialize.assert_called_once_with(backend=gs.cpu)

    def test_invalid_backend_and_init_failure(self):
        with self.assertRaises(ValueError): setup_genesis('typo')
        with patch.object(gs, '_initialized', False), patch.object(gs, 'init', side_effect=RuntimeError('failure')):
            with self.assertRaisesRegex(RuntimeError, 'failure'): setup_genesis('cpu')

    def test_repeated_init_and_backend_switch(self):
        with patch.object(gs, '_initialized', True), patch.object(gs, 'backend', gs.cpu), patch.object(gs, 'init') as initialize:
            setup_genesis('cpu')
            initialize.assert_not_called()
            with self.assertRaises(RuntimeError): setup_genesis('gpu')

if __name__ == '__main__': unittest.main()
