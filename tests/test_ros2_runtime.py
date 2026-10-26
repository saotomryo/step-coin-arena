"""Container ROS2 remains DDS; only the sidecar executable changes."""
import os,sys,unittest
from unittest.mock import patch
from xrobocon.ros2_runtime import worker_command
class RuntimeTests(unittest.TestCase):
    def test_system_uses_same_python_and_worker(self):
        with patch.dict(os.environ,{'XROBOCON_ROS2_RUNTIME':'system'}):
            cmd=worker_command('/app','/data/config.json')
            self.assertEqual(cmd,[sys.executable,'/app/scripts/ros2_worker.py','--role','bridge','--config','/data/config.json'])
    def test_pixi_default_keeps_existing_mac_environment(self):
        with patch.dict(os.environ,{},clear=True):
            cmd=worker_command('/project','/data/config.json')
            self.assertEqual(cmd[0],'/project/.tools/pixi');self.assertIn('--no-install',cmd)
    def test_invalid_mode_is_not_silent_fallback(self):
        with patch.dict(os.environ,{'XROBOCON_ROS2_RUNTIME':'other'}),self.assertRaises(ValueError):worker_command('/app','config.json')
