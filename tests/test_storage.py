"""Storage works after relocation and never falls back from a missing disk."""
import json,os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from xrobocon.storage import resource_root

class StorageTests(unittest.TestCase):
    def test_relative_override_is_project_relative_not_cwd(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{'XROBOCON_RESOURCE_ROOT':'data'}):
            self.assertEqual(resource_root(d),(Path(d)/'data').resolve())
    def test_legacy_volume_still_required(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{},clear=True):
            p=Path(d)/'ros2';p.mkdir();(p/'storage.local.json').write_text(json.dumps(dict(volume='/missing-disk',resource_root='data')))
            with patch('xrobocon.storage.os.path.ismount',return_value=False),self.assertRaisesRegex(RuntimeError,'no internal fallback'):resource_root(d)
    def test_relative_local_config_and_override_priority(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{},clear=True):
            p=Path(d)/'ros2';p.mkdir();(p/'storage.local.json').write_text(json.dumps(dict(resource_root='data')))
            self.assertEqual(resource_root(d),(Path(d)/'data').resolve())
            with patch.dict(os.environ,{'XROBOCON_RESOURCE_ROOT':'other'}):self.assertEqual(resource_root(d),(Path(d)/'other').resolve())
    def test_no_implicit_internal_fallback(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{},clear=True),self.assertRaisesRegex(RuntimeError,'XROBOCON_RESOURCE_ROOT'):resource_root(d)
    def test_override_missing_external_volume_rejected(self):
        with patch.dict(os.environ,{'XROBOCON_RESOURCE_ROOT':'/Volumes/Missing/data'}),patch('xrobocon.storage.os.path.ismount',return_value=False),self.assertRaises(RuntimeError):resource_root()
