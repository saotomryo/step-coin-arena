import unittest
from pathlib import Path
from xrobocon.match_setup import MatchCatalog,build_match_config
ROOT=Path(__file__).resolve().parents[1]
class MatchSetupTests(unittest.TestCase):
    def setUp(self):self.catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json')
    def test_independent_color_choices_and_ros2_preserved(self):
        choices=self.catalog.defaults();choices['self']['control']='safe';choices['opponent']['strategy']='fixed'
        config=build_match_config(self.catalog,choices)
        self.assertEqual(config['agents']['self']['control_profile'],'safe')
        self.assertEqual(config['agents']['opponent']['kind'],'tier')
        self.assertEqual(config['transport']['kind'],'ros2')
        self.assertEqual(set(config['simulation']['robot_models']),{'self','opponent'})
        self.assertTrue(Path(config['simulation']['scenario']).is_absolute())
    def test_invalid_selection_does_not_mutate_defaults(self):
        choices=self.catalog.defaults();choices['self']['robot']='missing'
        with self.assertRaises(ValueError):build_match_config(self.catalog,choices)
        self.assertNotEqual(self.catalog.defaults()['self']['robot'],'missing')
    def test_model_assets_exist_and_control_strategy_are_separate(self):
        config=build_match_config(self.catalog,self.catalog.defaults())
        for spec in config['simulation']['robot_models'].values():self.assertTrue(Path(spec['robot_asset']).is_file())
        self.assertEqual(config['match_selection']['self']['strategy'],'adaptive')
    def test_incompatible_selection_rejected(self):
        from copy import deepcopy
        data=deepcopy(self.catalog.data);data['robots']['front_bogie_r18']['controls']=['safe']
        catalog=MatchCatalog(self.catalog.path,data)
        with self.assertRaises(ValueError):build_match_config(catalog,catalog.defaults())
    def test_missing_participant_rejected(self):
        with self.assertRaises(ValueError):build_match_config(self.catalog,{'self':self.catalog.defaults()['self']})
    def test_launcher_prepares_config_and_collects_result(self):
        import tempfile,json
        from unittest.mock import patch,Mock
        from xrobocon.match_launcher import MatchLauncher
        with tempfile.TemporaryDirectory() as tmp:
            process=Mock();process.poll.return_value=None
            with patch('xrobocon.match_launcher.subprocess.Popen',return_value=process):
                launcher=MatchLauncher(self.catalog,tmp);trial=launcher.launch(self.catalog.defaults())
            self.assertTrue(launcher.running)
            with self.assertRaises(ValueError):launcher.launch(self.catalog.defaults())
            saved=json.loads((trial/'selected-config.json').read_text());self.assertEqual(saved['agents']['self']['kind'],'adaptive_tier')
            (trial/'run').mkdir();(trial/'run/session-state.json').write_text(json.dumps({'scores':{'self':15,'opponent':15},'control_status':'finished'}))
            process.poll.return_value=0;process.returncode=0
            self.assertEqual(launcher.poll()['scores']['self'],15)
            with patch('xrobocon.match_launcher.subprocess.Popen',return_value=Mock(poll=lambda:None)):
                next_trial=launcher.launch(self.catalog.defaults())
            self.assertNotEqual(trial,next_trial);launcher.log.close()
    def test_preview_closed_before_match_and_on_replacement(self):
        import tempfile
        from unittest.mock import Mock,patch
        from xrobocon.match_launcher import MatchLauncher
        with tempfile.TemporaryDirectory() as tmp:
            launcher=MatchLauncher(self.catalog,tmp)
            preview=Mock();preview.poll.return_value=None
            with patch('xrobocon.match_launcher.subprocess.Popen',return_value=preview):launcher.preview('front_bogie_r18')
            with patch('xrobocon.match_launcher.subprocess.Popen',return_value=Mock(poll=lambda:None)):
                launcher.launch(self.catalog.defaults())
            preview.terminate.assert_called_once();preview.wait.assert_called_once()
            self.assertIsNone(launcher.preview_process);launcher.log.close()
    def test_preview_invalid_model_never_starts_process(self):
        import tempfile
        from unittest.mock import patch
        from xrobocon.match_launcher import MatchLauncher
        with tempfile.TemporaryDirectory() as tmp,patch('xrobocon.match_launcher.subprocess.Popen') as popen:
            with self.assertRaises(ValueError):MatchLauncher(self.catalog,tmp).preview('unknown')
            popen.assert_not_called()
