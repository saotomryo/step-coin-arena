import unittest,tempfile,json
from copy import deepcopy
from pathlib import Path
from xrobocon.match_setup import MatchCatalog
from xrobocon.workshop.design import source_snapshot,apply_proposal,WorkshopLibrary,validate_proposal
ROOT=Path(__file__).resolve().parents[1]
def proposal():return dict(title='低重心の試作機',summary='車体重心を少し下げ、速度を調整',model_changes=[dict(kind='geom',name='base_link_geom_0',attribute='pos',value='-.03 0 -.06')],drive_changes=[],motion_changes=[dict(profile='baseline',key='max_speed',value=.13)])
class WorkshopTests(unittest.TestCase):
    def setUp(self):self.catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json');self.source=source_snapshot(self.catalog,'front_bogie_r18')
    def test_apply_preserves_original_and_control_binding(self):
        original=deepcopy(self.source);new=apply_proposal(self.source,proposal(),'robot')
        self.assertEqual(self.source,original);self.assertNotEqual(new['xml'],original['xml']);self.assertEqual(new['tuning']['motion']['baseline']['max_speed'],.13)
        self.assertGreater(new['summary']['mass'],0)
    def test_reject_code_paths_topology_nan_and_wrong_mode(self):
        for edit in (dict(kind='body',name='base_link',attribute='file',value='/tmp/x'),dict(kind='joint',name='missing',attribute='damping',value='1'),dict(kind='geom',name='base_link_geom_0',attribute='mass',value='nan')):
            p=proposal();p['model_changes']=[edit]
            with self.assertRaises(ValueError):apply_proposal(self.source,p,'robot')
        with self.assertRaises(ValueError):apply_proposal(self.source,proposal(),'control')
    def test_save_load_branch_and_integrity(self):
        new=apply_proposal(self.source,proposal(),'robot')
        with tempfile.TemporaryDirectory() as d:
            library=WorkshopLibrary(d);v1=library.save(new,'重心を下げて');loaded=library.load(v1)
            v2=library.save(apply_proposal(loaded,proposal(),'robot'),'別案として保存')
            self.assertEqual(len(library.list()),2);self.assertEqual(library.load(v2)['parent'],v1.name)
            (v1/'model.xml').write_text('corrupt')
            with self.assertRaises(ValueError):library.load(v1)
    def test_control_only_and_invalid_limits(self):
        p=proposal();p['model_changes']=[];new=apply_proposal(self.source,p,'control');self.assertEqual(new['xml'],self.source['xml'])
        p['motion_changes'][0]['value']=-1
        with self.assertRaises(ValueError):apply_proposal(self.source,p,'control')

class WorkflowTests(unittest.TestCase):
    def setUp(self):
        from xrobocon.workshop.controller import WorkshopController
        class Generator:
            model='test';reasoning_effort='low'
            def propose(self,snapshot,prompt,mode,cancelled):return proposal()
        self.temp=tempfile.TemporaryDirectory();self.controller=WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'),self.temp.name,Generator());self.controller.choose_robot('front_bogie_r18')
    def tearDown(self):self.controller.close();self.temp.cleanup()
    def test_prompt_apply_save_register_and_fork(self):
        import time
        c=self.controller;c.propose('低重心にして')
        for _ in range(100):
            c.poll()
            if c.future is None:break
            time.sleep(.001)
        self.assertIsNotNone(c.proposal);c.apply()
        with self.assertRaises(ValueError):c.register()
        saved=c.save();catalog=c.register();data=MatchCatalog.load(catalog)
        rid='workshop_'+c.current['robot_id'];self.assertIn(rid,data.options('robots'))
        from xrobocon.match_setup import build_match_config
        choices=data.defaults();choices['self']['robot']=rid;choices['self']['control']='baseline'
        config=build_match_config(data,choices);self.assertEqual(config['simulation']['robot_models']['self']['robot_asset'],str(saved/'model.xml'))
        c.load_revision(saved);self.assertEqual(c.current['revision_id'],saved.name)
    def test_invalid_apply_cannot_replace_current_snapshot(self):
        c=self.controller;c.proposal=proposal();c.proposal['motion_changes'][0]['value']=0
        before=deepcopy(c.current)
        with self.assertRaises(ValueError):c.apply()
        self.assertEqual(c.current,before);self.assertIsNone(c.directory)
    def test_saved_diagnostic_is_restored_and_invalidated_on_edit(self):
        from xrobocon.workshop.design import write_json
        c=self.controller;c.proposal=proposal();c.apply();saved=c.save()
        evaluations=saved/'evaluations';evaluations.mkdir();write_json(evaluations/'1.json',dict(model_sha256=c.current['model_sha256'],result={'passed':True}))
        c.load_revision(saved);self.assertTrue(c.last_result['passed'])
        c.proposal=proposal();c.apply();self.assertNotIn('diagnostic_result',c.current)
    def test_diagnostic_command_uses_generated_catalog(self):
        from unittest.mock import patch,Mock
        c=self.controller;c.proposal=proposal();c.apply()
        process=Mock();process.poll.return_value=None;process.wait.return_value=0
        with patch('xrobocon.workshop.controller.subprocess.Popen',return_value=process) as run:
            output=c.diagnostic('flat');command=run.call_args.args[0]
            self.assertIn('--catalog',command);self.assertEqual(command[command.index('--suite')+1],'flat')
            self.assertTrue((output/'catalog.json').is_file());self.assertFalse((output/'run').exists())
            process.poll.return_value=0;process.returncode=0
    def test_preview_before_edit_uses_snapshot_without_saving_revision(self):
        from unittest.mock import patch,Mock
        c=self.controller;before=deepcopy(c.current);process=Mock();process.poll.return_value=None
        with patch('xrobocon.workshop.controller.subprocess.Popen',return_value=process) as run:
            c.preview();command=run.call_args.args[0];asset=Path(command[command.index('--asset')+1])
            self.assertTrue(asset.exists());self.assertEqual(asset.read_text(),before['xml'])
            self.assertIsNone(c.directory);self.assertEqual(c.library.list(),[])
            process.poll.return_value=0;process.returncode=0
    def test_robot_only_changes_geometry_and_rejects_control_edits(self):
        c=self.controller;p=proposal()
        with self.assertRaises(ValueError):apply_proposal(c.current,p,'robot_only')
        p['motion_changes']=[];new=apply_proposal(c.current,p,'robot_only')
        self.assertEqual(new['tuning'],c.current['tuning']);self.assertNotEqual(new['xml'],c.current['xml'])

class PreviewUpdateTests(unittest.TestCase):
    setUp=WorkflowTests.setUp
    tearDown=WorkflowTests.tearDown
    def test_pending_preview_uses_candidate_without_applying(self):
        from unittest.mock import patch,Mock
        c=self.controller;before=deepcopy(c.current);c.proposal=proposal()
        process=Mock();process.poll.return_value=None
        with patch('xrobocon.workshop.controller.subprocess.Popen',return_value=process):
            c.preview()
            asset=c.preview_job['out']/'model.xml'
            self.assertNotEqual(asset.read_text(),before['xml'])
            self.assertEqual(c.current,before);self.assertIsNone(c.directory)
            self.assertFalse(c.busy)
            process.poll.return_value=0
    def test_open_preview_follows_apply_and_new_proposal(self):
        from unittest.mock import patch,Mock
        c=self.controller;process=Mock();process.poll.return_value=None
        with patch('xrobocon.workshop.controller.subprocess.Popen',return_value=process):
            c.preview();asset=c.preview_job['out']/'model.xml';before=asset.read_text()
            c.proposal=proposal();c.poll()
            self.assertNotEqual(asset.read_text(),before)
            self.assertIn('未適用',c.presentation()['status'])
            c.apply();c.poll()
            self.assertEqual(asset.read_text(),c.current['xml'])
            self.assertNotIn('未適用',c.presentation()['status'])
            process.poll.return_value=0
    def test_invalid_proposal_keeps_current_and_is_explicit(self):
        c=self.controller;c.proposal=proposal();c.proposal['motion_changes'][0]['value']=-1
        view=c.presentation()
        self.assertIn('エラー',view['status']);self.assertIsNone(view['snapshot'])
