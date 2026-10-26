"""Robot identity and failed-run -> control improvement regressions."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
from xrobocon.match_setup import MatchCatalog
from xrobocon.workshop.controller import WorkshopController,catalog_with_revision
from xrobocon.workshop.examples import lifting_robot
from xrobocon.workshop.design import write_json
ROOT=Path(__file__).resolve().parents[1]

class IterationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.c=WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'),self.temp.name)
    def tearDown(self):self.c.close();self.temp.cleanup()
    def new(self):
        self.c.mode='new_robot';self.c.proposal=lifting_robot();self.c.apply();self.c.save()
        return dict(self.c.current)
    def wait(self):
        for _ in range(100):
            self.c.poll()
            if not self.c.busy:return
            time.sleep(.005)
        self.fail('inference did not finish')

    def test_new_robot_is_independent_but_control_edit_keeps_identity(self):
        first=self.new();self.c.register()
        self.c.mode='program';self.c.proposal=lifting_robot();self.c.apply();self.c.save()
        second=self.c.current
        self.assertEqual(first['robot_id'],second['robot_id'])
        self.assertEqual(second['parent'],first['revision_id'])
        self.c.register();catalog=json.loads((Path(self.temp.name)/'catalog.json').read_text())
        registered=[r for r in catalog['robots'].values() if r.get('workshop_robot_id')==first['robot_id']]
        self.assertEqual(len(registered),1)
        third=self.new();self.c.register()
        self.assertNotEqual(first['robot_id'],third['robot_id'])
        self.assertIsNone(third['parent'])
        self.assertNotEqual(first['revision_id'],third['revision_id'])

    def test_save_as_new_robot_detaches_old_revision(self):
        first=self.new()
        self.c.save_as_new_robot('四足ロボットB')
        self.assertNotEqual(first['robot_id'],self.c.current['robot_id'])
        self.assertEqual(self.c.current['robot_name'],'四足ロボットB')
        self.assertIsNone(self.c.current['parent'])
        self.assertEqual(self.c.library.load(Path(self.temp.name)/'revisions'/first['revision_id'])['robot_id'],first['robot_id'])

    def complete_failed_run(self):
        process=Mock();process.poll.return_value=None
        with patch('xrobocon.workshop.controller.subprocess.Popen',return_value=process):
            job=self.c.diagnostic('joints')
        (job/'run').mkdir()
        write_json(job/'run/report.json',dict(results=[dict(passed=False,error='tipped',trials=[])]))
        (job/'process.log').write_text('physics warning: leg slips\n')
        (job/'run/joints.jsonl').write_text('\n'.join(json.dumps(dict(case='turn',position=[.1],targets=[.5],torques=[4.],tilt=95)) for _ in range(10)))
        process.returncode=1;process.poll.return_value=1
        self.c.poll()
        return job

    def test_failed_trace_is_persisted_reloaded_and_sent_to_generator(self):
        self.new();job=self.complete_failed_run()
        self.c.load_revision(self.c.directory)
        class Generator:
            model='test';reasoning_effort='low'
            def propose(inner,snapshot,prompt,mode,cancelled):
                self.payload=snapshot
                return lifting_robot()
        self.c.new_generator=Generator()
        self.c.improve_from_logs('旋回時の転倒を減らす')
        self.wait()
        context=self.payload['diagnostic_context']
        self.assertIn('leg slips',json.dumps(context))
        self.assertIn('95',json.dumps(context))
        self.assertTrue(context['runs'][0]['matches_current'])
        self.assertTrue((job/'input/controller.py').exists())
        self.assertTrue((job/'diagnostic.json').exists())
        self.assertIsNotNone(self.c.proposal)

    def test_partial_run_without_report_keeps_logs(self):
        self.new()
        process=Mock();process.poll.return_value=None
        with patch('xrobocon.workshop.controller.subprocess.Popen',return_value=process):job=self.c.diagnostic('joints')
        (job/'process.log').write_text('ImportError: missing dependency')
        process.poll.return_value=2;process.returncode=2;self.c.poll()
        self.assertIn('missing dependency',self.c.log_context_text())
        self.assertFalse(self.c.last_result['complete'])

    def test_updated_control_requires_new_matching_run(self):
        self.new();self.complete_failed_run()
        p=lifting_robot();p['controller_code']=p['controller_code'].replace('.6 *', '.5 *')
        self.c.mode='program';self.c.proposal=p;self.c.apply()
        with self.assertRaises(ValueError):self.c.improve_from_logs('改善')

    def test_rejected_design_can_be_repaired_without_losing_the_robot(self):
        first=self.new()
        self.c.mode='program';bad=lifting_robot();bad['controller_code']='invalid python ???'
        self.c.proposal=bad
        with self.assertRaises(ValueError):self.c.apply()
        self.assertEqual(self.c.current['revision_id'],first['revision_id'])
        self.assertIn('invalid',self.c.log_context_text())
        class Generator:
            model='test';reasoning_effort='low'
            def propose(inner,snapshot,prompt,mode,cancelled,feedback=None):
                self.feedback=feedback
                return lifting_robot()
        self.c.new_generator=Generator();self.c.improve_from_logs('エラーを修正');self.wait();self.c.apply()
        self.assertIn('invalid python',self.feedback['proposal']['controller_code'])
        self.assertEqual(self.c.current['robot_id'],first['robot_id'])
        self.assertTrue(self.c.current['improvement_basis'][0].startswith('validation-'))

    def test_legacy_revisions_get_stable_family_without_rewriting_old_manifest(self):
        first=self.new();path=self.c.directory
        raw=json.loads((path/'manifest.json').read_text());raw.pop('robot_id');raw.pop('robot_name')
        write_json(path/'manifest.json',raw);before=(path/'manifest.json').read_bytes()
        one=self.c.library.load(path);two=self.c.library.load(path)
        self.assertEqual(one['robot_id'],two['robot_id'])
        self.assertEqual((path/'manifest.json').read_bytes(),before)

    def test_cancelled_run_and_dry_run_are_not_treated_as_success(self):
        self.new();job=self.complete_failed_run()
        record=json.loads((job/'diagnostic.json').read_text());record['dry_run']=True;write_json(job/'diagnostic.json',record)
        with self.assertRaises(ValueError):self.c.improve_from_logs('改善')

    def test_generator_rejection_is_saved_even_for_new_robot(self):
        from xrobocon.workshop.inference import RejectedDesign
        class Generator:
            model='test';reasoning_effort='low'
            def propose(inner,snapshot,prompt,mode,cancelled):
                raise RejectedDesign(lifting_robot(),'compiler failed on an invalid joint')
        self.c.new_generator=Generator();self.c.propose('新規機体','new_robot');self.wait()
        self.assertIsNone(self.c.current)
        self.assertIsNotNone(self.c.rejected_pending)
        self.assertTrue(Path(self.c.rejected_pending['file']).is_file())
        self.assertIn('compiler failed',self.c.log_context_text())

    def test_trace_summary_retains_worst_and_last_not_just_beginning(self):
        from xrobocon.workshop.diagnostics import trace_evidence
        path=Path(self.temp.name)/'trace.jsonl'
        rows=[dict(time=i,tilt=99 if i==3 else i,joint_names=['a'],position=[i],targets=[0],tracking_errors=[i],torques=[2],saturation=[True]) for i in range(5)]
        path.write_text('\n'.join(json.dumps(row) for row in rows))
        summary=trace_evidence(path)
        self.assertEqual(summary['first']['time'],0)
        self.assertEqual(summary['worst']['time'],3)
        self.assertEqual(summary['last']['time'],4)
        self.assertEqual(summary['worst_joints'][0]['max_abs_error'],4)
