"""Bounded iteration semantics, model settings and cancellation regressions."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.examples import lifting_robot
from xrobocon.match_setup import MatchCatalog
ROOT=Path(__file__).resolve().parents[1]

class AutomaticTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.c=WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'),self.tmp.name)
        self.c.mode='new_robot';self.c.proposal=lifting_robot();self.c.apply();self.c.save()
    def tearDown(self):self.c.close();self.tmp.cleanup()
    def test_settings_apply_to_both_generators_and_persist(self):
        self.c.configure_inference('gpt-6.1-sol','high',300)
        self.assertEqual(self.c.generator.model,'gpt-6.1-sol')
        self.assertEqual(self.c.new_generator.reasoning_effort,'high')
        other=WorkshopController(self.c.catalog,self.tmp.name)
        try:self.assertEqual(other.inference_settings['timeout'],300)
        finally:other.close()
    def test_limit_means_at_most_n_improvements_plus_final_validation(self):
        from xrobocon.workshop.automation import ImprovementLoop
        calls=[]
        def diagnostic(suite):calls.append('test');self.c.last_result={'results':[dict(complete=True,passed=False)]};self.c.job={'root':Path(self.tmp.name)}
        def improve(prompt,**kwargs):calls.append('improve');self.c.mode='program';self.c.proposal=lifting_robot()
        with patch.object(self.c,'diagnostic',side_effect=diagnostic),patch.object(self.c,'improve_from_logs',side_effect=improve):
            loop=ImprovementLoop(self.c,2,'flat','旋回を改善',False,120)
            for _ in range(20):loop.tick()
        self.assertEqual(calls,['test','improve','test','improve','test'])
        self.assertFalse(loop.active);self.assertEqual(loop.state['reason'],'limit_reached')
        self.assertEqual(loop.state['improvement_attempts'],2)
    def test_success_can_stop_early_but_smoke_is_not_performance_pass(self):
        from xrobocon.workshop.automation import report_passed
        self.assertTrue(report_passed(dict(results=[dict(passed=True,complete=True)]),'flat'))
        self.assertFalse(report_passed(dict(results=[dict(passed=True,complete=True,status='smoke_only_performance_unrated')]),'joints'))
        self.assertFalse(report_passed(dict(results=[]),'flat'))
    def test_cancel_and_timeout_do_not_start_more_calls(self):
        from xrobocon.workshop.automation import ImprovementLoop
        loop=ImprovementLoop(self.c,2,'flat','',True,1)
        loop.cancel();loop.tick();self.assertFalse(loop.active)
        self.assertEqual(loop.state['reason'],'cancelled')
        loop=ImprovementLoop(self.c,2,'flat','',True,1)
        loop.stage_started-=2;loop.tick();self.assertEqual(loop.state['reason'],'timeout')

    def test_active_loop_blocks_manual_mutation(self):
        with patch.object(self.c,'diagnostic'):
            self.c.start_automation(2,'flat')
            with self.assertRaises(ValueError):self.c.configure_inference('gpt-6.1-sol','high',300)
            with self.assertRaises(ValueError):self.c.choose_robot('front_bogie_r18')
            self.c.cancel()
        self.assertFalse(self.c.automation.active)

    def test_early_success_uses_no_inference(self):
        from xrobocon.workshop.automation import ImprovementLoop
        def diagnostic(suite):
            self.c.last_result={'results':[dict(complete=True,passed=True)]}
            self.c.job={'root':Path(self.tmp.name)}
        with patch.object(self.c,'diagnostic',side_effect=diagnostic),patch.object(self.c,'improve_from_logs') as improve:
            loop=ImprovementLoop(self.c,3,'flat','',True,120)
            loop.tick();loop.tick()
        improve.assert_not_called();self.assertEqual(loop.state['reason'],'passed')

    def test_validation_rejections_consume_attempt_limit(self):
        from xrobocon.workshop.automation import ImprovementLoop
        def diagnostic(suite):
            self.c.last_result={'results':[dict(complete=True,passed=False)]};self.c.job={'root':Path(self.tmp.name)}
        def improve(prompt,**kwargs):
            self.c.proposal=None;self.c.rejected_pending=dict(file='failed.json',mode='program')
        with patch.object(self.c,'diagnostic',side_effect=diagnostic),patch.object(self.c,'improve_from_logs',side_effect=improve):
            loop=ImprovementLoop(self.c,2,'flat','',False,120)
            for _ in range(20):loop.tick()
        self.assertEqual(loop.state['improvement_attempts'],2)
        self.assertEqual(loop.state['reason'],'limit_reached')

    def test_model_error_stops_instead_of_retrying(self):
        from xrobocon.workshop.automation import ImprovementLoop
        loop=ImprovementLoop(self.c,3,'flat')
        loop.state['phase']='wait_improve';self.c.proposal=None
        self.c.message='model unavailable'
        loop.tick()
        self.assertEqual(loop.state['reason'],'inference_error')
        self.assertEqual(loop.state['improvement_attempts'],0)
