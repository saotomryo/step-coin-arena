import unittest
from dataclasses import replace
from threading import Event
import time
from xrobocon.codex_tier_agent import CodexTierAgent
from xrobocon.coin_selector import selection_input, validate_choice
from tests import test_route_strategy

class Backend:
    def __init__(self, cid=0, error=None):
        self.cid=cid; self.error=error; self.release=Event();self.calls=[]
    def select(self,payload,cancelled):
        self.calls.append(payload)
        while not self.release.wait(.001):
            if cancelled.is_set():raise RuntimeError('cancelled')
        if self.error:raise self.error
        return {'coin_id':self.cid}

class CodexStrategyTests(unittest.TestCase):
    def setUp(self):
        self.obs=test_route_strategy.RouteStrategyTests().obs();self.agent=CodexTierAgent();self.backend=Backend();self.agent.selector=self.backend
    def tearDown(self):self.agent.close()
    def settle(self):
        for _ in range(1000):
            if self.agent._future and self.agent._future.done():return
            time.sleep(.001)
        self.fail('inference did not complete')
    def test_async_initial_stop_then_rule_route(self):
        self.assertEqual(self.agent.act(self.obs,.01).linear,0)
        for _ in range(30):self.agent.act(self.obs,.01)
        self.assertTrue(self.agent.decision_pending);self.assertEqual(len(self.backend.calls),1)
        self.backend.release.set();self.settle();self.agent.act(self.obs,.01)
        self.assertEqual(self.agent.target_id,0);self.assertEqual(self.agent.route,(0,));self.assertFalse(self.agent.decision_pending)
        self.assertIn('decision',self.agent.plan_event)
        self.agent.phase='climb';self.assertEqual(self.agent.act(self.obs,.01).angular,0)
        self.agent.phase='coins';self.agent.act(self.obs,.01)
        self.assertIsNotNone(self.agent.follower)
    def test_higher_target_uses_intermediate_radial_ascent(self):
        self.backend.cid=3;self.backend.release.set();self.agent.act(self.obs,.01);self.settle();self.agent.act(self.obs,.01)
        self.assertEqual(self.agent.route,())
        self.agent.phase='coins';self.agent.act(self.obs,.01)
        self.assertEqual(self.agent.stage,1);self.assertEqual(self.agent.phase,'align')
        self.agent.phase='coins';self.agent.act(self.obs,.01)
        self.assertEqual(self.agent.stage,2);self.assertEqual(self.agent.route,(3,))
        self.assertEqual(len(self.backend.calls),1)
    def test_opponent_acquisition_discards_stale_target_and_retries(self):
        self.agent.act(self.obs,.01)
        coins=tuple(dict(c,status='collected',owner='opponent') if c['id']==0 else c for c in self.obs.competition.coins)
        updated=replace(self.obs,competition=replace(self.obs.competition,coins=coins))
        self.agent.act(updated,.01);self.backend.release.set();self.settle();self.agent.act(updated,.01)
        self.assertIsNone(self.agent.target_id);self.assertTrue(self.agent.decision_pending)
        self.settle();self.agent.act(updated,.01)
        self.assertIsNotNone(self.agent.decision_error);self.assertEqual(self.agent.act(updated,.01).linear,0)
        self.assertEqual(len(self.backend.calls),2)
    def test_backend_failure_stops_without_heuristic_fallback(self):
        self.backend.error=TimeoutError('test timeout');self.backend.release.set()
        self.agent.act(self.obs,.01);self.settle();cmd=self.agent.act(self.obs,.01)
        self.assertEqual(cmd.linear,0);self.assertIn('timeout',self.agent.decision_error)
        for _ in range(20):self.agent.act(self.obs,.01)
        self.assertEqual(len(self.backend.calls),1)
    def test_reset_cancels_old_request(self):
        self.agent.act(self.obs,.01);old=self.agent._cancel;self.agent.reset()
        self.assertTrue(old.is_set());self.assertIsNone(self.agent._future);self.assertIsNone(self.agent.target_id)
    def test_input_contains_rules_positions_and_no_lower_candidates(self):
        p=selection_input(self.obs,1,self.agent.motion,{'time_limit':180})
        self.assertEqual(p['movement_rules']['candidate_ids'],[2,3]);self.assertEqual(len(p['state']['coins']),4)
        self.assertIn('own_pose',p['state']);self.assertIn('other_robots',p['state'])
        self.assertEqual(p['rules']['configured']['time_limit'],180)
    def test_output_validation(self):
        for obj in ({'coin_id':True},{'coin_id':99},{'coin_id':None},{'coin_id':0,'extra':1}):
            with self.assertRaises(ValueError):validate_choice(obj,[0])
        self.assertIsNone(validate_choice({'coin_id':None},[]))
    def test_catalog_all_models_can_select_codex_without_network(self):
        from xrobocon.match_setup import MatchCatalog,build_match_config
        from pathlib import Path
        catalog=MatchCatalog.load(Path(__file__).resolve().parents[1]/'configs/match_catalog.json')
        for robot in catalog.options('robots'):
            choices=catalog.defaults();choices['self'].update(robot=robot,control='baseline',strategy='codex_target')
            cfg=build_match_config(catalog,choices)
            self.assertEqual(cfg['agents']['self']['kind'],'codex_tier')
            self.assertEqual(cfg['agents']['self']['game_rules']['time_limit'],120)
            self.assertEqual(cfg['transport']['kind'],'ros2')

class CodexBackendTests(unittest.TestCase):
    def run_backend(self, body, timeout=2):
        import tempfile,sys
        from pathlib import Path
        from xrobocon.coin_selector import CodexCoinSelector
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'fake-codex'
            path.write_text('#!'+sys.executable+'\nimport sys,json,time\nfrom pathlib import Path\n'+body)
            path.chmod(0o700)
            return CodexCoinSelector(str(path),timeout).select({'movement_rules':{'candidate_ids':[7]}})
    def test_structured_output_from_subprocess(self):
        result=self.run_backend("assert sys.argv[sys.argv.index('--model')+1]=='gpt-6-luna'\nassert 'model_reasoning_effort=\"low\"' in sys.argv\nsys.stdin.read()\nPath(sys.argv[sys.argv.index('-o')+1]).write_text(json.dumps({'coin_id':7}))\n")
        self.assertEqual(result,{'coin_id':7})
    def test_subprocess_invalid_choice_rejected(self):
        with self.assertRaises(ValueError):
            self.run_backend("sys.stdin.read()\nPath(sys.argv[sys.argv.index('-o')+1]).write_text(json.dumps({'coin_id':8}))\n")
    def test_subprocess_timeout_and_exit_are_reported(self):
        with self.assertRaises(TimeoutError):self.run_backend('sys.stdin.read()\ntime.sleep(10)\n',timeout=1)
        with self.assertRaises(RuntimeError):self.run_backend('sys.exit(2)\n')
    def test_panel_displays_inference_pending_target_and_error(self):
        from xrobocon.native_panel import decision_status
        def status(**kw):return decision_status({'agent_states':{'self':kw}})
        self.assertIn('推論待ち',status(decision_pending=True))
        self.assertIn('コイン7',status(target_id=7))
        self.assertIn('エラー',status(decision_error='timeout'))

    def test_model_defaults_and_validation(self):
        from xrobocon.coin_selector import CodexCoinSelector
        selector=CodexCoinSelector()
        self.assertEqual((selector.model,selector.reasoning_effort),('gpt-6-luna','low'))
        for options in ({'model':''},{'model':'--bad'},{'reasoning_effort':'unknown'}):
            with self.assertRaises(ValueError):CodexCoinSelector(**options)
