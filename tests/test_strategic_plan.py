"""Persistent AI plans must survive acquisitions and expire only on explicit guards."""
import unittest,time
from copy import deepcopy
from dataclasses import replace
from tests import test_route_strategy
from xrobocon.strategic_plan import validate_plan, CodexPlanSelector
from xrobocon.strategic_agent import StrategicTierAgent

def plan(ids=(0,1,2,3), **stops):
    conditions=dict(target_coin_unavailable=True,max_plan_seconds=40,min_remaining_seconds=20,max_blocked_seconds=5,score_gap_change=None)
    conditions.update(stops)
    return dict(strategy='Collect the outer pair, then ascend for higher scores.',coin_ids=list(ids),stop_conditions=conditions)

class Backend:
    model='test';reasoning_effort='low'
    def __init__(self,result):self.result=result;self.calls=[]
    def select(self,payload,cancelled):self.calls.append(deepcopy(payload));return deepcopy(self.result)

class StrategicPlanTests(unittest.TestCase):
    def setUp(self):
        self.obs=test_route_strategy.RouteStrategyTests().obs();self.agent=StrategicTierAgent(game_rules={'participant':'self'})
        self.backend=Backend(plan());self.agent.selector=self.backend
    def tearDown(self):self.agent.close()
    def finish_inference(self,obs=None):
        obs=obs or self.obs
        for _ in range(1000):
            if self.agent._future is not None and self.agent._future.done():break
            time.sleep(.001)
        else:self.fail('inference incomplete')
        self.agent.act(obs,.01)
    def start(self):self.agent.act(self.obs,.01);self.finish_inference();self.agent.phase='coins'
    def change_coin(self,cid,owner):
        coins=tuple(dict(c,status='collected',owner=owner) if c['id']==cid else c for c in self.obs.competition.coins)
        return replace(self.obs,competition=replace(self.obs.competition,coins=coins))
    def test_initial_plan_and_own_acquisition_do_not_request_again(self):
        self.start();self.assertEqual(self.agent.remaining_coin_ids,[0,1,2,3]);self.assertEqual(self.agent.target_id,0)
        self.agent.act(self.change_coin(0,'self'),.01)
        self.assertEqual(self.agent.target_id,1);self.assertEqual(len(self.backend.calls),1)
        self.assertEqual(self.agent.active_plan['coin_ids'],[0,1,2,3])
    def test_unrelated_opponent_acquisition_does_not_replan(self):
        self.backend.result=plan((0,2,3));self.start();self.agent.act(self.change_coin(1,'opponent'),.01)
        self.assertEqual(len(self.backend.calls),1);self.assertEqual(self.agent.target_id,0)
    def test_future_planned_coin_taken_by_opponent_triggers_replan(self):
        self.start();self.agent.act(self.change_coin(2,'opponent'),.01)
        self.assertTrue(self.agent.decision_pending);self.assertEqual(self.agent.replan_reason,'target_coin_unavailable')
    def test_time_limit_and_remaining_threshold(self):
        self.start();self.agent.act(replace(self.obs,timestamp=41),.01)
        self.assertEqual(self.agent.replan_reason,'max_plan_seconds')
        self.agent.reset();self.start()
        self.agent.act(replace(self.obs,timestamp=1,competition=replace(self.obs.competition,remaining=19)),.01)
        self.assertEqual(self.agent.replan_reason,'min_remaining_seconds')
    def test_threshold_already_crossed_at_acceptance_does_not_loop(self):
        self.obs=replace(self.obs,competition=replace(self.obs.competition,remaining=10));self.start()
        self.agent.act(replace(self.obs,timestamp=1),.01);self.assertEqual(len(self.backend.calls),1)
    def test_score_change_guard(self):
        self.backend.result=plan(score_gap_change=3);self.start()
        self.agent.act(replace(self.obs,competition=replace(self.obs.competition,scores={'self':0,'opponent':3})),.01)
        self.assertEqual(self.agent.replan_reason,'score_gap_change')
    def test_plan_completion_requests_new_plan(self):
        self.backend.result=plan((0,));self.start();self.agent.act(self.change_coin(0,'self'),.01)
        self.assertEqual(self.agent.replan_reason,'plan_completed');self.assertTrue(self.agent.decision_pending)
    def test_validation_rejects_duplicate_unavailable_descent_and_bad_guard(self):
        coins=self.obs.competition.coins
        for result in (plan((0,0)),plan((3,0)),plan((99,)),plan((),max_plan_seconds=0),plan(max_plan_seconds=True),plan(target_coin_unavailable=False),plan(max_blocked_seconds=float('nan'))):
            with self.assertRaises(ValueError):validate_plan(result,coins,0)
        self.assertEqual(validate_plan(plan(),coins,0)['coin_ids'],[0,1,2,3])
    def test_stale_inference_is_rejected_and_requested_again(self):
        self.agent.act(self.obs,.01);self.finish_inference(self.change_coin(2,'opponent'))
        self.assertTrue(self.agent.decision_pending);self.assertIsNone(self.agent.active_plan)
    def test_empty_board_waits_without_inference(self):
        empty=replace(self.obs,competition=replace(self.obs.competition,coins=tuple(dict(c,status='collected') for c in self.obs.competition.coins)))
        self.agent.act(empty,.01);self.assertEqual(self.backend.calls,[])
    def test_blocked_guard_and_movement_reset(self):
        self.start();self.agent._was_moving=True
        self.agent._monitor(self.obs,3)
        self.assertEqual(self.agent._blocked,3)
        from xrobocon.navigation import Pose2D
        moved=replace(self.obs,pose=Pose2D(self.obs.pose.x+.06,self.obs.pose.y,self.obs.pose.yaw))
        self.agent._monitor(moved,3);self.assertEqual(self.agent._blocked,0)
        self.agent._monitor(moved,5)
        self.assertEqual(self.agent.replan_reason,'max_blocked_seconds')
    def test_recovery_reset_discards_strategy_and_cancels_request(self):
        self.start();cancel=self.agent._cancel;self.agent.reset()
        self.assertIsNone(self.agent.active_plan);self.assertEqual(self.agent.remaining_coin_ids,[])
        self.assertTrue(cancel.is_set());self.assertTrue(self.agent._force_replan)
    def test_unrelated_spawn_does_not_infer(self):
        self.backend.result=plan((0,2,3));self.start()
        coins=tuple(dict(c,spawn_count=2) if c['id']==1 else c for c in self.obs.competition.coins)
        self.agent.act(replace(self.obs,competition=replace(self.obs.competition,coins=coins)),.01)
        self.assertEqual(len(self.backend.calls),1)
    def test_catalog_all_models_compile_without_network(self):
        from pathlib import Path
        from xrobocon.match_setup import MatchCatalog,build_match_config
        c=MatchCatalog.load(Path(__file__).resolve().parents[1]/'configs/match_catalog.json')
        for robot in c.options('robots'):
            choices=c.defaults();choices['opponent'].update(robot=robot,control='baseline',strategy='codex_plan')
            v=build_match_config(c,choices)
            self.assertEqual(v['agents']['opponent']['game_rules']['participant'],'opponent')
            self.assertEqual(v['agents']['opponent']['kind'],'strategic_tier')
            self.assertEqual(v['transport']['kind'],'ros2')
    def test_future_upper_target_still_uses_rule_ascent(self):
        self.backend.result=plan((2,3));self.start();self.agent.act(self.obs,.01)
        self.assertEqual(self.agent.stage,1);self.assertEqual(self.agent.phase,'align')
        self.assertEqual(self.agent.route,(2,));self.assertEqual(len(self.backend.calls),1)
    def test_invalid_inference_does_not_retry_every_tick(self):
        self.backend.result=plan((0,0));self.agent.act(self.obs,.01);self.finish_inference()
        for _ in range(20):self.agent.act(self.obs,.01)
        self.assertIsNotNone(self.agent.decision_error);self.assertEqual(len(self.backend.calls),1)

    def test_request_includes_bounded_position_history(self):
        self.start()
        for t in range(1,20):self.agent.act(replace(self.obs,timestamp=t),.01)
        self.assertLessEqual(len(self.agent._recent_positions),6)
        self.assertIn('recent_robot_positions',self.backend.calls[0]['state'])
        self.assertEqual(self.backend.calls[0]['state']['recent_robot_positions'][0]['timestamp'],0)
