import unittest
from dataclasses import replace
from math import pi
from xrobocon.agent import Observation,PublicCompetition
from xrobocon.navigation import Pose2D,VelocityCommand
from xrobocon.collision_avoidance import CollisionAvoidance

class AvoidanceTests(unittest.TestCase):
    def obs(self,x=1.4,y=0,t=0):
        return Observation(Pose2D(0,0,0),t,'test',PublicCompetition('running',180,{},()),{'ground_z':0}, {'red':[x,y,0]})
    def test_predicts_approach_but_allows_divergence(self):
        a=CollisionAvoidance();o=self.obs()
        self.assertIsNotNone(a.threat(o,VelocityCommand(.22,0)))
        self.assertIsNone(a.threat(o,VelocityCommand(-.22,0)))
    def test_crossing_motion_and_stopped_command(self):
        a=CollisionAvoidance();a.observe(self.obs(1,1,0))
        o=self.obs(1,.9,.1);a.observe(o)
        self.assertIsNotNone(a.threat(o,VelocityCommand(.22,0)))
        self.assertIsNone(a.threat(o,VelocityCommand(0,0)))
    def test_stop_then_choose_clear_ring_leg(self):
        from tests.test_route_strategy import RouteStrategyTests
        from xrobocon.adaptive_tier_agent import AdaptiveTierAgent
        o=RouteStrategyTests().obs();a=AdaptiveTierAgent(avoidance={'enabled':True})
        a.act(o,.01);a.phase='coins'
        o=replace(o,other_robots={'red':[4,1,.105]},competition=replace(o.competition,coins=o.competition.coins+(dict(o.competition.coins[0],id=99,pos=[3.51,-1.92,.105]),)))
        a.avoidance.blocked_until=0
        self.assertTrue(a.avoidance.reroute(a,o))
        self.assertEqual(a.plan_event['reason'],'collision_avoidance')
        self.assertIsNotNone(a.follower)
    def test_invalid_settings(self):
        for opts in ({'clearance':-1},{'horizon':float('nan')},{'pause':-1}):
            with self.assertRaises(ValueError):CollisionAvoidance(**opts)
    def test_climbing_stops_without_lateral_turn_or_push_assist(self):
        from types import SimpleNamespace
        a=CollisionAvoidance();agent=SimpleNamespace(phase='climb')
        command=a.filter(agent,self.obs(.9,0),VelocityCommand(.14,0,climb_stage=1))
        self.assertEqual((command.linear,command.angular,command.climb_stage),(0,0,None))
        command=a.filter(agent,self.obs(.9,0,.2),VelocityCommand(.14,0,climb_stage=1))
        self.assertEqual((command.linear,command.angular),(0,0))
    def test_clear_opponent_resumes_after_pause(self):
        from types import SimpleNamespace
        a=CollisionAvoidance();agent=SimpleNamespace(phase='climb')
        a.filter(agent,self.obs(.9),VelocityCommand(.14,0))
        command=a.filter(agent,self.obs(3,0,1),VelocityCommand(.14,0))
        self.assertEqual(command.linear,.14);self.assertFalse(a.waiting)
    def test_runner_exposes_same_snapshot_without_shared_mutation(self):
        from xrobocon.runner import CompetitionRunner
        from xrobocon.competition import CompetitionManager
        seen={}
        class Sim:
            def reset(self):pass
            def read_pose(self,p):return Pose2D(0,0,0)
            def truth_positions(self):return {'blue':[0,0,0],'red':[2,0,0]}
            def step(self,commands,dt):pass
            def speeds(self):return {'blue':0,'red':0}
            def stop(self):pass
        class Agent:
            def __init__(self,name):self.name=name
            def reset(self):pass
            def act(self,obs,dt):
                seen[self.name]={k:list(v) for k,v in obs.other_robots.items()}
                for pos in obs.other_robots.values():pos[0]=99
                return VelocityCommand(0,0)
        manager=CompetitionManager([],participants=['blue','red'])
        runner=CompetitionRunner(Sim(),manager,{p:Agent(p) for p in ('blue','red')})
        runner.start();runner.tick()
        self.assertEqual(seen,{'blue':{'red':[2,0,0]},'red':{'blue':[0,0,0]}})
    def test_no_reachable_coin_uses_space_on_same_tier(self):
        from tests.test_route_strategy import RouteStrategyTests
        from xrobocon.adaptive_tier_agent import AdaptiveTierAgent
        o=RouteStrategyTests().obs();a=AdaptiveTierAgent(avoidance={'enabled':True})
        a.act(o,.01);a.phase='coins'
        o=replace(o,other_robots={'red':[4,1,.105]})
        self.assertTrue(a.avoidance.reroute(a,o))
        self.assertTrue(a.avoidance_retreat)
        self.assertEqual(a.plan_event['reason'],'collision_retreat')
        self.assertLess(a.follower.points[-1][1],0)
    def test_stop_then_detour_with_full_filter(self):
        from tests.test_route_strategy import RouteStrategyTests
        from xrobocon.adaptive_tier_agent import AdaptiveTierAgent
        o=replace(RouteStrategyTests().obs(),pose=Pose2D(4,0,pi/2),other_robots={'red':[4,1,.105]})
        a=AdaptiveTierAgent(avoidance={'enabled':True});a.act(replace(o,other_robots={}),.01);a.phase='coins'
        command=a.avoidance.filter(a,o,VelocityCommand(.22,0))
        self.assertEqual(command.linear,0);self.assertEqual(a.avoidance.event['kind'],'stop')
        a.avoidance.filter(a,replace(o,timestamp=1),VelocityCommand(.22,0))
        self.assertEqual(a.avoidance.event['kind'],'reroute');self.assertFalse(a.avoidance.waiting)
