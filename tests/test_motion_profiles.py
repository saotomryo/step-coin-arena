import unittest
from math import pi
from xrobocon.motion_profiles import resolve_profile
from xrobocon.tier_agent import FixedTierAgent
from tests import test_tier_agent
class MotionProfileTests(unittest.TestCase):
    def test_default_matches_explicit_baseline_commands(self):
        a=FixedTierAgent([[0],[1],[2]]);b=FixedTierAgent([[0],[1],[2]],control_profile='baseline')
        obs=test_tier_agent.TierAgentTests().observation()
        for _ in range(140):self.assertEqual(a.act(obs,.01),b.act(obs,.01));self.assertEqual(a.phase,b.phase)
        self.assertEqual(a.motion.climb_speed,.12);self.assertEqual(a.motion.settle_time,1)
    def test_phase_limits_and_time_are_independent(self):
        agent=FixedTierAgent([[0],[1],[2]],control_profile='fast',motion_limits={'climb_speed':.11})
        obs=test_tier_agent.TierAgentTests().observation()
        agent.act(obs,.01);self.assertEqual(agent.follower.max_speed,.11)
        for _ in range(49):agent.act(obs,.01)
        self.assertEqual(agent.phase,'coins');agent.act(obs,.01)
        self.assertEqual(agent.follower.max_speed,.22)
    def test_invalid_and_custom_limits(self):
        for changes in ({'settle_time':0},{'max_speed':float('nan')},{'oops':2}):
            with self.assertRaises(ValueError):resolve_profile('fast',.12,.3,changes)
        with self.assertRaises(ValueError):resolve_profile('unknown',.12,.3)
        self.assertNotEqual(resolve_profile('safe',.12,.3),resolve_profile('fast',.12,.3))

    def test_commands_match_frozen_controller_through_all_phases(self):
        from importlib.util import spec_from_file_location,module_from_spec
        from xrobocon.agent import Observation,PublicCompetition
        from xrobocon.navigation import Pose2D
        spec=spec_from_file_location('legacy_tier','baselines/control_20261014/tier_agent.py');module=module_from_spec(spec);spec.loader.exec_module(module)
        a=FixedTierAgent([[0],[1],[2]]);b=module.FixedTierAgent([[0],[1],[2]])
        coins=[dict(id=i,pos=[r,0,h+.005],tier=3-i,status='available',owner=None) for i,(r,h) in enumerate([(4,.1),(2.6,.35),(1.2,.6)])]
        for i in range(1500):
            if a.phase=='coins':coins[a.stage]['status']='collected'
            coin=coins[a.stage]
            obs=Observation(Pose2D(coin['pos'][0],0,pi),i*.01,'test',PublicCompetition('running',300,{},tuple(coins)),dict(speed=0,ground_z=coin['pos'][2]-.005,steering=[0]*6))
            self.assertEqual(a.act(obs,.01),b.act(obs,.01));self.assertEqual((a.phase,a.stage),(b.phase,b.stage))
            if a.done:break
        self.assertTrue(a.done);self.assertTrue(b.done)
