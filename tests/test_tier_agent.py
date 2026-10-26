"""Terrain phase transitions depend on public collection and measured stability."""
import unittest
from xrobocon.agent import Observation,PublicCompetition
from xrobocon.navigation import Pose2D
from xrobocon.tier_agent import FixedTierAgent
from math import pi

class TierAgentTests(unittest.TestCase):
    def observation(self,z=.1,status='available'):
        coins=tuple(dict(id=i,pos=[r,0,h+.005],tier=3-i,status=status if i==0 else 'available',owner=None) for i,(r,h) in enumerate([(4,.1),(2.6,.35),(1.2,.6)]))
        return Observation(Pose2D(4,0,pi),1,'test',PublicCompetition('running',300,{},coins),dict(speed=0,ground_z=z,steering=[0]*6))
    def test_climb_keeps_steering_straight_after_alignment(self):
        from dataclasses import replace
        agent=FixedTierAgent([[0],[1],[2]])
        obs=replace(self.observation(z=0),pose=Pose2D(5.2,.02,pi-.02))
        self.assertEqual(agent.act(obs,.01).angular,0)

    def test_straightening_accepts_small_heading_drift(self):
        from dataclasses import replace
        agent=FixedTierAgent([[0],[1],[2]])
        agent.phase='straighten'
        obs=replace(self.observation(),pose=Pose2D(4,0,pi+.04))
        for _ in range(100):agent.act(obs,.01)
        self.assertEqual(agent.phase,'climb')

    def test_alignment_uses_actual_radial_direction(self):
        from dataclasses import replace
        agent=FixedTierAgent([[0],[1],[2]],gate_angle=pi/2)
        agent.phase='align'
        obs=replace(self.observation(),pose=Pose2D(-.08,2.6,-pi/2))
        self.assertGreater(agent.act(obs,.01).angular,.001)

    def test_cannot_finish_climb_with_only_xy_goal(self):
        agent=FixedTierAgent([[0],[1],[2]])
        for _ in range(120):agent.act(self.observation(z=0),.01)
        self.assertEqual(agent.phase,'climb')
        for _ in range(100):agent.act(self.observation(),.01)
        self.assertEqual(agent.phase,'coins')
        self.assertEqual(agent.stage,0)
    def test_authority_collection_starts_return_gate(self):
        agent=FixedTierAgent([[0],[1],[2]])
        for _ in range(100):agent.act(self.observation(),.01)
        agent.act(self.observation(status='collected'),.01)
        self.assertEqual(agent.phase,'return_gate')
        self.assertFalse(agent.done)
    def test_coin_targets_cannot_cut_across_different_rings(self):
        agent=FixedTierAgent([[0,1],[2],[3]])
        obs=self.observation()
        from dataclasses import replace
        coins=list(obs.competition.coins)+[dict(id=3,pos=[1.2,0,.605],tier=1,status="available",owner=None)]
        with self.assertRaises(ValueError):agent.act(replace(obs,competition=replace(obs.competition,coins=tuple(coins))),.01)

    def test_duplicate_targets_rejected(self):
        with self.assertRaises(ValueError):FixedTierAgent([[0],[0],[2]])

if __name__=='__main__':unittest.main()
