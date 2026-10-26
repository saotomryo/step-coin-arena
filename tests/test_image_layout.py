"""Checks photo reconstruction assumptions, not unverified official rules."""
import json,unittest
from pathlib import Path
from collections import Counter
from math import hypot
from xrobocon.tier_agent import FixedTierAgent

class ImageLayoutTests(unittest.TestCase):
    def test_visible_locations_and_fixed_targets_cover_all_spots(self):
        root=Path(__file__).resolve().parents[1]
        c=json.loads((root/'configs/framework_image_layout.json').read_text())
        self.assertEqual(Counter(coin['tier'] for coin in c['competition']['coins']),{3:9,2:7,1:2})
        ids=[i for agent in c['agents'].values() for group in agent['tier_coin_ids'] for i in group]
        self.assertEqual(sorted(ids),list(range(18)))
        for coin in c['competition']['coins']:
            self.assertLessEqual(coin['pos'][0],1e-6)
        for spawn in c['simulation']['spawns'].values():
            self.assertLess(spawn['pos'][0],-4.65)
            self.assertLess(hypot(*spawn['pos'][:2]),6.875)
    def test_return_gate_uses_short_arc_across_angle_wrap(self):
        from math import pi
        from xrobocon.agent import Observation,PublicCompetition
        from xrobocon.navigation import Pose2D
        agent=FixedTierAgent([[0],[1],[2]],gate_angles=[-pi,200*pi/180,225*pi/180])
        agent.phase='return_gate';agent.angle=-pi/2
        coins=tuple(dict(id=i,pos=[r,0,h],tier=3-i,status='available',owner=None) for i,(r,h) in enumerate([(4,.105),(2.6,.355),(.95,.605)]))
        obs=Observation(Pose2D(0,-4,0),0,'test',PublicCompetition('running',600,{},coins),dict(speed=0,ground_z=.1,steering=[0]*6))
        agent.act(obs,.01)
        self.assertLessEqual(len(agent.follower.points),7)
        self.assertTrue(all(x<=1e-6 for x,y in agent.follower.points))

    def test_gate_changes_with_stage_and_rejects_invalid_angles(self):
        agent=FixedTierAgent([[0],[1],[2]],gate_angles=[3,2.8,2.4])
        self.assertEqual(agent.gate_angle,3)
        agent.stage=2;self.assertEqual(agent.gate_angle,2.4)
        with self.assertRaises(ValueError):FixedTierAgent([[0],[1],[2]],gate_angles=[3])

if __name__=='__main__':unittest.main()
