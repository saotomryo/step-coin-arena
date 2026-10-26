import unittest
import numpy as np
from types import SimpleNamespace
from xrobocon.game import XRoboconGame

class CoinTests(unittest.TestCase):
    def make_game(self, tier=1, height=.605):
        robot=SimpleNamespace(get_pos=lambda:np.array([1.,0.,.88]))
        spec={'rules':{'time_limit':180.,'coin_radius':.2,'upper_tier_stay_time':2.},
              'coins':[{'pos':[1,0,height],'tier':tier}]}
        game=XRoboconGame(None,robot,spec,robot_height_offset=.28)
        game.start()
        return game
    def test_top_requires_continuous_stay_and_scores_five(self):
        g=self.make_game()
        g.update(1);self.assertEqual(g.score,0)
        g.robot.get_pos=lambda:np.array([2.,0.,.88]);g.update(.1)
        g.robot.get_pos=lambda:np.array([1.,0.,.88]);g.update(1)
        self.assertEqual(g.score,0)
        g.update(1);self.assertEqual(g.score,5)
        g.update(1);self.assertEqual(g.score,5)
    def test_wrong_surface_height_is_not_collected(self):
        g=self.make_game(tier=2,height=.355);g.update(3)
        self.assertEqual(g.score,0)
    def test_bottom_immediate_one_point(self):
        g=self.make_game(tier=3,height=.105)
        g.robot.get_pos=lambda:np.array([1.,0.,.38]);g.update(.01)
        self.assertEqual(g.score,1)

