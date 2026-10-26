import unittest
from math import hypot,pi
from xrobocon.coin_routes import ring_path,tier_coin_plans
from xrobocon.game import XRoboconGame

class RingTests(unittest.TestCase):
    def test_ring_segments_clear_inner_tier(self):
        for radius,inner in [(4,3.25),(2.55,1.85)]:
            points=[(radius,0)]+ring_path(0,pi/2,radius)
            for a,b in zip(points,points[1:]):
                midpoint=((a[0]+b[0])/2,(a[1]+b[1])/2)
                self.assertGreater(hypot(*midpoint)-.45,inner)
    def test_each_route_has_two_targets_per_tier(self):
        spots=XRoboconGame(None,None).spots
        for direction in ['clockwise','counterclockwise']:
            plans=tier_coin_plans(spots,direction)
            self.assertEqual([[s['tier'] for s in p] for p in plans],[[3,3],[2,2],[1,1]])
            self.assertEqual(len({s['id'] for p in plans for s in p}),6)
