import unittest
from xrobocon.navigation import Pose2D
from xrobocon.route_controller import MixedRouteController
class RouteTests(unittest.TestCase):
    def test_alignment_and_height_gate(self):
        c=MixedRouteController(.15);c.phase='align'
        command=c.update(Pose2D(-.55,0,.3),.15,0,0,.01)
        self.assertEqual(command.linear,0);self.assertEqual(c.phase,'align')
        c.update(Pose2D(-.55,0,0),.15,0,0,.01);self.assertEqual(c.phase,'cross')
        c.update(Pose2D(.45,0,0),.15,0,0,.01)
        self.assertEqual(c.phase,'cross');self.assertFalse(c.on_top_verified)
        c.update(Pose2D(.45,0,0),.20,0,0,.01)
        self.assertEqual(c.phase,'platform');self.assertTrue(c.on_top_verified)
    def test_abort_is_sticky(self):
        c=MixedRouteController(.15)
        self.assertEqual(c.update(Pose2D(0,0,0),.15,0,46,.01).linear,0)
        self.assertEqual(c.phase,'abort_posture')
        c.update(Pose2D(0,0,0),.15,0,0,.01);self.assertEqual(c.phase,'abort_posture')
