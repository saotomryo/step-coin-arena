import unittest
from types import SimpleNamespace
import numpy as np
from xrobocon.robot_markers import RobotMarkers,COLORS
class RobotMarkersTests(unittest.TestCase):
    def test_markers_follow_each_robot_without_recreating_nodes(self):
        positions={'self':[1,2,3],'opponent':[4,5,6]};draws=[];updates=[]
        robots={name:SimpleNamespace(get_pos=lambda name=name:SimpleNamespace(cpu=lambda:SimpleNamespace(tolist=lambda:positions[name]))) for name in positions}
        scene=SimpleNamespace(draw_debug_sphere=lambda pos,**kw:draws.append((pos,kw)) or len(draws),update_debug_objects=lambda nodes,poses:updates.append((nodes,poses)))
        markers=RobotMarkers(scene,robots);positions['self']=[2,3,4];markers.sync();markers.sync()
        self.assertEqual(len(draws),2);self.assertEqual(draws[0][1]['color'],COLORS['self']);self.assertEqual(draws[1][1]['color'],COLORS['opponent'])
        np.testing.assert_allclose(updates[-1][1][0][:3,3],[2,3,4.28]);np.testing.assert_allclose(updates[-1][1][1][:3,3],[4,5,6.28])
