import json
import tempfile
import unittest
from pathlib import Path
from math import pi
from xrobocon.navigation import Pose2D, WaypointFollower
from xrobocon.scenario import load_scenario

class NavigationTests(unittest.TestCase):
    def test_speed_acceleration_and_turn(self):
        f = WaypointFollower([(2, 0)])
        command = f.update(Pose2D(0, 0, 0), 0.1)
        self.assertAlmostEqual(command.linear, 0.03)
        for _ in range(100):
            command = f.update(Pose2D(0, 0, 0), 0.1)
        self.assertAlmostEqual(command.linear, 0.25)
        f = WaypointFollower([(0, 2)])
        command = f.update(Pose2D(0, 0, 0), 0.1)
        self.assertAlmostEqual(command.linear, 0)
        self.assertAlmostEqual(command.angular, 0.1)

    def test_pass_through_and_final_stop(self):
        f = WaypointFollower([(0, 0), (1, 0)])
        self.assertGreater(f.update(Pose2D(0, 0, 0), 0.1).linear, 0)
        self.assertEqual(f.index, 1)
        self.assertEqual(f.update(Pose2D(1, 0, 0), 0.1).linear, 0)
        self.assertTrue(f.done)

    def test_wrapped_heading_and_invalid_input(self):
        f = WaypointFollower([(-2, 0)])
        self.assertAlmostEqual(f.update(Pose2D(0, 0, -pi), 0.1).angular, 0)
        with self.assertRaises(ValueError): f.update(Pose2D(0, 0, 0), 0)
        with self.assertRaises(ValueError): WaypointFollower([])
        with self.assertRaises(ValueError): WaypointFollower([(float('nan'), 0)])

    def test_field_and_coin_reset_use_specification(self):
        from xrobocon.field import XRoboconField
        from xrobocon.game import XRoboconGame
        field = XRoboconField({'tiers': [{'radius': 1, 'height': 0.2},
                                         {'radius': 2, 'height': 0.1}], 'friction': 0.4})
        self.assertEqual(field.get_terrain_height(0, 0), 0.2)
        self.assertEqual(field.get_terrain_height(1.5, 0), 0.1)
        self.assertEqual(field.get_terrain_height(3, 0), 0)
        flat = XRoboconField({'tiers': [], 'friction': 0.8})
        self.assertEqual(flat.get_terrain_height(0, 0), 0)
        specification = {'rules': {'time_limit': 30, 'coin_radius': 0.1,
                                  'upper_tier_stay_time': 1},
                         'coins': [{'pos': [1.5, 0, 0.105], 'tier': 2}]}
        game = XRoboconGame(field, None, specification)
        game.spots[0]['collected'] = True
        game.start()
        self.assertFalse(game.spots[0]['collected'])
        self.assertEqual(game.time_limit, 30)
        self.assertEqual(len(game.spots), 1)

    def test_scenario_validation(self):
        data = load_scenario('configs/scenarios/flat_baseline.json')
        self.assertEqual(data['field']['tiers'], [])
        for changes in ({'robot_type': 'unknown'}, {'schema_version': 99},
                        {'coins': [{'tier': 1, 'pos': [0, 0, 0]}]}):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'invalid.json'
                path.write_text(json.dumps(dict(data, **changes)))
                with self.assertRaises(ValueError): load_scenario(path)

if __name__ == '__main__': unittest.main()
