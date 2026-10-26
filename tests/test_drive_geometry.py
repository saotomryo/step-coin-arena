import unittest
from types import SimpleNamespace
from xrobocon.pid_drive import DifferentialPIDDrive

class DriveGeometryTests(unittest.TestCase):
    def test_steering_matches_rigid_body_velocity(self):
        from math import cos, sin
        from xrobocon.scenario import load_scenario
        from xrobocon.navigation import VelocityCommand
        spec=load_scenario('configs/scenarios/passive_front_bogie_r18_steering_flat.json')
        robot=SimpleNamespace(robot_type=spec['robot_type'],asset_path=spec['robot_asset'],
                              entity=SimpleNamespace(get_joint=lambda name:SimpleNamespace(dofs_idx_local=[0])))
        drive=DifferentialPIDDrive(robot)
        for command in (VelocityCommand(0,.3), VelocityCommand(.12,.3),VelocityCommand(-.12,.3)):
            speeds=drive.velocity_targets(command)
            for x,y,angle,speed in zip(drive.wheel_longitudinal_offsets,drive.wheel_lateral_offsets,drive.steering_targets,speeds):
                self.assertAlmostEqual(speed*drive.radius*cos(angle),command.linear-command.angular*y)
                self.assertAlmostEqual(speed*drive.radius*sin(angle),command.angular*x)

    def test_registered_models_resolve_named_joints(self):
        expected = {'tristar': (.06,.28,6,2), 'standard': (.025,.22,2,0),
                    'rocker_bogie': (.06,.30,6,0), 'rocker_bogie_large': (.09,.45,6,0)}
        for kind,(radius,track,wheels,frames) in expected.items():
            calls=[]
            def joint(name):
                calls.append(name)
                return SimpleNamespace(dofs_idx_local=[len(calls)+5])
            robot=SimpleNamespace(robot_type=kind,entity=SimpleNamespace(get_joint=joint))
            drive=DifferentialPIDDrive(robot)
            self.assertAlmostEqual(drive.radius,radius)
            self.assertAlmostEqual(drive.track,track)
            self.assertEqual(len(drive.indices),wheels)
            self.assertEqual(len(drive.frames),frames)
            self.assertEqual(len(set(calls)),len(calls))

    def test_saved_rigid_profile_uses_actual_asset(self):
        from xrobocon.scenario import load_scenario
        specification=load_scenario('configs/scenarios/rigid_sixwheel_flat.json')
        calls=[]
        def joint(name):
            calls.append(name)
            return SimpleNamespace(dofs_idx_local=[len(calls)+5])
        robot=SimpleNamespace(robot_type=specification['robot_type'],asset_path=specification['robot_asset'],entity=SimpleNamespace(get_joint=joint))
        drive=DifferentialPIDDrive(robot)
        self.assertAlmostEqual(drive.radius,.08)
        self.assertAlmostEqual(drive.track,.30)
        self.assertEqual(len(drive.indices),6)

    def test_staggered_wheels_turn_with_individual_radii(self):
        from xrobocon.scenario import load_scenario
        from xrobocon.navigation import VelocityCommand
        spec=load_scenario('configs/scenarios/passive_front_bogie_r18_flat.json')
        robot=SimpleNamespace(robot_type=spec['robot_type'],asset_path=spec['robot_asset'],
                              entity=SimpleNamespace(get_joint=lambda name:SimpleNamespace(dofs_idx_local=[0])))
        drive=DifferentialPIDDrive(robot)
        targets=drive.velocity_targets(VelocityCommand(0,1))
        self.assertAlmostEqual(targets[0],-.24/.18)
        self.assertAlmostEqual(targets[1],-.14/.18)
        for left,right in zip(targets[:3],targets[3:]):self.assertAlmostEqual(left,-right)
