"""Wheel speed PID and MJCF-name-based differential drive adapter."""
from math import isfinite
import xml.etree.ElementTree as ET
from pathlib import Path
from xrobocon.navigation import Pose2D

class PID:
    """Derivative on measurement; conditional integration prevents windup."""
    def __init__(self, kp, ki, kd, limit, feedback_limit=None):
        if not all(isfinite(v) and v >= 0 for v in (kp, ki, kd)) or not isfinite(limit) or limit <= 0:
            raise ValueError('invalid PID gains/limit')
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.feedback_limit = limit if feedback_limit is None else feedback_limit
        if not isfinite(self.feedback_limit) or not 0 < self.feedback_limit <= limit:
            raise ValueError('feedback limit must be positive and within total torque limit')
        self.integral = 0.0
        self.previous_measurement = None

    def update(self, target, measurement, dt, feedforward=0.0):
        if not all(isfinite(v) for v in (target, measurement, dt, feedforward)) or dt <= 0:
            raise ValueError('finite values and positive dt required')
        error = target-measurement
        derivative = 0 if self.previous_measurement is None else (measurement-self.previous_measurement)/dt
        proposed = self.integral+error*dt
        feedback = self.kp*error+self.ki*proposed-self.kd*derivative
        raw = feedback+feedforward
        if (abs(raw) <= self.limit and abs(feedback) <= self.feedback_limit) or feedback*error < 0:
            self.integral = proposed
        self.previous_measurement = measurement
        feedback = max(-self.feedback_limit,min(self.feedback_limit,self.kp*error+self.ki*self.integral-self.kd*derivative))
        return max(-self.limit, min(self.limit, feedback+feedforward))

class GroundTruthPose:
    """Only for baseline tests. Replace with estimator without changing follower."""
    def __init__(self, robot): self.robot = robot
    def read_pose(self):
        from math import radians
        pos = self.robot.get_pos().cpu().numpy()
        return Pose2D(float(pos[0]), float(pos[1]), radians(float(self.robot.get_euler()[2])))

class DifferentialPIDDrive:
    """Read geometry from MJCF; resolve joints by name, never hard-coded DOF indices.

    Supports the five registered geometries. Rocker/Bogie suspension joints
    stay passive; wheel indices are resolved by name.
    """
    def __init__(self, robot, kp=0.3, ki=0.5, kd=0.0, torque_limit=1.0, frame_kp=20.0, frame_torque_limit=3.0, feedback_limit=None, steering_kp=12., steering_kd=.8, steering_torque_limit=2., frame_kd=2.,turn_gain=1.):
        from xrobocon.robot_configs import get_robot_config
        if robot.robot_type not in ('standard', 'tristar', 'tristar_large', 'rocker_bogie', 'rocker_bogie_large'):
            raise ValueError('unknown differential-drive robot')
        self.robot = robot
        asset_path = getattr(robot, 'asset_path', Path(__file__).parent/'assets'/get_robot_config(robot.robot_type)['xml_file'])
        root = ET.parse(asset_path).getroot()
        if not all(isfinite(v) and v > 0 for v in (frame_kp, frame_torque_limit)):
            raise ValueError('positive finite frame gains and limits required')
        if not all(isfinite(v) and v > 0 for v in (steering_kp,steering_torque_limit)) or not isfinite(steering_kd) or steering_kd < 0:
            raise ValueError("invalid steering gains/limit")
        self.steering_kp=steering_kp;self.steering_kd=steering_kd;self.steering_torque_limit=steering_torque_limit
        if not isfinite(frame_kd) or frame_kd<0:raise ValueError("invalid frame damping")
        if not isfinite(turn_gain) or turn_gain<=0:raise ValueError("positive turn gain required")
        self.turn_gain=turn_gain
        self.frame_kd=frame_kd
        self.frame_kp = frame_kp
        self.frame_torque_limit = frame_torque_limit
        self.frames = []
        self.frame_target = 0.0
        self.wheels_enabled = True
        if robot.robot_type == 'standard':
            names = ['left_wheel_joint', 'right_wheel_joint']
            left = root.find(".//body[@name='left_wheel']")
            right = root.find(".//body[@name='right_wheel']")
            self.track = abs(float(left.get('pos').split()[1])-float(right.get('pos').split()[1]))
        elif robot.robot_type in ('rocker_bogie', 'rocker_bogie_large'):
            names = [f'{side}_wheel_{position}_joint' for side in ('left','right')
                     for position in ('front','middle','rear')]
            left = root.find(".//body[@name='left_wheel_front']")
            right = root.find(".//body[@name='right_wheel_front']")
            offsets = {}
            def walk(element, y):
                for body in element.findall('body'):
                    absolute_y = y + float(body.get('pos','0 0 0').split()[1])
                    offsets[body.get('name')] = absolute_y
                    walk(body, absolute_y)
            walk(root.find('worldbody'), 0)
            self.track = abs(offsets['left_wheel_front']-offsets['right_wheel_front'])
        else:
            names = [f'{side}_wheel_{i}_joint'  for side in ('left', 'right') for i in (1,2,3)]
            left = root.find(".//body[@name='left_tristar']")
            right = root.find(".//body[@name='right_tristar']")
            self.track = abs(float(left.get('pos').split()[1])-float(right.get('pos').split()[1]))
            self.frames = [robot.entity.get_joint(f'{side}_tristar_joint').dofs_idx_local[0] for side in ('left', 'right')]
            left = left.find("body[@name='left_wheel_1']")
        self.radius = float(left.find('geom').get('size').split()[0])
        self.wheel_lateral_offsets = ([offsets[name.removesuffix('_joint')] for name in names]
                                      if robot.robot_type in ('rocker_bogie', 'rocker_bogie_large')
                                      else [self.track/2]*(len(names)//2)+[-self.track/2]*(len(names)//2))
        self.indices = [robot.entity.get_joint(n).dofs_idx_local[0] for n in names]
        self.controllers = [PID(kp, ki, kd, torque_limit, feedback_limit) for _ in names]
        self.targets = [0.0]*len(names)
        self.torques = [0.0]*len(names)
        self.torque_feedforward = [0.0]*len(names)
        self.steering = []
        self.wheel_longitudinal_offsets = []
        coordinates = {}
        def locate(element, x):
            for body in element.findall('body'):
                bx = x + float(body.get('pos','0 0 0').split()[0])
                coordinates[body.get('name')] = bx
                locate(body, bx)
        locate(root.find('worldbody'), 0)
        for name in names:
            steering_name = name.removesuffix('_joint')+'_steering_joint'
            if root.find(f".//joint[@name='{steering_name}']") is not None:
                self.steering.append(robot.entity.get_joint(steering_name).dofs_idx_local[0])
                self.wheel_longitudinal_offsets.append(coordinates[name.removesuffix('_joint')])
        if self.steering and len(self.steering) != len(names):
            raise ValueError('all wheels must have steering joints')
        self.steering_targets = [0.0]*len(self.steering)

    def apply_velocity(self, command, dt):
        import genesis as gs
        import torch
        velocity = self.robot.entity.get_dofs_velocity().cpu().numpy()
        position = self.robot.entity.get_dofs_position().cpu().numpy()
        force = torch.zeros(self.robot.n_dofs, device=gs.device)
        self.targets = self.velocity_targets(command)
        for i, index in enumerate(self.indices):
            self.torques[i] = self.controllers[i].update(self.targets[i], float(velocity[index]), dt, self.torque_feedforward[i]) if self.wheels_enabled else 0.0
            force[index] = self.torques[i]
        for index in self.frames:
            force[index] = max(-self.frame_torque_limit, min(self.frame_torque_limit, self.frame_kp*(self.frame_target-float(position[index]))-self.frame_kd*float(velocity[index])))
        for index, target in zip(self.steering, self.steering_targets):
            force[index] = max(-self.steering_torque_limit, min(self.steering_torque_limit, self.steering_kp*(target-float(position[index]))-self.steering_kd*float(velocity[index])))
        self.robot.entity.control_dofs_force(force)

    def velocity_targets(self, command):
        """Wheel angular speeds for each wheel's individual lateral position."""
        from dataclasses import replace
        command=replace(command,angular=command.angular*self.turn_gain)
        if self.steering:
            from math import atan2, hypot, pi
            targets = []
            for i, (x,y) in enumerate(zip(self.wheel_longitudinal_offsets,self.wheel_lateral_offsets)):
                vx, vy = command.linear-command.angular*y, command.angular*x
                if hypot(vx,vy) < 1e-6:
                    targets.append(0.0)
                    continue
                angle = atan2(vy,vx); sign = 1
                if angle > pi/2: angle -= pi; sign = -1
                elif angle < -pi/2: angle += pi; sign = -1
                self.steering_targets[i] = angle
                targets.append(sign*hypot(vx,vy)/self.radius)
            return targets
        return [(command.linear-command.angular*y)/self.radius for y in self.wheel_lateral_offsets]

    def set_frame_target(self, angle):
        """Shared left/right frame angle in radians, for synchronized step motion."""
        if not isfinite(angle):
            raise ValueError('frame target must be finite')
        self.frame_target = angle

    def stop(self):
        from xrobocon.navigation import VelocityCommand
        self.apply_velocity(VelocityCommand(0, 0), 0.01)
