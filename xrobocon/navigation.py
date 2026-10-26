"""Simulator-independent navigation contracts and conservative waypoint control.

Pose providers supply estimates; simulator truth remains an evaluation concern.
Commands are body velocities, not legacy RL torque actions.
"""
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi
from typing import Protocol

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # radians

class PoseProvider(Protocol):
    def read_pose(self) -> Pose2D: ...

@dataclass(frozen=True)
class VelocityCommand:
    linear: float
    angular: float
    climb_stage: int | None = None
    straighten: bool = False
    reset_drive: bool = False
    frame_target: float | None = None

class Drive(Protocol):
    def apply_velocity(self, command: VelocityCommand, dt: float) -> None: ...
    def stop(self) -> None: ...

class WaypointFollower:
    """Distance/heading P control with acceleration limits; reset on each new route.

    Intermediate points pass through. Final goal commands zero inside tolerance.
    A wheel-feedback drive adapter must enforce physical speed/torque limits.
    """
    def __init__(self, points, max_speed=0.25, max_turn=0.6,
                 acceleration=0.3, turn_acceleration=1.0, tolerance=0.1):
        if not points or any(len(p) != 2 or not all(isfinite(v) for v in p) for p in points):
            raise ValueError('points must be nonempty finite XY pairs')
        limits = (max_speed, max_turn, acceleration, turn_acceleration, tolerance)
        if any(not isfinite(v) or v <= 0 for v in limits):
            raise ValueError('limits must be finite and positive')
        self.points = tuple(tuple(p) for p in points)
        self.max_speed, self.max_turn = max_speed, max_turn
        self.acceleration, self.turn_acceleration = acceleration, turn_acceleration
        self.tolerance = tolerance
        self.index = 0
        self.previous = VelocityCommand(0, 0)
        self.done = False

    def update(self, pose: Pose2D, dt: float) -> VelocityCommand:
        if not isfinite(dt) or dt <= 0 or not all(isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            raise ValueError('finite pose and positive dt required')
        while hypot(self.points[self.index][0]-pose.x, self.points[self.index][1]-pose.y) <= self.tolerance:
            if self.index == len(self.points)-1:
                self.done = True
                self.previous = VelocityCommand(0, 0)
                return self.previous
            self.index += 1
        if self.done:
            return VelocityCommand(0, 0)
        dx, dy = self.points[self.index][0]-pose.x, self.points[self.index][1]-pose.y
        error = (atan2(dy, dx)-pose.yaw+pi) % (2*pi)-pi
        linear = min(self.max_speed, 0.8*hypot(dx, dy)) * max(0, cos(error))
        angular = max(-self.max_turn, min(self.max_turn, 2*error))
        def limited(target, old, bound):
            return old + max(-bound*dt, min(bound*dt, target-old))
        self.previous = VelocityCommand(limited(linear, self.previous.linear, self.acceleration),
                                        limited(angular, self.previous.angular, self.turn_acceleration))
        return self.previous
