"""Guarded step sequence using replaceable range, speed and posture observations.

Stall is inferred contact, not measured collision. No lidar implementation here.
"""
from dataclasses import dataclass
from math import isfinite, pi
from xrobocon.navigation import VelocityCommand

@dataclass(frozen=True)
class StepObservation:
    edge_distance: float
    heading_error: float  # radians, relative to step normal
    forward_speed: float
    pitch: float  # radians
    frame_left: float
    frame_right: float
    on_top: bool

class GuardedStepController:
    def __init__(self, climb_speed=0.03, frame_direction=1):
        if not isfinite(climb_speed) or not 0 <= climb_speed <= 0.1:
            raise ValueError('climb_speed must be in [0,0.1]')
        if frame_direction not in (-1, 1):
            raise ValueError('frame_direction must be -1 or 1')
        self.climb_speed = climb_speed
        self.frame_direction = frame_direction
        self.phase = 'approach'
        self.frame_target = 0.0
        self.stall_time = 0.0
        self.phase_time = 0.0
        self.contact_inferred = False

    def update(self, obs, dt):
        values = (obs.edge_distance, obs.heading_error, obs.forward_speed,
                  obs.pitch, obs.frame_left, obs.frame_right, dt)
        if not all(isfinite(v) for v in values) or dt <= 0:
            raise ValueError('finite observation and positive dt required')
        self.phase_time += dt
        if abs(obs.pitch) > pi/6:
            self.phase = 'abort_posture'
        if self.phase_time > (20 if self.phase == 'climb' else 10) and self.phase not in ('approach', 'recover'):
            self.phase = 'abort_timeout'
        if self.phase.startswith('abort'):
            # Avoid continuing a growing frame target after instability.
            self.frame_target = (obs.frame_left+obs.frame_right)/2
            return VelocityCommand(0, 0)
        turn = max(-0.3, min(0.3, 2*obs.heading_error))
        if self.phase == 'approach' and obs.edge_distance < 0.25:
            self.phase = 'align'; self.phase_time = 0
        if self.phase == 'align':
            if abs(obs.heading_error) < 0.05:
                self.phase = 'contact'; self.phase_time = 0
            else:
                return VelocityCommand(0,turn)
        if self.phase == 'contact':
            # Range gates stall detection so braking or distant obstacles do not trigger climbing.
            stalled = obs.edge_distance < 0.18 and abs(obs.forward_speed) < 0.01
            self.stall_time = self.stall_time+dt if stalled else 0
            if self.stall_time >= 0.5:
                self.contact_inferred = True
                self.phase = 'climb'; self.phase_time = 0
            return VelocityCommand(0.04,turn)
        if self.phase == 'climb':
            if obs.on_top:
                self.phase = 'recover'; self.phase_time = 0
                return VelocityCommand(0,0)
            tracking_error = max(abs(self.frame_target-obs.frame_left), abs(self.frame_target-obs.frame_right))
            if tracking_error < 0.12:
                self.frame_target = self.frame_direction*min(2*pi/3,abs(self.frame_target)+0.15*dt)
            return VelocityCommand(self.climb_speed,turn)
        if self.phase == 'recover':
            return VelocityCommand(0,0)
        return VelocityCommand(0.08,turn)
