"""Agent baseline independent of physics and competition authority."""
from dataclasses import dataclass,field
from typing import Protocol
from xrobocon.sensors import SensorSample
from xrobocon.navigation import Pose2D,VelocityCommand,WaypointFollower

@dataclass(frozen=True)
class PublicCompetition:
    status: str
    remaining: float
    scores: dict
    coins: tuple
    collection_counts: dict = field(default_factory=dict)

@dataclass(frozen=True)
class Observation:
    pose: Pose2D
    timestamp: float
    source: str
    competition: PublicCompetition
    telemetry: dict = field(default_factory=dict)
    other_robots: dict = field(default_factory=dict)
    sensors: dict[str,SensorSample] = field(default_factory=dict)

class RobotAgent(Protocol):
    def reset(self): ...
    def act(self,observation: Observation,dt: float) -> VelocityCommand: ...

class FixedCoinAgent:
    """Fixed target order. Wait at goal until the authority marks it collected.

    Straight planar legs only; multi-tier planning is a separate extension.
    """
    def __init__(self,coin_ids,max_speed=.12,max_turn=.3,control_profile=None,motion_limits=None):
        if not coin_ids or len(set(coin_ids))!=len(coin_ids):raise ValueError('unique nonempty targets required')
        from xrobocon.motion_profiles import resolve_profile
        self.control_profile=control_profile or "baseline"
        self.motion=resolve_profile(control_profile,max_speed,max_turn,motion_limits)
        self.coin_ids=tuple(coin_ids);self.max_speed=self.motion.max_speed;self.max_turn=self.motion.max_turn;self.reset()
    def reset(self):
        self.index=0;self.follower=None;self.done=False
    def act(self,observation,dt):
        if observation.competition.status!='running' or self.done:return VelocityCommand(0,0)
        coins={c['id']:c for c in observation.competition.coins}
        while self.index<len(self.coin_ids):
            coin=coins.get(self.coin_ids[self.index])
            if coin is None:raise ValueError('target absent from public layout')
            if coin['status']=='collected':self.index+=1;self.follower=None;continue
            if coin['status']!='available':return VelocityCommand(0,0)
            if self.follower is None:self.follower=WaypointFollower([coin['pos'][:2]],max_speed=self.max_speed,max_turn=self.max_turn,acceleration=self.motion.acceleration,turn_acceleration=self.motion.turn_acceleration)
            return self.follower.update(observation.pose,dt)
        self.done=True;return VelocityCommand(0,0)
