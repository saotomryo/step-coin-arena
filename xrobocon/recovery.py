"""Simulation-time stuck detection, independent of physics, agents and scoring."""
from math import isfinite,sqrt,atan2,sin,cos
class StuckMonitor:
    def __init__(self,timeout=20,min_distance=.05,min_rotation=.15):
        for value in (timeout,min_distance,min_rotation):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not isfinite(value) or value<=0:raise ValueError('positive finite recovery thresholds required')
        self.timeout=timeout;self.min_distance=min_distance;self.min_rotation=min_rotation;self.reset()
    def reset(self,participant=None):
        if participant is None:self.samples={}
        else:self.samples.pop(participant,None)
    def update(self,participant,position,yaw,dt,active,rotation_only=False):
        if len(position)!=3 or not all(isfinite(v) for v in (*position,yaw,dt)) or dt<=0:raise ValueError('finite pose and positive dt required')
        if not active:self.reset(participant);return None
        sample=self.samples.setdefault(participant,dict(position=tuple(position),yaw=yaw,elapsed=0.))
        distance=sqrt(sum((a-b)**2 for a,b in zip(position,sample['position'])))
        rotation=abs(atan2(sin(yaw-sample['yaw']),cos(yaw-sample['yaw'])))
        if distance>=self.min_distance or rotation_only and rotation>=self.min_rotation:
            sample.update(position=tuple(position),yaw=yaw,elapsed=0.)
        else:sample['elapsed']+=dt
        if sample['elapsed']+1e-9>=self.timeout:
            seconds=sample['elapsed'];self.reset(participant);return seconds
        return None


def monitoring_state(phase,command,tilt_degrees):
    """Intentional waits are exempt; overturned bodies are always monitored."""
    fallen=tilt_degrees>60
    active=fallen or (phase not in ('align','straighten') and (abs(command.linear)>=.005 or abs(command.angular)>=.01 or phase=='climb'))
    return active,not fallen and abs(command.linear)<.005
