"""Fixed radial ascent and ring routes, independent of simulator and scoring."""
from dataclasses import replace
from math import atan2,cos,sin,hypot,pi,isfinite
from xrobocon.navigation import WaypointFollower,VelocityCommand
from xrobocon.coin_routes import ring_path

class FixedTierAgent:
    def __init__(self,tier_coin_ids,gate_angle=0,max_speed=.12,max_turn=.3,gate_angles=None):
        if len(tier_coin_ids)!=3 or any(not group for group in tier_coin_ids):raise ValueError('three nonempty tier target groups required')
        ids=[i for group in tier_coin_ids for i in group]
        if len(ids)!=len(set(ids)):raise ValueError('unique targets required')
        if not all(isfinite(v) and v>0 for v in (max_speed,max_turn)):raise ValueError('positive finite speed limits required')
        if not isfinite(gate_angle):raise ValueError('finite gate angle required')
        self.gate_angles=tuple(gate_angles if gate_angles is not None else [gate_angle]*3)
        if len(self.gate_angles)!=3 or not all(isfinite(v) for v in self.gate_angles):raise ValueError('three finite gate angles required')
        self.groups=tier_coin_ids;self.max_speed=max_speed;self.max_turn=max_turn;self.reset()
    def reset(self):
        self.stage=0;self.phase='climb';self.index=0;self.follower=None;self.stable=0;self.done=False;self.angle=self.gate_angle;self.checkpoints=[];self.reset_pending=True
    @property
    def gate_angle(self):return self.gate_angles[self.stage]
    def _point(self,r):return (r*cos(self.gate_angle),r*sin(self.gate_angle))
    def _transition(self,phase,observation):
        self.checkpoints.append(dict(phase=self.phase,stage=self.stage,time=observation.timestamp))
        self.phase=phase;self.follower=None;self.stable=0;self.reset_pending=True
    def act(self,observation,dt):
        if observation.competition.status!='running' or self.done:return VelocityCommand(0,0)
        coins={c['id']:c for c in observation.competition.coins}
        group=[coins[i] for i in self.groups[self.stage]]
        radius=hypot(*group[0]['pos'][:2]);height=group[0]['pos'][2]
        if any(c['tier']!=3-self.stage or abs(hypot(*c['pos'][:2])-radius)>.001 or abs(c['pos'][2]-height)>.001 for c in group):raise ValueError('targets must share the expected tier, height and ring radius')
        telemetry=observation.telemetry;speed=telemetry.get('speed',float('inf'))
        heading=atan2(-observation.pose.y,-observation.pose.x) if self.phase in ('align','straighten') else self.gate_angle+pi
        error=atan2(sin(heading-observation.pose.yaw),cos(heading-observation.pose.yaw))
        reset=self.reset_pending;self.reset_pending=False
        if self.phase in ('align','straighten'):
            ready=abs(error)<(.05 if self.phase=='straighten' else .03) and speed<.03
            if self.phase=='straighten':ready=ready and max([abs(v) for v in telemetry.get('steering',[float('inf')])])<.03
            self.stable=self.stable+1 if ready else 0
            if self.stable>=100:self._transition('straighten' if self.phase=='align' else 'climb',observation)
            return VelocityCommand(0,max(-.2,min(.2,error)) if self.phase=='align' else 0,straighten=self.phase=='straighten',reset_drive=reset)
        if self.phase=='coins':
            while self.index<len(group) and group[self.index]['status']=='collected':
                self.index+=1;self.follower=None;reset=True
            if self.index==len(group):
                if self.stage==2:self.done=True;self._transition('done',observation);return VelocityCommand(0,0,reset_drive=True)
                self._transition('return_gate',observation)
            else:
                coin=group[self.index]
                if coin['status']!='available':return VelocityCommand(0,0,reset_drive=reset)
                if self.follower is None:
                    end=atan2(coin['pos'][1],coin['pos'][0]);end=self.angle+atan2(sin(end-self.angle),cos(end-self.angle))
                    self.follower=WaypointFollower(ring_path(self.angle,end,radius),max_speed=self.max_speed,max_turn=self.max_turn);self.angle=end
                return replace(self.follower.update(observation.pose,dt),reset_drive=reset)
        if self.follower is None:
            next_gate=self.gate_angles[self.stage+1] if self.phase=='return_gate' else self.gate_angle
            end=self.angle+atan2(sin(next_gate-self.angle),cos(next_gate-self.angle))
            points=[self._point(radius)] if self.phase=='climb' else ring_path(self.angle,end,radius)
            self.follower=WaypointFollower(points,max_speed=self.max_speed,max_turn=self.max_turn)
        command=self.follower.update(observation.pose,dt)
        ready=self.follower.done and speed<.05
        if self.phase=='climb':ready=ready and abs(telemetry.get('ground_z',float('inf'))-height)<.03
        self.stable=self.stable+1 if ready else 0
        if self.stable>=100:
            if self.phase=='climb':self.index=0;self.angle=self.gate_angle;self._transition('coins',observation)
            else:self._transition('align',observation);self.stage+=1;self.angle=self.gate_angle
        return replace(command,angular=0 if self.phase=='climb' else command.angular,climb_stage=self.stage if self.phase=='climb' else None,reset_drive=reset)
