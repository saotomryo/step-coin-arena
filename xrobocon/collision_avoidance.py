"""Agent-side predictive stop and local ring detour using public robot positions.

A disc approximation is deliberately conservative. No sideways detour is made
while climbing. This is a baseline, not a complete multi-robot motion planner.
"""
from math import atan2,cos,sin,hypot,isfinite,pi
from xrobocon.navigation import VelocityCommand,WaypointFollower
from xrobocon.coin_routes import ring_path

class CollisionAvoidance:
    def __init__(self,clearance=1.15,horizon=2.,pause=.6):
        if not all(isfinite(v) and v>0 for v in (clearance,horizon,pause)):
            raise ValueError('positive finite avoidance parameters required')
        self.clearance=clearance;self.horizon=horizon;self.pause=pause;self.reset()
    def reset(self):
        self.previous=None;self.velocities={};self.blocked_until=None;self.event=None
        self.waiting=False;self.last_reroute=-float('inf')
    def observe(self,obs):
        self.velocities={}
        if self.previous and obs.timestamp>self.previous[0]:
            elapsed=obs.timestamp-self.previous[0]
            for name,pos in obs.other_robots.items():
                old=self.previous[1].get(name)
                if old:self.velocities[name]=((pos[0]-old[0])/elapsed,(pos[1]-old[1])/elapsed)
        self.previous=(obs.timestamp,{k:tuple(v) for k,v in obs.other_robots.items()})
    def threat(self,obs,command):
        if abs(command.linear)<1e-6:return None
        vx=command.linear*cos(obs.pose.yaw);vy=command.linear*sin(obs.pose.yaw)
        for name,pos in obs.other_robots.items():
            x=pos[0]-obs.pose.x;y=pos[1]-obs.pose.y
            ov=self.velocities.get(name,(0,0));rx=ov[0]-vx;ry=ov[1]-vy
            closing=x*rx+y*ry
            if closing>=-1e-6:continue
            t=min(self.horizon,max(0,-closing/max(rx*rx+ry*ry,1e-12)))
            if hypot(x+rx*t,y+ry*t)<self.clearance:return name
        return None
    def reroute(self,agent,obs):
        """Choose the shortest clear arc to an available same-tier coin.

        Both arc directions are considered. Inside the clearance disc a path
        must move away monotonically until clear; this lets a yielding robot
        escape instead of repeatedly selecting a path through the other robot.
        """
        start=atan2(obs.pose.y,obs.pose.x);radius=hypot(obs.pose.x,obs.pose.y)
        choices=[];obstacles=list(obs.other_robots.values())
        for coin in obs.competition.coins:
            if coin['status']!='available' or coin['tier']!=3-agent.stage:continue
            end=atan2(coin['pos'][1],coin['pos'][0]);delta=atan2(sin(end-start),cos(end-start))
            for turn in (delta,delta-2*pi if delta>0 else delta+2*pi):
                if abs(turn)<.03:continue
                points=ring_path(start,start+turn,radius)
                safe=True
                for obstacle in obstacles:
                    initial=hypot(obs.pose.x-obstacle[0],obs.pose.y-obstacle[1])
                    distances=[hypot(x-obstacle[0],y-obstacle[1]) for x,y in points]
                    previous=initial
                    for distance in distances:
                        if distance<self.clearance and (initial>=self.clearance or distance<previous-.01):safe=False;break
                        previous=distance
                    if distances[-1]<self.clearance:safe=False
                    if not safe:break
                if safe:choices.append((abs(turn),coin['id'],start+turn,points))
        retreat=False
        if not choices:
            # No reachable coin: create room on this tier before replanning.
            available=[c for c in obs.competition.coins if c['status']=='available' and c['tier']==3-agent.stage]
            if not available:return False
            initial=min((hypot(obs.pose.x-p[0],obs.pose.y-p[1]) for p in obstacles),default=float('inf'))
            for turn in (-.5,.5):
                points=ring_path(start,start+turn,radius)
                distances=[min((hypot(x-p[0],y-p[1]) for p in obstacles),default=float('inf')) for x,y in points]
                if min(distances)>=initial-.01 and distances[-1]>initial+.1:
                    choices.append((-distances[-1],available[0]['id'],start+turn,points))
            if not choices:return False
            retreat=True
        _,target,end,points=min(choices,key=lambda v:v[0])
        agent.avoidance_retreat=retreat
        agent.route=(target,);agent.groups[agent.stage]=[target];agent.index=0;agent.angle=end
        agent.follower=WaypointFollower(points,max_speed=agent.motion.max_speed,max_turn=agent.motion.max_turn,acceleration=agent.motion.acceleration,turn_acceleration=agent.motion.turn_acceleration)
        agent.reset_pending=True
        agent.plan_event=dict(time=obs.timestamp,stage=agent.stage,reason='collision_retreat' if retreat else 'collision_avoidance',coin_ids=[target],excluded_ids=[c['id'] for c in obs.competition.coins if c['status']=='collected'],waypoints=points)
        agent.replans.append(dict(agent.plan_event));return True
    def filter(self,agent,obs,command):
        self.event=None;self.observe(obs)
        if obs.competition.status!='running':return command
        threat=self.threat(obs,command)
        if self.waiting:
            if obs.timestamp<self.blocked_until:return VelocityCommand(0,0,straighten=agent.phase=='climb',reset_drive=True)
            if agent.phase=='coins' and obs.timestamp-self.last_reroute>=self.pause:
                changed=self.reroute(agent,obs);self.last_reroute=obs.timestamp
                if changed:
                    self.waiting=False;self.event=dict(kind='reroute',time=obs.timestamp)
                    return VelocityCommand(0,0,reset_drive=True)
            if threat is None:
                self.waiting=False;self.event=dict(kind='resume',time=obs.timestamp)
            else:return VelocityCommand(0,0,straighten=agent.phase=='climb',reset_drive=True)
        if threat:
            self.waiting=True;self.blocked_until=obs.timestamp+self.pause
            self.event=dict(kind='stop',time=obs.timestamp,robot=threat,phase=agent.phase)
            return VelocityCommand(0,0,straighten=agent.phase=='climb',reset_drive=True)
        return command
