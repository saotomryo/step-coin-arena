"""Tri-star mobility adapter: actuator targets from robot observations, no physics imports."""
from dataclasses import replace
from math import hypot,isfinite,pi

class ClusterMobilityAgent:
    def __init__(self,agent,tiers=(4.65,3.25,1.85),carrier_rate=.30,trigger_distance=.42,stall_seconds=.35,wheel_speed=.04,max_cycles=4,tracking_window=.45,clearance_distance=.75,heights=(.1,.35,.6)):
        if len(tiers)!=3 or any(not isfinite(v) or v<=0 for v in tiers):raise ValueError('three positive tier radii required')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) or v<=0 for v in (carrier_rate,trigger_distance,stall_seconds,wheel_speed,tracking_window,clearance_distance)):raise ValueError('positive finite mobility limits required')
        if type(max_cycles) is not int or not 1<=max_cycles<=8:raise ValueError('max_cycles must be 1..8')
        if len(heights)!=3 or any(not isfinite(v) or v<=0 for v in heights):raise ValueError("three positive heights required")
        self.heights=tuple(heights);self.clearance_distance=clearance_distance
        self.tracking_window=tracking_window
        self.agent=agent;self.tiers=tuple(tiers);self.rate=carrier_rate;self.trigger_distance=trigger_distance;self.stall_seconds=stall_seconds;self.wheel_speed=wheel_speed;self.max_cycles=max_cycles;self.reset()
    def __getattr__(self,name):return getattr(self.agent,name)
    @property
    def done(self):return self.agent.done and not self.rotating
    def reset(self):
        self.agent.reset();self.frame_target=0.;self.goal=0.;self.rotating=False;self.stall=0.;self.settled=0.;self.cycles={};self.mobility_events=[];self.progress_origin=None;self.progress_stage=None
    def act(self,observation,dt):
        if not isfinite(dt) or dt<=0:raise ValueError('positive dt required')
        command=self.agent.act(observation,dt)
        samples=getattr(observation,'sensors',{})
        if 'joint_states' in samples:
            data=samples['joint_states'].data
            if len(data['name'])!=len(data['position']):raise ValueError('malformed JointState')
            positions=dict(zip(data['name'],data['position']))
            frames=[positions.get(side+'_tristar_joint',float('nan')) for side in ('left','right')]
        else:frames=observation.telemetry.get('frames',[])
        if len(frames)!=2 or not all(isfinite(v) for v in frames):raise ValueError('two carrier encoder readings required')
        if observation.competition.status!='running':return replace(command,linear=0,angular=0,climb_stage=None,frame_target=self.frame_target)
        if getattr(self.agent,'decision_pending',False) or getattr(self.agent,'decision_error',None):
            return replace(command,linear=0,angular=0,climb_stage=None,frame_target=self.frame_target)
        avoidance=getattr(self.agent,'avoidance',None)
        if avoidance is not None and avoidance.waiting:return replace(command,linear=0,angular=0,climb_stage=None,frame_target=self.frame_target)
        stage=command.climb_stage
        if not self.rotating and stage is not None:
            distance=hypot(observation.pose.x,observation.pose.y)-self.tiers[stage]
            near=-self.clearance_distance<distance<self.trigger_distance
            if not near or self.progress_stage!=stage:
                self.progress_origin=None;self.stall=0;self.progress_stage=stage
            if near:
                if self.progress_origin is None:self.progress_origin=distance
                if self.progress_origin-distance>.015:self.progress_origin=distance;self.stall=0
                else:self.stall+=dt
            stalled=near and self.stall>=self.stall_seconds
            if distance<0 and abs(observation.telemetry.get('ground_z',self.heights[stage])-self.heights[stage])>.03:
                command=replace(command,linear=max(command.linear,self.wheel_speed),angular=0)
            if stalled and self.cycles.get(stage,0)<self.max_cycles:
                self.goal=self.frame_target+2*pi/3;self.rotating=True;self.stall=0;self.settled=0;self.cycles[stage]=self.cycles.get(stage,0)+1
                self.mobility_events.append(dict(stage=stage,goal=self.goal,cycle=self.cycles[stage]))
        if self.rotating:
            self.frame_target=max(self.frame_target,min(self.goal,self.frame_target+self.rate*dt,min(frames)+self.tracking_window))
            reached=abs(self.frame_target-self.goal)<1e-6 and max(abs(v-self.goal) for v in frames)<.10
            self.settled=self.settled+dt if reached else 0
            if self.settled>=.3:self.rotating=False;self.progress_origin=None;self.stall=0
            command=replace(command,linear=self.wheel_speed,angular=0)
        # Tri-star never enters the passive rocker-bogie force model.
        return replace(command,climb_stage=None,frame_target=self.frame_target)
