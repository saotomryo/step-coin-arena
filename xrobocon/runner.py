"""Only orchestrates components; scoring belongs to CompetitionManager."""
from copy import deepcopy
from dataclasses import asdict
from math import isfinite
from typing import Protocol
from xrobocon.agent import Observation,PublicCompetition
from xrobocon.navigation import VelocityCommand
from xrobocon.recovery import StuckMonitor,monitoring_state

class Simulator(Protocol):
    def reset(self): ...
    def read_pose(self,participant): ...
    def truth_positions(self): ...
    def step(self,commands,dt): ...
    def speeds(self): ...
    def stop(self): ...

class CompetitionRunner:
    def __init__(self,simulator,manager,agents,dt=.01,recovery=None,transport=None):
        if not isfinite(dt) or dt<=0:raise ValueError('positive finite dt required')
        if set(agents)!=set(manager.participants):raise ValueError('agents and participants must match')
        self.transport=transport
        self.simulator=simulator;self.manager=manager;self.agents=dict(agents);self.dt=dt
        options=dict(recovery or {});enabled=options.pop('enabled',False)
        if not isinstance(enabled,bool):raise ValueError('recovery enabled must be boolean')
        self.recovery=StuckMonitor(**options) if enabled else None
        if self.recovery and not hasattr(simulator,'respawn'):raise ValueError('recovery requires per-robot respawn')
    def reset_recovery(self):
        if self.recovery:self.recovery.reset()
    def start(self):
        self.simulator.reset();self.manager.start();self.reset_recovery()
        for agent in self.agents.values():agent.reset()
        if self.transport:self.transport.reset(self.agents)
        self._sync_display()
    def _sync_display(self):
        return self.simulator.sync_competition_display(self.manager.snapshot()) if hasattr(self.simulator,'sync_competition_display') else []
    def tick(self):
        state=self.manager.snapshot()
        if state['status']!='running':raise RuntimeError('competition is not running')
        # Whitelist public fields; owner/positions are intentionally public in
        # this baseline. Private dwell counters and event history are excluded.
        coins=tuple({k:deepcopy(c[k]) for k in ('id','pos','tier','status','owner','points','dwell','radius','height_tolerance','spawn_count')} for c in state['coins'])
        positions=deepcopy(self.simulator.truth_positions())
        commands={};observations={};inputs={}
        for participant,agent in self.agents.items():
            public=PublicCompetition(state['status'],state['remaining'],deepcopy(state['scores']),deepcopy(coins),deepcopy(state['collection_counts']))
            observation=Observation(self.simulator.read_pose(participant),state['elapsed'],'truth_baseline',public,
                self.simulator.read_telemetry(participant) if hasattr(self.simulator,'read_telemetry') else {},
                {p:deepcopy(pos) for p,pos in positions.items() if p!=participant},
                self.simulator.read_sensors(participant) if hasattr(self.simulator,'read_sensors') else {})
            inputs[participant]=observation
            observations[participant]=dict(pose=dict(x=observation.pose.x,y=observation.pose.y,yaw=observation.pose.yaw),timestamp=observation.timestamp,source=observation.source,other_robots=deepcopy(observation.other_robots),sensors={k:asdict(v) for k,v in observation.sensors.items()})
        if self.transport:
            try:
                commands,metadata=self.transport.exchange(inputs,self.dt)
                for participant,values in metadata.items():self.agents[participant].update(values)
            except BaseException:
                self.simulator.stop();raise
        else:
            for participant,agent in self.agents.items():commands[participant]=agent.act(inputs[participant],self.dt)
        self.simulator.step(commands,self.dt)
        self.manager.advance(self.dt,self.simulator.truth_positions())
        recovery_events=[]
        if self.recovery and self.manager.status=='running':
            positions=self.simulator.truth_positions()
            for participant,agent in self.agents.items():
                command=commands[participant];obs=inputs[participant]
                active,rotation_only=monitoring_state(getattr(agent,'phase',None),command,obs.telemetry.get('tilt_degrees',0))
                if (getattr(agent,'decision_pending',False) or getattr(agent,'decision_error',None)) and obs.telemetry.get('tilt_degrees',0)<=60:active=False
                seconds=self.recovery.update(participant,positions[participant],self.simulator.read_pose(participant).yaw,self.dt,active,rotation_only=rotation_only)
                if seconds is not None:
                    self.simulator.respawn(participant);agent.reset()
                    if self.transport:self.transport.reset([participant])
                    spawn=self.simulator.truth_positions()[participant]
                    self.manager.record_recovery(participant,seconds,positions[participant],spawn)
                    recovery_events.append(deepcopy(self.manager.events[-1]))
        display_changes=self._sync_display()
        if self.manager.status!='running':self.simulator.stop()
        return dict(transport=deepcopy(self.transport.last_exchange) if self.transport else {'kind':'direct_baseline'},avoidance_events={p:deepcopy(a.avoidance.event) for p,a in self.agents.items() if getattr(a,'avoidance',None) and a.avoidance.event},route_plans={p:deepcopy(a.plan_event) for p,a in self.agents.items() if getattr(a,'plan_event',None) is not None},recovery_events=recovery_events,recovery_counts=dict(self.manager.recovery_counts),display_changes=display_changes,time=self.manager.elapsed,observations=observations,commands={p:dict(linear=c.linear,angular=c.angular,climb_stage=c.climb_stage,straighten=c.straighten,reset_drive=c.reset_drive,frame_target=c.frame_target) for p,c in commands.items()},positions=self.simulator.truth_positions(),speeds=self.simulator.speeds(),scores=dict(self.manager.scores),agent_states={p:dict(phase=getattr(a,'phase','coins'),stage=getattr(a,'stage',None),done=getattr(a,'done',False),strategy_label=getattr(a,'strategy_label',None),remaining_coin_ids=getattr(a,'remaining_coin_ids',[]),replan_reason=getattr(a,'replan_reason',None),target_id=getattr(a,'target_id',None),decision_pending=getattr(a,'decision_pending',False),decision_error=getattr(a,'decision_error',None)) for p,a in self.agents.items()})

    def close(self):
        self.simulator.stop()
        if self.transport:self.transport.close()
        else:
            for agent in self.agents.values():
                if hasattr(agent,'close'):agent.close()
