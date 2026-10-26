"""Replaceable coin-coverage strategy built on the existing tier controller."""
from math import atan2,hypot
from xrobocon.tier_agent import FixedTierAgent
from xrobocon.navigation import VelocityCommand,WaypointFollower
from xrobocon.route_planner import UpwardCoveragePlanner,RoutePlanner
class AdaptiveTierAgent(FixedTierAgent):
    def __init__(self,max_speed=.12,max_turn=.3,control_profile=None,motion_limits=None,speed_fraction=.75,ascent_overhead=10,top_route='arc',avoidance=None):
        from xrobocon.collision_avoidance import CollisionAvoidance
        options=dict(avoidance or {});enabled=options.pop('enabled',False)
        if not isinstance(enabled,bool):raise ValueError('avoidance enabled must be boolean')
        self.avoidance=CollisionAvoidance(**options) if enabled else None
        self.top_route=top_route
        self.planner:RoutePlanner=UpwardCoveragePlanner(speed_fraction,ascent_overhead,top_route)
        super().__init__([[0],[1],[2]],max_speed=max_speed,max_turn=max_turn,control_profile=control_profile,motion_limits=motion_limits)
    def reset(self):
        self.avoidance_retreat=False
        if self.avoidance:self.avoidance.reset()
        super().reset();self.phase='align';self.initialized=False;self.signature=None;self.route=();self.replans=[];self.plan_event=None;self.needs_plan=True
    def _plan(self,obs,reason):
        coins={c['id']:c for c in obs.competition.coins};group=[coins[i] for i in self.all_groups[self.stage]]
        radius=hypot(*group[0]['pos'][:2]);start=atan2(obs.pose.y,obs.pose.x)
        approach=max(0,hypot(obs.pose.x,obs.pose.y)-radius)/self.motion.climb_speed if self.phase in ('align','straighten','climb') else 0
        preview=self.planner.plan([[coins[i] for i in ids] for ids in self.all_groups],self.stage,start,self.motion.max_speed,self.motion.climb_speed,max(0,obs.competition.remaining-approach))
        plan=preview.current
        self.preview_count=preview.total_count
        self.route=plan.coin_ids;self.groups[self.stage]=list(plan.coin_ids or self.all_groups[self.stage]);self.index=0
        if self.phase in ('coins','wait'):self.follower=None;self.reset_pending=True;self.angle=start
        self.plan_event=dict(time=obs.timestamp,stage=self.stage,reason=reason,coin_ids=list(plan.coin_ids),excluded_ids=[c['id'] for c in obs.competition.coins if c['status']=='collected'],estimated_seconds=plan.estimated_seconds,distance=plan.distance,expected_total_coins=preview.total_count,estimated_total_seconds=preview.estimated_seconds,tier_routes=[list(r.coin_ids) for r in preview.tier_routes])
        self.replans.append(dict(self.plan_event));self.needs_plan=False
    def act(self,observation,dt):
        command=self._act_route(observation,dt)
        return self.avoidance.filter(self,observation,command) if self.avoidance else command
    def _act_route(self,observation,dt):
        self.plan_event=None
        if self.avoidance_retreat and self.follower is not None and self.follower.done:
            self.needs_plan=True;self.avoidance_retreat=False
        if observation.competition.status!='running':return VelocityCommand(0,0)
        if not self.initialized:
            self.all_groups=[[c['id'] for c in observation.competition.coins if c['tier']==3-stage] for stage in range(3)]
            if any(not g for g in self.all_groups):raise ValueError('adaptive tier strategy requires three nonempty rings')
            for ids in self.all_groups:
                group=[c for c in observation.competition.coins if c['id'] in ids]
                radii=[hypot(*c['pos'][:2]) for c in group];heights=[c['pos'][2] for c in group]
                if min(radii)<=0 or max(radii)-min(radii)>.001 or max(heights)-min(heights)>.001:raise ValueError('strategy requires concentric constant-height coin rings')
            self.groups=[list(g) for g in self.all_groups]
            angle=atan2(observation.pose.y,observation.pose.x);self.gate_angles=(angle,angle,angle);self.angle=angle;self.initialized=True
        signature=(frozenset((c['id'],c['status'],c.get('owner'),c.get('spawn_count',0)) for c in observation.competition.coins),tuple(sorted(observation.competition.collection_counts.items())))
        if signature!=self.signature or self.needs_plan:
            old=self.signature
            previous_collected={v[0] for v in old[0] if v[1]=='collected'} if old else set()
            collected={c['id'] for c in observation.competition.coins if c['status']=='collected'}
            reason='initial' if old is None else 'coin_acquisition' if collected-previous_collected or signature[1]!=old[1] else 'spawn_changed' if signature!=old else 'entered_tier'
            self._plan(observation,reason);self.signature=signature
        if self.phase in ('coins','wait'):
            if not self.route:
                higher=any(c['status']=='available' and c['tier']<3-self.stage for c in observation.competition.coins)
                if self.stage<2 and higher and self.preview_count>0:
                    self._transition('align',observation);self.stage+=1
                    angle=atan2(observation.pose.y,observation.pose.x);angles=list(self.gate_angles);angles[self.stage]=angle;self.gate_angles=tuple(angles);self.angle=angle
                    self._plan(observation,'ascend_from_current_position')
                else:
                    self.phase='wait';self.done=False;return VelocityCommand(0,0)
            elif self.phase=='wait':self.phase='coins';self.follower=None;self.reset_pending=True
        if self.top_route=='direct' and self.phase=='coins' and self.stage==2 and self.route and self.follower is None:
            # The top has no inner step: a chord stays entirely on this tier.
            coin=next(c for c in observation.competition.coins if c['id']==self.route[0])
            self.follower=WaypointFollower([coin['pos'][:2]],max_speed=self.motion.max_speed,max_turn=self.motion.max_turn,acceleration=self.motion.acceleration,turn_acceleration=self.motion.turn_acceleration)
        previous=self.phase;command=super().act(observation,dt)
        if previous=='straighten' and self.phase=='climb':
            # Anchor the normal at the actual pose after alignment, not an old waypoint.
            angle=atan2(observation.pose.y,observation.pose.x);angles=list(self.gate_angles);angles[self.stage]=angle;self.gate_angles=tuple(angles);self.angle=angle;self.follower=None
        if previous=='climb' and self.phase=='coins':self.needs_plan=True
        return command
