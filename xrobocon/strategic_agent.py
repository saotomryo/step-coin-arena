"""Execute a persistent AI coin sequence; replan only on machine-checked guards."""
from collections import deque
from copy import deepcopy
from math import atan2,hypot,sin,cos
from xrobocon.codex_tier_agent import CodexTierAgent
from xrobocon.coin_selector import selection_input
from xrobocon.strategic_plan import CodexPlanSelector,validate_plan,GUARDS


class StrategicTierAgent(CodexTierAgent):
    def __init__(self,selector=None,**kwargs):
        super().__init__(selector=selector,**kwargs)
        self.selector=CodexPlanSelector(**(selector or {}))

    def reset(self):
        super().reset()
        self.active_plan=None;self.remaining_coin_ids=[];self.strategy_label=None
        self.replan_reason='initial_or_recovery';self._force_replan=True;self._last_board=None
        self._plan_started=0.;self._remaining_at_accept=0.;self._gap_at_accept=0.
        self._generations={};self._progress_pose=None;self._blocked=0.;self._was_moving=False
        self._previous_plan=None;self._event_reason=None;self._recent_positions=deque(maxlen=6)

    def _gap(self,obs):
        scores=obs.competition.scores;participant=self.game_rules.get('participant','self')
        return scores.get(participant,0)-max((v for p,v in scores.items() if p!=participant),default=0)

    def _accept_result(self,result,obs):
        # Validate request and current snapshots: no stale plan may start driving.
        validate_plan(result,self._request['state']['coins'],self._request['movement_rules']['current_stage'])
        try: accepted=validate_plan(result,obs.competition.coins,self.stage)
        except ValueError:
            self._force_replan=True;self.replan_reason='stale_response';raise
        self.active_plan=accepted;self.strategy_label=accepted['strategy']
        self.remaining_coin_ids=list(accepted['coin_ids']);self.target_id=self.remaining_coin_ids[0]
        self._plan_started=obs.timestamp;self._remaining_at_accept=obs.competition.remaining
        self._gap_at_accept=self._gap(obs);self._blocked=0.;self._progress_pose=obs.pose;self._was_moving=False
        self._generations={c['id']:c.get('spawn_count',0) for c in obs.competition.coins}
        self._force_replan=False;self._event_reason='plan_accepted'

    def _invalidate(self,reason):
        self._previous_plan=deepcopy(self.active_plan)
        self.active_plan=None;self.remaining_coin_ids=[];self.target_id=None;self.strategy_label=None
        self._force_replan=True;self.replan_reason=reason;self._event_reason=reason;self.needs_plan=True
        self.follower=None;self.reset_pending=True

    def _monitor(self,obs,dt):
        if self.active_plan is None:return
        coins={c['id']:c for c in obs.competition.coins};participant=self.game_rules.get('participant','self')
        old=list(self.remaining_coin_ids)
        self.remaining_coin_ids=[i for i in old if not (coins.get(i,{}).get('status')=='collected' and coins[i].get('owner')==participant and coins[i].get('spawn_count',0)==self._generations[i])]
        if not self.remaining_coin_ids:self._invalidate('plan_completed');return
        if any(i not in coins or coins[i]['status']!='available' or coins[i].get('spawn_count',0)!=self._generations[i] for i in self.remaining_coin_ids):
            self._invalidate('target_coin_unavailable');return
        guards=self.active_plan['stop_conditions'];reason=None
        if obs.timestamp-self._plan_started>=guards['max_plan_seconds']:reason='max_plan_seconds'
        elif self._remaining_at_accept>guards['min_remaining_seconds']>=obs.competition.remaining:reason='min_remaining_seconds'
        elif guards['score_gap_change'] is not None and abs(self._gap(obs)-self._gap_at_accept)>=guards['score_gap_change']:reason='score_gap_change'
        if self._progress_pose is None:self._progress_pose=obs.pose
        moved=hypot(obs.pose.x-self._progress_pose.x,obs.pose.y-self._progress_pose.y)>=.05
        turned=abs(atan2(sin(obs.pose.yaw-self._progress_pose.yaw),cos(obs.pose.yaw-self._progress_pose.yaw)))>=.15
        if moved or turned:self._blocked=0.;self._progress_pose=obs.pose
        elif self._was_moving or (self.avoidance is not None and self.avoidance.waiting):self._blocked+=dt
        else:self._blocked=0.
        if reason is None and self._blocked>=guards['max_blocked_seconds']:reason='max_blocked_seconds'
        if reason:self._invalidate(reason);return
        self.target_id=self.remaining_coin_ids[0]
        if old!=self.remaining_coin_ids:self.needs_plan=True;self._event_reason='plan_progress'

    def _plan(self,obs,reason):
        board=tuple((c['id'],c['status'],c.get('owner'),c.get('spawn_count',0)) for c in obs.competition.coins)
        available=[c for c in obs.competition.coins if c['status']=='available' and c['tier']<=3-self.stage]
        # Failures retry only on a new board; own acquisitions in a valid plan do not infer.
        retry=self.active_plan is None and self.decision_error and board!=self._last_board
        if self._future is None and available and (self._force_replan or retry):
            payload=selection_input(obs,self.stage,self.motion,self.game_rules)
            payload['state']['recent_robot_positions']=list(self._recent_positions)
            payload['allowed_stop_conditions']=deepcopy(GUARDS)
            payload['planning_context']=dict(reason=self.replan_reason,previous_plan=self._previous_plan,
                current_plan=self.active_plan,remaining_coin_ids=list(self.remaining_coin_ids))
            self.decision_error=None;self._force_replan=False;self._last_board=board
            self._start_request(payload)
        elif not available and self.active_plan is None:self._force_replan=True
        target=self.target_id
        self.route=(target,) if target in self.all_groups[self.stage] else ()
        self.preview_count=int(target is not None)
        self.groups[self.stage]=list(self.route or self.all_groups[self.stage]);self.index=0
        if self.phase in ('coins','wait'):
            self.follower=None;self.reset_pending=True;self.angle=atan2(obs.pose.y,obs.pose.x)
        self.plan_event=dict(time=obs.timestamp,stage=self.stage,strategy='codex_plan',
            reason=self._event_reason or reason,replan_reason=self.replan_reason,strategy_label=self.strategy_label,
            coin_ids=list(self.remaining_coin_ids),target_id=self.target_id,
            stop_conditions=deepcopy(self.active_plan['stop_conditions']) if self.active_plan else None,
            decision_pending=self.decision_pending,decision_error=self.decision_error,
            model=getattr(self.selector,'model',None),reasoning_effort=getattr(self.selector,'reasoning_effort',None))
        if self._decision is not None:self.plan_event['decision']=self._decision;self._decision=None
        if self.decision_pending:self.plan_event['request']=self._request
        self.replans.append(deepcopy(self.plan_event));self.needs_plan=False;self._event_reason=None

    def act(self,observation,dt):
        if observation.competition.status=='running':
            if not self._recent_positions or observation.timestamp-self._recent_positions[-1]['timestamp']>=2:
                self._recent_positions.append(dict(timestamp=observation.timestamp,
                    own_pose=dict(x=observation.pose.x,y=observation.pose.y,yaw=observation.pose.yaw),
                    other_robots=deepcopy(observation.other_robots)))
            self._monitor(observation,dt)
            if self.active_plan is None and self._future is None and self._force_replan and any(
                c['status']=='available' and c['tier']<=3-self.stage for c in observation.competition.coins):self.needs_plan=True
        command=super().act(observation,dt)
        self._was_moving=not self.decision_pending and (abs(command.linear)>=.005 or abs(command.angular)>=.01)
        return command
