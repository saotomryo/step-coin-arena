"""Codex selects a coin; the unchanged radial/ring controller drives to it."""
from concurrent.futures import Future
from math import atan2
from threading import Event, Thread
import time
from xrobocon.adaptive_tier_agent import AdaptiveTierAgent
from xrobocon.coin_selector import CodexCoinSelector, selection_input, validate_choice
from xrobocon.navigation import VelocityCommand


class CodexTierAgent(AdaptiveTierAgent):
    def __init__(self, selector=None, game_rules=None, **kwargs):
        self.selector = CodexCoinSelector(**(selector or {}))
        self.game_rules = game_rules or {}
        super().__init__(**kwargs)

    def reset(self):
        if hasattr(self, '_cancel'): self._cancel.set()
        self._cancel = Event(); self._future = None; self._request = None
        self.target_id = None; self.decision_pending = False; self.decision_error = None
        self._decision = None; self._selection_signature = None
        super().reset()

    def _plan(self, obs, reason):
        signature = tuple((c['id'], c['status'], c.get('spawn_count', 0), c.get('owner'))
                          for c in obs.competition.coins)
        candidates = [c['id'] for c in obs.competition.coins
                      if c['status'] == 'available' and c['tier'] <= 3-self.stage]
        if self.target_id not in candidates: self.target_id = None
        # Only one request at a time. A changed board is coalesced until it returns.
        if signature != self._selection_signature and self._future is None:
            self._selection_signature = signature
            self.target_id = None; self.decision_error = None
            if candidates:
                payload = selection_input(obs, self.stage, self.motion, self.game_rules)
                self._start_request(payload)
        self.route = tuple(i for i in self.all_groups[self.stage] if i == self.target_id)
        self.preview_count = int(self.target_id is not None)
        self.groups[self.stage] = list(self.route or self.all_groups[self.stage]); self.index = 0
        if self.phase in ('coins', 'wait'):
            self.follower = None; self.reset_pending = True; self.angle = atan2(obs.pose.y, obs.pose.x)
        self.plan_event = dict(time=obs.timestamp, stage=self.stage, reason=reason,
            strategy='codex_target', model=getattr(self.selector,'model',None),
            reasoning_effort=getattr(self.selector,'reasoning_effort',None), coin_ids=list(self.route), target_id=self.target_id,
            decision_pending=self.decision_pending, decision_error=self.decision_error,
            excluded_ids=[c['id'] for c in obs.competition.coins if c['status'] != 'available'])
        if self._decision is not None:
            self.plan_event['decision'] = self._decision; self._decision = None
        if self.decision_pending and self._request is not None:
            self.plan_event['request'] = self._request
        self.replans.append(dict(self.plan_event)); self.needs_plan = False

    def _start_request(self, payload):
        future = Future(); cancel = self._cancel; selector = self.selector
        self._future = future; self._request = payload; self.decision_pending = True
        def infer():
            started = time.monotonic()
            try: future.set_result((selector.select(payload, cancel), time.monotonic()-started))
            except Exception as error: future.set_exception(error)
        self._thread = Thread(target=infer, name='coin-inference', daemon=True)
        self._thread.start()

    def _accept_result(self, result, observation):
        candidates = selection_input(observation, self.stage, self.motion)['movement_rules']['candidate_ids']
        self.target_id = validate_choice(result, candidates)

    def close(self):
        self._cancel.set()
        if hasattr(self, '_thread'): self._thread.join(timeout=3)

    def act(self, observation, dt):
        if observation.competition.status == 'finished':
            self._cancel.set()
            self.plan_event = None
            return VelocityCommand(0, 0)
        if self._future is not None and self._future.done():
            future = self._future; self._future = None; self.decision_pending = False
            try:
                result, latency = future.result()
                self._accept_result(result, observation)
                self._decision = dict(output=result, wall_seconds=latency, request=self._request,
                    model=getattr(self.selector,'model',None), reasoning_effort=getattr(self.selector,'reasoning_effort',None))
            except Exception as error:
                self.target_id = None; self.decision_error = str(error)
                self._decision = dict(error=self.decision_error, request=self._request)
                # Invalid/stale output can be retried once the board changes; no fake AI fallback.
            self.needs_plan = True
        command = super().act(observation, dt)
        if self.target_id is None or self.decision_pending or self.decision_error:
            # A deliberate stop does not keep a climb command active for stuck detection.
            return VelocityCommand(0, 0, reset_drive=True)
        return command
