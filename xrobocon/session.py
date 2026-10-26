"""Operator lifecycle separate from scoring, agents and the physics adapter.

Only the simulation thread calls these methods. Pause freezes simulation time.
"""
class CompetitionSession:
    def __init__(self,runner,on_finish=None):
        self.runner=runner;self.status='ready';self.round_id=0;self.on_finish=on_finish
    def _finish(self,reason):
        self.runner.manager.finish(reason);self.runner.simulator.stop();self.status='finished'
        if self.on_finish:self.on_finish(self.round_id,self.runner.manager.snapshot(),reason)
    def command(self,action):
        if action=='start' and self.status in ('ready','finished'):
            self.runner.start();self.round_id+=1;self.status='running'
        elif action=='pause' and self.status=='running':self.status='paused'
        elif action=='resume' and self.status=='paused':self.status='running'
        elif action=='stop' and self.status in ('running','paused'):self._finish('operator_stop')
        elif action=='reset':
            if self.status in ('running','paused'):self._finish('reset')
            self.runner.simulator.reset();self.runner.manager.reset()
            for agent in self.runner.agents.values():agent.reset()
            self.runner.reset_recovery();self.runner._sync_display();self.status='ready'
        else:raise ValueError(f'{action} is unavailable while {self.status}')
    def fail(self):
        self._finish('error');self.status='error'
    def tick(self):
        if self.status!='running':return None
        row=self.runner.tick();row['round_id']=self.round_id
        if self.runner.manager.status=='finished':self._finish('time_limit')
        return row
    def snapshot(self):
        state=self.runner.manager.snapshot()
        return dict(control_status=self.status,round_id=self.round_id,status=state['status'],elapsed=state['elapsed'],remaining=state['remaining'],scores=state['scores'],collection_counts=state['collection_counts'],
                    coins=[{k:c[k] for k in ('id','pos','tier','status','owner','spawn_count')} for c in state['coins']],events=state['events'][-50:],recovery_counts=state['recovery_counts'],recovery_blocked_seconds=state['recovery_blocked_seconds'])
