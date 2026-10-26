"""Finite host-side diagnostic/repair state machine, advanced by the UI timer."""
import time
from datetime import datetime,timezone
from copy import deepcopy
from pathlib import Path
from contextlib import contextmanager
from xrobocon.workshop.design import ensure_storage,write_json


def report_passed(report,suite):
    if suite=='joints' or not isinstance(report,dict) or report.get('error'):return False
    rows=report.get('results',[])
    return bool(rows) and all(r.get('complete') is True and r.get('passed') is True and not r.get('error')
        and r.get('status')!='smoke_only_performance_unrated' for r in rows)


class ImprovementLoop:
    def __init__(self,controller,limit,suite,prompt='',stop_on_pass=True,stage_timeout=600,allow_robot_changes=True):
        if type(limit) is not int or not 1<=limit<=30:raise ValueError('改善回数は1〜30回で指定してください')
        if suite not in ('flat','steps','all','joints'):raise ValueError('unknown diagnostic suite')
        if type(stage_timeout) not in (int,float) or not 1<=stage_timeout<=3600:raise ValueError('工程の制限時間は1〜3600秒です')
        self.controller=controller;self.active=True;self.stage_timeout=stage_timeout
        self.accepted=None;self.accepted_result=None;self.accepted_directory=None
        self.directory=ensure_storage(controller.library.root)/'jobs'/f'auto-{time.time_ns()}'
        self.directory.mkdir(parents=True)
        self.state=dict(status='running',phase='improve' if controller.rejected_pending else 'test',
            limit=limit,suite=suite,prompt=prompt,initial_prompt=prompt,prompt_version=1,prompt_history=[dict(version=1,prompt=prompt,changed_at=self.now(),reason='initial')],allow_robot_changes=bool(allow_robot_changes),creation_prompt=(controller.rejected_pending or {}).get('creation_prompt') if controller.rejected_pending and controller.rejected_pending['mode']=='new_robot' else (controller.current or {}).get('creation_prompt'),stop_on_pass=bool(stop_on_pass),improvement_attempts=0,history=[],
            inference_settings=dict(controller.inference_settings),starting_revision=(controller.current or {}).get('revision_id'),
            robot_id=None if controller.rejected_pending and controller.rejected_pending['mode']=='new_robot' else (controller.current or {}).get('robot_id'),stage_timeout=stage_timeout)
        self.stage_started=time.monotonic();self._save()

    @staticmethod
    def now():return datetime.now(timezone.utc).isoformat()

    def _save(self):
        target=self.directory/'loop.json';temporary=target.with_suffix('.tmp')
        write_json(temporary,self.state);temporary.replace(target)

    @contextmanager
    def internal(self):
        self.controller._automatic_call=True
        try:yield
        finally:self.controller._automatic_call=False

    def finish(self,reason,error=None):
        self.active=False;self.state.update(status='finished',reason=reason,error=error,
            final_revision=(self.controller.current or {}).get('revision_id'))
        self._save();self.controller.message=f'自動改善終了: {reason} / 改善 {self.state["improvement_attempts"]}/{self.state["limit"]}回 / {self.directory}'

    def cancel(self):
        self.finish('cancelled')
        self.controller.cancel()

    def transition(self,phase):
        self.state['phase']=phase;self.stage_started=time.monotonic();self._save()

    def select_tested_revision(self):
        from xrobocon.workshop.selection import regression
        from xrobocon.workshop.diagnostics import bounded
        c=self.controller;candidate=(c.current or {}).get('revision_id')
        restore,reason=regression(self.accepted_result,c.last_result,self.state['suite']) if self.accepted else (False,'initial baseline')
        event=dict(kind='selection',at=self.now(),candidate_revision=candidate,
            previous_accepted_revision=(self.accepted or {}).get('revision_id'),
            action='rollback' if restore else 'continue',reason=reason,
            candidate_result=bounded(c.last_result,5000),candidate_summary=(c.current or {}).get('proposal',{}).get('summary','')[:1500])
        if restore:
            c.current=deepcopy(self.accepted);c.directory=self.accepted_directory;c.saved=True;c.proposal=None
            c.last_result=deepcopy(self.accepted_result);c.pending_basis=None;c.rejected_pending=None
            c.require_constraint_review=False;c.pending_creation_prompt=None
        else:
            self.accepted=deepcopy(c.current);self.accepted_result=deepcopy(c.last_result);self.accepted_directory=c.directory
        event['accepted_revision']=(c.current or {}).get('revision_id')
        self.state['accepted_revision']=event['accepted_revision'];self.state['history'].append(event)

    def tick(self):
        if not self.active:return
        if time.monotonic()-self.stage_started>self.stage_timeout:
            self.finish('timeout');self.controller.cancel();return
        if self.controller.working:return
        c=self.controller
        try:
            with self.internal():
                phase=self.state['phase']
                if phase=='test':
                    c.diagnostic(self.state['suite']);self.transition('wait_test')
                elif phase=='wait_test':
                    self.state['history'].append(dict(kind='diagnostic',revision=(c.current or {}).get('revision_id'),
                        result=c.last_result,job=str(c.job['root']) if c.job else None))
                    self.select_tested_revision()
                    if self.state['stop_on_pass'] and report_passed(c.last_result,self.state['suite']):self.finish('passed')
                    elif self.state['improvement_attempts']>=self.state['limit']:self.finish('limit_reached')
                    else:self.transition('improve')
                elif phase=='improve':
                    if self.state['improvement_attempts']>=self.state['limit']:
                        self.finish('limit_reached');return
                    self.state['improvement_attempts']+=1;self._save()
                    c.improve_from_logs(self.state['prompt'],allow_robot_changes=self.state['allow_robot_changes'],preserve_creation_constraints=True)
                    self.state['history'].append(dict(kind='prompt_used',at=self.now(),attempt=self.state['improvement_attempts'],
                        prompt_version=self.state['prompt_version'],prompt=self.state['prompt'],effective_prompt=c.prompt))
                    self.transition('wait_improve')
                elif phase=='wait_improve':
                    if c.proposal is None:
                        if c.rejected_pending:
                            self.state['history'].append(dict(kind='rejected',attempt=c.rejected_pending['file'],inference=getattr(c,'inference',None)))
                            self.transition('improve');return
                        self.state['history'].append(dict(kind='inference_error',error=c.message,inference=getattr(c,'inference',None)))
                        self.finish('inference_error',c.message);return
                    before_xml=(c.current or {}).get('xml');before_control=(c.current or {}).get('controller')
                    try:c.apply()
                    except ValueError as error:
                        self.state['history'].append(dict(kind='rejected',error=str(error),attempt=(c.rejected_pending or {}).get('file')))
                        self.transition('improve');return
                    used_version=self.state['prompt_version']
                    c.current['improvement_prompt_usage']=dict(loop=str(self.directory),attempt=self.state['improvement_attempts'],
                        version=used_version,prompt=self.state['prompt'],effective_prompt=c.prompt,used_at=self.state['history'][-1].get('at'))
                    c.save()
                    identity=c.current.get('robot_id')
                    if c.current.get('creation_prompt')!=self.state['creation_prompt']:raise ValueError('自動改善中に元の作成プロンプトが変化しました')
                    if self.state['robot_id'] is not None and identity!=self.state['robot_id']:raise ValueError('自動改善中に機体IDが変化しました')
                    self.state['robot_id']=identity
                    self.state['history'].append(dict(kind='revision',revision=c.current['revision_id'],at=self.now(),prompt_version=used_version,
                        basis=c.current.get('improvement_basis'),inference=getattr(c,'inference',None),
                        model_changed=before_xml!=c.current.get('xml'),control_changed=before_control!=c.current.get('controller'),constraint_review=c.current.get('constraint_review')))
                    self.transition('test')
        except Exception as error:self.finish('error',str(error))
