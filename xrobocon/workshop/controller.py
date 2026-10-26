"""UI-independent workshop workflow and isolated native tool processes."""
from concurrent.futures import Future
from copy import deepcopy
from pathlib import Path
from threading import Event,Thread
import json,os,signal,subprocess,sys,tempfile,time
from xrobocon.match_setup import MatchCatalog
from xrobocon.workshop.design import source_snapshot,apply_proposal,WorkshopLibrary,ensure_storage,write_json
from xrobocon.workshop.inference import RobotDesignGenerator, NewRobotGenerator, RobotImprovementGenerator, ParameterImprovementGenerator, RejectedDesign
ROOT=Path(__file__).resolve().parents[2]

def absolute_catalog(catalog):
    data=deepcopy(catalog.data);data['base_config']=str((catalog.path.parent/data['base_config']).resolve())
    for robot in data['robots'].values():
        for key in ('robot_asset','visual_asset','tuning_file','controller_file'):
            if robot.get(key):robot[key]=str((catalog.path.parent/robot[key]).resolve())
    return data

def catalog_with_revision(catalog,snapshot,directory):
    data=absolute_catalog(catalog);directory=Path(directory).resolve();entry=deepcopy(snapshot['entry'])
    entry.pop('visual_asset',None);entry['robot_asset']=str(directory/'model.xml');entry['tuning_file']=str(directory/'tuning.json')
    if 'controller' in snapshot:
        entry['drive_kind']='program';entry['controller_file']=str(directory/'controller.json')
    entry['label']=snapshot.get('robot_name',snapshot['proposal']['title']);entry['description']='Workshop試作。診断結果を確認して利用してください。';entry['recommended_control']='baseline'
    from xrobocon.workshop.identity import catalog_id
    robot_id=catalog_id(snapshot,directory)
    if snapshot.get('robot_id'):entry.update(workshop_robot_id=snapshot['robot_id'],workshop_revision=directory.name,robot_name=snapshot['robot_name'])
    data['robots'][robot_id]=entry
    return data,robot_id

class WorkshopController:
    def __init__(self,catalog,root,generator=None):
        self.require_constraint_review=False;self.pending_creation_prompt=None
        self.automation=None;self._automatic_call=False
        self.rejected_pending=None
        self.pending_basis=None
        self.catalog=catalog;self.library=WorkshopLibrary(root);self.generator=generator or RobotDesignGenerator(timeout=120)
        self.new_generator=NewRobotGenerator(timeout=180)
        self.improvement_generator=RobotImprovementGenerator(timeout=180)
        self.parameter_improvement_generator=ParameterImprovementGenerator(timeout=180)
        self.inference_settings=dict(model='gpt-6-luna',reasoning_effort='low',timeout=180)
        settings=self.library.root/'inference-settings.json'
        self._set_inference(**(json.loads(settings.read_text()) if settings.is_file() else self.inference_settings))
        self.preview_job=None
        self.current=None;self.proposal=None;self.directory=None;self.saved=False;self.future=None;self.cancelled=Event()
        self.process=None;self.log=None;self.job=None;self.last_result=None;self.mode='robot';self.prompt='';self.message='機体を選び、プロンプトを入力してください。'
    @property
    def working(self):return self.future is not None or self.process is not None and self.process.poll() is None
    @property
    def busy(self):return self.working or bool(self.automation and self.automation.active)
    def _idle(self):
        if self.working or self.automation and self.automation.active and not self._automatic_call:raise ValueError('処理の終了または取消しを待ってください')
    def _set_inference(self,model,reasoning_effort,timeout):
        # Reuse backend validation; no network call or silent model substitution.
        checked=RobotDesignGenerator(model=model,reasoning_effort=reasoning_effort,timeout=timeout)
        self.inference_settings=dict(model=checked.model,reasoning_effort=checked.reasoning_effort,timeout=checked.timeout)
        for generator in (self.generator,self.new_generator,self.improvement_generator,self.parameter_improvement_generator):
            generator.model=checked.model;generator.reasoning_effort=checked.reasoning_effort;generator.timeout=checked.timeout

    def configure_inference(self,model,reasoning_effort='low',timeout=180):
        self._idle();self._set_inference(model,reasoning_effort,timeout)
        ensure_storage(self.library.root).mkdir(parents=True,exist_ok=True)
        path=self.library.root/'inference-settings.json';temporary=path.with_suffix('.tmp')
        write_json(temporary,self.inference_settings);temporary.replace(path)
        self.message=f'推論設定: {model} / {reasoning_effort} / {timeout}秒'

    def start_automation(self,limit,suite,prompt='',stop_on_pass=True,stage_timeout=600,allow_robot_changes=True):
        self._idle()
        if not self.rejected_pending:
            if self.current is None or self.directory is None:raise ValueError('先に機体の変更案を適用してください')
            self.save()
        if suite=='joints' and 'controller' not in (self.current or {}) and not self.rejected_pending:raise ValueError('関節検証は生成制御の機体を選択してください')
        from xrobocon.workshop.automation import ImprovementLoop
        self.automation=ImprovementLoop(self,limit,suite,prompt,stop_on_pass,stage_timeout,allow_robot_changes)
        self.message=f'自動改善開始: 最大{limit}回、検証={suite}。各改訂を保存します。'
        return self.automation.directory

    def choose_robot(self,robot_id):
        self._idle()
        self.require_constraint_review=False;self.pending_creation_prompt=None
        self.rejected_pending=None
        entry=self.catalog.options('robots')[robot_id]
        directory=(self.catalog.path.parent/entry['robot_asset']).resolve().parent
        if directory.parent==(self.library.root/'revisions').resolve() and (directory/'manifest.json').is_file():
            self.load_revision(directory);return
        self.current=source_snapshot(self.catalog,robot_id);self.proposal=None;self.directory=None;self.saved=False
        self.last_result=None;self.pending_basis=None;self.message='既存機体を読み込みました。元ファイルは変更しません。'
    def load_revision(self,directory):
        self._idle();self.require_constraint_review=False;self.pending_creation_prompt=None;self.rejected_pending=None;self.current=self.library.load(directory);self.directory=Path(directory);self.saved=True;self.proposal=None
        self.pending_basis=None
        self.last_result=self.current.get('diagnostic_result');self.message='保存版を読み込みました。この版から分岐して修正できます。'
    def propose(self,prompt,mode='robot',basis=None,feedback=None,require_review=False):
        self._idle()
        if self.current is None and mode!='new_robot':raise ValueError('機体を選択してください')
        if not prompt.strip():raise ValueError('プロンプトを入力してください')
        self.require_constraint_review=require_review
        self.pending_creation_prompt=(feedback.get('creation_prompt') if feedback and 'creation_prompt' in feedback else prompt) if mode=='new_robot' else (self.current or {}).get('creation_prompt')
        self.rejected_pending=None
        self._inference_started=time.monotonic()
        self.pending_basis=deepcopy(basis)
        self.prompt=prompt;self.mode=mode;self.proposal=None;self.cancelled=Event();future=Future();self.future=future
        snapshot=None if mode=='new_robot' else deepcopy(self.current)
        if snapshot is not None:
            from xrobocon.workshop.diagnostics import inference_context
            snapshot['diagnostic_context']=inference_context(self.current,self.library.root)
            from xrobocon.workshop.reasoning import mechanism_context
            snapshot['mechanism_context']=mechanism_context(self.current)
            from xrobocon.workshop.reasoning import step_improvement_context
            snapshot['step_improvement_context']=step_improvement_context(snapshot)
            if feedback:snapshot['validation_feedback']=deepcopy(feedback)
            if self.automation and self.automation.active:
                snapshot['improvement_history']=deepcopy([e for e in self.automation.state['history'] if e['kind']=='selection'][-6:])
        generator=(self.improvement_generator if mode in ('new_robot','mechanism','program') else self.parameter_improvement_generator) if require_review else self.new_generator if mode in ('new_robot','mechanism','program') else self.generator
        self._active_generator=generator;cancelled=self.cancelled
        def run():
            started=time.monotonic()
            try:
                kwargs=dict(feedback=feedback) if feedback and mode in ('new_robot','mechanism','program') else {}
                future.set_result((generator.propose(snapshot,prompt,mode,cancelled,**kwargs),time.monotonic()-started))
            except Exception as error:future.set_exception(error)
        self.thread=Thread(target=run,name='workshop-inference',daemon=True);self.thread.start();self.message='Codexで変更案を作成中…'
    def poll(self):
        if self.future is not None and self.future.done():
            future=self.future;self.future=None
            try:
                result,elapsed=future.result()
                used_generator=self._active_generator
                self.inference=dict(model=getattr(used_generator,'model',None),reasoning_effort=getattr(used_generator,'reasoning_effort',None),wall_seconds=elapsed)
                self.inference.update(prompt_template_version=getattr(used_generator,'prompt_version',None),prompt_template_instruction=getattr(used_generator,'instruction',None),web_research_allowed=bool(getattr(used_generator,'allow_web_research',False)),research_execution=deepcopy(getattr(used_generator,'last_research',None)))
                if self.cancelled.is_set():self.message='生成を取り消しました。';return
                self.proposal=result;self.message='変更案を確認し、「適用・検証」を押してください。'
            except Exception as error:
                used_generator=self._active_generator
                self.inference=dict(model=getattr(used_generator,'model',None),reasoning_effort=getattr(used_generator,'reasoning_effort',None),wall_seconds=time.monotonic()-self._inference_started,error=str(error))
                self.inference.update(prompt_template_version=getattr(used_generator,'prompt_version',None),prompt_template_instruction=getattr(used_generator,'instruction',None),web_research_allowed=bool(getattr(used_generator,'allow_web_research',False)),research_execution=deepcopy(getattr(used_generator,'last_research',None)))
                if isinstance(error,RejectedDesign) and not self.cancelled.is_set():self._record_rejection(error.proposal,str(error))
                self.message='生成を取り消しました。' if self.cancelled.is_set() else f'生成失敗: {error}（失敗案は保存済み。「ログから制御を改善」で再生成できます）' if isinstance(error,RejectedDesign) else f'生成失敗: {error}'
        if self.process is not None and self.process.poll() is not None:
            code=self.process.returncode
            if self.log:self.log.close();self.log=None
            job=self.job;self.process=None
            if job and job['kind']=='diagnostic':
                report_path=job['out']/'report.json'
                try:
                    if job.get('dry_run') and code==0:result=json.loads(job['log_path'].read_text())
                    else:result=json.loads(report_path.read_text())
                except (OSError,ValueError):result=dict(complete=False,error=f'exit {code}; report missing or invalid; see {job["log_path"]}')
                self._record_diagnostic(job,result,code)
            elif code:self.message=f'子画面が終了しました（exit {code}）。ログを確認してください。'
        if self.automation:self.automation.tick()
        self._sync_preview()

    def _record_diagnostic(self,job,result,code):
        from xrobocon.workshop.diagnostics import finish_run
        self.last_result=result
        reference=finish_run(job,result,code)
        self.current['diagnostic_result']=deepcopy(result)
        self.current.setdefault('diagnostic_runs',[]).append(reference)
        if self.saved:
            evaluations=self.directory/'evaluations';evaluations.mkdir(exist_ok=True)
            write_json(evaluations/f'{time.time_ns()}.json',dict(model_sha256=self.current['model_sha256'],result=result,run_manifest=reference))
        self.message=('検証終了' if code==0 else '検証失敗／未完了（ログ保存済み）')+f': {job["root"]}。「ログから制御を改善」で次の案を作れます。'

    def performance_view(self):
        from xrobocon.workshop.scoring import score_snapshot,score_report,compare_scores
        current=self.current or {}
        key=(id(self.current),id(self.last_result),tuple(current.get('diagnostic_runs',[])))
        if getattr(self,'_performance_key',None)==key:return self._performance_cache
        score=score_snapshot(current,self.library.root) if self.current else score_report(None)
        baseline=None;parent=current.get('parent')
        if isinstance(parent,str) and Path(parent).name==parent:
            directory=self.library.root/'revisions'/parent
            if directory.is_dir():
                try:baseline=score_snapshot(self.library.load(directory),self.library.root)
                except (ValueError,OSError):pass
        value=dict(current=score,parent_revision=parent,baseline=baseline,comparison=compare_scores(score,baseline))
        self._performance_key=key;self._performance_cache=value
        return value

    def log_context_text(self):
        from xrobocon.workshop.diagnostics import inference_context,tail
        context=inference_context(self.current,self.library.root) if self.current else dict(runs=[])
        if self.rejected_pending:context['validation_failure']=self.rejected_pending
        if self.process is not None and self.job and self.job['kind']=='diagnostic':
            context['running_log']=dict(file=str(self.job['log_path']),tail=tail(self.job['log_path']))
        return json.dumps(context,ensure_ascii=False,indent=2)

    def improve_from_logs(self,prompt='',allow_robot_changes=False,preserve_creation_constraints=False):
        self._idle()
        if self.rejected_pending:
            failed=deepcopy(self.rejected_pending)
            return self.propose((prompt.strip()+'\n' if prompt.strip() else '')+'validation_feedbackの失敗案と検証エラーを確認して修正してください。実行成功は未検証と明記してください。',failed['mode'],basis=[failed['attempt_id']],feedback=failed,require_review=failed.get('require_review',False))
        from xrobocon.workshop.diagnostics import inference_context
        if self.current is None:raise ValueError('機体を選択してください')
        context=inference_context(self.current,self.library.root)
        matching=[run for run in context['runs'] if run['matches_current']]
        if not matching:raise ValueError('現在のモデル・制御・設定で動作検証を行ってください。古い版やdry-runのログだけでは改善を開始しません。')
        mode='mechanism' if allow_robot_changes else 'program' if 'controller' in self.current else 'control'
        instructions='元の作成プロンプトの制約を守り、ログから機体と制御を併せて改善してください。機体変更は必要な場合のみ行い、制約との照合結果を返してください。' if allow_robot_changes else '機体形状は変更せず、ログに基づいて制御を改善してください。'
        prompt=(prompt.strip()+'\n' if prompt.strip() else '')+instructions+'失敗原因・変更理由をsummaryに記載し、未検証の成功を断定しないでください。'
        self.propose(prompt,mode,basis=[run['run_id'] for run in matching],require_review=allow_robot_changes or preserve_creation_constraints)

    def _record_rejection(self,proposal,error):
        out=ensure_storage(self.library.root)/'jobs'/f'validation-{time.time_ns()}'
        out.mkdir(parents=True)
        record=dict(attempt_id=out.name,mode=self.mode,proposal=deepcopy(proposal),error=error,prompt=self.prompt,
                    robot_id=(self.current or {}).get('robot_id'),file=str(out/'failure.json'),creation_prompt=self.pending_creation_prompt,require_review=self.require_constraint_review)
        write_json(out/'failure.json',record)
        self.rejected_pending=record

    def _candidate_snapshot(self):
        candidate={k:v for k,v in self.proposal.items() if k not in ('prompt_revision','diagnostic_analysis')}
        if self.require_constraint_review:
            from xrobocon.workshop.constraints import validate_review
            validate_review(candidate.get('constraint_review'),(self.current or {}).get('creation_prompt'))
            candidate={k:v for k,v in candidate.items() if k!='constraint_review'}
        if self.mode in ('new_robot','mechanism','program'):
            from xrobocon.workshop.generated import apply_design
            if self.mode=='program':
                from xrobocon.workshop.design import canonical_xml
                if canonical_xml(candidate['mjcf'])!=canonical_xml(self.current['xml']):raise ValueError('制御コードのみの生成では機体を変更できません')
            updated=apply_design(None if self.mode=='new_robot' else self.current,candidate)
        else:updated=apply_proposal(self.current,candidate,self.mode)
        if 'prompt_revision' in self.proposal or 'diagnostic_analysis' in self.proposal:updated['proposal']=deepcopy(self.proposal)
        updated.pop('constraint_review',None)
        if self.require_constraint_review:
            updated['constraint_review']=deepcopy(self.proposal['constraint_review']);updated['proposal']=deepcopy(self.proposal)
        return updated

    def apply(self):
        self._idle()
        if self.proposal is None:raise ValueError('先に変更案を生成してください')
        try:
            updated=self._candidate_snapshot()
        except ValueError as error:
            self._record_rejection(self.proposal,str(error))
            raise
        self.rejected_pending=None
        if self.mode=='new_robot':
            updated['creation_prompt']=self.pending_creation_prompt if self.pending_creation_prompt is not None else self.prompt or None
            updated['creation_prompt_source']='creation_request' if updated['creation_prompt'] else 'unavailable'
        else:
            updated.setdefault('creation_prompt',None);updated.setdefault('creation_prompt_source','unavailable')
        updated['mode']=self.mode
        updated.pop('improvement_basis',None)
        updated.pop('improvement_prompt_usage',None)
        if self.pending_basis:updated['improvement_basis']=deepcopy(self.pending_basis)
        updated['inference']=getattr(self,'inference',dict(model=getattr(self.generator,'model',None),reasoning_effort=getattr(self.generator,'reasoning_effort',None)))
        # Validation precedes draft creation; no partial failed draft replaces the current model.
        root=ensure_storage(self.library.root)/'drafts';root.mkdir(parents=True,exist_ok=True)
        directory=Path(tempfile.mkdtemp(prefix='draft-',dir=root))
        (directory/'model.xml').write_text(updated['xml']);write_json(directory/'tuning.json',updated['tuning'])
        write_json(directory/'proposal.json',dict(prompt=self.prompt,mode=self.mode,proposal=self.proposal))
        from xrobocon.workshop.generated import write_bundle
        write_bundle(directory,updated)
        self.current=updated;self.directory=directory;self.saved=False;self.last_result=None
        self.message='モデルと制御設定の検証成功。3D確認・診断後に保存できます。'
        return directory
    def save(self):
        self._idle()
        if self.directory is None:raise ValueError('先に変更案を適用してください')
        if self.proposal is not None and self.proposal!=self.current.get('proposal'):raise ValueError('先に変更案を適用してください')
        if self.saved:return self.directory
        path=self.library.save(self.current,self.prompt);self.current=self.library.load(path);self.directory=path;self.saved=True
        self.message=f'保存しました: {path.name}';return path
    def save_as_new_robot(self,name):
        self._idle()
        if self.directory is None:raise ValueError('先に変更案を適用してください')
        if self.proposal is not None and self.proposal!=self.current.get('proposal'):raise ValueError('先に変更案を適用してください')
        from xrobocon.workshop.identity import new_robot
        snapshot=new_robot(self.current,name)
        path=self.library.save(snapshot,self.prompt)
        self.current=self.library.load(path);self.directory=path;self.saved=True;self.last_result=None
        self.message=f'新しいロボット「{name}」として保存しました。対戦へ登録できます。'
        return path

    def _spawn(self,command,kind,out):
        self._idle();ensure_storage(out);out.mkdir(parents=True,exist_ok=True)
        self.log=(out/'process.log').open('w')
        try:self.process=subprocess.Popen(command,cwd=ROOT,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:self.log.close();self.log=None;raise
        self.job=dict(kind=kind,out=out,log_path=out/'process.log')
    def presentation(self):
        """Validated display candidate, cached until the input objects change."""
        key=(id(self.current),id(self.proposal),self.mode,self.saved,self.require_constraint_review)
        if getattr(self,'_presentation_key',None)==key:return self._presentation_cache
        pending=self.proposal is not None and self.proposal!=(self.current or {}).get('proposal')
        try:
            snapshot=self._candidate_snapshot() if pending else self.current
            status='未適用の変更案（診断・保存は適用後）' if pending else '保存済みの機体' if self.saved else '適用済み・未保存の機体' if self.directory else '読込中の機体'
            value=dict(snapshot=snapshot,status=status,pending=pending,error=None)
        except (ValueError,KeyError,TypeError) as error:
            value=dict(snapshot=None,status='変更案の検証エラー（3D更新不可）',pending=True,error=str(error))
        self._presentation_inputs=(self.current,self.proposal)
        self._presentation_key=key;self._presentation_cache=value
        return value

    def _sync_preview(self):
        job=self.preview_job
        if not job:return
        if job['process'].poll() is not None:
            if job['process'].returncode:self.message=f'3Dビューが終了しました。表示ログを確認してください: {job["out"] / "process.log"}'
            job['log'].close();self.preview_job=None;return
        view=self.presentation()
        if job.get('view') is view:return
        job['view']=view
        snapshot=view['snapshot']
        label=view['status']
        if snapshot:
            temporary=job['out']/'model.tmp.xml';temporary.write_text(snapshot['xml']);temporary.replace(job['out']/'model.xml')
            label=snapshot.get('proposal',{}).get('title',snapshot['entry']['label'])+' / '+label
        metadata=job['out']/'view.tmp.json';write_json(metadata,dict(label=label,error=view['error']));metadata.replace(job['out']/'view.json')

    def preview(self,smoke=False,screenshot=None):
        self._idle()
        view=self.presentation()
        if not view['snapshot']:raise ValueError(view['error'] or '機体を選択してください')
        self._sync_preview()
        if self.preview_job:
            self.message='3Dビューは最新の表示対象に更新されています。';return
        out=ensure_storage(self.library.root)/'jobs'/f'preview-{time.time_ns()}'
        out.mkdir(parents=True,exist_ok=False)
        asset=out/'model.xml';asset.write_text(view['snapshot']['xml'])
        command=[sys.executable,str(ROOT/'scripts/preview_robot.py'),'--asset',str(asset),'--watch','--label',view['status']]
        if smoke:command.append('--smoke')
        if screenshot:command.extend(['--screenshot',str(screenshot)])
        log=(out/'process.log').open('w')
        try:process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:log.close();raise
        self.preview_job=dict(process=process,out=out,log=log)
        self._sync_preview()
        self.message='3Dビューを開きました。編集を続けると表示も更新されます。静止形状の確認用です。'
    def diagnostic(self,suite='flat',dry_run=False):
        self._idle()
        if suite not in ('flat','steps','all','joints'):raise ValueError('unknown diagnostic suite')
        if self.directory is None:raise ValueError('先に変更案を適用してください')
        out=ensure_storage(self.library.root)/'jobs'/f'diagnostic-{time.time_ns()}';out.mkdir(parents=True)
        data,robot_id=catalog_with_revision(self.catalog,self.current,self.directory);path=out/'catalog.json';write_json(path,data)
        command=[sys.executable,str(ROOT/'scripts/tune_robot.py'),'--catalog',str(path),'--robot',robot_id,'--profile','baseline','--suite',suite,'--out',str(out/'run')]
        if suite=='joints':
            if 'controller' not in self.current:raise ValueError('関節動作確認は新規生成制御の機体を選んでください')
            command=[sys.executable,str(ROOT/'scripts/test_generated_robot.py'),'--directory',str(self.directory),'--out',str(out/'run')]
        if dry_run:command.append('--dry-run')
        from xrobocon.workshop.diagnostics import start_run
        start_run(out,self.current,suite,self.directory,dry_run)
        try:self._spawn(command,'diagnostic',out)
        except Exception as error:
            job=dict(root=out,out=out/'run',log_path=out/'process.log')
            self._record_diagnostic(job,dict(complete=False,error=str(error)),-1)
            raise
        self.job['root']=out
        self.job['out']=out/'run' if not dry_run else out
        self.job['dry_run']=dry_run
        # tune_robot owns creation of the result folder; log outside it.
        self.message='既存の診断を実行中…（競技用ROS2とは別の直接観測による診断）'
        return out
    def register(self):
        self._idle()
        if not self.saved:raise ValueError('先に版を保存してください')
        data,robot_id=catalog_with_revision(self.catalog,self.current,self.directory)
        output=ensure_storage(self.library.root)/'catalog.json'
        if output.exists():
            old=json.loads(output.read_text())
            data['robots'].update({k:v for k,v in old['robots'].items() if k.startswith('workshop_') and k!=robot_id})
        temporary=output.with_suffix('.tmp.json');write_json(temporary,data)
        MatchCatalog.load(temporary);temporary.replace(output)
        self.catalog=MatchCatalog.load(output)
        self.message='対戦カタログに登録しました。診断結果を確認して使用してください。'
        return output
    def cancel(self):
        if self.automation and self.automation.active:self.automation.finish('cancelled')
        self.cancelled.set()
        if self.process is not None and self.process.poll() is None:
            if self.job:self.job['cancelled']=True
            os.killpg(self.process.pid,signal.SIGINT)
    def close(self):
        if self.preview_job:
            job=self.preview_job;process=job['process']
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            job['log'].close();self.preview_job=None
        self.cancel()
        if hasattr(self,'thread'):self.thread.join(timeout=3)
        if self.process is not None:
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
            if self.job and self.job['kind']=='diagnostic':self.poll()
        if self.log:self.log.close();self.log=None
