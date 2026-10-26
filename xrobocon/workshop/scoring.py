"""Descriptive control-performance scores, never an exploration/selection policy."""
import json
import math
from pathlib import Path
from xrobocon.workshop.diagnostics import fingerprint

VERSION='control-performance-v1'
CASES=('straight','turn','route','three_steps')
LABELS=dict(straight='直進・停止',turn='旋回・移動',route='連続経路',three_steps='三段登坂')
POINTS=dict(straight=((1.,0.),),turn=((0.,1.),),route=((.6,0.),(.6,.6),(0.,.6)))
BUDGET=dict(straight=25.,turn=25.,route=25.,three_steps=120.)
WEIGHTS=dict(achievement=40.,progress=30.,time=15.,stability=10.,headroom=5.)

def number(v):return type(v) in (int,float) and math.isfinite(v)
def clip(v):return max(0.,min(1.,v))

def path_progress(case,trace):
    """Approximate completed waypoint distance from sampled XY, not a new trial."""
    points=POINTS[case];lengths=[];last=(0.,0.)
    for p in points:lengths.append(math.dist(last,p));last=p
    total=sum(lengths);index=0;best=0.;count=0;valid=0
    try:
        with Path(trace).open('rb') as stream:
            for line in stream:
                count+=1
                if count>20000 or stream.tell()>8_000_000:break
                try:r=json.loads(line);pose=r['pose'];xy=(pose['x'],pose['y'])
                except (ValueError,KeyError,TypeError):continue
                if not all(number(v) for v in xy):continue
                valid+=1
                while index<len(points) and math.dist(xy,points[index])<=.10:index+=1
                remaining=0. if index==len(points) else math.dist(xy,points[index])+sum(lengths[index+1:])
                best=max(best,clip(1-remaining/total))
    except OSError:return None
    return best if valid else None

def score_trial(trial,trace=None):
    case=trial['case'];passed=trial.get('passed') is True
    if passed:progress=1.;progress_source='passed'
    elif case=='three_steps' and number(trial.get('coins')):
        progress=clip(trial['coins']/3);progress_source='three_target_coins'
    elif trace is not None:
        progress=path_progress(case,trace);progress_source='sampled_waypoints'
    elif case in ('straight','turn') and number(trial.get('final_error')) and trial['final_error']>=0:
        progress=clip(1-trial['final_error']);progress_source='endpoint_distance_1m'
    else:progress=None;progress_source='missing_trajectory'
    elapsed=trial.get('simulation_seconds');tilt=trial.get('max_tilt_degrees');saturation=trial.get('torque_saturated_fraction')
    components=dict(achievement=40. if passed else 0.,progress=None if progress is None else 30*progress,
        time=15*clip(1-elapsed/BUDGET[case]) if passed and number(elapsed) and elapsed>=0 else 0. if not passed else None,
        stability=10*clip(1-tilt/60) if number(tilt) and tilt>=0 else None,
        headroom=5*(1-saturation) if number(saturation) and 0<=saturation<=1 else None)
    score=round(sum(components.values()),1) if type(trial.get('passed')) is bool and all(v is not None for v in components.values()) else None
    return dict(case=case,label=LABELS[case],score=score,passed=passed,status=trial.get('status'),
        progress=None if progress is None else round(progress,4),progress_source=progress_source,
        components={k:round(v,2) if v is not None else None for k,v in components.items()},
        raw={k:trial.get(k) for k in ('simulation_seconds','final_error','max_tilt_degrees','torque_saturated_fraction','coins')},
        note='未評価: 採点に必要な情報が不足' if score is None else '簡易指標。探索先や改善可能性を示す値ではありません。')

def score_report(report,trace_root=None):
    result=dict(policy_version=VERSION,score=None,scope=None,cases={c:dict(case=c,label=LABELS[c],score=None,note='未評価') for c in CASES},comparison_key=None,note='未評価')
    if not isinstance(report,dict):return result
    plan=report.get('plan',{});suite=plan.get('suite');result['scope']=suite
    rows=report.get('results',[])
    if suite not in ('flat','steps','all') or plan.get('protocol_version')!=1:
        result['note']='未評価: 関節スモークまたは未対応の検証条件';return result
    if len(rows)!=1:
        result['note']='未評価: 制御候補を個別に検証してください';return result
    row=rows[0]
    if report.get('error') or row.get('error') or row.get('complete') is not True or row.get('status')=='smoke_only_performance_unrated':
        result['note']='未評価: 検証が未完了または実行エラー';return result
    trials=row.get('trials',[]);seen=set()
    for trial in trials:
        case=trial.get('case')
        if case not in CASES or case in seen:continue
        seen.add(case);trace=None
        if trace_root is not None and case in POINTS:
            folder=Path(trace_root).resolve();candidate=row.get('candidate','baseline')
            if isinstance(candidate,str) and Path(candidate).name==candidate:
                path=folder/candidate/(case+'.jsonl')
                if path.is_file() and path.resolve().is_relative_to(folder):trace=path
        result['cases'][case]=score_trial(trial,trace)
    expected=('straight','turn','route') if suite=='flat' else ('three_steps',) if suite=='steps' else CASES
    values=[result['cases'][c]['score'] for c in expected]
    if all(v is not None for v in values):result['score']=round(sum(values)/len(values),1)
    result['comparison_key']=[VERSION,plan.get('protocol_version'),suite,row.get('candidate'),plan.get('fixture_signature')]
    result['note']='評価対象の平均。未検証項目は総合点へ混ぜません。' if result['score'] is not None else '未評価項目があるため総合点は未算出'
    result['fixture_assumption']='calibration protocol v1: fixed goals, dt=.01, 25s flat/120s steps'
    return result

def score_snapshot(snapshot,library):
    """Read only input-bound, finished diagnostics; old logs need no migration."""
    current=fingerprint(snapshot);root=Path(library).resolve();merged=score_report(None);covered=set();keys={};runs=[]
    for reference in reversed(snapshot.get('diagnostic_runs',[])):
        path=Path(reference).resolve()
        if not path.is_relative_to(root/'jobs') or not path.is_file():continue
        try:record=json.loads(path.read_text())
        except (OSError,ValueError):continue
        if record.get('fingerprint')!=current or record.get('robot_id')!=snapshot.get('robot_id') or record.get('dry_run') or record.get('status')!='finished':continue
        cached=record.get('control_score')
        if isinstance(cached,dict) and cached.get('policy_version')==VERSION:score=dict(cached)
        else:
            report=record.get('result');output=Path(record.get('report',str(path.parent/'report.json'))).resolve().parent
            if not output.is_relative_to(path.parent):output=None
            score=score_report(report,output)
        scope=score.get('scope')
        if scope not in ('flat','steps','all'):continue
        cases=('straight','turn','route') if scope=='flat' else ('three_steps',) if scope=='steps' else CASES
        for case in cases:
            if case in covered:continue
            covered.add(case);merged['cases'][case]=dict(score['cases'][case],run_id=record.get('run_id'))
            keys[case]=score.get('comparison_key')
        runs.append(record.get('run_id'))
        if len(covered)==4:break
    if not covered:return merged
    values=[merged['cases'][c]['score'] for c in covered]
    merged['score']=round(sum(values)/len(values),1) if all(v is not None for v in values) else None
    merged['scope']='all' if len(covered)==4 else 'flat' if covered==set(CASES[:3]) else 'steps'
    known=all(keys[c] and keys[c][-1] is not None for c in covered)
    merged['comparison_key']=[VERSION,1,sorted(covered),'per_case',[[c,keys[c]] for c in sorted(covered)] if known else None]
    merged.update(run_ids=runs,fingerprint=current,note='同一モデル・制御・設定の項目別最新評価。未検証項目は平均に含めず、未完了項目があれば総合は未算出。')
    return merged

def compare_scores(current,baseline):
    # Old fixture provenance is unknown: display scores, but never invent a delta.
    key=current.get('comparison_key')
    if not key or not baseline or key!=baseline.get('comparison_key') or key[-1] is None:
        return dict(delta=None,note='比較不可: 同一検証条件の記録がない、または条件が異なります')
    a=current.get('score');b=baseline.get('score')
    return dict(delta=round(a-b,1) if a is not None and b is not None else None,note='親版との差。再開元や探索方針の自動決定には使用しません。')
