"""Persistent, input-bound diagnostic runs and bounded inference evidence."""
import json
import math
from pathlib import Path
from xrobocon.workshop.design import digest,write_json


def fingerprint(snapshot):
    return dict(model=digest(snapshot['xml'].encode()),
                control=digest(json.dumps(snapshot.get('controller'),sort_keys=True).encode()),
                tuning=digest(json.dumps(snapshot['tuning'],sort_keys=True).encode()))


def tail(path, limit=6000):
    path=Path(path)
    if not path.is_file():return ''
    with path.open('rb') as stream:
        stream.seek(max(0,path.stat().st_size-limit))
        return stream.read(limit).decode('utf-8',errors='replace')


def trace_evidence(path):
    """Scan a bounded trace, retaining start/end and worst finite numeric error/tilt."""
    samples=[];last=None;worst=None;worst_score=-1.;count=0;malformed=0;scanned=0
    metrics={}
    with Path(path).open('rb') as stream:
        for line in iter(lambda:stream.readline(256_000),b''):
            scanned+=len(line)
            if scanned>8_000_000 or count>=20000:break
            try:row=json.loads(line)
            except (ValueError,UnicodeDecodeError):malformed+=1;continue
            if not isinstance(row,dict):continue
            count+=1
            names=row.get('joint_names',[])
            for index,name in enumerate(names):
                item=metrics.setdefault(name,dict(joint=name,max_abs_error=0.,max_abs_torque=0.,saturated_samples=0,samples=0))
                for key,field in [('tracking_errors','max_abs_error'),('torques','max_abs_torque')]:
                    values=row.get(key,[])
                    if index<len(values) and type(values[index]) in (int,float) and math.isfinite(values[index]):item[field]=max(item[field],abs(values[index]))
                saturated=row.get('saturation',[])
                item['saturated_samples']+=int(index<len(saturated) and bool(saturated[index]))
                item['samples']+=1
            if len(samples)<2:samples.append(row)
            last=row
            values=[row.get(k) for k in ('tilt','error','tracking_error_max','max_tilt_degrees')]
            score=max((abs(v) for v in values if type(v) in (int,float) and math.isfinite(v)),default=0.)
            if score>worst_score:worst_score=score;worst=row
    if worst is not None and worst not in samples:samples.append(worst)
    if last is not None and last not in samples:samples.append(last)
    def compact(row):
        keys=('time','case','step','pose','error','tilt','tracking_error_max','joint_names','modes','position','velocity','targets','torques','tracking_errors','saturation')
        def simplify(value):
            if isinstance(value,float):return round(value,5) if math.isfinite(value) else str(value)
            if isinstance(value,list):return [simplify(v) for v in value[:6]]
            if isinstance(value,dict):return {k:simplify(v) for k,v in value.items()}
            return value
        return {key:simplify(row[key]) for key in keys if key in row}
    ranked=sorted(metrics.values(),key=lambda item:(item['saturated_samples']/max(1,item['samples']),item['max_abs_error']),reverse=True)
    return dict(file=str(path),rows_scanned=count,malformed_rows=malformed,
                truncated=scanned>8_000_000 or count>=20000,
                joint_count=len(metrics),worst_joints=ranked[:6],
                first=compact(samples[0]) if samples else None,
                worst=compact(worst) if worst is not None else None,last=compact(last) if last is not None else None,
                note='Samples show up to six joints; full arrays remain in the raw trace.')


def start_run(job, snapshot, suite, directory, dry_run=False):
    from xrobocon.workshop.generated import write_bundle
    root=Path(job);inputs=root/'input';inputs.mkdir()
    (inputs/'model.xml').write_text(snapshot['xml']);write_json(inputs/'tuning.json',snapshot['tuning'])
    write_bundle(inputs,snapshot)
    record=dict(run_id=root.name,robot_id=snapshot.get('robot_id'),revision_id=snapshot.get('revision_id'),
        fingerprint=fingerprint(snapshot),suite=suite,dry_run=dry_run,status='running',directory=str(directory),files=[])
    write_json(root/'diagnostic.json',record)
    return record


def finish_run(job, report, code):
    root=Path(job['root']);path=root/'diagnostic.json'
    record=json.loads(path.read_text());record.update(status='cancelled' if job.get('cancelled') else 'finished',exit_code=code,
        report=str(Path(job['out'])/'report.json'),log=str(job['log_path']),result=report)
    record['files']=[str(p) for p in sorted(root.rglob('*')) if p.is_file() and 'input' not in p.relative_to(root).parts and p!=path]
    from xrobocon.workshop.scoring import score_report
    record['control_score']=score_report(report,job['out'])
    write_json(path,record)
    return str(path)


def bounded(value, maximum):
    text=json.dumps(value,ensure_ascii=False,default=str)
    return value if len(text)<=maximum else dict(truncated=True,excerpt=text[:maximum])


def inference_context(snapshot, library, limit=2):
    root=Path(library).resolve();runs=[]
    for reference in reversed(snapshot.get('diagnostic_runs',[])):
        path=Path(reference).resolve()
        if not path.is_relative_to(root/'jobs') or not path.is_file():continue
        try:record=json.loads(path.read_text())
        except (ValueError,OSError):continue
        if record.get('dry_run') or record.get('status')=='running':continue
        if record.get('robot_id')!=snapshot.get('robot_id'):continue
        evidence=dict(run_id=record['run_id'],suite=record['suite'],status=record['status'],exit_code=record.get('exit_code'),
            input_fingerprint=record['fingerprint'],matches_current=record['fingerprint']==fingerprint(snapshot),
            result=bounded(record.get('result'),5000),report=record.get('report'),log=record.get('log'),traces=[])
        # Only read artifacts under this run, never paths supplied by generated code.
        log=root/'jobs'/record['run_id']/'process.log'
        evidence['process_tail']=tail(log,3000)
        for trace in sorted(path.parent.rglob('*.jsonl'))[:3]:
            if trace.resolve().is_relative_to(path.parent):
                try:evidence['traces'].append(bounded(trace_evidence(trace),6000))
                except (ValueError,OSError):evidence['traces'].append(dict(file=str(trace),error='trace could not be summarized'))
        runs.append(evidence)
        if len(runs)>=limit:break
    return dict(scope='Diagnostic evidence only. Failure and incomplete runs are retained; passing smoke is not task success.',
                runs=runs)
