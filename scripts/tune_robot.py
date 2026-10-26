"""Run shared calibration fixtures and export a reviewable recommendation.

Diagnostic observations are direct by design. Production matches keep ROS2.
No candidate changes mass, geometry or the hardware wheel torque limit.
"""
import argparse,json,sys,copy,time,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.match_setup import MatchCatalog,build_match_config
from xrobocon.robot_tuning import load_tuning,select_candidate,tuning_signature,verify_comparison,verify_asset_binding,resolve_locomotion
from xrobocon.tuning_trials import run_flat,run_steps,asset_fingerprint
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',default='front_bogie_r20_wide');p.add_argument('--profile',choices=['baseline','safe','fast'],default=None);p.add_argument('--scan',action='store_true',help='compare baseline, safe, fast on the same model')
    p.add_argument('--catalog',default=str(ROOT/'configs/match_catalog.json'))
    p.add_argument('--suite',choices=['flat','steps','all'],default='all');p.add_argument('--tuning',help='alternate tuning JSON; original is never overwritten');p.add_argument('--out',help='new output directory; defaults to mounted external SSD');p.add_argument('--viewer',action='store_true');p.add_argument('--dry-run',action='store_true');args=p.parse_args()
    catalog=MatchCatalog.load(args.catalog)
    if args.robot not in catalog.options('robots'):p.error('unknown robot')
    robot=catalog.options('robots')[args.robot]
    args.profile=args.profile or robot.get('recommended_control','baseline')
    if not robot.get('tuning_file') and not args.tuning:p.error('robot needs a tuning_file or --tuning')
    tuning=load_tuning(args.tuning or catalog.path.parent/robot['tuning_file']);profiles=['baseline','safe','fast'] if args.scan else [args.profile]
    configs={}
    for profile in profiles:
        choices=catalog.defaults();choices['self'].update(robot=args.robot,control=profile)
        config=build_match_config(catalog,choices)
        verify_asset_binding(tuning,config['simulation']['robot_models']['self']['robot_asset'])
        config['simulation']['robot_models']['self']['drive_parameters']=copy.deepcopy(tuning['drive'])
        if tuning.get('locomotion'):config['agents']['self']['locomotion']=resolve_locomotion(tuning,config['simulation']['scenario'])
        config['agents']['self']['motion_limits']=copy.deepcopy(tuning['motion'].get(profile,{}));configs[profile]=config
    fingerprint=asset_fingerprint(next(iter(configs.values())))
    plan=dict(protocol_version=1,tuning_signature=tuning_signature(tuning),robot=args.robot,suite=args.suite,candidates=profiles,asset=fingerprint,observation_source='direct diagnostic fixture; production ROS2 unchanged',wheel_torque_limit=next(iter(configs.values()))['simulation']['torque_limit'])
    import hashlib
    fixture_files=[ROOT/'xrobocon/tuning_trials.py',ROOT/'configs/scenarios/passive_front_bogie_r18_steering_flat.json']
    if args.suite in ('steps','all'):fixture_files.append(Path(next(iter(configs.values()))['simulation']['scenario']))
    from importlib.metadata import version
    fixture_bytes=b''.join(hashlib.sha256(path.read_bytes()).digest() for path in fixture_files)
    plan['fixture_signature']=hashlib.sha256(fixture_bytes+version('genesis-world').encode()).hexdigest()
    if args.dry_run:print(json.dumps(dict(plan=plan,tuning=tuning),indent=2,ensure_ascii=False));return
    if args.out:out=Path(args.out)
    else:
        from xrobocon.storage import resource_root
        out=resource_root()/'evaluation_results'/f'tuning-{args.robot}-{time.time_ns()}'
    out.mkdir(parents=True,exist_ok=False);(out/'plan.json').write_text(json.dumps(plan,indent=2));(out/'input-tuning.json').write_text(json.dumps(tuning,indent=2))
    results=[]
    for profile,config in configs.items():
        folder=out/profile
        if args.scan:
            # Independent native physics processes bound peak memory between candidates.
            command=[sys.executable,str(Path(__file__).resolve()),'--catalog',str(Path(args.catalog).resolve()),'--robot',args.robot,'--profile',profile,'--suite',args.suite,'--out',str(folder)]
            command.extend(['--tuning',str((out/'input-tuning.json').resolve())])
            if args.viewer:command.append('--viewer')
            print(f'Running {profile}: {folder}',flush=True)
            with (out/(profile+'.log')).open('w') as log:process=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            report_path=folder/'report.json'
            if report_path.is_file():
                child=json.loads(report_path.read_text());result=child['results'][0]
                try:verify_comparison(child['plan'],plan)
                except ValueError as exc:result.update(complete=False,passed=False,error=dict(type='ComparisonChanged',message=str(exc)))
            else:result=dict(candidate=profile,complete=False,passed=False,simulation_seconds=0,error=dict(type='ChildProcessError',message=f'exit={process.returncode}; see {profile}.log'),trials=[])
            results.append(result);print(json.dumps(result),flush=True);continue
        folder.mkdir();(folder/'config.json').write_text(json.dumps(config,indent=2));rows=[];error=None
        try:
            if args.suite in ('flat','all'):rows.extend(run_flat(config,folder,profile,viewer=args.viewer))
            if args.suite in ('steps','all'):rows.extend(run_steps(config,folder,viewer=args.viewer))
        except Exception as exc:error=dict(type=type(exc).__name__,message=str(exc))
        expected=4 if args.suite=='all' else 3 if args.suite=='flat' else 1
        result=dict(candidate=profile,complete=len(rows)==expected,passed=len(rows)==expected and all(r['passed'] for r in rows) and error is None,simulation_seconds=sum(r['simulation_seconds'] for r in rows),error=error,trials=rows)
        results.append(result);(folder/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    recommended=select_candidate(results)
    report=dict(plan=plan,results=results,recommended_profile=recommended,recommendation_scope=args.suite,automatic_application=False)
    (out/'report.json').write_text(json.dumps(report,indent=2))
    if recommended:
        exported=copy.deepcopy(tuning);exported['recommended_profile']=recommended;exported['calibration']=dict(suite=args.suite,asset=fingerprint,report=str(out/'report.json'))
        (out/'recommended-tuning.json').write_text(json.dumps(exported,indent=2))
    print(json.dumps(dict(output=str(out),recommended_profile=recommended,all_passed=all(r['passed'] for r in results))),flush=True)
    if not recommended:raise SystemExit(1)
if __name__=='__main__':main()
