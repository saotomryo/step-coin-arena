"""Compare control profiles with the same field, routes, rules and motor limits.

One run is a smoke comparison, not an estimate of failure probability.
"""
import argparse,json,time,sys
from pathlib import Path
from math import hypot
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime

def main():
    p=argparse.ArgumentParser();p.add_argument('--profile',choices=('baseline','safe','fast'),required=True);p.add_argument('--out',required=True);p.add_argument('--config',default='configs/competition_image.json');p.add_argument('--seconds',type=float,default=180);args=p.parse_args()
    # Keep the original config immutable; a resolved experiment config lives beside its output.
    source=Path(args.config).resolve();config=json.loads(source.read_text())
    config['simulation']['scenario']=str((source.parent/config['simulation']['scenario']).resolve())
    config['competition']['time_limit']=args.seconds
    for options in config['agents'].values():options['control_profile']=args.profile
    root=Path(args.out)
    if root.exists():raise FileExistsError(root)
    root.parent.mkdir(parents=True,exist_ok=True)
    config_path=root.parent/(root.name+'-config.json');config_path.write_text(json.dumps(config,indent=2))
    config,sim,manager,agents,runner=build_runtime(config_path,root,False)
    phase_times={p:{} for p in agents};distances={p:0. for p in agents};max_tilts={p:0. for p in agents};error=None;plan_events=[];avoidance_events=[];began=time.perf_counter()
    try:
        runner.start();previous=sim.truth_positions()
        with (root/'trace.jsonl').open('w') as trace:
            while manager.status=='running':
                phases={p:f'{a.stage}:{a.phase}' if hasattr(a,'stage') else 'flat' for p,a in agents.items()}
                row=runner.tick()
                if row.get('avoidance_events'):avoidance_events.append(dict(time=row['time'],events=row['avoidance_events']))
                if row.get('route_plans'):plan_events.append(dict(time=row['time'],plans=row['route_plans']))
                for p in agents:
                    phase_times[p][phases[p]]=phase_times[p].get(phases[p],0)+config['dt']
                    pos=row['positions'][p];distances[p]+=hypot(pos[0]-previous[p][0],pos[1]-previous[p][1]);previous[p]=pos
                    angle=sim.robots[p].get_euler();max_tilts[p]=max(max_tilts[p],abs(float(angle[0])),abs(float(angle[1])))
                if row.get('avoidance_events') or row.get('route_plans') or row.get('recovery_events') or round(row['time']/config['dt'])%100==0:trace.write(json.dumps(row)+'\n');trace.flush()
    except Exception as exc:error=dict(type=type(exc).__name__,message=str(exc))
    finally:
        runner.close();state=manager.snapshot()
        report=dict(profile=args.profile,elapsed=manager.elapsed,wall_seconds=time.perf_counter()-began,error=error,scores=manager.scores,collection_counts=manager.collection_counts,robot_contact_steps=sim.robot_contact_steps,recovery_counts=state['recovery_counts'],recovery_blocked_seconds=state['recovery_blocked_seconds'],max_tilt_degrees=max_tilts,distance_metres=distances,phase_seconds=phase_times,parameters={p:asdict(a.motion) for p,a in agents.items()},agent_checkpoints={p:getattr(a,'checkpoints',[]) for p,a in agents.items()},final_phases={p:getattr(a,'phase',None) for p,a in agents.items()},torque_limit=config['simulation']['torque_limit'],observations='ros2/truth_baseline' if runner.transport else 'truth_baseline',trial_count=1,route_plan_events=plan_events,avoidance_events=avoidance_events)
        (root/'state.json').write_text(json.dumps(state,indent=2));(root/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    if error:raise SystemExit(1)

if __name__=='__main__':main()
