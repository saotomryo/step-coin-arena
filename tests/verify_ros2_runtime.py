"""Real DDS delivery to independent agent processes, including standard sensors."""
import argparse,json,os,sys,signal,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);parser.add_argument('--ticks',type=int,default=110);parser.add_argument('--domain-id',type=int,default=38);parser.add_argument('--check-timeout',action='store_true');args=parser.parse_args()
    out=Path(args.out);source=Path('configs/competition_ros2.json').resolve();config=json.loads(source.read_text())
    config['simulation']['scenario']=str((source.parent/config['simulation']['scenario']).resolve())
    config['transport']['domain_id']=args.domain_id
    out.parent.mkdir(parents=True,exist_ok=True);staged=out.parent/(out.name+'-config.json');staged.write_text(json.dumps(config,indent=2))
    config,sim,manager,agents,runner=build_runtime(staged,out,False)
    checks=[];sources=set();pids=set();last=None
    try:
        runner.start()
        with (out/'trace.jsonl').open('w') as trace:
            for _ in range(args.ticks):
                last=runner.tick();trace.write(json.dumps(last,allow_nan=False)+'\n')
                wire=last['transport'];sources.update(wire['sources'].values());pids.update(wire['agent_pids'].values())
                for name,stamps in wire['observed_sensor_stamps'].items():
                    obs=last['observations'][name]
                    assert set(stamps)==set(obs['sensors'])
                    for kind,stamp in stamps.items():assert abs(stamp-obs['sensors'][kind]['timestamp'])<1e-8
        assert len(pids)==2 and os.getpid() not in pids;checks.append('two independent ROS2 agent processes, not Genesis PID')
        assert sources=={'ros2/truth_baseline'};checks.append('both agents receive observations through ROS2 only')
        checks.append('standard sensor topic stamps match every expected sample')
        counts=last['transport']['published_sensor_counts']
        assert all(c['imu']>c['joint_states']>c['contacts']>c['lidar'] for c in counts.values());checks.append('sensor topics published at independent configured rates')
        # An act() call in this process must be rejected, not silently fall back.
        try:agents['self'].act(None,.01)
        except RuntimeError:checks.append('direct agent invocation prohibited in ROS2 mode')
        else:raise AssertionError('local act unexpectedly accepted')
        preserved=agents['opponent'].phase;runner.transport.reset(['self']);agents['self'].reset();runner.tick()
        assert agents['opponent'].phase==preserved;checks.append('individual agent reset retains other agent state')
        runner.start();row=runner.tick();assert row['time']==.01;checks.append('new round resets simulator clock and ROS2 agents')
        assert all(all(t==0 for t in stamps.values()) for stamps in row['transport']['observed_sensor_stamps'].values());checks.append('round reset does not consume stale samples')
        if args.check_timeout:
            blue_pid=row['transport']['agent_pids']['self'];clock=manager.elapsed;started=time.monotonic()
            os.kill(blue_pid,signal.SIGTERM)
            try:runner.tick()
            except (RuntimeError,TimeoutError):
                assert manager.elapsed==clock
                checks.append('lost agent stops physics without advancing competition or direct fallback')
            else:raise AssertionError('missing ROS2 agent did not stop the runner')
        report=dict(passed=True,checks=checks,physics_pid=os.getpid(),agent_pids=sorted(pids),sources=sorted(sources),sensor_publish_counts=counts,state=manager.snapshot(),runtime=runner.transport.runtime)
        (out/'verification.json').write_text(json.dumps(report,indent=2));print(f'ROS2 runtime: {len(checks)} checks passed',flush=True)
    finally:runner.close()
if __name__=='__main__':main()
