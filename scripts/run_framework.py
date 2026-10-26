"""Configurable local baseline with independent agent, manager and physics."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.agent import FixedCoinAgent
from xrobocon.competition import CompetitionManager
from xrobocon.runner import CompetitionRunner

def build_runtime(config_file,output,viewer=False):
    config_path=Path(config_file).resolve();config=json.loads(config_path.read_text())
    if config.get('schema_version')!=1:raise ValueError('unsupported framework schema')
    if config['dt']!=.01:raise ValueError('current Genesis adapter requires dt=.01')
    mission=config['competition'];participants=mission['participants']
    spawns=config['simulation'].get('spawns')
    if spawns is not None and set(spawns)!=set(participants):raise ValueError('spawns must match participants')
    if spawns is None and len(participants)!=1:raise ValueError('multiple robots require explicit spawns')
    from xrobocon.sensors import validate_sensor_config
    validate_sensor_config(config.get('sensors',{}),participants)
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    resolved=dict(config);resolved['simulation']=dict(config['simulation'],scenario=str((config_path.parent/config['simulation']['scenario']).resolve()))
    if 'robot_models' in resolved['simulation']:
        resolved['simulation']['robot_models']={name:dict(model,robot_asset=str((config_path.parent/model['robot_asset']).resolve())) if model.get('robot_asset') else dict(model) for name,model in resolved['simulation']['robot_models'].items()}
    for model in resolved['simulation'].get('robot_models',{}).values():
        if model.get('controller_file'):model['controller_file']=str((config_path.parent/model['controller_file']).resolve())
    (out/'config.json').write_text(json.dumps(resolved,indent=2))
    from xrobocon.genesis_simulator import GenesisSimulator
    simulator=GenesisSimulator(resolved['simulation']['scenario'],participants[0],viewer,config['simulation']['torque_limit'],spawns=spawns,field_mesh_dir=out/'field-mesh' if config['simulation'].get('climb_assist') else None,climb_assist=config['simulation'].get('climb_assist',False),coins=mission['coins'],show_coins=config['simulation'].get('show_coins'),camera=config['simulation'].get('camera'),sensors=config.get('sensors'),robot_models=resolved['simulation'].get('robot_models'))
    import shutil,hashlib
    shutil.copyfile(simulator.robot.asset_path,out/'robot.xml')
    (out/'asset.json').write_text(json.dumps(dict(sha256=hashlib.sha256((out/'robot.xml').read_bytes()).hexdigest(),source=str(simulator.robot.asset_path)),indent=2))
    assets={}
    for name,robot in simulator.robots.items():
        filename=f'robot-{name}.xml';shutil.copyfile(robot.asset_path,out/filename)
        assets[name]=dict(file=filename,source=str(robot.asset_path),sha256=hashlib.sha256((out/filename).read_bytes()).hexdigest())
    (out/'robot-assets.json').write_text(json.dumps(assets,indent=2))
    manager=CompetitionManager(mission['coins'],participants,mission['time_limit'])
    from xrobocon.agent_factory import make_agent
    agents={name:make_agent(spec) for name,spec in config['agents'].items()}
    options=dict(config.get('transport',{'kind':'direct'}));kind=options.pop('kind')
    if kind not in ('direct','ros2'):raise ValueError('unknown transport kind')
    transport=None
    if kind=='ros2':
        from xrobocon.ros2_transport import Ros2Transport,RemoteRobotAgent
        try:transport=Ros2Transport(out/'config.json',out,options)
        except BaseException:simulator.stop();raise
        agents={name:RemoteRobotAgent(agent) for name,agent in agents.items()}
    runner=CompetitionRunner(simulator,manager,agents,config['dt'],transport=transport,recovery=mission.get('recovery',{'enabled':True,'timeout':20,'min_distance':.05,'min_rotation':.15}))
    return config,simulator,manager,agents,runner

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'configs/framework_flat.json'));p.add_argument('--out',required=True);p.add_argument('--viewer',action='store_true');args=p.parse_args()
    config,simulator,manager,agents,runner=build_runtime(args.config,args.out,args.viewer)
    out=Path(args.out)
    stable=0;status='time_limit'
    try:
        runner.start()
        with (out/'trace.jsonl').open('w') as trace:
            while manager.status=='running':
                began=time.perf_counter();row=runner.tick();trace.write(json.dumps(row)+'\n')
                stable=stable+1 if all(a.done for a in agents.values()) and all(v<.05 for v in row['speeds'].values()) else 0
                if config.get('stop_when_targets_collected',False) and stable>=100:status='targets_collected_and_stopped';break
                if args.viewer:time.sleep(max(0,config['dt']-(time.perf_counter()-began)))
    except KeyboardInterrupt:
        status='interrupted';raise
    except Exception as error:
        status='error';(out/'error.json').write_text(json.dumps(dict(type=type(error).__name__,message=str(error))));raise
    finally:
        runner.close()
        state=manager.snapshot();(out/'state.json').write_text(json.dumps(state,indent=2))
        (out/'summary.json').write_text(json.dumps(dict(status=status,elapsed=state['elapsed'],scores=state['scores'],collection_counts=state['collection_counts'],observation_source='ros2/truth_baseline' if runner.transport else 'truth_baseline',robot_contact_steps=simulator.robot_contact_steps,recovery_counts=state['recovery_counts'],recovery_blocked_seconds=state['recovery_blocked_seconds'],agent_checkpoints={p:getattr(a,'checkpoints',[]) for p,a in agents.items()}),indent=2))
    print(json.dumps(dict(status=status,elapsed=manager.elapsed,scores=manager.scores)),flush=True)

if __name__=='__main__':main()
