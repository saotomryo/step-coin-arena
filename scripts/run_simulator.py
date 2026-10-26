"""Standalone physics/viewer: no competition manager or robot agents."""
import argparse,json,time,sys
from dataclasses import asdict
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.genesis_simulator import GenesisSimulator

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',default='configs/competition_image.json')
    parser.add_argument('--out',required=True)
    parser.add_argument('--viewer',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--seconds',type=float,default=None)
    args=parser.parse_args()
    if args.seconds is not None and args.seconds<=0:parser.error('--seconds must be positive')
    path=Path(args.config).resolve();config=json.loads(path.read_text());spec=config['simulation']
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    simulator=GenesisSimulator(str((path.parent/spec['scenario']).resolve()),viewer=args.viewer,torque_limit=spec['torque_limit'],spawns=spec['spawns'],field_mesh_dir=out/'field-mesh' if spec.get('climb_assist') else None,climb_assist=spec.get('climb_assist',False),coins=config['competition']['coins'],show_coins=True,camera=spec.get('camera'),sensors=config.get('sensors'),robot_models={name:dict(model,robot_asset=str((path.parent/model['robot_asset']).resolve())) if model.get('robot_asset') else dict(model) for name,model in spec.get('robot_models',{}).items()})
    elapsed=0
    try:
        simulator.reset()
        while (args.seconds is None or elapsed<args.seconds) and (not args.viewer or simulator.scene.viewer.is_alive()):
            started=time.perf_counter();simulator.step_physics();elapsed+=.01
            if simulator.robot_markers:simulator.robot_markers.sync()
            if args.viewer:time.sleep(max(0,.01-(time.perf_counter()-started)))
    except KeyboardInterrupt:pass
    finally:
        simulator.stop()
        (out/'physics-state.json').write_text(json.dumps(dict(elapsed=elapsed,positions=simulator.truth_positions(),sensors={name:{kind:asdict(sample) for kind,sample in simulator.read_sensors(name).items()} for name in simulator.robots},mode='standalone_physics'),indent=2))
        print(f'Standalone physics completed: {elapsed:.2f}s',flush=True)

if __name__=='__main__':main()
