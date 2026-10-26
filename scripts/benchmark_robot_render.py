"""Short identical two-robot Genesis viewer workload; no ROS or agent timing."""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.genesis_simulator import GenesisSimulator
from xrobocon.navigation import VelocityCommand
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['simple','detailed'],required=True);p.add_argument('--out',required=True);p.add_argument('--steps',type=int,default=300);args=p.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    config=json.loads((ROOT/'configs/competition_strategy_adaptive_direct_data.json').read_text());spec=config['simulation']
    asset=ROOT/('xrobocon/assets/robot_component_r18.xml' if args.mode=='simple' else 'xrobocon/assets/visual/robot_component_r18_detailed.xml')
    start=time.perf_counter()
    sim=GenesisSimulator(str(ROOT/'configs'/spec['scenario']),viewer=True,torque_limit=6,spawns=spec['spawns'],field_mesh_dir=out/'field-mesh',climb_assist=True,coins=config['competition']['coins'],show_coins=True,robot_models={name:dict(robot_type='rocker_bogie',robot_asset=str(asset)) for name in spec['spawns']})
    build=time.perf_counter()-start
    try:
        sim.reset()
        # Validate imported Genesis inertial data, not just MuJoCo source.
        bodies={}
        for name,robot in sim.robots.items():
            link=robot.entity.get_link('base_link');desc=link.desc
            np.testing.assert_allclose(desc.inertial_pos,robot.base_com,atol=1e-8)
            bodies[name]=dict(base_com=list(map(float,desc.inertial_pos)),base_mass=float(desc.mass),collision_geoms=robot.entity.n_geoms)
        commands={name:VelocityCommand(.12,0) for name in sim.robots}
        for _ in range(50):sim.step(commands,.01)
        start=time.perf_counter()
        for _ in range(args.steps):
            sim.step(commands,.01)
            if sim.robot_markers:sim.robot_markers.sync()
        elapsed=time.perf_counter()-start
        report=dict(mode=args.mode,steps=args.steps,simulation_seconds=args.steps*.01,wall_seconds=elapsed,steps_per_second=args.steps/elapsed,build_seconds=build,imported=bodies,positions=sim.truth_positions(),scope='short viewer+physics+PID; no ROS2; warm-up excluded; one trial')
        (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    finally:sim.stop()
if __name__=='__main__':main()
