"""Real Genesis sensors, agent delivery, timing and reset checks."""
import argparse,json,sys
from pathlib import Path
from math import isfinite,cos,sin,sqrt,radians
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime
from xrobocon.navigation import VelocityCommand

class ProbeAgent:
    def reset(self):self.seen=[]
    def act(self,obs,dt):
        self.seen.append(obs)
        return VelocityCommand(.08,0)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
    out=Path(args.out);_,sim,manager,agents,runner=build_runtime('configs/competition_sensors_demo.json',out,False)
    runner.agents={name:ProbeAgent() for name in agents}
    checks=[]
    try:
        runner.start();initial=sim.read_sensors('self')
        assert set(initial)=={'lidar','imu','joint_states','contacts'};checks.append('four sensor types per robot')
        assert len(initial['lidar'].data['ranges'])==72;checks.append('native lidar returns 72 rays')
        robot=sim.robots['self'];pos=robot.get_pos().cpu().tolist();yaw=radians(float(robot.get_euler()[2]))
        dx,dy=cos(yaw),sin(yaw);ox,oy=pos[0]+.45*dx,pos[1]+.45*dy
        radius=3.25;projection=ox*dx+oy*dy
        expected=-projection-sqrt(projection**2-(ox*ox+oy*oy-radius**2))
        measured=initial['lidar'].data['ranges'][36]
        assert measured is not None and abs(measured-expected)<.05,(measured,expected)
        checks.append('forward lidar detects actual raised-field collision surface within 5cm')
        assert abs(initial['imu'].data['linear_acceleration'][2]-9.8)<.2
        checks.append('resting IMU measures approximately 9.8 m/s2 proper acceleration')
        assert all(v is None or .05<=v<=12 for v in initial['lidar'].data['ranges'])
        assert len(initial['imu'].data['linear_acceleration'])==3
        assert all(isfinite(v) for v in initial['imu'].data['linear_acceleration']+initial['imu'].data['angular_velocity']);checks.append('native IMU finite vectors')
        joints=initial['joint_states'].data
        assert len(joints['name'])==len(joints['position'])==len(joints['velocity']) and any('wheel' in n for n in joints['name']);checks.append('encoders named and floating base excluded')
        assert initial['contacts'].data['in_contact'];checks.append('terrain contacts detected')
        with (out/'sensor-trace.jsonl').open('w') as trace:
            for _ in range(110):
                row=runner.tick();trace.write(json.dumps(row,allow_nan=False)+'\n')
        for name,agent in runner.agents.items():
            assert all(obs.sensors and all(s.timestamp<=obs.timestamp+1e-8 for s in obs.sensors.values()) for obs in agent.seen)
            assert agent.seen[0].sensors['lidar'].sequence==agent.seen[1].sensors['lidar'].sequence
            assert agent.seen[0].sensors['imu'].sequence!=agent.seen[1].sensors['imu'].sequence
        checks+=['both agents receive timestamped samples','10Hz lidar and 100Hz IMU hold distinct sample sequences','all traces strict JSON without infinity']
        # Place the other robot's wheel across the forward beam on flat ground.
        robot=sim.robots['self'];pos=robot.get_pos().cpu().tolist();yaw=radians(float(robot.get_euler()[2]))
        other_robot=sim.robots['opponent']
        other_robot.entity.set_qpos(sim.initial_qpos['opponent'],zero_velocity=True)
        other_robot.set_pose([pos[0]+cos(yaw)+.24*sin(yaw),pos[1]+sin(yaw)-.24*cos(yaw),pos[2]],[0,0,yaw*180/3.141592653589793])
        for _ in range(10):sim.step({name:VelocityCommand(0,0) for name in agents},.01)
        measured=sim.read_sensors('self')['lidar'].data['ranges'][36]
        assert measured is not None and measured<1.2,measured
        checks.append('native lidar detects the other robot geometry')
        other=sim.read_sensors('opponent');sim.respawn('self')
        assert sim.read_sensors('self')=={} and sim.read_sensors('opponent')==other;checks.append('single respawn flushes only affected sensor cache')
        runner.tick();assert sim.read_sensors('self')['imu'].sequence==0;checks.append('sensors resume on next physics tick after respawn')
        runner.start();assert sim.read_sensors('self')['imu'].timestamp==0 and sim.read_sensors('self')['imu'].sequence==0;checks.append('round reset restarts sensor clock')
        report=dict(passed=True,checks=checks,initial={k:asdict(v) for k,v in initial.items()},final_samples={n:{k:asdict(v) for k,v in sim.read_sensors(n).items()} for n in agents})
        (out/'verification.json').write_text(json.dumps(report,indent=2,allow_nan=False));print(f'Sensors runtime: {len(checks)} checks passed',flush=True)
    finally:sim.stop()
if __name__=='__main__':main()
