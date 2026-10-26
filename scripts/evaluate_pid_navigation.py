"""Headless closed-loop PID trials, CSV trace and explicit success criteria."""
import argparse
import csv
import json
from math import hypot
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--backend', choices=['cpu', 'gpu'], default='cpu')
    p.add_argument('--robot', choices=['standard','tristar','tristar_large','rocker_bogie','rocker_bogie_large'], default=None)
    p.add_argument('--scenario')
    p.add_argument('--case', choices=['straight','turn','route'])
    p.add_argument('--wheel-torque', type=float, default=1)
    p.add_argument('--out', required=True)
    p.add_argument('--seconds', type=float, default=25)
    p.add_argument('--max-speed',type=float,default=.25)
    p.add_argument('--max-turn',type=float,default=.6)
    p.add_argument('--feedback-limit',type=float)
    p.add_argument('--telemetry', action='store_true')
    p.add_argument('--wheel-ki', type=float, default=.5)
    args = p.parse_args()
    from math import isfinite
    if not isfinite(args.wheel_torque) or args.wheel_torque <= 0: p.error('positive finite wheel torque required')
    if args.seconds <= 0: p.error('positive seconds required')
    out = Path(args.out); out.mkdir(parents=True, exist_ok=False)
    from xrobocon.scenario import load_scenario
    source = args.scenario or Path(__file__).resolve().parents[1]/'configs/scenarios/flat_baseline.json'
    specification = load_scenario(source)
    args.robot = args.robot or specification['robot_type']
    specification['robot_type'] = args.robot
    (out/'scenario.json').write_text(json.dumps(specification, indent=2))
    import genesis as gs
    import numpy as np
    from xrobocon.base_env import XRoboconBaseEnv
    from xrobocon.pid_drive import DifferentialPIDDrive, GroundTruthPose
    from xrobocon.navigation import WaypointFollower, VelocityCommand
    env = XRoboconBaseEnv(scenario_path=out/'scenario.json', backend=args.backend)
    results = []
    cases = [('straight', [(1,0)]), ('turn', [(0,1)]), ('route', [(0.6,0),(0.6,0.6),(0,0.6)])]
    if args.case: cases = [case for case in cases if case[0] == args.case]
    for name, points in cases:
        env.scene.reset()
        env.robot.set_pose((0,0,0.35), (0,0,0))
        drive = DifferentialPIDDrive(env.robot, torque_limit=args.wheel_torque,feedback_limit=args.feedback_limit, ki=args.wheel_ki)
        pose_source = GroundTruthPose(env.robot)
        for _ in range(150):
            drive.apply_velocity(VelocityCommand(0,0),0.01); env.scene.step()
        follower = WaypointFollower(points,max_speed=args.max_speed,max_turn=args.max_turn)
        settled_steps = 0; status = 'timeout'; max_tilt = 0; max_speed = 0
        telemetry = (out/f'{name}-contacts.jsonl').open('w') if args.telemetry else None
        with (out/f'{name}.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['step','x','y','yaw_rad','roll_deg','pitch_deg','speed','waypoint','v_cmd','w_cmd','wheel_targets','wheel_torques'])
            for step in range(int(args.seconds/0.01)):
                pose = pose_source.read_pose()
                command = follower.update(pose,0.01)
                drive.apply_velocity(command,0.01); env.scene.step()
                post = pose_source.read_pose(); euler = env.robot.get_euler()
                speed = float(np.linalg.norm(env.robot.get_vel().cpu().numpy()[:2]))
                tilt = float(max(abs(euler[0]),abs(euler[1])))
                max_tilt = max(max_tilt,tilt); max_speed = max(max_speed,speed)
                writer.writerow([step,post.x,post.y,post.yaw,euler[0],euler[1],speed,follower.index,command.linear,command.angular,json.dumps(drive.targets),json.dumps(drive.torques)])
                if telemetry and step % 10 == 0:
                    from xrobocon.contact_telemetry import read_contact_telemetry
                    record = read_contact_telemetry(env.robot, (step+1)*.01)
                    record.update(dof_positions=env.robot.entity.get_dofs_position().cpu().tolist(),
                                  dof_velocities=env.robot.entity.get_dofs_velocity().cpu().tolist(),
                                  wheel_indices=drive.indices, wheel_targets=drive.targets,
                                  wheel_torques=drive.torques)
                    telemetry.write(json.dumps(record)+'\n')
                if not np.isfinite([post.x,post.y,post.yaw,speed,tilt]).all():
                    status='nonfinite'; break
                if tilt > 60:
                    status='tipped'; break
                error = hypot(post.x-points[-1][0],post.y-points[-1][1])
                settled_steps = settled_steps+1 if follower.done and error <= 0.15 and speed < 0.05 else 0
                if settled_steps >= 100:
                    status='reached_and_stopped'; break
        if telemetry: telemetry.close()
        drive.stop()
        result = dict(case=name,status=status,steps=step+1,final_error=hypot(post.x-points[-1][0],post.y-points[-1][1]),max_tilt=max_tilt,max_speed=max_speed,final_speed=speed)
        results.append(result)
        (out/'summary.json').write_text(json.dumps(dict(robot=args.robot,max_speed=args.max_speed,max_turn=args.max_turn,wheel_ki=args.wheel_ki,steering_joints=len(drive.steering),feedback_limit=args.feedback_limit,wheel_torque=args.wheel_torque,seconds=args.seconds,backend=str(gs.backend),device=str(gs.device),radius=drive.radius,track=drive.track,trials=results),indent=2))
        print(json.dumps(result),flush=True)

if __name__ == '__main__': main()
