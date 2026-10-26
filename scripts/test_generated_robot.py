"""Isolated Genesis joint/motion smoke; not a claim of task or walking success."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.workshop.generated import load_bundle
from xrobocon.workshop.design import write_json
from xrobocon.navigation import VelocityCommand
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--directory',required=True)
    parser.add_argument('--out',required=True)
    parser.add_argument('--viewer',action='store_true')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args();directory=Path(args.directory).resolve()
    bundle=load_bundle(directory/'controller.json',directory/'model.xml')
    plan=dict(protocol_version=1,scope='joint control smoke, not locomotion performance certification',
        observation_source='direct diagnostic; production observations remain ROS2',joints=bundle['joints'])
    if args.dry_run:print(json.dumps(plan));return
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False);write_json(out/'plan.json',plan)
    from xrobocon.genesis_simulator import GenesisSimulator
    import numpy as np
    sim=None;trials=[];error=None
    try:
        sim=GenesisSimulator(str(ROOT/'configs/scenarios/passive_front_bogie_r18_steering_flat.json'),
            spawns={'self':dict(pos=[0,0,.65],euler=[0,0,0])},viewer=args.viewer,
            robot_models={'self':dict(robot_type='standard',robot_asset=str(directory/'model.xml'),drive_kind='program',controller_file=str(directory/'controller.json'))})
        sim.reset();drive=sim.drives['self']
        with (out/'joints.jsonl').open('w') as trace:
            for case,command in [('hold',VelocityCommand(0,0)),('forward',VelocityCommand(.08,0)),('turn',VelocityCommand(0,.2)),('mechanism',VelocityCommand(0,0,climb_stage=0))]:
                start=sim.robot.entity.get_dofs_position().cpu().numpy().copy()
                start_pose=sim.read_pose('self')
                max_tilt=0.;max_torque=0.;max_motion=0.;start_targets=None
                for step in range(200):
                    sim.step({'self':command},.01)
                    pos=sim.robot.entity.get_dofs_position().cpu().numpy()
                    if not np.isfinite(pos).all():raise ValueError('nonfinite physics state')
                    max_tilt=max(max_tilt,*map(abs,map(float,sim.robot.get_euler()[:2])))
                    max_torque=max(max_torque,*map(abs,drive.torques))
                    max_motion=max(max_motion,float(np.max(np.abs(pos[drive.indices]-start[drive.indices]))))
                    if start_targets is None:start_targets=list(drive.targets)
                    if step%10==0:
                        velocity=sim.robot.entity.get_dofs_velocity().cpu().numpy()
                        measurement=[float(pos[i] if spec['mode']=='position' else velocity[i]) for spec,i in zip(drive.specs,drive.indices)]
                        errors=[target-actual for target,actual in zip(drive.targets,measurement)]
                        record=dict(time=drive.time,case=case,step=step,command=asdict(command),joint_names=[s['name'] for s in drive.specs],modes=[s['mode'] for s in drive.specs],
                            position=pos[drive.indices].tolist(),velocity=velocity[drive.indices].tolist(),targets=list(drive.targets),tracking_errors=errors,
                            tracking_error_max=max(map(abs,errors)),torques=list(drive.torques),torque_limits=[c.limit for c in drive.controllers],
                            saturation=[abs(t)>=c.limit*.99 for t,c in zip(drive.torques,drive.controllers)],pose=asdict(sim.read_pose('self')),tilt=max_tilt)
                        trace.write(json.dumps(record,allow_nan=False)+'\n');trace.flush()
                end_pose=sim.read_pose('self')
                displacement=float(np.hypot(end_pose.x-start_pose.x,end_pose.y-start_pose.y))
                yaw_change=float(np.arctan2(np.sin(end_pose.yaw-start_pose.yaw),np.cos(end_pose.yaw-start_pose.yaw)))
                trials.append(dict(displacement=displacement,yaw_change=yaw_change,case=case,passed=max_tilt<90,simulation_seconds=2.,max_tilt_degrees=max_tilt,max_joint_motion=max_motion,max_torque=max_torque,targets=start_targets))
    except Exception as exc:error=dict(type=type(exc).__name__,message=str(exc))
    finally:
        if sim:
            sim.stop()
            if args.viewer:sim.scene.viewer.stop()
    complete=len(trials)==4
    passed=complete and error is None and all(t['passed'] for t in trials)
    report=dict(plan=plan,results=[dict(candidate='generated',status='smoke_only_performance_unrated',passed=passed,complete=complete,error=error,trials=trials)],error=error)
    write_json(out/'report.json',report)
    print(json.dumps(dict(output=str(out),passed=passed,error=error)))
    if not passed:raise SystemExit(1)
if __name__=='__main__':main()
