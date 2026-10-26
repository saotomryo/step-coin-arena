"""CPU mixed-route validation with a fixed motor torque cap throughout."""
import argparse
import csv
import hashlib
import json
from math import hypot,isfinite
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--viewer',action='store_true')
    p.add_argument('--realtime',action='store_true')
    p.add_argument('--trials',type=int,choices=[1,3],default=3)
    p.add_argument('--out',required=True)
    p.add_argument('--backend',choices=['cpu','gpu'],default='cpu')
    p.add_argument('--seconds',type=float,default=120)
    args=p.parse_args()
    if not isfinite(args.seconds) or args.seconds<=0:p.error('positive seconds required')
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    from xrobocon.common import setup_genesis
    from xrobocon.scenario import load_scenario
    from xrobocon.robot import XRoboconRobot
    from xrobocon.pid_drive import DifferentialPIDDrive,GroundTruthPose
    from xrobocon.route_controller import MixedRouteController
    from xrobocon.navigation import VelocityCommand
    import genesis as gs
    import numpy as np
    setup_genesis(args.backend)
    root=Path(__file__).resolve().parents[1]
    spec=load_scenario(root/'configs/scenarios/rigid_sixwheel_flat.json')
    scene=gs.Scene(viewer_options=gs.options.ViewerOptions(camera_pos=(2.3,-3.5,2.4),camera_lookat=(0,0,.1),camera_fov=40,refresh_rate=60),rigid_options=gs.options.RigidOptions(dt=.01,gravity=(0,0,-9.8)),show_viewer=args.viewer)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Box(pos=(.75,0,.025),size=(1.5,2,.05),fixed=True),material=gs.materials.Rigid(friction=.8),surface=gs.surfaces.Default(color=(.55,.75,.9)))
    if args.viewer:
        for x,y,z,color in [(-1,-.5,.03,(1,.5,.1)),(-1,0,.03,(1,.5,.1)),(-.55,0,.03,(1,.5,.1)),
                            (.45,0,.08,(1,1,.1)),(.75,0,.08,(.1,.8,.2)),(.9,.4,.08,(.2,.4,1))]:
            scene.add_entity(gs.morphs.Sphere(pos=(x,y,z),radius=.035,fixed=True,collision=False),surface=gs.surfaces.Default(color=color))
    robot=XRoboconRobot(scene,robot_type=spec['robot_type'],asset_path=spec['robot_asset'],pos=(-1.4,-.5,.35))
    scene.build();robot.post_build()
    settings=dict(wheel_torque_limit=3.,kp=.3,ki=.5,kd=0.,radius=.08,track=.3)
    results=[]
    for yaw in (0,5,-5)[:args.trials]:
        scene.reset();robot.set_pose((-1.4,-.5,.35),(0,0,yaw))
        drive=DifferentialPIDDrive(robot,torque_limit=settings['wheel_torque_limit'])
        source=GroundTruthPose(robot)
        for _ in range(150):drive.apply_velocity(VelocityCommand(0,0),.01);scene.step()
        controller=MixedRouteController(float(robot.get_pos()[2]))
        phase_order=[];max_tilt=0.;max_speed=0.;max_torque=0.;stable=0;status='timeout'
        with (out/f'yaw{yaw}.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['step','phase','x','y','z','yaw_rad','roll','pitch','speed','v_cmd','w_cmd','torques','torque_limit'])
            for step in range(int(args.seconds/.01)):
                started=time.perf_counter()
                pose=source.read_pose();angles=robot.get_euler();pos=robot.get_pos().cpu().numpy()
                command=controller.update(pose,float(pos[2]),float(angles[0]),float(angles[1]),.01)
                drive.apply_velocity(command,.01);scene.step()
                post=source.read_pose();pos=robot.get_pos().cpu().numpy();angles=robot.get_euler()
                speed=float(np.linalg.norm(robot.get_vel().cpu().numpy()[:2]));tilt=float(max(abs(angles[0]),abs(angles[1])))
                max_tilt=max(max_tilt,tilt);max_speed=max(max_speed,speed);max_torque=max(max_torque,max(abs(v) for v in drive.torques))
                if not phase_order or phase_order[-1]!=controller.phase:
                    phase_order.append(controller.phase)
                    print(f'DEMO yaw={yaw} phase={controller.phase} t={step*.01:.1f}s',flush=True)
                w.writerow([step,controller.phase,*pos,post.yaw,*angles[:2],speed,command.linear,command.angular,json.dumps(drive.torques),settings['wheel_torque_limit']])
                error=hypot(post.x-.9,post.y-.4)
                if not np.isfinite([*pos,*angles,speed]).all():status='nonfinite';break
                if controller.phase.startswith('abort'):status=controller.phase;break
                if tilt>60:status='tipped';break
                stopped=(controller.phase=='done' and error<.15 and speed<.05 and controller.on_top_verified
                         and abs(pos[2]-controller.start_height-.05)<.03)
                stable=stable+1 if stopped else 0
                if args.realtime:time.sleep(max(0,.01-(time.perf_counter()-started)))
                if stable>=100:status='route_completed_and_stopped';break
        result=dict(start_yaw=yaw,status=status,steps=step+1,seconds=(step+1)*.01,phase_order=phase_order,
                    on_top_verified=controller.on_top_verified,final_position=pos.tolist(),final_error=error,
                    max_tilt=max_tilt,max_speed=max_speed,max_torque=max_torque)
        results.append(result)
        manifest=dict(device=str(gs.device),step_height=.05,motor_settings=settings,
                      asset_sha256=hashlib.sha256(Path(spec['robot_asset']).read_bytes()).hexdigest(),
                      note='truth pose; fixed torque cap, ideal torque source, no motor speed/thermal curve',trials=results)
        (out/'summary.json').write_text(json.dumps(manifest,indent=2))
        print(json.dumps(result),flush=True)

if __name__=='__main__':main()
