"""Isolated perpendicular step trials: locked frame versus fixed frame sequence."""
import argparse
import csv
import json
from math import hypot, pi, isfinite
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--start-yaw', type=float, default=0)
    p.add_argument('--robot', choices=['tristar','rocker_bogie','rocker_bogie_large'], default='tristar')
    p.add_argument('--wheel-torque', type=float, default=1)
    p.add_argument('--wheel-radius', type=float, default=0.06)
    p.add_argument('--rigid-suspension', action='store_true', help='generate a level six-wheel rigid variant')
    p.add_argument('--front-support', action='store_true')
    p.add_argument('--out',required=True)
    p.add_argument('--height',type=float,default=0.1)
    p.add_argument('--mode', choices=['guarded_sequence','wheel_only','frame_sequence'], default='guarded_sequence')
    p.add_argument('--frame-kp', type=float, default=20)
    p.add_argument('--frame-torque', type=float, default=3)
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--variant', choices=['baseline','low_wheel_torque','wheel_stop_during_rotation','reverse_frame','free_wheels','reverse_free_wheels'])
    p.add_argument('--rear-support', action='store_true', help='test a separate rear support sphere asset')
    p.add_argument('--sweep', action='store_true', help='compare six drive allocations on the same step')
    p.add_argument('--backend',choices=['cpu','gpu'],default='cpu')
    args=p.parse_args()
    if any(not isfinite(v) or v <= 0 for v in (args.seconds,args.frame_kp,args.frame_torque,args.wheel_torque,args.wheel_radius)): p.error('positive limits required')
    if not isfinite(args.start_yaw) or abs(args.start_yaw)>180: p.error('yaw must be finite within +/-180 degrees')
    if args.variant and not args.sweep: p.error('variant requires sweep')
    if args.wheel_radius != .06 and not args.rigid_suspension: p.error('radius override requires rigid suspension')
    if args.rigid_suspension and not .06 <= args.wheel_radius <= .08: p.error('radius must be in [.06,.08] to avoid wheel overlap')
    if args.rigid_suspension and args.robot != 'rocker_bogie': p.error('rigid suspension requires rocker_bogie')
    if args.robot != 'tristar' and (args.mode != 'wheel_only' or args.rear_support or args.front_support): p.error('rocker model uses wheel_only without added supports')
    if args.sweep and args.mode != 'guarded_sequence': p.error('sweep requires guarded_sequence')
    if not 0 < args.height <= 0.3: p.error('height must be in (0,0.3]')
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    from xrobocon.common import setup_genesis
    import genesis as gs
    import numpy as np
    from xrobocon.robot import XRoboconRobot
    from xrobocon.pid_drive import DifferentialPIDDrive,GroundTruthPose
    from xrobocon.navigation import WaypointFollower,VelocityCommand
    from xrobocon.step_controller import GuardedStepController,StepObservation
    setup_genesis(args.backend)
    scene=gs.Scene(rigid_options=gs.options.RigidOptions(dt=0.01,gravity=(0,0,-9.8)),show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Box(pos=(0.75,0,args.height/2),size=(1.5,2,args.height),fixed=True),material=gs.materials.Rigid(friction=0.8))
    asset_path = None
    if args.rear_support or args.front_support:
        import xml.etree.ElementTree as ET
        asset = ET.parse(Path(__file__).resolve().parents[1]/'xrobocon/assets/robot_tristar.xml')
        base = asset.getroot().find(".//body[@name='base_link']")
        for enabled, name, pos in [(args.rear_support,'rear_support','-0.22 0 -0.095'),
                                   (args.front_support,'front_support','0.06 0 -0.095')]:
            if enabled:
                support = ET.SubElement(base, 'body', name=name, pos=pos)
                ET.SubElement(support, 'geom', type='sphere', size='0.025', mass='0.05', friction='0.1 0.005 0.0001', rgba='0.2 0.7 0.2 1')
        asset_path = out/'robot_tristar_support.xml'
        asset.write(asset_path)
    if args.rigid_suspension:
        import xml.etree.ElementTree as ET
        asset = ET.parse(Path(__file__).resolve().parents[1]/'xrobocon/assets/robot_rocker_bogie.xml')
        for body in asset.getroot().iter('body'):
            for joint in list(body.findall('joint')):
                if 'rocker_joint' in joint.get('name','') or 'bogie_joint' in joint.get('name',''):
                    body.remove(joint)
            if body.get('name') in ('left_wheel_rear','right_wheel_rear'):
                xyz = body.get('pos').split(); xyz[2] = '-0.07'; body.set('pos',' '.join(xyz))
        for body in asset.getroot().iter('body'):
            if '_wheel_' in body.get('name',''):
                geom = body.find('geom'); size = geom.get('size').split()
                size[0] = str(args.wheel_radius); geom.set('size',' '.join(size))
        asset_path = out/'robot_rigid_sixwheel.xml' 
        asset.write(asset_path)
    robot=XRoboconRobot(scene,asset_path=asset_path,pos=(-0.6,0,0.35),robot_type=args.robot);scene.build();robot.post_build()
    results=[]
    variants = [('baseline', args.wheel_torque, .03, 1, False)]
    if args.sweep:
        variants += [('low_wheel_torque', .3, .03, 1, False),
                     ('wheel_stop_during_rotation', 1.0, 0.0, 1, False),
                     ('reverse_frame', 1.0, .03, -1, False),
                     ('free_wheels', 1.0, 0.0, 1, True),
                     ('reverse_free_wheels', 1.0, 0.0, -1, True)]
    if args.variant: variants = [v for v in variants if v[0] == args.variant]
    for label, torque_limit, climb_speed, frame_direction, free_wheels in variants:
        mode = args.mode
        scene.reset();robot.set_pose((-0.6,0,0.35),(0,0,args.start_yaw))
        drive=DifferentialPIDDrive(robot, torque_limit=torque_limit,frame_kp=args.frame_kp,frame_torque_limit=args.frame_torque);pose_source=GroundTruthPose(robot)
        for _ in range(150): drive.apply_velocity(VelocityCommand(0,0),0.01);scene.step()
        base_z=float(robot.get_pos()[2]);follower=WaypointFollower([(0.6,0)],max_speed=0.12)
        guard=GuardedStepController(climb_speed=climb_speed, frame_direction=frame_direction)
        phase='approach';angle=0.;stopped=0;max_tilt=0.;max_x=-0.6;max_z=base_z;status='timeout'
        with (out/f'{label}-{mode}.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['step','phase','x','y','z','roll','pitch','yaw','speed','frame_target','left_frame','right_frame'])
            for step in range(int(args.seconds/.01)):
                pose=pose_source.read_pose();command=follower.update(pose,0.01)
                if mode=='guarded_sequence':
                    before=robot.get_pos().cpu().numpy();angles=robot.entity.get_dofs_position().cpu().numpy()[drive.frames]
                    from math import radians,cos,sin
                    yaw=pose.yaw;vel=robot.get_vel().cpu().numpy()
                    # Known-edge baseline: range is analytical, NOT a simulated lidar reading.
                    observation=StepObservation(-pose.x,-yaw,float(vel[0]*cos(yaw)+vel[1]*sin(yaw)),
                        radians(float(robot.get_euler()[1])),float(angles[0]),float(angles[1]),
                        bool(before[0]>.3 and abs(before[2]-(base_z+args.height))<.05))
                    command=guard.update(observation,.01);phase=guard.phase;angle=guard.frame_target
                    if phase=='recover':command=follower.update(pose,.01)
                if mode=='frame_sequence'  and phase=='approach' and pose.x > -0.20: phase='climb'
                if mode=='frame_sequence' and phase=='climb':
                    angle=min(2*pi/3,angle+0.5*0.01)
                    if angle>=2*pi/3: phase='recover'
                # Hold the requested angle; this is only a first fixed sequence, not a validated gait.
                drive.wheels_enabled = not (free_wheels and phase == 'climb')
                drive.set_frame_target(angle);drive.apply_velocity(command,0.01);scene.step()
                pos=robot.get_pos().cpu().numpy();euler=robot.get_euler();speed=float(np.linalg.norm(robot.get_vel().cpu().numpy()[:2]))
                frame=robot.entity.get_dofs_position().cpu().numpy()[drive.frames]
                tilt=float(max(abs(euler[0]),abs(euler[1])));max_tilt=max(max_tilt,tilt);max_x=max(max_x,float(pos[0]));max_z=max(max_z,float(pos[2]))
                writer.writerow([step,phase,*pos,*euler,speed,angle,*frame])
                if not np.isfinite([*pos,*euler,speed]).all():status='nonfinite';break
                if phase.startswith('abort'):
                    status=phase;break
                if tilt>60:status='tipped';break
                atop=pos[0]>0.3 and abs(pos[2]-(base_z+args.height))<0.05
                stopped=stopped+1 if follower.done and hypot(pos[0]-.6,pos[1])<.15 and atop and speed<.05 else 0
                if stopped>=100:status='climbed_and_stopped';break
        results.append(dict(label=label,free_wheels=free_wheels,wheel_torque_limit=torque_limit,climb_speed=climb_speed,frame_direction=frame_direction,mode=mode,status=status,steps=step+1,max_tilt=max_tilt,max_x=max_x,max_z=max_z,final_position=pos.tolist(),base_z=base_z,frame_target=angle,frame_actual=frame.tolist(),contact_inferred=guard.contact_inferred))
        (out/'summary.json').write_text(json.dumps(dict(height=args.height,start_yaw=args.start_yaw,robot=args.robot,rear_support=args.rear_support,front_support=args.front_support,rigid_suspension=args.rigid_suspension,device=str(gs.device),wheel_radius=drive.radius,wheel_torque_limit=args.wheel_torque,frame_torque_limit=args.frame_torque,frame_kp=args.frame_kp,seconds=args.seconds,frame_speed=.15,trials=results),indent=2))
        print(json.dumps(results[-1]),flush=True)

if __name__=='__main__':main()
