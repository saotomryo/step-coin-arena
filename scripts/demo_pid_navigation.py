"""Live CPU PID demonstration: three waypoints, real-time pacing, no RL model."""
import argparse
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['cpu','gpu'], default='cpu')
    parser.add_argument('--repeat', type=int, default=3)
    args = parser.parse_args()
    if args.repeat <= 0: parser.error('repeat must be positive')
    from xrobocon.common import setup_genesis
    import genesis as gs
    from xrobocon.robot import XRoboconRobot
    from xrobocon.pid_drive import DifferentialPIDDrive, GroundTruthPose
    from xrobocon.navigation import WaypointFollower, VelocityCommand
    setup_genesis(args.backend)
    points = [(0.6,0),(0.6,0.6),(0,0.6)]
    scene = gs.Scene(
        viewer_options=gs.options.ViewerOptions(camera_pos=(1.6,-1.5,1.7),
            camera_lookat=(0.3,0.3,0.05),camera_fov=40,refresh_rate=60),
        rigid_options=gs.options.RigidOptions(dt=0.01,gravity=(0,0,-9.8)),
        show_viewer=True)
    scene.add_entity(gs.morphs.Plane())
    for point,color in zip(points,[(1,0.3,0.1),(0.1,0.8,0.2),(0.2,0.4,1)]):
        scene.add_entity(gs.morphs.Sphere(pos=(*point,0.025),radius=0.025,
            fixed=True,collision=False),surface=gs.surfaces.Default(color=color))
    robot = XRoboconRobot(scene,robot_type='tristar',pos=(0,0,0.35))
    scene.build(); robot.post_build()
    for episode in range(args.repeat):
        scene.reset(); robot.set_pose((0,0,0.35),(0,0,0))
        drive = DifferentialPIDDrive(robot); pose_source = GroundTruthPose(robot)
        follower = WaypointFollower(points)
        print(f'DEMO {episode+1}/{args.repeat}: origin -> red -> green -> blue',flush=True)
        for step in range(150+2500):
            started = time.perf_counter()
            command = VelocityCommand(0,0) if step < 150 else follower.update(pose_source.read_pose(),0.01)
            drive.apply_velocity(command,0.01); scene.step()
            if step%100==0:
                pose=pose_source.read_pose()
                print(f't={step*0.01:.1f} waypoint={follower.index+1} x={pose.x:.3f} y={pose.y:.3f} done={follower.done}',flush=True)
            time.sleep(max(0,0.01-(time.perf_counter()-started)))
        print(f'DEMO_FINISHED reached={follower.done}',flush=True)

if __name__ == '__main__': main()
