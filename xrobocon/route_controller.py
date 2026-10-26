"""Fixed-drive route controller: ground -> alignment -> step -> platform.

Pose/range/height inputs currently come from truth; replace at the runner boundary.
Motor parameters stay in Drive and do not change with phases.
"""
from math import isfinite, pi
from xrobocon.navigation import WaypointFollower,VelocityCommand

class MixedRouteController:
    def __init__(self, start_height, step_height=0.05):
        if not all(isfinite(v) for v in (start_height,step_height)) or step_height <= 0:
            raise ValueError('finite heights and positive step height required')
        self.start_height=start_height;self.step_height=step_height
        self.phase='ground'
        self.ground=WaypointFollower([(-1,-.5),(-1,0),(-.55,0)],max_speed=.18)
        self.cross=WaypointFollower([(.45,0)],max_speed=.12)
        self.platform=WaypointFollower([(.75,0),(.9,.4)],max_speed=.18)
        self.on_top_verified=False

    def update(self, pose, height, roll, pitch, dt):
        if not all(isfinite(v) for v in (height,roll,pitch)):
            raise ValueError('finite posture required')
        if max(abs(roll),abs(pitch)) > 45:
            self.phase='abort_posture'
        if self.phase.startswith('abort') or self.phase=='done': return VelocityCommand(0,0)
        if self.phase=='ground':
            command=self.ground.update(pose,dt)
            if self.ground.done:self.phase='align'
            return command
        if self.phase=='align':
            error=(-pose.yaw+pi)%(2*pi)-pi
            if abs(error)<.05:self.phase='cross'
            return VelocityCommand(0,max(-.4,min(.4,2*error)))
        if self.phase=='cross':
            command=self.cross.update(pose,dt)
            atop=(pose.x>.3 and abs(height-self.start_height-self.step_height)<.03
                  and max(abs(roll),abs(pitch))<5)
            if self.cross.done and atop:
                self.on_top_verified=True;self.phase='platform'
            return command
        if self.phase=='platform':
            command=self.platform.update(pose,dt)
            if self.platform.done:self.phase='done'
            return command
        raise RuntimeError('unknown phase')
