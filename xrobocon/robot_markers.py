"""Robot identity markers: rendering only, no bodies, mass or collision shapes."""
import numpy as np
COLORS={'self':(0.1,0.5,1.0,1.0),'opponent':(1.0,0.15,0.15,1.0)}
class RobotMarkers:
    def __init__(self,scene,robots):
        self.scene=scene;self.robots=robots;self.nodes={}
        for name,robot in robots.items():
            self.nodes[name]=scene.draw_debug_sphere(self.position(robot),radius=.13,color=COLORS.get(name,(.9,.75,.1,1)))
    @staticmethod
    def position(robot):
        pos=np.asarray(robot.get_pos().cpu().tolist(),dtype=float)
        return pos+np.array([0,0,.28])
    def sync(self):
        poses=[]
        for robot in self.robots.values():
            pose=np.eye(4);pose[:3,3]=self.position(robot);poses.append(pose)
        self.scene.update_debug_objects(tuple(self.nodes.values()),tuple(poses))
