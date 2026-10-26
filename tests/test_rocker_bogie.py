"""
ロッカー・ボギー型ロボットのテストスクリプト
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import genesis as gs
import numpy as np
from xrobocon.robot_configs import get_robot_config
from xrobocon.field import XRoboconField
from xrobocon.robot import XRoboconRobot

def test_rocker_bogie():
    print("Initializing Genesis...")
    gs.init(backend=gs.gpu)
    
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2.0, -2.0, 1.0),
            camera_lookat=(0.0, 0.0, 0.2),
            camera_fov=40,
        ),
        rigid_options=gs.options.RigidOptions(
            dt=0.01,
            gravity=(0.0, 0.0, -9.8),
        ),
    )
    
    plane = scene.add_entity(gs.morphs.Plane())
    
    # テスト用段差 (高さ10cm)
    step = scene.add_entity(
        gs.morphs.Box(
            pos=(1.0, 0.0, 0.05),
            size=(0.5, 1.0, 0.1),
            fixed=True
        )
    )
    
    # ロボット作成
    robot = XRoboconRobot(
        scene,
        pos=(0.0, 0.0, 0.2),
        euler=(0, 0, 0),
        robot_type='rocker_bogie'
    )
    
    scene.build()
    robot.post_build()
    
    print("\nSimulation Start. The robot should climb the 10cm step.")
    
    for i in range(1000):
        # 単純に前進 (左右ともプラスのトルク)
        action = np.array([0.8, 0.8], dtype=np.float32)
        
        # 最大トルクを掛けて適用 (robot_configs.pyの定義値)
        torque = 40.0
        robot.set_actions(action * torque)
        
        scene.step()
        
        if i % 100 == 0:
            pos = robot.get_pos().cpu().numpy()
            print(f"Step {i}: Pos X={pos[0]:.2f}, Z={pos[2]:.2f}")

if __name__ == "__main__":
    test_rocker_bogie()