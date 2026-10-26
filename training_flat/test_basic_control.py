import time
import numpy as np
import xrobocon.common as common
import genesis as gs
from xrobocon.robot import XRoboconRobot
from xrobocon.field import XRoboconField

def test_basic_control():
    """
    Tri-starロボットの基本制御テスト (4D Action Space)
    各モーター群（フレーム、ホイール）が独立して制御できるか確認する
    """
    print("Initializing Genesis...")
    common.setup_genesis(backend=gs.gpu)
    
    scene = gs.Scene(
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(3.0, -3.0, 2.5),
            camera_lookat=(0.0, 0.0, 0.5),
            camera_fov=40,
        ),
        rigid_options=gs.options.RigidOptions(
            dt=0.01,
            gravity=(0.0, 0.0, -9.8),
        ),
        show_viewer=True,
    )
    
    # 地面
    plane = scene.add_entity(gs.morphs.Plane())
    
    # フィールド（参考用）
    field = XRoboconField()
    field.build(scene)
    
    print("Creating Robot...")
    robot = XRoboconRobot(
        scene,
        pos=(5.0, -1.0, 0.25), # 平地エリア (高めにスポーンして埋まり回避)
        euler=(0, 0, 90),
        robot_type='tristar'
    )
    
    scene.build()
    robot.post_build()
    
    # DOF名の確認
    print("\n" + "="*60)
    print("🔍 Robot DOF Analysis")
    print("="*60)
    # Genesis 0.3.x API check: entity might have .dofs or .joints
    # We'll try to iterate and print names if possible, or just print n_dofs
    print(f"Total DOFs: {robot.n_dofs}")
    
    # Try to access joint names if available (Genesis API specific)
    # Assuming standard MJCF loading preserves names
    try:
        # Note: This is a best-effort guess at the API for debugging
        # If this fails, we will rely on the structure observed in XML
        pass 
    except:
        pass
        
    print("="*60 + "\n")
    print("This script will test 4 control modes sequentially.")
    print("Please observe the robot's behavior in the viewer.")
    print("="*60 + "\n")
    
    # テストシーケンス
    sequences = [
        {
            "name": "Phase 1: Wheels Only (Forward)",
            "desc": "フレームは固定(0)、ホイールのみ前進(0.5)",
            "action": [0.0, 0.0, 0.5, 0.5],
            "steps": 300
        },
        {
            "name": "Phase 2: Wheels Only (Turn Right)",
            "desc": "フレームは固定(0)、ホイールで右旋回",
            "action": [0.0, 0.0, 0.5, -0.5],
            "steps": 300
        },
        {
            "name": "Phase 3: Frames Only (Rotate Forward)",
            "desc": "ホイールは停止(0)、フレームのみ回転(0.5)",
            "action": [0.5, 0.5, 0.0, 0.0],
            "steps": 300
        },
        {
            "name": "Phase 4: Combined (Climb Mode)",
            "desc": "フレームとホイールを同時に駆動",
            "action": [0.5, 0.5, 0.5, 0.5],
            "steps": 300
        },
        {
            "name": "Phase 5: Stop",
            "desc": "全モーター停止",
            "action": [0.0, 0.0, 0.0, 0.0],
            "steps": 100
        }
    ]
    
    for seq in sequences:
        print(f"\n▶ {seq['name']}")
        print(f"  {seq['desc']}")
        print(f"  Action: {seq['action']}")
        
        for i in range(seq['steps']):
            robot.set_actions(seq['action'])
            scene.step()
            
            # 10ステップごとにログ出力
            if i % 50 == 0:
                pos = robot.entity.get_pos()
                vel = robot.entity.get_vel()
                dofs_vel = robot.entity.get_dofs_velocity()
                
                # 速度の大きさ
                speed = np.linalg.norm(vel.cpu().numpy())
                
                # ホイールの回転速度 (左: 7,8,9, 右: 11,12,13)
                # 0-5: Free, 6: LFrame, 7-9: LWheels, 10: RFrame, 11-13: RWheels
                wheel_vel_l = dofs_vel[7].item() if len(dofs_vel) > 7 else 0
                wheel_vel_r = dofs_vel[11].item() if len(dofs_vel) > 11 else 0
                
                print(f"  Step {i}: Speed={speed:.4f} m/s, Pos={pos.cpu().numpy()}, WheelVel(L/R)={wheel_vel_l:.2f}/{wheel_vel_r:.2f}")
            
            time.sleep(0.01)
            
    print("\nTest Complete!")

if __name__ == "__main__":
    test_basic_control()
