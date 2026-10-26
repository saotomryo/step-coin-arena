"""
大型Tri-starロボットのテストスクリプト
ロボット設定の読み込みと物理シミュレーションの動作確認
"""
import genesis as gs
import numpy as np
from xrobocon.robot_configs import get_robot_config, list_robots, get_max_step_height
from xrobocon.field import XRoboconField
from xrobocon.robot import XRoboconRobot

def test_tristar_large():
    """大型Tri-starロボットのテスト"""
    
    # ロボット一覧表示
    list_robots()
    
    # 大型Tri-star設定取得
    config = get_robot_config('tristar_large')
    
    print("\n\n大型Tri-starロボットの詳細:")
    print("="*70)
    print(f"名前: {config['name']}")
    print(f"説明: {config['description']}")
    print(f"\n物理パラメータ:")
    print(f"  ベースサイズ: {config['physics']['base_size']}")
    print(f"  ホイール半径: {config['physics']['wheel_radius']*100:.1f}cm")
    print(f"  フレーム半径: {config['physics']['frame_radius']*100:.1f}cm")
    print(f"  総重量: {config['physics']['mass']:.1f}kg")
    print(f"\n登坂能力:")
    print(f"  最大登坂高さ: {config['capabilities']['max_step_height']*100:.1f}cm")
    print(f"  目標段差: {config['capabilities']['target_tier']}")
    print(f"\n制御パラメータ:")
    print(f"  アクション次元: {config['control']['action_space_dim']}D")
    print(f"  最大トルク: {config['control']['max_torque']}")
    print(f"\n開始位置:")
    print(f"  平地: Z={config['start_positions']['flat']['z_offset']*100:.1f}cm")
    print(f"  段差: Z={config['start_positions']['step']['z_offset']*100:.1f}cm")
    print("="*70)
    
    # Genesis初期化
    gs.init(backend=gs.gpu)
    
    # シーン作成
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(3.0, -3.0, 2.5),
            camera_lookat=(0.0, 0.0, 0.5),
            camera_fov=40,
        ),
        rigid_options=gs.options.RigidOptions(
            dt=0.01,
            gravity=(0.0, 0.0, -9.8),
        ),
    )
    
    # 地面
    plane = scene.add_entity(gs.morphs.Plane())
    
    # フィールド（段差）
    field = XRoboconField()
    field.build(scene)
    
    # 大型Tri-starロボット作成
    start_z = config['start_positions']['step']['z_offset']
    robot = XRoboconRobot(
        scene,
        pos=(5.5, 0.0, start_z),  # Tier 3の外側
        euler=(0, 0, 180),         # 中心向き
        robot_type='tristar_large'
    )
    
    # シーンビルド
    scene.build()
    robot.post_build()
    
    print("\n\nシミュレーション開始:")
    print("  ロボットは Tier 3 (10cm) の外側に配置されています")
    print("  静止状態で物理シミュレーションを実行します")
    print("  ロボットが安定して接地しているか確認してください")
    print("\nビューアを閉じると終了します...")
    
    # 静止状態で100ステップ実行
    for i in range(100):
        # 何もしない（静止）
        action = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        robot.set_actions(action)
        scene.step()
        
        if i % 20 == 0:
            pos = robot.get_pos().cpu().numpy()
            print(f"Step {i:3d}: Position = ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    print("\nテスト完了")
    print("ロボットが安定していれば、トレーニングに使用できます")

if __name__ == "__main__":
    test_tristar_large()
