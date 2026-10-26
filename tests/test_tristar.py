"""
Tri-star ロボットのテストスクリプト
段差を配置して、ロボットの動作を確認
"""
import genesis as gs
from xrobocon.field import XRoboconField
from xrobocon.robot import XRoboconRobot
from xrobocon.game import XRoboconGame

def test_tristar_robot():
    """Tri-starロボットのテスト"""
    # Genesis初期化
    gs.init(backend=gs.gpu)
    
    scene = gs.Scene(
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2.0, -2.0, 1.5),
            camera_lookat=(0.5, 0.0, 0.1),
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
    
    # 段差を追加（テスト用）
    # 小段差: 3cm
    step_small = scene.add_entity(
        gs.morphs.Box(
            pos=(0.5, 0.0, 0.015),
            size=(0.5, 0.5, 0.03),
            fixed=True,
        ),
        material=gs.materials.Rigid(friction=1.0),
        surface=gs.surfaces.Default(color=(0.8, 0.8, 0.8))
    )
    
    # 中段差: 6cm
    step_medium = scene.add_entity(
        gs.morphs.Box(
            pos=(0.5, 0.8, 0.03),
            size=(0.5, 0.5, 0.06),
            fixed=True,
        ),
        material=gs.materials.Rigid(friction=1.0),
        surface=gs.surfaces.Default(color=(0.6, 0.6, 0.6))
    )
    
    # Tri-starロボット
    robot = XRoboconRobot(
        scene,
        pos=(0.0, 0.0, 0.08),
        euler=(0, 0, 0),
        robot_type='tristar'
    )
    
    # ゲームロジック
    game = XRoboconGame(field, robot)
    field.add_coin_spots(scene, game.spots)
    
    scene.build()
    robot.post_build()
    
    print("\n" + "="*70)
    print("🎮 Tri-star ロボットテスト")
    print("="*70)
    print("操作:")
    print("  W: 前進")
    print("  S: 後退")
    print("  A: 左回転")
    print("  D: 右回転")
    print("  Q: 終了")
    print("="*70 + "\n")
    
    # シミュレーション実行
    # 自動前進テスト
    print("自動前進テスト開始（10秒間）...")
    for i in range(1000):
        # 前進
        robot.set_wheel_torques(0.5, 0.5)
        scene.step()
    
    print("テスト完了！")

if __name__ == "__main__":
    test_tristar_robot()
