"""
段差乗り越えシナリオの可視化スクリプト
各シナリオでのロボット開始位置とターゲット位置を視覚的に確認
"""
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import xrobocon.common as common
import genesis as gs
import numpy as np
import time
from xrobocon.field import XRoboconField
from xrobocon.robot import XRoboconRobot
from xrobocon.game import XRoboconGame

def visualize_scenario(scenario_name, start_pos, start_euler, target_pos):
    """特定のシナリオを可視化"""
    print(f"\n{'='*70}")
    print(f"📍 シナリオ: {scenario_name}")
    print(f"{'='*70}")
    print(f"  開始位置: ({start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f})")
    print(f"  開始姿勢: Yaw={start_euler[2]:.1f}°")
    print(f"  目標位置: ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})")
    
    # 距離計算
    dist = np.linalg.norm(np.array(start_pos[:2]) - np.array(target_pos[:2]))
    print(f"  目標距離: {dist:.2f}m")
    print(f"{'='*70}\n")
    
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
    
    # フィールド
    field = XRoboconField()
    field.build(scene)
    
    # ロボット（開始位置）
    robot = XRoboconRobot(scene, pos=start_pos, euler=start_euler, robot_type='tristar')
    
    # ゲームロジック
    game = XRoboconGame(field, robot)
    field.add_coin_spots(scene, game.spots)
    
    # ターゲットマーカー（赤い球）
    target_marker = scene.add_entity(
        gs.morphs.Sphere(
            pos=target_pos,
            radius=0.15,
            fixed=True,
        ),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(1.0, 0.0, 0.0))  # 赤色
    )
    
    scene.build()
    robot.post_build()
    
    print("🎮 シミュレーション開始（10秒間表示）")
    print("   - 青いロボット: 開始位置")
    print("   - 赤い球: 目標位置")
    print("   - ウィンドウを閉じるか、10秒経過で次のシナリオへ\n")
    
    # 10秒間表示
    for i in range(1000):  # 10秒 @ 100Hz
        scene.step()
        time.sleep(0.01)
    
    print("✅ シナリオ表示完了\n")

def main():
    """全シナリオを順番に表示"""
    print("\n" + "="*70)
    print("🎯 ステップコイン・アリーナ 訓練シナリオ可視化 (Phase 3-2b: 段差乗り越え)")
    print("="*70)
    print("\n各シナリオを10秒ずつ表示します。")
    print("ウィンドウを閉じると次のシナリオに進みます。\n")
    
    # Genesis初期化 (共通モジュールを使用)
    common.setup_genesis()
    
    scenarios = [
        {
            'name': 'Scenario 1: 正面段差登坂 (Ground -> Tier 3)',
            'start_pos': (5.5, 0.0, 0.25), # Tier 3 (R=4.65) の外側
            'start_euler': (0, 0, 180),    # 中心方向
            'target_pos': (4.0, 0.0, 0.15), # Tier 3の上 (Z=0.1)
        },
        {
            'name': 'Scenario 2: 2段目登坂 (Tier 3 -> Tier 2)',
            'start_pos': (4.0, 0.0, 0.15), # Tier 3の上 (R=4.0)
            'start_euler': (0, 0, 180),    # 中心方向
            'target_pos': (2.5, 0.0, 0.4), # Tier 2の上 (Z=0.35 + マージン)
        },
    ]
    
    for i, scenario in enumerate(scenarios, 1):
        print(f"\n{'#'*70}")
        print(f"  シナリオ {i}/{len(scenarios)}")
        print(f"{'#'*70}")
        
        try:
            visualize_scenario(
                scenario['name'],
                scenario['start_pos'],
                scenario['start_euler'],
                scenario['target_pos']
            )
        except Exception as e:
            print(f"⚠️  エラー: {e}")
            print("次のシナリオに進みます...\n")
            continue
    
    print("\n" + "="*70)
    print("✅ 全シナリオの表示が完了しました")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
