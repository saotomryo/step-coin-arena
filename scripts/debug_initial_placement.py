"""
ロボットの初期配置と接地状態をデバッグするスクリプト
"""
import genesis as gs
import numpy as np
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xrobocon.step_env import XRoboconStepEnv

def debug_initial_placement():
    """初期配置時のロボット状態を詳しく確認"""
    
    # 環境作成
    env = XRoboconStepEnv(render_mode="human", robot_type='tristar')
    
    print("\n" + "="*70)
    print("ロボット初期配置デバッグ")
    print("="*70)
    
    # リセット
    obs, info = env.reset()
    scenario_type = info.get('scenario_type', 'unknown')
    
    print(f"\nシナリオ: {scenario_type}")
    print(f"観測データ (最初の6要素 - 位置と姿勢):")
    print(f"  Position: ({obs[0]:.4f}, {obs[1]:.4f}, {obs[2]:.4f})")
    print(f"  Euler:    ({obs[3]:.4f}, {obs[4]:.4f}, {obs[5]:.4f})")
    
    # 最初の10ステップを詳細にログ
    print("\n" + "-"*70)
    print("最初の10ステップの詳細ログ")
    print("-"*70)
    print(f"{'Step':>4} | {'X':>8} | {'Y':>8} | {'Z':>8} | {'Roll':>8} | {'Pitch':>8} | {'Yaw':>8} | {'Reward':>10}")
    print("-"*70)
    
    # 静止アクション（何もしない）
    action = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    
    for step in range(10):
        obs, reward, terminated, truncated, _ = env.step(action)
        
        pos_x, pos_y, pos_z = obs[0], obs[1], obs[2]
        roll, pitch, yaw = obs[3], obs[4], obs[5]
        
        print(f"{step:4d} | {pos_x:8.4f} | {pos_y:8.4f} | {pos_z:8.4f} | {roll:8.2f} | {pitch:8.2f} | {yaw:8.2f} | {reward:10.2f}")
        
        if terminated or truncated:
            print(f"\n⚠️  エピソード終了 (Step {step})")
            break
    
    # 最終状態の詳細
    print("\n" + "-"*70)
    print("最終状態の詳細")
    print("-"*70)
    
    robot_pos = env.robot.get_pos().cpu().numpy()
    robot_euler = env.robot.get_euler()
    robot_vel = env.robot.get_vel().cpu().numpy()
    
    print(f"Position:        ({robot_pos[0]:.4f}, {robot_pos[1]:.4f}, {robot_pos[2]:.4f})")
    print(f"Euler (deg):     ({robot_euler[0]:.2f}, {robot_euler[1]:.2f}, {robot_euler[2]:.2f})")
    print(f"Velocity:        ({robot_vel[0]:.4f}, {robot_vel[1]:.4f}, {robot_vel[2]:.4f})")
    print(f"Speed (XY):      {np.linalg.norm(robot_vel[:2]):.4f} m/s")
    print(f"Speed (Z):       {robot_vel[2]:.4f} m/s")
    
    print("\n" + "="*70)
    print("デバッグ完了")
    print("="*70)
    print("\n推奨される開始高さ:")
    print("  - Z=0.05: ホイールが地面に接地")
    print("  - Z=0.10: 少し浮いた状態（ジャンプの原因）")
    print("  - Z=0.03: 地面に埋もれる（物理エラーの原因）")
    print("\n現在の設定を確認し、必要に応じて調整してください。")

if __name__ == "__main__":
    debug_initial_placement()
