"""
Tri-star専用報酬システムのテスト
フレーム角度報酬とエッジ接近報酬の動作を確認
"""
import genesis as gs
import numpy as np
from xrobocon.step_env import XRoboconStepEnv
from xrobocon.robot_configs import get_robot_config

def test_specialized_rewards():
    """専用報酬システムのテスト"""
    
    print("\n" + "="*70)
    print("Tri-star Large 専用報酬システムのテスト")
    print("="*70)
    
    # ロボット設定確認
    config = get_robot_config('tristar_large')
    print("\nロボット設定:")
    print(f"  名前: {config['name']}")
    print(f"  専用報酬使用: {config['reward_params']['use_specialized_rewards']}")
    print(f"  高さ報酬重み: {config['reward_params']['height_gain_weight']}")
    print(f"  フレーム角度報酬重み: {config['reward_params']['frame_angle_weight']}")
    print(f"  目標フレーム角度(近): {config['reward_params']['target_frame_angle_near']}度")
    print(f"  目標フレーム角度(遠): {config['reward_params']['target_frame_angle_far']}度")
    
    # 環境作成
    env = XRoboconStepEnv(render_mode="human", robot_type='tristar_large')
    obs, info = env.reset()
    
    print(f"\nシナリオ: {info.get('scenario_type', 'unknown')}")
    print("\n手動制御テスト:")
    print("  アクション: [frame_L, frame_R, wheel_L, wheel_R]")
    print("  フレームを前傾させて段差に接近する動作を確認")
    
    # テストシーケンス (各フェーズを長くする)
    test_actions = [
        # Step 1-30: 前進のみ（フレーム水平）
        *[np.array([0.0, 0.0, 0.3, 0.3], dtype=np.float32) for _ in range(30)],
        
        # Step 31-60: フレーム前傾 + 前進
        *[np.array([0.5, 0.5, 0.3, 0.3], dtype=np.float32) for _ in range(30)],
        
        # Step 61-90: フレーム維持 + 前進
        *[np.array([0.0, 0.0, 0.3, 0.3], dtype=np.float32) for _ in range(30)],
    ]
    
    print("\n" + "-"*70)
    print(f"{'Step':>4} | {'Pos_X':>7} | {'Pos_Z':>7} | {'Frame':>7} | {'Reward':>10} | {'Dist':>7}")
    print("-"*70)
    
    import time
    
    for i, action in enumerate(test_actions):
        obs, reward, terminated, truncated, _ = env.step(action)
        
        # 状態取得
        robot_pos = env.robot.get_pos().cpu().numpy()
        frame_angles = env.robot.get_frame_angles()
        avg_frame = (frame_angles[0] + frame_angles[1]) / 2.0 if frame_angles else 0.0
        
        target_pos = np.array(env.current_target['pos'])
        dist = np.linalg.norm(robot_pos[:2] - target_pos[:2])
        
        if i % 5 == 0:  # 5ステップごとに表示
            print(f"{i:4d} | {robot_pos[0]:7.3f} | {robot_pos[2]:7.3f} | {avg_frame:7.1f} | {reward:10.2f} | {dist:7.3f}")
        
        # ゆっくり表示 (0.05秒待機)
        time.sleep(0.05)
        
        if terminated or truncated:
            print(f"\nエピソード終了 (Step {i})")
            break
    
    print("-"*70)
    print("\nテスト完了")
    print("="*70)
    print("\n観察ポイント:")
    print("  1. フレームを前傾させた時（Step 11-20）に報酬が変化するか")
    print("  2. 段差に近づくにつれて報酬が増加するか")
    print("  3. 高さを獲得した時に大きな報酬が得られるか")

if __name__ == "__main__":
    test_specialized_rewards()
