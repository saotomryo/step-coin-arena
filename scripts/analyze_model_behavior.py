import argparse
import time
import numpy as np
import torch
import genesis as gs
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from stable_baselines3 import PPO
from xrobocon.step_env import XRoboconStepEnv
from xrobocon.robot_configs import get_robot_config

def analyze_behavior(model_path, robot_type='tristar_large', env_type='step', episodes=1):
    """学習済みモデルの動作と報酬内訳を詳細に解析する"""
    
    print(f"\n{'='*80}")
    print(f"モデル動作解析: {model_path}")
    print(f"ロボット: {robot_type}, 環境: {env_type}")
    print(f"{'='*80}\n")
    
    # 環境作成
    env = XRoboconStepEnv(render_mode="human", robot_type=robot_type)
    
    # モデルロード
    try:
        model = PPO.load(model_path)
        print("モデル読み込み成功")
    except Exception as e:
        print(f"モデル読み込みエラー: {e}")
        return

    # ロボット設定取得（報酬重みなどの参照用）
    config = get_robot_config(robot_type)
    reward_params = config['reward_params']
    
    for ep in range(episodes):
        obs, info = env.reset()
        scenario_type = info.get('scenario_type', 'unknown')
        print(f"\nEpisode {ep+1} Start - Scenario: {scenario_type}")
        print(f"{'-'*100}")
        print(f"{'Step':>4} | {'Action (F_L, F_R, W_L, W_R)':>25} | {'Reward':>8} | {'Dist':>6} | {'Z-Vel':>6} | {'Frame':>6} | {'Notes'}")
        print(f"{'-'*100}")
        
        total_reward = 0
        terminated = False
        truncated = False
        step = 0
        
        while not (terminated or truncated):
            # アクション決定
            action, _ = model.predict(obs, deterministic=True)
            
            # 状態取得（ステップ前）
            prev_dist = env.prev_dist
            prev_height = env.prev_height
            
            # ステップ実行
            obs, reward, terminated, truncated, info = env.step(action)
            
            # 詳細データ取得
            robot_pos = env.robot.get_pos().cpu().numpy()
            
            # 速度取得（Tensor -> float変換）
            vel = env.robot.get_vel()
            if hasattr(vel, 'cpu'): vel = vel.cpu().numpy()
            z_vel = abs(vel[2])
            
            # フレーム角度
            frame_angles = env.robot.get_frame_angles()
            avg_frame = (frame_angles[0] + frame_angles[1]) / 2.0 if frame_angles else 0.0
            
            # ターゲット距離
            target_pos = np.array(env.current_target['pos'])
            dist = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # 報酬内訳の推定（env内部の値は直接取れないため再計算）
            # 1. 距離報酬
            dist_reward = (prev_dist - dist) * 100.0
            
            # 2. 高さ報酬
            z_diff = robot_pos[2] - prev_height
            height_reward = 0.0
            if z_diff > 0:
                h_weight = reward_params.get('height_gain_weight', 500.0)
                height_reward = z_diff * h_weight
                
            # 3. ペナルティ
            z_pen = 0.0
            if z_vel > 0.1:
                z_pen = -(z_vel - 0.1) * reward_params.get('z_velocity_penalty_weight', 0.0)
                
            frame_vel = abs(action[0]) + abs(action[1])
            f_pen = -frame_vel * reward_params.get('frame_velocity_penalty_weight', 0.0)
            
            # 表示（5ステップごと、または重要な変化があった時）
            if step % 10 == 0 or reward > 10.0 or reward < -5.0:
                act_str = f"[{action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f}, {action[3]:.2f}]"
                note = ""
                if dist_reward > 1.0: note += "前進 "
                if dist_reward < -1.0: note += "後退 "
                if height_reward > 1.0: note += "登坂 "
                if z_pen < -0.5: note += "JUMP! "
                
                print(f"{step:4d} | {act_str:>25} | {reward:8.2f} | {dist:6.2f} | {z_vel:6.2f} | {avg_frame:6.1f} | {note}")
                
                # 報酬内訳詳細表示
                if step % 50 == 0:
                    print(f"       Breakdown -> Dist: {dist_reward:.2f}, Height: {height_reward:.2f}, Z-Pen: {z_pen:.2f}, F-Pen: {f_pen:.2f}")

            total_reward += reward
            step += 1
            time.sleep(0.02) # 少しゆっくり
            
        print(f"{'-'*100}")
        print(f"Episode Finish. Total Reward: {total_reward:.2f}, Steps: {step}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='xrobocon_ppo_tristar_large_step.zip')
    parser.add_argument('--robot', type=str, default='tristar_large')
    args = parser.parse_args()
    
    analyze_behavior(args.model, args.robot)
