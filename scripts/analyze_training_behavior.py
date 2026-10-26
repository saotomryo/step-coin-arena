import os
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
import sys
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import gymnasium as gym
import numpy as np
import torch
import logging
import genesis as gs
import xrobocon.common as common
from stable_baselines3 import PPO
from xrobocon.env import XRoboconEnv
import pandas as pd

def analyze_behavior(model_path, episodes=3):
    """
    訓練済みモデルの挙動を詳細に解析する
    """
    print(f"Analyzing model: {model_path}")
    
    # Genesis初期化とログレベル抑制
    try:
        gs.init(backend=gs.gpu, logging_level='warning')
    except Exception as e:
        print(f"Genesis init warning: {e}")
    
    # 環境作成 (可視化なしで高速に実行)
    env = XRoboconEnv(render_mode=None, robot_type='tristar')
    
    # モデル読み込み
    model = common.load_trained_model(model_path, env)
    
    all_logs = []
    
    # レポート出力用バッファ
    report_lines = []
    def log_print(text):
        print(text)
        report_lines.append(text)
    
    for ep in range(episodes):
        obs, _ = env.reset()
        done = False
        step = 0
        
        log_print(f"\n--- Episode {ep+1} ---")
        log_print(f"Start Pos: {env.robot.get_pos().cpu().numpy()}")
        log_print(f"Target: {env.current_target['pos']}")
        
        while not done:
            # アクション決定
            action, _ = model.predict(obs, deterministic=True)
            
            # 状態取得 (Step前)
            pos_prev = env.robot.get_pos().cpu().numpy()
            dist_prev = np.linalg.norm(pos_prev[:2] - np.array(env.current_target['pos'])[:2])
            
            # Step実行
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            # 状態取得 (Step後)
            pos_curr = env.robot.get_pos().cpu().numpy()
            dist_curr = np.linalg.norm(pos_curr[:2] - np.array(env.current_target['pos'])[:2])
            dist_delta = dist_prev - dist_curr # 正なら近づいている
            
            vel = env.robot.get_vel().cpu().numpy()
            speed = np.linalg.norm(vel)
            euler = env.robot.get_euler()
            
            # ログ記録
            log = {
                'episode': ep,
                'step': step,
                'reward': reward,
                'dist': dist_curr,
                'dist_delta': dist_delta,
                'speed': speed,
                'action_frame_l': action[0],
                'action_frame_r': action[1],
                'action_wheel_l': action[2],
                'action_wheel_r': action[3],
                'roll': euler[0],
                'pitch': euler[1],
                'yaw': euler[2]
            }
            all_logs.append(log)
            
            if step % 100 == 0:
                print(f"Step {step}: Reward={reward:.4f}, Dist={dist_curr:.4f}, Speed={speed:.4f}, Action={action}")
            
            step += 1
            
    env.close()
    
    # データ分析
    df = pd.DataFrame(all_logs)
    
    log_print("\n" + "="*60)
    log_print("📊 Analysis Report")
    log_print("="*60)
    
    log_print(f"Total Steps Analyzed: {len(df)}")
    log_print(f"Average Reward per Step: {df['reward'].mean():.4f}")
    log_print(f"Average Speed: {df['speed'].mean():.4f} m/s")
    log_print(f"Average Distance Delta (Progress): {df['dist_delta'].mean():.6f} m/step")
    
    log_print("\n--- Action Statistics ---")
    log_print(str(df[['action_frame_l', 'action_frame_r', 'action_wheel_l', 'action_wheel_r']].describe()))
    
    log_print("\n--- Correlation with Reward ---")
    log_print(str(df.corr()['reward'].sort_values(ascending=False)))
    
    # 問題点の診断
    log_print("\n--- Diagnosis ---")
    if df['speed'].mean() < 0.05:
        log_print("⚠️  Robot is moving too slowly. Check friction, torque limits, or action magnitude.")
    
    if df['dist_delta'].mean() <= 0:
        log_print("⚠️  Robot is NOT moving towards the target on average.")
    else:
        log_print("✅  Robot is moving towards the target on average.")
        
    if df['action_wheel_l'].abs().mean() < 0.1 and df['action_wheel_r'].abs().mean() < 0.1:
        log_print("⚠️  Wheel actions are very small. Agent might be afraid to move.")

    # レポート保存
    with open("analysis_report.txt", "w") as f:
        f.write("\n".join(report_lines))
    print(f"\nAnalysis report saved to analysis_report.txt")

if __name__ == "__main__":
    analyze_behavior("xrobocon_ppo_tristar_flat.zip")
