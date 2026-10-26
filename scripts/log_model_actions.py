"""
学習済みモデルのアクション出力をログに記録するスクリプト
手動操作のトルク調整の参考にする
"""
# MPSを無効化してCPUを強制使用
import os
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
os.environ['PYTORCH_MPS_HIGH_WATERMARK_RATIO'] = '0.0'  # MPSを完全に無効化

import numpy as np
from stable_baselines3 import PPO
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xrobocon.step_env import XRoboconStepEnv
import argparse
import torch


def log_model_actions(model_path, num_steps=500, robot_type='tristar_large'):
    """モデルのアクション出力を記録"""
    
    # 環境とモデルのロード
    env = XRoboconStepEnv(render_mode="human", robot_type=robot_type)
    
    # CPUでモデルをロード
    model = PPO.load(model_path, device='cpu')
    
    obs, info = env.reset()
    
    action_log = []
    
    print("\n" + "="*70)
    print("モデルアクション記録開始")
    print("="*70)
    print(f"ロボット: {robot_type}")
    print(f"ステップ数: {num_steps}")
    print("="*70)
    
    for step in range(num_steps):
        # モデルの予測
        action, _ = model.predict(obs, deterministic=True)
        
        # アクションを記録
        action_log.append(action.copy())
        
        # 10ステップごとにログ出力
        if step % 10 == 0:
            robot_pos = env.robot.get_pos().cpu().numpy()
            robot_vel = env.robot.get_vel().cpu().numpy()
            robot_euler = env.robot.get_euler()
            
            print(f"Step {step:3d} | "
                  f"Action: Frame={action[0]:.3f},{action[1]:.3f} Wheel={action[2]:.3f},{action[3]:.3f} | "
                  f"Pos: Z={robot_pos[2]:.3f} | Vel: Z={robot_vel[2]:.3f} | "
                  f"Pitch={robot_euler[1]:.1f}°")
        
        # ステップ実行
        obs, reward, terminated, truncated, _ = env.step(action)
        
        if terminated or truncated:
            print(f"\nエピソード終了 (Step {step})")
            obs, info = env.reset()
    
    # 統計情報を出力
    action_log = np.array(action_log)
    
    print("\n" + "="*70)
    print("アクション統計")
    print("="*70)
    print(f"Frame L: mean={action_log[:, 0].mean():.3f}, std={action_log[:, 0].std():.3f}, "
          f"min={action_log[:, 0].min():.3f}, max={action_log[:, 0].max():.3f}")
    print(f"Frame R: mean={action_log[:, 1].mean():.3f}, std={action_log[:, 1].std():.3f}, "
          f"min={action_log[:, 1].min():.3f}, max={action_log[:, 1].max():.3f}")
    print(f"Wheel L: mean={action_log[:, 2].mean():.3f}, std={action_log[:, 2].std():.3f}, "
          f"min={action_log[:, 2].min():.3f}, max={action_log[:, 2].max():.3f}")
    print(f"Wheel R: mean={action_log[:, 3].mean():.3f}, std={action_log[:, 3].std():.3f}, "
          f"min={action_log[:, 3].min():.3f}, max={action_log[:, 3].max():.3f}")
    print("="*70)
    
    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, required=True, help='Path to trained model')
    parser.add_argument('--steps', type=int, default=500, help='Number of steps to log')
    parser.add_argument('--robot', type=str, default='tristar_large', help='Robot type')
    
    args = parser.parse_args()
    
    log_model_actions(args.model, args.steps, args.robot)
