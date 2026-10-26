import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from xrobocon.step_env import XRoboconStepEnv
import argparse
import glob
import os

def train_bc(demo_dir='demonstrations', output_model='xrobocon_ppo_tristar_large_bc', epochs=50, batch_size=64, lr=3e-4):
    print(f"\n{'='*60}")
    print(f"Behavior Cloning Training")
    print(f"{'='*60}")
    
    # 1. デモンストレーションデータの読み込み
    obs_list = []
    actions_list = []
    
    demo_files = glob.glob(os.path.join(demo_dir, "*.npz"))
    if not demo_files:
        print(f"Error: No demonstration files found in {demo_dir}")
        return
        
    print(f"Loading {len(demo_files)} demonstration files...")
    for f in demo_files:
        data = np.load(f)
        obs_list.append(data['obs'])
        actions_list.append(data['actions'])
        
    # 結合
    observations = np.concatenate(obs_list, axis=0)
    actions = np.concatenate(actions_list, axis=0)
    
    print(f"Total samples: {len(observations)}")
    
    # Tensor変換
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if torch.backends.mps.is_available():
        device = torch.device("mps")
        
    obs_tensor = torch.FloatTensor(observations).to(device)
    actions_tensor = torch.FloatTensor(actions).to(device)
    
    dataset = TensorDataset(obs_tensor, actions_tensor)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
    
    # 2. PPOモデルの作成 (Student)
    env = XRoboconStepEnv(render_mode=None, robot_type='tristar_large')
    model = PPO("MlpPolicy", env, verbose=1)
    
    # ポリシーネットワークの抽出
    policy = model.policy.to(device)
    optimizer = optim.Adam(policy.parameters(), lr=lr)
    loss_fn = nn.MSELoss()
    
    # 3. 教師あり学習 (Behavior Cloning)
    print(f"\nStarting training on {device}...")
    policy.train()
    
    for epoch in range(epochs):
        total_loss = 0
        for batch_obs, batch_actions in dataloader:
            optimizer.zero_grad()
            
            # ポリシーからアクション分布を取得 -> 平均値(mean)を使用
            # get_distribution returns a distribution object
            dist = policy.get_distribution(batch_obs)
            pred_actions = dist.mode() # 決定論的アクション (mean)
            
            loss = loss_fn(pred_actions, batch_actions)
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
            
        avg_loss = total_loss / len(dataloader)
        if (epoch + 1) % 5 == 0:
            print(f"Epoch {epoch+1}/{epochs}, Loss: {avg_loss:.6f}")
            
    print("\nTraining finished.")
    
    # 4. モデル保存
    model.save(output_model)
    print(f"Model saved to {output_model}.zip")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, default='demonstrations', help='Directory containing .npz files')
    parser.add_argument('--out', type=str, default='xrobocon_ppo_tristar_large_bc', help='Output model name')
    parser.add_argument('--epochs', type=int, default=50, help='Training epochs')
    args = parser.parse_args()
    
    train_bc(args.dir, args.out, args.epochs)
