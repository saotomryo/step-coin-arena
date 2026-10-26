"""
XROBOCON RL Training Script - Slope Climbing (Transfer Learning)
平地移動モデルから転移学習してスロープ登坂を訓練
"""
import os
import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from xrobocon.env import XRoboconEnv

class ProgressCallback(BaseCallback):
    """訓練進捗を表示するカスタムコールバック"""
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []
        self.episode_lengths = []
        self.current_episode_reward = 0
        self.current_episode_length = 0
        
    def _on_step(self) -> bool:
        self.current_episode_reward += self.locals['rewards'][0]
        self.current_episode_length += 1
        
        if self.locals['dones'][0]:
            self.episode_rewards.append(self.current_episode_reward)
            self.episode_lengths.append(self.current_episode_length)
            
            # 直近10エピソードの平均を表示
            if len(self.episode_rewards) % 10 == 0:
                recent_rewards = self.episode_rewards[-10:]
                recent_lengths = self.episode_lengths[-10:]
                avg_reward = sum(recent_rewards) / len(recent_rewards)
                avg_length = sum(recent_lengths) / len(recent_lengths)
                
                print(f"\nステップ {self.num_timesteps:,} | "
                      f"エピソード {len(self.episode_rewards)} | "
                      f"平均報酬 {avg_reward:.2f} | "
                      f"平均ステップ {avg_length:.1f}")
            
            self.current_episode_reward = 0
            self.current_episode_length = 0
        
        return True

def train_slope_model(steps=5000, base_model='xrobocon_ppo.zip'):
    """スロープ登坂モデルを訓練（転移学習）"""
    # Mac (MPS) 用の環境変数設定
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
    
    # 環境作成
    env = XRoboconEnv(render_mode=None)
    
    # 転移学習: 平地移動モデルから開始
    if os.path.exists(base_model):
        print(f"\n{'='*70}")
        print(f"転移学習: {base_model} から開始")
        print(f"{'='*70}\n")
        model = PPO.load(base_model, env=env)
        # reset_num_timesteps=Falseで、既存の訓練を継続
        model.learn(
            total_timesteps=steps,
            callback=ProgressCallback(),
            progress_bar=True,
            reset_num_timesteps=False
        )
    else:
        print(f"\n{'='*70}")
        print(f"警告: ベースモデル {base_model} が見つかりません")
        print(f"新規訓練を開始します")
        print(f"{'='*70}\n")
        model = PPO("MlpPolicy", env, verbose=1)
        model.learn(
            total_timesteps=steps,
            callback=ProgressCallback(),
            progress_bar=True
        )
    
    # モデル保存
    model.save('xrobocon_ppo_slope')
    print(f"\n{'='*70}")
    print(f"モデルを保存しました: xrobocon_ppo_slope.zip")
    print(f"{'='*70}\n")

def test_slope_model(episodes=5):
    """スロープ登坂モデルをテスト"""
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
    
    env = XRoboconEnv(render_mode="human")
    model = PPO.load('xrobocon_ppo_slope', env=env)
    
    print(f"\n{'='*70}")
    print(f"スロープ登坂モデルのテスト ({episodes}エピソード)")
    print(f"{'='*70}\n")
    
    for episode in range(episodes):
        obs, _ = env.reset()
        total_reward = 0
        steps = 0
        
        print(f"\nエピソード {episode + 1}/{episodes}")
        
        while steps < 1000:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            total_reward += reward
            steps += 1
            
            if terminated or truncated:
                break
        
        print(f"  報酬: {total_reward:.2f}, ステップ: {steps}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='XROBOCON Slope Climbing RL Training')
    parser.add_argument('--train', action='store_true', help='訓練モード')
    parser.add_argument('--test', action='store_true', help='テストモード')
    parser.add_argument('--steps', type=int, default=5000, help='訓練ステップ数（デフォルト: 5000）')
    parser.add_argument('--episodes', type=int, default=5, help='テストエピソード数（デフォルト: 5）')
    parser.add_argument('--base', type=str, default='xrobocon_ppo.zip', help='ベースモデル（デフォルト: xrobocon_ppo.zip）')
    args = parser.parse_args()
    
    if args.train:
        train_slope_model(steps=args.steps, base_model=args.base)
    elif args.test:
        test_slope_model(episodes=args.episodes)
    else:
        print("--train または --test を指定してください")
