"""
ステップコイン・アリーナ RL Training Script - Step Climbing (Tri-star Robot)
平地移動モデルから転移学習して段差登坂を訓練
"""
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# 共通モジュールを最初にインポートして設定（MPS Fallback等）を適用
import xrobocon.common as common

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

def train_step_model(steps=10000, base_model='xrobocon_ppo.zip', env_type='flat', robot_type='tristar', save_name='xrobocon_ppo_tristar_flat'):
    """ロボットの訓練（転移学習）"""
    
    # 環境作成
    if env_type == 'step':
        from xrobocon.step_env import XRoboconStepEnv
        env = XRoboconStepEnv(render_mode=None, robot_type=robot_type)
        print(f"環境: 段差乗り越え (Step Climbing), ロボット: {robot_type}")
    else:
        env = XRoboconEnv(render_mode=None, robot_type=robot_type)
        print(f"環境: 平地移動 (Flat Ground), ロボット: {robot_type}")
    
    # 転移学習: ベースモデルから開始
    if os.path.exists(base_model):
        print(f"\n{'='*70}")
        print(f"転移学習: {base_model} から開始")
        print(f"ロボットタイプ: Tri-star")
        print(f"{'='*70}\n")
        
        # モデルロード
        model = common.load_trained_model(base_model, env)
        
        # 学習率を少し下げる（微調整のため）
        model.learning_rate = 0.0001
        
        # reset_num_timesteps=Falseで、既存の訓練を継続
        try:
            model.learn(
                total_timesteps=steps,
                callback=ProgressCallback(),
                progress_bar=True,
                reset_num_timesteps=False
            )
        except KeyboardInterrupt:
            print("\n\n訓練が中断されました。モデルを保存しています...")
            model.save(save_name)
            print(f"モデルを保存しました: {save_name}.zip")
            return
            
    elif base_model == 'scratch':
        print(f"\n{'='*70}")
        print(f"新規訓練を開始します (スクラッチ)")
        print(f"ロボットタイプ: Tri-star")
        print(f"{'='*70}\n")
        model = PPO("MlpPolicy", env, verbose=1)
        try:
            model.learn(
                total_timesteps=steps,
                callback=ProgressCallback(),
                progress_bar=True
            )
        except KeyboardInterrupt:
            print("\n\n訓練が中断されました。モデルを保存しています...")
            model.save(save_name)
            print(f"モデルを保存しました: {save_name}.zip")
            return

    else:
        print(f"\n{'='*70}")
        print(f"警告: ベースモデル {base_model} が見つかりません")
        print(f"新規訓練を開始します")
        print(f"ロボットタイプ: Tri-star")
        print(f"{'='*70}\n")
        model = PPO("MlpPolicy", env, verbose=1)
        try:
            model.learn(
                total_timesteps=steps,
                callback=ProgressCallback(),
                progress_bar=True
            )
        except KeyboardInterrupt:
            print("\n\n訓練が中断されました。モデルを保存しています...")
            model.save(save_name)
            print(f"モデルを保存しました: {save_name}.zip")
            return
    
    # モデル保存
    model.save(save_name)
    print(f"\n{'='*70}")
    print(f"モデルを保存しました: {save_name}.zip")
    print(f"{'='*70}\n")

def test_step_model(episodes=5, env_type='flat', robot_type='tristar', model_path='xrobocon_ppo_tristar_flat'):
    """モデルをテスト"""
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
    
    if env_type == 'step':
        from xrobocon.step_env import XRoboconStepEnv
        env = XRoboconStepEnv(render_mode="human", robot_type=robot_type)
    else:
        env = XRoboconEnv(render_mode="human", robot_type=robot_type)
        
    model = PPO.load(model_path, env=env)
    
    print(f"\n{'='*70}")
    print(f"Tri-star モデルテスト ({env_type}) ({episodes}エピソード)")
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
    parser = argparse.ArgumentParser(description='ステップコイン・アリーナ Step Climbing RL Training')
    parser.add_argument('--train', action='store_true', help='訓練モード')
    parser.add_argument('--test', action='store_true', help='テストモード')
    parser.add_argument('--steps', type=int, default=10000, help='訓練ステップ数（デフォルト: 10000）')
    parser.add_argument('--episodes', type=int, default=5, help='テストエピソード数（デフォルト: 5）')
    parser.add_argument('--base', type=str, default='xrobocon_ppo.zip', help='ベースモデル')
    parser.add_argument('--env', type=str, default='flat', choices=['flat', 'step'], help='環境タイプ (flat, step)')
    parser.add_argument('--save_name', type=str, default='xrobocon_ppo_tristar_flat', help='保存モデル名')
    parser.add_argument('--robot', type=str, default='tristar', help='ロボットタイプ (tristar, tristar_large)')
    args = parser.parse_args()
    
    if args.train:
        train_step_model(steps=args.steps, base_model=args.base, env_type=args.env, robot_type=args.robot, save_name=args.save_name)
    elif args.test:
        test_step_model(episodes=args.episodes, env_type=args.env, robot_type=args.robot, model_path=args.save_name)
    else:
        print("--train または --test を指定してください")
