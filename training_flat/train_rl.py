import os
import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback
from xrobocon.env import XRoboconEnv

class ProgressCallback(BaseCallback):
    """訓練進捗を表示するカスタムコールバック"""
    
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_count = 0
        self.episode_rewards = []
        
    def _on_step(self) -> bool:
        # エピソード終了時に情報を表示
        if self.locals.get('dones')[0]:
            self.episode_count += 1
            episode_reward = self.locals.get('rewards')[0]
            self.episode_rewards.append(episode_reward)
            
            # 10エピソードごとに詳細情報を表示
            if self.episode_count % 10 == 0:
                avg_reward = sum(self.episode_rewards[-10:]) / min(10, len(self.episode_rewards))
                print(f"\n{'='*70}")
                print(f"📊 訓練進捗")
                print(f"{'='*70}")
                print(f"  総ステップ数:     {self.num_timesteps:,} / {self.locals.get('total_timesteps', 0):,}")
                print(f"  エピソード数:     {self.episode_count}")
                print(f"  直近10回平均報酬: {avg_reward:.2f}")
                print(f"{'='*70}\n")
        
        return True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--train", action="store_true", help="Train the agent")
    parser.add_argument("--test", action="store_true", help="Test the trained agent")
    parser.add_argument("--steps", type=int, default=100000, help="Total timesteps for training")
    args = parser.parse_args()
    
    # 環境作成
    # train時は描画なし(高速化)、test時は描画あり
    render_mode = "human" if args.test else None
    env = XRoboconEnv(render_mode=render_mode)
    
    # 環境チェック (初回のみ)
    # check_env(env)
    
    model_path = "xrobocon_ppo"
    
    if args.train:
        print(f"\n{'='*70}")
        print(f"🚀 訓練開始")
        print(f"{'='*70}")
        print(f"  目標ステップ数: {args.steps:,}")
        print(f"  モデル保存先:   {model_path}.zip")
        print(f"{'='*70}\n")
        
        # 既存のモデルがあれば読み込んで継続訓練
        if os.path.exists(model_path + ".zip"):
            print(f"✅ 既存モデル発見: {model_path}.zip")
            print("📂 モデルを読み込んで訓練を継続します...\n")
            model = PPO.load(model_path, env=env, tensorboard_log="./ppo_xrobocon_tensorboard/")
        else:
            print("🆕 既存モデルなし。新規訓練を開始します...\n")
            model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_xrobocon_tensorboard/")
        
        # カスタムコールバックで訓練
        progress_callback = ProgressCallback()
        model.learn(
            total_timesteps=args.steps,
            reset_num_timesteps=False,
            callback=progress_callback,
            progress_bar=True  # プログレスバーを表示
        )
        
        model.save(model_path)
        
        print(f"\n{'='*70}")
        print(f"✅ 訓練完了！")
        print(f"{'='*70}")
        print(f"  総エピソード数: {progress_callback.episode_count}")
        print(f"  モデル保存先:   {model_path}.zip")
        print(f"{'='*70}\n")
        
    if args.test:
        if not os.path.exists(model_path + ".zip"):
            print("❌ モデルが見つかりません。先に訓練を実行してください。")
            return
            
        print("📂 モデルを読み込んでテスト中...")
        model = PPO.load(model_path)
        
        obs, _ = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            
            # 環境側で描画更新されるはず
            
if __name__ == "__main__":
    main()
