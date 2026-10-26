import os
import argparse
import time
import sys
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from stable_baselines3 import PPO
from xrobocon.step_hard_env import XRoboconStepHardEnv
from xrobocon.common import load_trained_model

def train_step_hard_loop(total_timesteps=100000, chunk_size=10000, base_model_path=None, robot_type='tristar', save_name=None):
    """
    段差乗り越え特化訓練ループ（段差80%、平地20%）
    
    Args:
        total_timesteps: 総ステップ数
        chunk_size: 1回の実行（Genesis再起動）あたりのステップ数
        base_model_path: ベースとなるモデルのパス
        robot_type: ロボットタイプ ('tristar', 'tristar_large', etc.)
        save_name: 保存するモデル名（拡張子なし）。Noneの場合はデフォルト名を使用
    """
    
    # モデル保存ディレクトリとモデル名
    model_dir = "."
    if save_name:
        model_name = save_name
    else:
        model_name = f"xrobocon_ppo_{robot_type}_step_hard"
    final_model_path = os.path.join(model_dir, f"{model_name}.zip")
    
    # 現在の進捗を確認
    current_steps = 0
    start_loop = 1
    
    # 既存のステップモデルがあればそこから再開、なければベースモデル（平地）から開始
    if os.path.exists(final_model_path):
        print(f"既存の段差モデルから再開します: {final_model_path}")
        current_base_model = final_model_path
    elif base_model_path and os.path.exists(base_model_path):
        print(f"平地モデルをベースに開始します: {base_model_path}")
        current_base_model = base_model_path
    else:
        print("警告: ベースモデルが見つかりません。スクラッチから学習します。")
        current_base_model = "scratch"

    # ループ計算
    num_loops = total_timesteps // chunk_size
    
    print(f"訓練開始 ({robot_type}): 全{total_timesteps}ステップ ({chunk_size}ステップ x {num_loops}ループ)")
    
    # スクリプトのパスを取得
    script_path = os.path.join(os.path.dirname(__file__), 'train_rl_step.py')
    
    for i in range(start_loop, num_loops + 1):
        print(f"\n{'='*60}")
        print(f"ループ {i}/{num_loops} | 累積ステップ: {(i-1)*chunk_size} -> {i*chunk_size}")
        print(f"{'='*60}")
        
        # train_rl_step.py をサブプロセスとして実行
        # メモリリーク回避のため、Genesisを毎回再起動する
        cmd = f"python {script_path} --train --steps {chunk_size} --base {current_base_model} --env step --robot {robot_type} --save_name {model_name}"
        
        print(f"コマンド実行: {cmd}")
        exit_code = os.system(cmd)
        
        if exit_code != 0:
            print(f"エラーが発生しました (Exit Code: {exit_code})。ループを中断します。")
            break
            
        # 次のループでは、今回保存したモデルをベースにする
        # これにより、学習が累積的に進む
        current_base_model = final_model_path
        print(f"次のループのベースモデル: {current_base_model}")
            
        # 少し待機（メモリ解放待ち）
        print("メモリ解放待機中...")
        time.sleep(5)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='段差乗り越え特化訓練ループ（段差80%）')
    parser.add_argument('--steps', type=int, default=100000, help='総学習ステップ数')
    parser.add_argument('--chunk', type=int, default=10000, help='1回の実行あたりのステップ数')
    parser.add_argument('--base', type=str, default=None, help='ベースモデルのパス')
    parser.add_argument('--robot', type=str, default='tristar', 
                        choices=['tristar', 'tristar_large', 'rocker_bogie', 'rocker_bogie_large'],
                        help='ロボットタイプ')
    parser.add_argument('--save_name', type=str, default=None, help='保存するモデル名（拡張子なし）')
    
    args = parser.parse_args()
    
    # ベースモデルのパスを処理
    base_model_path = None
    if args.base and args.base != 'scratch':
        base_model_path = args.base
    
    # 訓練実行
    train_step_hard_loop(
        total_timesteps=args.steps,
        chunk_size=args.chunk,
        base_model_path=base_model_path,
        robot_type=args.robot,
        save_name=args.save_name
    )
