import subprocess
import os
import time
import argparse

def train_loop(total_steps=100000, chunk_size=10000, initial_base='xrobocon_ppo.zip'):
    target_model = 'xrobocon_ppo_tristar_flat.zip'
    
    num_iterations = total_steps // chunk_size
    remainder = total_steps % chunk_size
    if remainder > 0:
        num_iterations += 1
    
    print(f"訓練ループ開始: 全 {total_steps} ステップ ({chunk_size} ステップ x {num_iterations} 回)")
    
    for i in range(num_iterations):
        current_steps = chunk_size
        if i == num_iterations - 1 and remainder > 0:
            current_steps = remainder
            
        print(f"\n{'='*60}")
        print(f"ループ {i+1}/{num_iterations} | 累積ステップ: {i*chunk_size} -> {(i*chunk_size) + current_steps}")
        print(f"{'='*60}")
        
        # ベースモデルの決定
        # ターゲットモデル（途中経過）が存在すればそれを使う
        # なければ初期ベースモデル（転移元）を使う
        if os.path.exists(target_model):
            base_model = target_model
            print(f"既存モデルから継続: {base_model}")
        else:
            base_model = initial_base
            print(f"新規/転移学習開始: {base_model}")
            
        cmd = [
            "python", "scripts/train_rl_step.py",
            "--train",
            "--steps", str(current_steps),
            "--base", base_model,
            "--save_name", target_model.replace('.zip', '') # 拡張子なしの名前を渡す
        ]
        
        print(f"コマンド実行: {' '.join(cmd)}")
        
        try:
            # サブプロセスとして実行（終了時にメモリがOSに返還される）
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"\nエラーが発生しました (Exit Code: {e.returncode})")
            print("ループを停止します。")
            break
        except KeyboardInterrupt:
            print("\nユーザーによる中断を受け付けました。ループを終了します。")
            break
            
        # OSがリソースを確実に回収するための短い待機
        print("メモリ解放待機中...")
        time.sleep(3)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Memory-safe Training Loop')
    parser.add_argument('--steps', type=int, default=100000, help='総ステップ数')
    parser.add_argument('--chunk', type=int, default=10000, help='1回あたりのステップ数')
    parser.add_argument('--base', type=str, default='xrobocon_ppo.zip', help='初期ベースモデル')
    
    args = parser.parse_args()
    
    train_loop(total_steps=args.steps, chunk_size=args.chunk, initial_base=args.base)
