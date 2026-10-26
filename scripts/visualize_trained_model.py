"""
訓練済みモデルの動作を可視化し、停滞時点のスクリーンショットを撮影
"""
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# 共通モジュールを最初にインポートして設定（MPS Fallback等）を適用
import xrobocon.common as common

import os
import numpy as np
import argparse
import genesis as gs
from xrobocon.env import XRoboconEnv
from xrobocon.step_env import XRoboconStepEnv
import xrobocon.common as common

def visualize_trained_model(model_path, env_type='flat', robot_type='tristar'):
    """訓練済みモデルの動作を可視化"""
    
    # 環境作成（描画あり）
    if env_type == 'step':
        env = XRoboconStepEnv(render_mode="human", robot_type=robot_type)
        # カメラ位置を段差エリアに調整
        env.scene.viewer.set_camera_pose(pos=np.array([8.0, -3.0, 2.5]), lookat=np.array([4.0, 0.0, 0.5]))
    else:
        env = XRoboconEnv(render_mode="human", robot_type=robot_type)
        # カメラ位置を訓練エリアに調整
        env.scene.viewer.set_camera_pose(pos=np.array([8.0, -3.0, 2.5]), lookat=np.array([5.0, 0.0, 0.0]))
    
    # モデル読み込み
    model = common.load_trained_model(model_path, env)
    
    print("\n" + "="*70)
    print(f"訓練済みモデルの動作を可視化 ({env_type} environment)")
    print("="*70)
    print("ロボットが目標に最も近づいた時点で一時停止します")
    print("Escキーでウィンドウを閉じると次のエピソードに進みます")
    print("="*70 + "\n")
    
    num_episodes = 5
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        scenario_type = info.get('scenario_type', 'default')
        
        print(f"\nエピソード {episode + 1}/{num_episodes} [{scenario_type}]")
        print(f"  初期位置: ({obs[0]:.2f}, {obs[1]:.2f}, {obs[2]:.2f})")
        print(f"  目標位置: ({env.current_target['pos'][0]:.2f}, {env.current_target['pos'][1]:.2f}, {env.current_target['pos'][2]:.2f})")
        
        min_dist = 999
        min_dist_step = 0
        
        for step in range(1000): # ステップ数を増やす
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            
            # 現在の距離を計算
            robot_pos = env.robot.get_pos().cpu().numpy()
            target_pos = np.array(env.current_target['pos'])
            dist = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            if dist < min_dist:
                min_dist = dist
                min_dist_step = step
            
            # 最小距離から離れ始めたら一時停止 (判定を少し緩く)
            if step > min_dist_step + 30 and step > 50 and min_dist < 2.0:
                print(f"  最小距離到達: Step {min_dist_step}, 距離 {min_dist:.3f}m")
                print(f"  現在のステップ: {step}")
                print(f"  ロボット位置: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})")
                print(f"  目標からの距離: {dist:.3f}m")
                print(f"\n  ⚠️  ロボットが目標から離れ始めました")
                print(f"  ビューアで状況を確認してください")
                print(f"  確認したらEnterキーを押して次のエピソードに進みます...")
                
                # ユーザーの入力を待つ
                # input() # 自動化のためコメントアウトするか、オプションにする
                # break # 一時停止せずに継続
            
            if terminated or truncated:
                print(f"  エピソード終了: Step {step}, 理由: {'Terminated' if terminated else 'Truncated'}")
                break
        
        print(f"  最終的な最小距離: {min_dist:.3f}m")
        
        # 成功判定
        success = False
        if env_type == 'step':
            # 最終的な高さもチェック
            robot_pos = env.robot.get_pos().cpu().numpy()
            target_pos = np.array(env.current_target['pos'])
            if min_dist < 0.5 and robot_pos[2] > target_pos[2] - 0.1:
                success = True
        else:
            if min_dist < 0.5:
                success = True
                
        print(f"  成功判定: {'成功' if success else '失敗'}")
    
    print("\n" + "="*70)
    print("可視化完了")
    print("="*70)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='ステップコイン・アリーナ RL Model Visualization')
    parser.add_argument('--model', type=str, default='xrobocon_ppo.zip', help='モデルファイルのパス')
    parser.add_argument('--env', type=str, default='flat', choices=['flat', 'step'], help='環境タイプ (flat, step)')
    parser.add_argument('--robot', type=str, default='tristar', help='ロボットタイプ')
    args = parser.parse_args()
    
    # Mac (MPS) 用の環境変数設定
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
    
    if not os.path.exists(args.model) and not os.path.exists(args.model + ".zip"):
         print(f"エラー: モデルファイルが見つかりません: {args.model}")
         exit(1)

    visualize_trained_model(args.model, args.env, args.robot)
