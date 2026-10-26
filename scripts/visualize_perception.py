"""
地形認識（Perception）の可視化スクリプト
ロボットが「見ている」5x5の高さマップをコンソールに表示します。
"""
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import xrobocon.common as common
import genesis as gs
import numpy as np
import time
import sys
from xrobocon.step_env import XRoboconStepEnv

def visualize_perception():
    """地形認識の様子を可視化"""
    print("\n" + "="*70)
    print("地形認識 (Terrain Perception) 可視化")
    print("="*70)
    print("ロボットが取得している 5x5 高さマップ をリアルタイム表示します。")
    print("数値は「足元からの相対高さ (m)」です。")
    print("----------------------------------------------------------------------")
    print("凡例:")
    print("  0.00 : 平地 (Ground)")
    print("  0.10 : 1段目 (Tier 3)")
    print("  0.35 : 2段目 (Tier 2)")
    print("----------------------------------------------------------------------\n")
    
    # 環境作成 (Step環境)
    env = XRoboconStepEnv(render_mode="human", robot_type='tristar')
    
    # シナリオ: 2段目登坂 (段差がわかりやすい)
    obs, info = env.reset()
    
    # カメラ調整
    env.scene.viewer.set_camera_pose(pos=np.array([6.0, -2.0, 2.0]), lookat=np.array([4.0, 0.0, 0.5]))
    
    print(f"シナリオ: {info.get('scenario_type', 'unknown')}")
    print("開始位置から前進します...\n")
    
    # 前進アクション (フレーム固定、ホイール前進)
    # Action: [Frame_L, Frame_R, Wheel_L, Wheel_R]
    action = np.array([0.0, 0.0, 0.5, 0.5], dtype=np.float32)
    
    try:
        for i in range(100): # 100ステップ実行
            # 1ステップ実行
            obs, reward, terminated, truncated, _ = env.step(action)
            
            # 観測データから高さマップを抽出
            # obs構造: [Pos(3), Euler(3), Vel(3), AngVel(3), Target(3), HeightMap(25)]
            # Total 40. HeightMap is the last 25.
            height_map_flat = obs[-25:]
            height_map_grid = height_map_flat.reshape(5, 5)
            
            # ロボット位置
            pos = env.robot.get_pos().cpu().numpy()
            
            # コンソール表示 (カーソルを上に移動して上書きっぽく見せるのは難しいので、ステップごとに表示)
            if i % 10 == 0: # 10ステップごとに表示
                print(f"\n--- Step {i} (Pos: x={pos[0]:.2f}, z={pos[2]:.2f}) ---")
                print("Robot's Eye View (Height Map 5x5):")
                
                # グリッド表示 (上側が前方)
                # 配列は [row, col] で、row=0がXマイナス(後方)? 
                # _get_height_mapの実装を確認すると:
                # i (row) が X (前方), j (col) が Y (左)
                # 表示するときは、上が前方(X+)、右が右(Y-)にしたい
                
                # i=4 (Front), j=0 (Right) ... j=4 (Left)
                # 行を逆順にして表示すれば、上が前方になる
                
                for row in range(4, -1, -1): # 4, 3, 2, 1, 0
                    line = "  "
                    for col in range(4, -1, -1): # 4(Left) ... 0(Right) -> Right to Left?
                        # Y軸は左がプラス。
                        # 表示は左側を左(Y+)、右側を右(Y-)にしたい。
                        # col=4 (Left), col=0 (Right)
                        val = height_map_grid[row, col]
                        
                        # 色付け (簡易)
                        char = f"{val:5.2f}"
                        if val > 0.05:
                            char = f"\033[91m{char}\033[0m" # 赤 (段差)
                        elif val < -0.05:
                            char = f"\033[94m{char}\033[0m" # 青 (穴)
                        else:
                            char = f"\033[90m{char}\033[0m" # グレー (平地)
                            
                        line += char + " "
                    print(line)
                print("      (Front)")
                
            time.sleep(0.1)
            
            if terminated or truncated:
                print("\nエピソード終了")
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        print("\n終了します")

if __name__ == "__main__":
    visualize_perception()
