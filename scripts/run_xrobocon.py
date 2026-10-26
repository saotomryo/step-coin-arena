import genesis as gs
import torch
import numpy as np
import os
import sys
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from xrobocon.robot import XRoboconRobot
from xrobocon.field import XRoboconField
from xrobocon.game import XRoboconGame

class XRoboconSimulator:
    """ステップコイン・アリーナ シミュレーター"""
    
    def __init__(self):
        gs.init(backend=gs.gpu)
        
        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3.0, -3.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            rigid_options=gs.options.RigidOptions(
                dt=0.01,
                gravity=(0.0, 0.0, -9.8),
            ),
            show_viewer=True,
        )
        
        # 地面
        self.plane = self.scene.add_entity(gs.morphs.Plane())
        
        # フィールド作成
        self.field = XRoboconField()
        self.field_entities = self.field.build(self.scene)
        
        # ロボット作成 (地面に配置)
        # Ramp 1: End at (1.5, 0). Length 1.5. Direction Y+ (90 deg).
        # Start at (1.5, -1.5).
        # ロボットをその手前 (1.5, -2.0) に配置。向きはY軸正方向 (90度)。
        # ロボットサイズを50%に縮小したため、Z位置も調整（0.1 → 0.05）
        self.robot = XRoboconRobot(self.scene, pos=(1.5, -2.0, 0.05), euler=(0, 0, 90))
        
        # ゲームロジック初期化
        self.game = XRoboconGame(self.field, self.robot)
        
        # コインスポット可視化
        self.spot_entities = self.field.add_coin_spots(self.scene, self.game.spots)
        
        self.scene.build()
        self.robot.post_build()
        
        # ゲーム開始
        self.game.start()
        
    def run(self):
        """シミュレーション実行"""
        print("Simulation started. Press Ctrl+C to exit.")
        
        # シミュレーションループ
        step = 0
        try:
            while True:
                # ゲーム更新
                self.game.update(0.01) # dt=0.01
                
                # 定期的に情報を表示
                if step % 100 == 0: # 1秒ごと
                    info = self.game.get_info()
                    print(f"Time: {info['time']:.1f}s, Score: {info['score']}, Collected: {info['collected_count']}/{info['total_spots']}")
                
                # Ramp 1 は Y軸正方向に登る。
                # すでにY軸正方向を向いているので、直進するだけ。
                self.robot.set_wheel_torques(20.0, 20.0)
                
                self.scene.step()
                step += 1
                
        except KeyboardInterrupt:
            print("Simulation stopped.")

if __name__ == "__main__":
    sim = XRoboconSimulator()
    sim.run()
