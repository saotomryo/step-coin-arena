import numpy as np
import genesis as gs
import time
import argparse
import os
import sys
import os
# Add parent directory to sys.path to allow importing xrobocon
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cv2
from xrobocon.step_env import XRoboconStepEnv

class ManualRecorder:
    def __init__(self, robot_type='tristar_large', output_dir='demonstrations'):
        # Use rgb_array mode to enable renderer but disable built-in viewer
        self.env = XRoboconStepEnv(render_mode="rgb_array", robot_type=robot_type)
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # アクション状態
        # [frame_L, frame_R, wheel_L, wheel_R]
        self.current_action = np.zeros(4, dtype=np.float32)
        
        # 記録データ
        self.observations = []
        self.actions = []
        self.rewards = []
        self.recording = False
        self.waiting_for_save_decision = False
        
        # カメラは環境初期化時に作成されているはず
        # self.env.scene.renderer が存在する
        
        print("\n" + "="*60)
        print("Manual Control Recorder (OpenCV)")
        print("="*60)
        print("Controls (Focus on the OpenCV Window):")
        print("  W / S     : Forward / Backward (Wheels)")
        print("  A / D     : Turn Left / Right (Wheels)")
        print("  I / K     : Frame Pitch Up / Down (Frames)")
        print("  Space     : Stop All")
        print("  R         : Start/Stop Recording")
        print("  ESC / Q   : Quit")
        print("="*60)

    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            print("\nRecording STARTED...")
            self.observations = []
            self.actions = []
            self.rewards = []
        else:
            print("\nRecording STOPPED.")
            print(f"Recorded {len(self.observations)} steps. Save this episode? (y/n)")
            self.waiting_for_save_decision = True

    def save_demonstration(self):
        if len(self.observations) == 0:
            print("No data to save.")
            return
            
        timestamp = int(time.time())
        filename = os.path.join(self.output_dir, f"demo_{timestamp}.npz")
        
        np.savez(filename, 
                 obs=np.array(self.observations), 
                 actions=np.array(self.actions),
                 rewards=np.array(self.rewards))
        
        print(f"Saved demonstration to {filename} ({len(self.observations)} steps)")

    def run(self):
        obs, info = self.env.reset()
        
        # カメラ取得
        camera = None
        if hasattr(self.env, 'camera') and self.env.camera:
            camera = self.env.camera
        else:
            print("Error: No camera found in the environment.")
            return

        try:
            while True:
                # 1. カメラ更新 (Follow Camera)
                if camera:
                    # ロボットの位置を取得
                    robot_pos = self.env.robot.get_pos().cpu().numpy()
                    
                    # カメラ位置を計算 (ロボットの少し後ろ・上)
                    # 現在のカメラ距離と角度を維持
                    # 簡易的に: ロボットの座標 + オフセット
                    # ズーム機能: self.camera_dist を調整
                    if not hasattr(self, 'camera_dist'):
                        self.camera_dist = 4.0
                        self.camera_pitch = 2.5
                    
                    # カメラ位置: ロボットからX軸方向に-dist, Z軸方向に+pitch
                    # より高度にするならロボットの向きに合わせるが、まずは固定アングル追従で十分
                    cam_pos = np.array([robot_pos[0] - self.camera_dist, robot_pos[1], robot_pos[2] + self.camera_pitch])
                    cam_lookat = np.array([robot_pos[0], robot_pos[1], robot_pos[2] + 0.5])
                    
                    camera.set_pose(pos=cam_pos, lookat=cam_lookat)

                    # レンダリング (OpenCV)
                    # render() returns (rgb, depth, segmentation, normal)
                    rgb, _, _, _ = camera.render()
                    # rgb is likely (H, W, 3) or (H, W, 4)
                    
                    # BGRに変換 (OpenCV用)
                    # Genesis returns RGB in [0, 1] or [0, 255]? Usually [0, 1] float or [0, 255] uint8.
                    # Assuming [0, 1] float based on typical renderers, or check type.
                    # If it's float, convert to uint8.
                    if rgb.dtype == np.float32 or rgb.dtype == np.float64:
                        rgb = (rgb * 255).astype(np.uint8)
                    
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    
                    # 録画中マーク
                    if self.recording:
                        cv2.circle(bgr, (30, 30), 10, (0, 0, 255), -1)
                        cv2.putText(bgr, "REC", (50, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # 保存待機中メッセージ
                    if self.waiting_for_save_decision:
                        cv2.putText(bgr, "Save? (Y/N)", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    
                    cv2.imshow("Manual Control", bgr)
                
                # 2. キー入力処理 (OpenCV)
                key = cv2.waitKey(1) & 0xFF
                
                if self.waiting_for_save_decision:
                    if key == ord('y'):
                        self.save_demonstration()
                        self.waiting_for_save_decision = False
                        print("Ready for next episode.")
                    elif key == ord('n'):
                        print("Discarded episode.")
                        self.waiting_for_save_decision = False
                        print("Ready for next episode.")
                    continue

                if key == 27 or key == ord('q'): # ESC or Q
                    break
                elif key == ord('r'):
                    self.toggle_recording()
                elif key == ord(' '):
                    self.current_action[:] = 0.0
                
                # ズーム操作
                elif key == ord('z'): # Zoom In
                    self.camera_dist = max(1.0, self.camera_dist - 0.2)
                elif key == ord('x'): # Zoom Out
                    self.camera_dist = min(10.0, self.camera_dist + 0.2)
                
                # 操作ロジック
                action_scale = 1.0  # トルクスケーリングの二重適用問題を修正したので1.0に戻す
                
                # Wheel操作時に自動的にFrameも制御(学習モデルの動作を模倣)
                # 注意: 強すぎると不安定になるので、弱めに設定
                auto_frame_control = False  # デフォルトOFF (I/Kキーで手動調整を推奨)
                frame_assist_strength = 0.2  # 自動制御の強度(0.0〜1.0)
                
                if key == ord('w'):  # 前進
                    self.current_action[2], self.current_action[3] = action_scale, action_scale
                    if auto_frame_control:
                        self.current_action[0], self.current_action[1] = -frame_assist_strength, 0.0
                elif key == ord('s'):  # 後退
                    self.current_action[2], self.current_action[3] = -action_scale, -action_scale
                    if auto_frame_control:
                        self.current_action[0], self.current_action[1] = frame_assist_strength, 0.0
                elif key == ord('a'): self.current_action[2], self.current_action[3] = -action_scale*0.5, action_scale*0.5
                elif key == ord('d'): self.current_action[2], self.current_action[3] = action_scale*0.5, -action_scale*0.5
                elif key == ord('i'): self.current_action[0], self.current_action[1] = action_scale, action_scale
                elif key == ord('k'): self.current_action[0], self.current_action[1] = -action_scale, -action_scale
                
                # アクション状態を表示 (画面上部に大きく)
                action_str = f"Frame:{self.current_action[0]:.1f} Wheel:{self.current_action[2]:.1f}"
                # 背景を描画 (視認性向上)
                cv2.rectangle(bgr, (5, 5), (400, 45), (255, 255, 255), -1)
                cv2.putText(bgr, action_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
                
                # コンソールにも出力 (10フレームごと)
                if not hasattr(self, 'frame_count'):
                    self.frame_count = 0
                self.frame_count += 1
                if self.frame_count % 10 == 0:
                    # ロボットの状態も出力
                    robot_pos = self.env.robot.get_pos().cpu().numpy()
                    robot_vel = self.env.robot.get_vel().cpu().numpy()
                    robot_euler = self.env.robot.get_euler()
                    print(f"Action: Frame={self.current_action[0]:.2f}, Wheel={self.current_action[2]:.2f} | "
                          f"Pos: Z={robot_pos[2]:.3f} | Vel: Z={robot_vel[2]:.3f} | "
                          f"Pitch={robot_euler[1]:.1f}°")
                
                # 描画完了後に表示
                cv2.imshow("Manual Control", bgr)

                # 3. シミュレーションステップ
                # トルク不足の可能性があるので、アクション値を少し強調する（環境側でクリップされるが）
                # しかし環境のmax_torqueが20なので、1.0を送れば20Nm。
                # 動かない場合は物理パラメータの問題か、アクションが適用されていないか。
                # とりあえずそのまま送る。
                
                next_obs, reward, terminated, truncated, _ = self.env.step(self.current_action)
                
                if self.recording:
                    self.observations.append(obs)
                    self.actions.append(self.current_action.copy())
                    self.rewards.append(reward)
                
                obs = next_obs
                
                if terminated or truncated:
                    if self.recording:
                        print(f"Episode finished. Reward: {sum(self.rewards):.2f}")
                        self.toggle_recording()
                    
                    obs, info = self.env.reset()
                    self.current_action[:] = 0.0
                    time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            cv2.destroyAllWindows()
            self.env.close()

if __name__ == "__main__":
    recorder = ManualRecorder()
    recorder.run()
