import numpy as np
from xrobocon.base_env import XRoboconBaseEnv

class XRoboconEnv(XRoboconBaseEnv):
    """
    ステップコイン・アリーナ RL Environment (Flat Ground)
    平地移動訓練用の環境です。
    """
    
    def __init__(self, render_mode=None, robot_type='tristar'):
        super().__init__(render_mode, robot_type)
        
    def reset(self, seed=None, options=None):
        # 親クラスのreset呼び出し（seed設定など）
        # super().reset(seed=seed) # BaseEnvのresetはNotImplementedErrorなので呼ばない
        if seed is not None:
            np.random.seed(seed)
        
        # シナリオ選択
        # Phase 3-2a: 平地移動訓練 (Tri-star Robot)
        # 地面（Tier 3の外側）での移動制御を学習
        
        # シナリオ選択 (Curriculum: Short 80%, Medium 10%, Long 10%)
        scenario_type = np.random.choice(
            ['flat_short', 'flat_medium', 'flat_long'],
            p=[0.8, 0.1, 0.1]
        )
        
        scenarios = [
        {
            'name': 'Scenario 1: 近距離移動 (平地)',
            'type': 'flat_short',
            'start_pos': (8.0, 0.0, 0.08),
            'start_euler': (0, 0, 0),
            'target_pos': (9.0, 0.0, 0.08), # 1m先
        },
        {
            'name': 'Scenario 2: 中距離移動 (平地)',
            'type': 'flat_medium',
            'start_pos': (8.0, 0.0, 0.08),
            'start_euler': (0, 0, 0),
            'target_pos': (10.0, 1.0, 0.08), # 2m以上先、斜め
        },
        {
            'name': 'Scenario 3: 長距離移動 (平地)',
            'type': 'flat_long',
            'start_pos': (8.0, 0.0, 0.08),
            'start_euler': (0, 0, 90),     # 横向き
            'target_pos': (8.0, 3.0, 0.08), # 3m先
        },
    ]
        
        selected_scenario = next(s for s in scenarios if s['type'] == scenario_type)
        start_pos = selected_scenario['start_pos']
        start_euler = selected_scenario['start_euler']
        target_pos = selected_scenario['target_pos']
            
        # ランダム性を少し加える（±5cm、±5度）
        start_x = start_pos[0] + np.random.uniform(-0.05, 0.05)
        start_y = start_pos[1] + np.random.uniform(-0.05, 0.05)
        start_yaw = start_euler[2] + np.random.uniform(-5, 5)
        
        # シーンリセット (物理状態のクリア)
        self.scene.reset()
        
        # ロボットの位置設定 (シーンリセット後に適用)
        self.robot.set_pose(
            pos=(start_x, start_y, start_pos[2]),
            euler_deg=(0, 0, start_yaw)
        )
        
        # ゲームリセット
        self.game.start()
        
        # ターゲット設定 (ロボットの現在位置に基づいてprev_distを計算するため、set_poseの後に行う)
        self.set_target(target_pos)
        
        # 高さトラッキング用（報酬計算に使用）
        self.prev_height = start_pos[2]
        
        return self._get_obs(), {}
        
    def _set_random_target(self):
        """(廃止予定) ランダムなターゲットを設定"""
        pass

    def step(self, action):
        # 共通のアクション適用
        self._apply_action(action)
        
        # 報酬計算
        reward = 0.0
        terminated = False
        truncated = False
        
        # 状態取得
        robot_pos = self.robot.get_pos().cpu().numpy()
        euler = self.robot.get_euler() # numpy array
        vel = self.robot.get_vel().cpu().numpy()
        speed = np.linalg.norm(vel)
        
        # 1. ターゲットへの接近報酬 (Goal-Conditioned)
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            dist = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # 距離が縮まったらプラス、離れたらマイナス (Shaping)
            # 重みをさらに強化 (50.0 -> 100.0) して、移動を最優先に
            reward += (self.prev_dist - dist) * 100.0
            self.prev_dist = dist
            
            # ターゲット到達判定 (距離0.5m以内 - 判定を緩和)
            z_diff = abs(robot_pos[2] - target_pos[2])
            if dist < 0.5 and z_diff < 0.2:
                # 到達ボーナス
                reward += 300.0
                
                # 停止ボーナス (速度が小さいほど高得点)
                if speed < 0.1:
                    reward += 100.0
                elif speed < 0.5:
                    reward += 50.0
                
                terminated = True
            
        # 2. 安定性報酬 (転倒防止)
        stability_penalty = abs(euler[0]) * 0.05 + abs(euler[1]) * 0.05
        reward -= stability_penalty
        
        # 3. 効率化のための追加ペナルティ
        # (a) フレーム使用ペナルティ (平地ではフレームをあまり使わないでほしい)
        # action[0], action[1] はフレームトルク
        # 重みを0.1 -> 0.5に強化して、フレームによる「這いずり移動」を抑制
        if self.robot_type == 'tristar':
            frame_penalty = (abs(action[0]) + abs(action[1])) * 0.5
            reward -= frame_penalty
            
        # (b) 速度超過ペナルティ (制御不能な暴走を防ぐ: 1.5m/s以上はペナルティ)
        # 重みを1.0 -> 2.0に強化
        if speed > 1.5:
            reward -= (speed - 1.5) * 2.0
        
        # 4. 転倒・落下判定 (終了条件)
        if abs(euler[0]) > 60 or abs(euler[1]) > 60:
            reward -= 100.0
            terminated = True
            
        if robot_pos[2] < 0.0:
            reward -= 100.0
            terminated = True
            
        # 時間切れ
        if not self.game.is_running:
            truncated = True
            
        return self._get_obs(), reward, terminated, truncated, {}
