import numpy as np
from xrobocon.base_env import XRoboconBaseEnv
from xrobocon.robot_configs import get_start_height

class XRoboconStepEnv(XRoboconBaseEnv):
    """
    ステップコイン・アリーナ RL Environment (Step Climbing)
    段差乗り越え（Tier 1への登坂）訓練用の環境です。
    """
    
    def __init__(self, render_mode=None, robot_type='tristar'):
        super().__init__(render_mode, robot_type)
        
        # ロボット設定から開始高さを取得
        self.start_z_offset = get_start_height(robot_type, 'step')
        
        # エピソード時間を延長 (5秒 = 500ステップ)
        self.game.time_limit = 5.0
        
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        
        # シナリオ選択
        # Phase 3-2b: 段差乗り越え訓練 (Curriculum Learning)
        # 1. Flat Easy (平地移動の基礎) - 50%
        # 2. Ground -> Tier 3 (高さ10cm) - 25%
        # 3. Tier 3 -> Tier 2 (高さ25cm) - 25%
        
        scenario_type = np.random.choice(
            ['flat_easy', 'step_straight', 'step_tier3_to_tier2'],
            p=[1.0, 0.0, 0.0] # Phase 1: 平地のみ (100%)
        )
        
        scenarios = [
        {
            'name': 'Scenario 0: 平地移動 (Flat Easy)',
            'type': 'flat_easy',
            'start_pos': (5.0, -2.0, self.start_z_offset), # 平地エリア
            'start_euler': (0, 0, 90),                      # Y軸プラス方向
            'target_pos': (5.0, 0.0, self.start_z_offset), # 2m先 (同じ高さ)
        },
        {
            'name': 'Scenario 1: 正面段差登坂 (Ground -> Tier 3)',
            'type': 'step_straight',
            'start_pos': (5.5, 0.0, self.start_z_offset), # Tier 3 (R=4.65) の外側
            'start_euler': (0, 0, 180),                    # 中心方向
            'target_pos': (4.0, 0.0, 0.1 + self.start_z_offset), # Tier 3の上
        },
        {
            'name': 'Scenario 2: 2段目登坂 (Tier 3 -> Tier 2)',
            'type': 'step_tier3_to_tier2',
            'start_pos': (4.0, 0.0, 0.1 + self.start_z_offset),  # Tier 3の上 (Z=0.1)
            'start_euler': (0, 0, 180),     # 中心方向
            'target_pos': (2.5, 0.0, 0.35 + self.start_z_offset),  # Tier 2の上 (Z=0.35)
        },
    ]
        
        selected_scenario = next(s for s in scenarios if s['type'] == scenario_type)
        start_pos = selected_scenario['start_pos']
        start_euler = selected_scenario['start_euler']
        target_pos = selected_scenario['target_pos']
            
        # ランダム性を少し加える
        start_x = start_pos[0] + np.random.uniform(-0.05, 0.05)
        start_y = start_pos[1] + np.random.uniform(-0.05, 0.05)
        start_yaw = start_euler[2] + np.random.uniform(-5, 5)
        
        # シーンリセット
        self.scene.reset()
        
        # ロボットの位置設定
        self.robot.set_pose(
            pos=(start_x, start_y, start_pos[2]),
            euler_deg=(0, 0, start_yaw)
        )
        
        # ゲームリセット
        self.game.start()
        
        # ターゲット設定
        self.set_target(target_pos)
        
        # 高さトラッキング用
        self.prev_height = start_pos[2]
        
        # アクション変化率計算用
        self.last_action = None
        
        self.current_scenario_type = scenario_type
        
        return self._get_obs(), {'scenario_type': scenario_type}
    
    def _calculate_tristar_climbing_rewards(self, robot_pos, action, config):
        """
        Tri-star専用の段差登坂報酬を計算
        
        Args:
            robot_pos: ロボット位置 (x, y, z)
            action: アクション [frame_L, frame_R, wheel_L, wheel_R]
            config: ロボット設定
            
        Returns:
            float: 追加報酬
        """
        reward = 0.0
        params = config['reward_params']
        
        # フレーム角度を取得
        frame_angles = self.robot.get_frame_angles()
        if frame_angles is None:
            return 0.0
        
        left_frame, right_frame = frame_angles
        avg_frame_angle = (left_frame + right_frame) / 2.0
        
        # 1. フレーム角度報酬
        # 段差に近い時は前傾（30度）、遠い時は水平（0度）を奨励
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            dist_to_target = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # 段差までの距離に応じて目標角度を調整
            if dist_to_target < params['distance_threshold']:
                # 近い: 前傾を奨励
                target_angle = params['target_frame_angle_near']
            else:
                # 遠い: 水平を奨励
                target_angle = params['target_frame_angle_far']
            
            # 目標角度との差分をペナルティ化
            angle_error = abs(avg_frame_angle - target_angle)
            angle_reward = -angle_error * params['frame_angle_weight'] / 100.0
            reward += angle_reward
        
        # 2. 段差エッジ接近報酬
        # ロボットの前方（上部ホイール位置）が段差エッジ付近にある場合に報酬
        if self.current_target:
            # 段差の高さ（Tier 3なら0.1m）
            step_height = 0.1  # TODO: シナリオから取得
            
            # ロボット前方の推定位置（フレーム半径分前方）
            frame_radius = config['physics']['frame_radius']
            robot_yaw_rad = np.radians(self.robot.get_euler()[2])
            front_x = robot_pos[0] + frame_radius * np.cos(robot_yaw_rad)
            front_y = robot_pos[1] + frame_radius * np.sin(robot_yaw_rad)
            
            # 段差エッジまでの水平距離
            edge_x = 5.5 - 0.1  # Tier 3のエッジ位置（半径4.65m付近）
            dist_to_edge = abs(front_x - edge_x)
            
            # エッジに近く、かつ適切な高さにいる場合に報酬
            if dist_to_edge < 0.1 and abs(robot_pos[2] - step_height) < params['edge_height_tolerance']:
                edge_reward = (0.1 - dist_to_edge) * params['edge_approach_weight']
                reward += edge_reward
        
        # 3. 安定性・ジャンプ抑制ペナルティ
        # Z軸速度（ジャンプ）へのペナルティ
        # get_vel() returns a tensor, so we need to convert it to float
        vel = self.robot.get_vel()
        if hasattr(vel, 'cpu'):
            vel = vel.cpu().numpy()
        z_vel = abs(vel[2])
        
        # Z軸速度ペナルティ (シナリオによって変える)
        z_penalty_weight = params.get('z_velocity_penalty_weight', 0.0)
        z_threshold = 0.05
        
        # 段差シナリオの場合はペナルティを緩和
        if hasattr(self, 'current_scenario_type') and 'step' in self.current_scenario_type:
             z_penalty_weight *= 0.1 # 1/10に緩和
             z_threshold = 0.2 # 閾値も緩和
             
        if z_vel > z_threshold:
            reward -= (z_vel - z_threshold) * z_penalty_weight
            
        # フレーム回転速度へのペナルティ（急激な動作を抑制）
        frame_vel = abs(action[0]) + abs(action[1]) # アクション値（-1.0~1.0）を速度の代用とする
        reward -= frame_vel * params.get('frame_velocity_penalty_weight', 0.0)
        
        # アクション変化率へのペナルティ (Action Rate Penalty)
        if self.last_action is not None:
            action_diff = np.abs(action - self.last_action).sum()
            reward -= action_diff * params.get('action_rate_penalty_weight', 0.0)
        
        # 4. 姿勢制御報酬 (Nose Up)
        # 段差手前では前輪を持ち上げる（ピッチ角をプラスにする）ことを奨励
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            dist_to_target = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # 段差に近い場合 (1.0m以内)
            if dist_to_target < 1.0:
                euler = self.robot.get_euler() # (roll, pitch, yaw) in degrees
                pitch = euler[1]
                
                # ピッチ角がプラス（前上がり）なら報酬
                if pitch > 0:
                    reward += pitch * params.get('pitch_reward_weight', 0.0) / 10.0 # 10度で weight 分の報酬
        
        return reward

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
            
            # 距離報酬 (平地と同じ)
            reward += (self.prev_dist - dist) * 100.0
            self.prev_dist = dist
            
            # 2. 高さ報酬 (段差を登ることを奨励)
            # 現在の高さと前回の高さの差分に報酬を与える
            z_diff = robot_pos[2] - self.prev_height
            if z_diff > 0: # 登っている時のみプラス
                # ロボット設定から高さ報酬の重みを取得
                from xrobocon.robot_configs import get_robot_config
                config = get_robot_config(self.robot_type)
                
                if config['reward_params'].get('use_specialized_rewards', False):
                    height_weight = config['reward_params']['height_gain_weight']
                else:
                    height_weight = 500.0  # デフォルト
                
                reward += z_diff * height_weight
            self.prev_height = robot_pos[2]
            
            # 2.5. Tri-star専用の段差登坂報酬 + 整列報酬
            from xrobocon.robot_configs import get_robot_config
            config = get_robot_config(self.robot_type)
            if config['reward_params'].get('use_specialized_rewards', False):
                specialized_reward = self._calculate_tristar_climbing_rewards(robot_pos, action, config)
                reward += specialized_reward
                
                # 整列報酬 (Alignment Reward)
                # ターゲット方向を向いているか
                if np.linalg.norm(robot_pos[:2] - target_pos[:2]) > 0.1:
                    target_vec = target_pos[:2] - robot_pos[:2]
                    target_angle = np.arctan2(target_vec[1], target_vec[0])
                    current_yaw = np.radians(euler[2])
                    
                    # 角度差 (-pi ~ pi)
                    angle_diff = np.arctan2(np.sin(target_angle - current_yaw), np.cos(target_angle - current_yaw))
                    
                    # 正面を向いているほど報酬 (最大 weight, 真後ろなら 0)
                    align_score = (1.0 - abs(angle_diff) / np.pi)
                    reward += align_score * config['reward_params'].get('alignment_reward_weight', 0.0)
            
            # ターゲット到達判定
            # ターゲットから0.5m以内 かつ 高さがターゲット付近 (ターゲット高さ - 10cm以上)
            if dist < 0.5 and robot_pos[2] > target_pos[2] - 0.1:
                # 到達ボーナス
                reward += 500.0
                terminated = True
            
        # 3. 安定性報酬 (転倒防止)
        # 段差登坂時はある程度の傾きは許容する必要があるが、転倒はNG
        stability_penalty = abs(euler[0]) * 0.02 + abs(euler[1]) * 0.02 # 平地より少し緩く
        reward -= stability_penalty
        
        # 4. 効率化のための追加ペナルティ
        # フレーム使用ペナルティ (段差ではフレームを使う必要があるかもしれないので、平地より緩くするか、あるいは最初は無しで様子見)
        # 一旦、平地と同じ設定にしておく（無駄な動きは抑制）
        if self.robot_type == 'tristar':
            frame_penalty = (abs(action[0]) + abs(action[1])) * 0.1 # 平地(0.5)より緩く
            reward -= frame_penalty
            
        # 速度超過ペナルティ
        if speed > 1.5:
            reward -= (speed - 1.5) * 2.0
        
        # 5. 転倒・落下判定 (終了条件)
        if abs(euler[0]) > 70 or abs(euler[1]) > 70: # 段差なので少し許容
            reward -= 100.0
            terminated = True
            
        if robot_pos[2] < 0.0:
            reward -= 100.0
            terminated = True
            
        # 時間切れ
        if not self.game.is_running:
            truncated = True
            
        # アクション保存
        self.last_action = action.copy()
            
        return self._get_obs(), reward, terminated, truncated, {}
