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
            p=[0.5, 0.25, 0.25] # Phase 2: 平地50%, 段差50%
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
            'start_pos': (5.5, 0.0, self.start_z_offset), # Tier 3の外側
            'start_euler': (0, 0, 180),                    # 中心方向
            # Tier 3の中央: (Tier 2半径3.25 + Tier 3半径4.65) / 2 = 3.95m
            'target_pos': (3.95, 0.0, 0.1 + self.start_z_offset), # Tier 3の中央
        },
        {
            'name': 'Scenario 2: 2段目登坂 (Tier 3 -> Tier 2)',
            'type': 'step_tier3_to_tier2',
            'start_pos': (3.95, 0.0, 0.1 + self.start_z_offset),  # Tier 3の中央
            'start_euler': (0, 0, 180),     # 中心方向
            # Tier 2の中央: (Tier 1半径1.85 + Tier 2半径3.25) / 2 = 2.55m
            'target_pos': (2.55, 0.0, 0.35 + self.start_z_offset),  # Tier 2の中央
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
    
    def _calculate_rocker_bogie_climbing_rewards(self, robot_pos, action, config):
        """
        Rocker-Bogie専用の段差登坂報酬を計算
        
        Args:
            robot_pos: ロボット位置 (x, y, z)
            action: アクション [left_drive, right_drive]
            config: ロボット設定
            
        Returns:
            float: 追加報酬
        """
        reward = 0.0
        params = config['reward_params']
        
        # 姿勢情報を取得
        euler = self.robot.get_euler()  # (roll, pitch, yaw) in degrees
        roll, pitch, yaw = euler
        
        # 速度情報を取得
        vel = self.robot.get_vel()
        if hasattr(vel, 'cpu'):
            vel = vel.cpu().numpy()
        
        # 1. 正面アプローチ報酬（ターゲット方向への整列）
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            dist_to_target = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # ターゲットへの方向ベクトル
            to_target = target_pos[:2] - robot_pos[:2]
            target_angle = np.degrees(np.arctan2(to_target[1], to_target[0]))
            
            # ロボットの向きとの角度差
            angle_diff = abs(((target_angle - yaw + 180) % 360) - 180)
            
            # 許容範囲内なら報酬、外ならペナルティ
            alignment_tolerance = params.get('alignment_tolerance', 15.0)
            if angle_diff < alignment_tolerance:
                alignment_reward = (alignment_tolerance - angle_diff) / alignment_tolerance
                reward += alignment_reward * params.get('alignment_reward_weight', 0.0)
            else:
                # 大きくずれている場合はペナルティ
                reward -= (angle_diff - alignment_tolerance) * 0.1
        
        # 2. 速度制御報酬（段差接近時の適切な速度）
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            dist_to_target = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # 段差に近い場合（distance_threshold以内）
            distance_threshold = params.get('distance_threshold', 0.5)
            if dist_to_target < distance_threshold:
                # 現在の速度
                current_speed = np.linalg.norm(vel[:2])
                optimal_speed = params.get('optimal_approach_speed', 0.3)
                
                # 最適速度との差分をペナルティ化
                speed_error = abs(current_speed - optimal_speed)
                speed_reward = -speed_error * params.get('approach_speed_weight', 0.0)
                reward += speed_reward
        
        # 3. ピッチ角報酬（段差登坂時の前傾姿勢）
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            dist_to_target = np.linalg.norm(robot_pos[:2] - target_pos[:2])
            
            # 段差までの距離に応じて目標ピッチ角を調整
            distance_threshold = params.get('distance_threshold', 0.5)
            if dist_to_target < distance_threshold:
                # 近い: 前傾を奨励
                target_pitch = params.get('target_pitch_near', 15.0)
            else:
                # 遠い: 水平を奨励
                target_pitch = params.get('target_pitch_far', 0.0)
            
            # 目標ピッチ角との差分をペナルティ化
            pitch_error = abs(pitch - target_pitch)
            pitch_reward = -pitch_error * params.get('pitch_reward_weight', 0.0) / 10.0
            reward += pitch_reward
        
        # 4. 高さ獲得ボーナス
        # 一定の高さ（Tier到達）時にボーナス
        height_bonus = params.get('height_gain_bonus', 0.0)
        if robot_pos[2] > 0.3:  # Tier 1到達（60cm）
            reward += height_bonus * 0.5
        if robot_pos[2] > 0.6:  # Tier 2到達（35cm）
            reward += height_bonus
        if robot_pos[2] > 0.95:  # Tier 3到達（10cm）
            reward += height_bonus * 1.5
        
        # 5. 安定性ペナルティ
        # ロール角ペナルティ（横転防止）
        max_safe_roll = params.get('max_safe_roll', 20.0)
        if abs(roll) > max_safe_roll:
            roll_penalty = (abs(roll) - max_safe_roll) * params.get('roll_penalty_weight', 0.0)
            reward -= roll_penalty
        
        # Z軸速度ペナルティ（ジャンプ抑制）
        z_vel = abs(vel[2])
        z_threshold = 0.1  # rocker_bogieは段差登坂時にZ速度が出るので閾値を緩く
        if z_vel > z_threshold:
            z_penalty = (z_vel - z_threshold) * params.get('z_velocity_penalty_weight', 0.0)
            reward -= z_penalty
        
        # 6. 継続的推進力報酬
        # 前進速度報酬（停止を防ぐ）
        forward_speed = vel[0]  # X軸方向の速度
        if forward_speed > 0:
            reward += forward_speed * params.get('forward_progress_weight', 0.0)
        
        # 7. アクション平滑化報酬（急激な操作を抑制）
        if self.last_action is not None:
            action_diff = np.abs(action - self.last_action).sum()
            smoothness_reward = -action_diff * params.get('action_smoothness_weight', 0.0)
            reward += smoothness_reward
        
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
            
            # 2.5. 専用報酬の計算
            from xrobocon.robot_configs import get_robot_config
            config = get_robot_config(self.robot_type)
            if config['reward_params'].get('use_specialized_rewards', False):
                # ロボットタイプに応じて専用報酬を計算
                if self.robot_type in ['tristar', 'tristar_large']:
                    specialized_reward = self._calculate_tristar_climbing_rewards(robot_pos, action, config)
                elif self.robot_type == 'rocker_bogie':
                    specialized_reward = self._calculate_rocker_bogie_climbing_rewards(robot_pos, action, config)
                else:
                    specialized_reward = 0.0
                
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
            
            # ターゲット付近でのボーナス報酬（距離に応じて増加）
            if dist < 1.0:
                # 1m以内に入ったら、距離に応じてボーナス
                proximity_bonus = (1.0 - dist) * 50.0  # 最大50.0
                reward += proximity_bonus
                
                # ターゲット付近での減速報酬
                # 距離が近いほど、低速であることに高い報酬
                speed = np.linalg.norm(vel)
                if dist < 0.7:
                    # 目標速度: 距離に応じて0.1～0.3m/s
                    target_speed = 0.1 + dist * 0.3  # 距離0.1m→0.13m/s, 距離0.7m→0.31m/s
                    
                    if speed < target_speed:
                        # 目標速度以下なら報酬
                        slowdown_bonus = (target_speed - speed) * 30.0
                        reward += slowdown_bonus
                    else:
                        # 目標速度より速いとペナルティ
                        slowdown_penalty = (speed - target_speed) * 20.0
                        reward -= slowdown_penalty
            
            # ターゲット到達判定
            # ターゲットから0.5m以内 かつ 高さがターゲット付近 (ターゲット高さ - 10cm以上)
            if dist < 0.5 and robot_pos[2] > target_pos[2] - 0.1:
                # 成功時、速度が低いほど高報酬
                speed = np.linalg.norm(vel)
                speed_bonus = max(0, (0.3 - speed) * 100.0)  # 速度0.3m/s以下でボーナス
                reward += 500.0 + speed_bonus  # 成功報酬 + 速度ボーナス
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
