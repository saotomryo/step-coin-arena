"""
報酬関数モジュール

各報酬要素を独立した関数として定義し、環境側で組み合わせて使用する。
パラメータは環境側から渡すことで、柔軟に調整可能。
"""
import numpy as np


def distance_progress_reward(current_dist, prev_dist, weight=100.0):
    """
    ターゲットへの接近報酬
    
    Args:
        current_dist: 現在のターゲットまでの距離
        prev_dist: 前回のターゲットまでの距離
        weight: 報酬の重み
    
    Returns:
        報酬値
    """
    return (prev_dist - current_dist) * weight


def proximity_bonus_reward(dist, threshold=1.0, max_bonus=50.0):
    """
    ターゲット付近でのボーナス報酬（距離に応じて増加）
    
    Args:
        dist: ターゲットまでの距離
        threshold: ボーナスが発生する距離閾値
        max_bonus: 最大ボーナス値
    
    Returns:
        報酬値
    """
    if dist < threshold:
        return (threshold - dist) / threshold * max_bonus
    return 0.0


def slowdown_reward(dist, speed, dist_threshold=0.7, base_speed=0.1, speed_factor=0.3, 
                    bonus_weight=30.0, penalty_weight=20.0):
    """
    ターゲット付近での減速報酬
    
    Args:
        dist: ターゲットまでの距離
        speed: 現在の速度
        dist_threshold: 減速を要求する距離閾値
        base_speed: 基準速度（最小）
        speed_factor: 距離に応じた速度係数
        bonus_weight: ボーナス報酬の重み
        penalty_weight: ペナルティの重み
    
    Returns:
        報酬値
    """
    if dist >= dist_threshold:
        return 0.0
    
    # 目標速度: 距離に応じて設定
    target_speed = base_speed + dist * speed_factor
    
    if speed < target_speed:
        # 目標速度以下なら報酬
        return (target_speed - speed) * bonus_weight
    else:
        # 目標速度より速いとペナルティ
        return -(speed - target_speed) * penalty_weight


def success_reward(dist, speed=None, dist_threshold=0.5, base_reward=500.0, 
                   speed_bonus_enabled=True, speed_threshold=0.3, speed_bonus_weight=100.0):
    """
    成功報酬（ターゲット到達時）
    
    Args:
        dist: ターゲットまでの距離
        speed: 現在の速度（Noneの場合は速度ボーナスなし）
        dist_threshold: 成功判定の距離閾値
        base_reward: 基本成功報酬
        speed_bonus_enabled: 速度ボーナスを有効にするか
        speed_threshold: 速度ボーナスの閾値
        speed_bonus_weight: 速度ボーナスの重み
    
    Returns:
        (報酬値, 成功フラグ)
    """
    if dist >= dist_threshold:
        return 0.0, False
    
    reward = base_reward
    
    if speed_bonus_enabled and speed is not None:
        # 速度が低いほど高報酬
        speed_bonus = max(0, (speed_threshold - speed) * speed_bonus_weight)
        reward += speed_bonus
    
    return reward, True


def height_gain_reward(current_height, prev_height, weight=500.0):
    """
    高さ獲得報酬（段差登坂時）
    
    Args:
        current_height: 現在の高さ
        prev_height: 前回の高さ
        weight: 報酬の重み
    
    Returns:
        報酬値
    """
    height_diff = current_height - prev_height
    if height_diff > 0:
        return height_diff * weight
    return 0.0


def alignment_reward(robot_yaw, target_angle, tolerance=15.0, weight=10.0):
    """
    ターゲット方向への整列報酬
    
    Args:
        robot_yaw: ロボットの向き（度）
        target_angle: ターゲット方向の角度（度）
        tolerance: 許容角度差（度）
        weight: 報酬の重み
    
    Returns:
        報酬値
    """
    # 角度差を計算（-180～180度）
    angle_diff = abs(((target_angle - robot_yaw + 180) % 360) - 180)
    
    if angle_diff < tolerance:
        # 許容範囲内なら報酬
        alignment_score = (tolerance - angle_diff) / tolerance
        return alignment_score * weight
    else:
        # 許容範囲外ならペナルティ
        return -(angle_diff - tolerance) * 0.1


def stability_penalty(roll, pitch, roll_weight=0.02, pitch_weight=0.02):
    """
    安定性ペナルティ（転倒防止）
    
    Args:
        roll: ロール角（度）
        pitch: ピッチ角（度）
        roll_weight: ロールペナルティの重み
        pitch_weight: ピッチペナルティの重み
    
    Returns:
        ペナルティ値（負の値）
    """
    return -(abs(roll) * roll_weight + abs(pitch) * pitch_weight)


def speed_limit_penalty(speed, max_speed=1.5, penalty_weight=2.0):
    """
    速度超過ペナルティ
    
    Args:
        speed: 現在の速度
        max_speed: 最大許容速度
        penalty_weight: ペナルティの重み
    
    Returns:
        ペナルティ値（負の値）
    """
    if speed > max_speed:
        return -(speed - max_speed) * penalty_weight
    return 0.0


def fall_penalty(robot_z, roll, pitch, min_height=0.0, max_tilt=70.0, penalty=-100.0):
    """
    転倒・落下ペナルティ
    
    Args:
        robot_z: ロボットの高さ
        roll: ロール角（度）
        pitch: ピッチ角（度）
        min_height: 最小許容高さ
        max_tilt: 最大許容傾き（度）
        penalty: ペナルティ値
    
    Returns:
        (ペナルティ値, 終了フラグ)
    """
    # 落下判定
    if robot_z < min_height:
        return penalty, True
    
    # 転倒判定
    if abs(roll) > max_tilt or abs(pitch) > max_tilt:
        return penalty, True
    
    return 0.0, False


class RewardConfig:
    """報酬設定クラス（パラメータをまとめて管理）"""
    
    def __init__(self):
        # 距離報酬
        self.distance_progress_weight = 100.0
        
        # 近接ボーナス
        self.proximity_threshold = 1.0
        self.proximity_max_bonus = 50.0
        
        # 減速報酬
        self.slowdown_dist_threshold = 0.7
        self.slowdown_base_speed = 0.1
        self.slowdown_speed_factor = 0.3
        self.slowdown_bonus_weight = 30.0
        self.slowdown_penalty_weight = 20.0
        
        # 成功報酬
        self.success_dist_threshold = 0.5
        self.success_base_reward = 500.0
        self.success_speed_bonus_enabled = True
        self.success_speed_threshold = 0.3
        self.success_speed_bonus_weight = 100.0
        
        # 高さ報酬
        self.height_gain_weight = 500.0
        
        # 整列報酬
        self.alignment_tolerance = 15.0
        self.alignment_weight = 10.0
        
        # 安定性ペナルティ
        self.stability_roll_weight = 0.02
        self.stability_pitch_weight = 0.02
        
        # 速度制限
        self.speed_limit_max = 1.5
        self.speed_limit_penalty_weight = 2.0
        
        # 転倒・落下
        self.fall_min_height = 0.0
        self.fall_max_tilt = 70.0
        self.fall_penalty = -100.0
