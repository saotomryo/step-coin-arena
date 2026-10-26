"""
ロボット設定の一元管理
各ロボットタイプごとの物理パラメータ、開始位置、制御パラメータを定義
"""

# ロボット設定辞書
ROBOT_CONFIGS = {
    'standard': {
        'name': '標準ホイール型',
        'xml_file': 'robot.xml',
        'description': '2輪差動駆動の標準ロボット',
        
        # 物理パラメータ
        'physics': {
            'base_radius': 0.1,      # ベース半径 (m)
            'wheel_radius': 0.025,   # ホイール半径 (m)
            'mass': 3.5,             # 総重量 (kg)
        },
        
        # 登坂能力
        'capabilities': {
            'max_step_height': 0.0125,  # 最大登坂高さ (m) = ホイール半径の50%
            'max_slope': 15,             # 最大登坂角度 (度)
        },
        
        # 制御パラメータ
        'control': {
            'action_space_dim': 2,       # アクション空間の次元
            'max_torque': 20.0,          # 最大トルク
            'max_speed': 2.0,            # 最大速度 (m/s)
        },
        
        # 開始位置（平地）
        'start_positions': {
            'flat': {
                'z_offset': 0.05,  # 地面からの高さ (m)
            },
            'step': {
                'z_offset': 0.05,
            }
        }
    },
    
    'tristar': {
        'name': 'Tri-star型（小型）',
        'xml_file': 'robot_tristar.xml',
        'description': '3輪星型ホイール、フレーム回転機構付き（小型版）',
        
        # 物理パラメータ
        'physics': {
            'base_size': (0.16, 0.12, 0.04),  # ベースサイズ (m)
            'wheel_radius': 0.03,              # ホイール半径 (m)
            'frame_radius': 0.08,              # フレーム半径 (m)
            'mass': 3.8,                       # 総重量 (kg)
        },
        
        # 登坂能力
        'capabilities': {
            'max_step_height': 0.05,   # 最大登坂高さ (m) = フレーム半径 - ホイール半径
            'max_slope': 25,           # 最大登坂角度 (度)
        },
        
        # 制御パラメータ
        'control': {
            'action_space_dim': 4,     # [frame_L, frame_R, wheel_L, wheel_R]
            'max_torque': 20.0,
            'max_speed': 1.5,
        },
        
        # 開始位置
        'start_positions': {
            'flat': {
                'z_offset': 0.08,  # 地面からの高さ (m)
            },
            'step': {
                'z_offset': 0.08,
            }
        },
        
        # 報酬パラメータ（段差登坂用）
        'reward_params': {
            'use_specialized_rewards': False,  # 小型は専用報酬を使わない
        }
    },
    
    'tristar_large': {
        'name': 'Tri-star型（大型）',
        'xml_file': 'robot_tristar_large.xml',
        'description': '3輪星型ホイール、フレーム回転機構付き（大型版・段差登坂用）',
        
        # 物理パラメータ
        'physics': {
            'base_size': (0.24, 0.18, 0.06),  # ベースサイズ (m) - 1.5倍
            'wheel_radius': 0.045,             # ホイール半径 (m) - 1.5倍
            'frame_radius': 0.15,              # フレーム半径 (m) - 約2倍
            'mass': 6.0,                       # 総重量 (kg)
        },
        
        # 登坂能力
        'capabilities': {
            'max_step_height': 0.105,  # 最大登坂高さ (m) = 0.15 - 0.045 = 10.5cm
            'max_slope': 30,           # 最大登坂角度 (度)
            'target_tier': 'Tier 3',   # 目標段差
        },
        
        # 制御パラメータ
        'control': {
            'action_space_dim': 4,     # [frame_L, frame_R, wheel_L, wheel_R]
            'max_torque': 30.0,        # 大型化に伴いトルク増加
            'max_speed': 1.2,          # 大型化で少し遅く
        },
        
        # 開始位置
        'start_positions': {
            'flat': {
                'z_offset': 0.12,  # 大型化に伴い高さ調整
            },
            'step': {
                'z_offset': 0.12,
            }
        },
        
        # 報酬パラメータ（段差登坂用）
        'reward_params': {
            'use_specialized_rewards': True,  # 大型は専用報酬を使用
            
            # フレーム角度報酬
            'frame_angle_weight': 2.0,        # フレーム角度報酬の重み
            'target_frame_angle_near': 30.0,  # 段差接近時の目標角度(度)
            'target_frame_angle_far': 0.0,    # 段差から離れた時の目標角度(度)
            'distance_threshold': 0.5,        # 「近い」と判定する距離(m)
            
            # 高さ報酬の調整
            'height_gain_weight': 800.0,      # 高さ獲得報酬（デフォルト500から増加）
            
            # 段差エッジ接近報酬
            'edge_approach_weight': 5.0,      # エッジ接近報酬の重み
            'edge_height_tolerance': 0.05,    # エッジ高さの許容範囲(m)
            
            # 安定性・ジャンプ抑制ペナルティ
            'z_velocity_penalty_weight': 5.0,     # Z軸速度（ジャンプ）へのペナルティ (1.0 -> 5.0)
            'frame_velocity_penalty_weight': 0.05, # フレーム回転速度へのペナルティ
            'action_rate_penalty_weight': 0.1,    # アクション変化率へのペナルティ (急激な操作抑制)
            
            # 姿勢制御報酬
            'pitch_reward_weight': 5.0, # 前傾姿勢（Nose Up）への報酬
            'alignment_reward_weight': 2.0, # ターゲット方向への整列報酬
        }
    },
    # xrobocon/robot_configs.py の ROBOT_CONFIGS 辞書内に追加

    'rocker_bogie': {
        'name': 'ロッカー・ボギー型',
        'xml_file': 'robot_rocker_bogie.xml',
        'description': '火星探査機スタイルの6輪サスペンションロボット',
        
        # 物理パラメータ
        'physics': {
            'base_size': (0.30, 0.20, 0.08),
            'wheel_radius': 0.06,  # 直径12cm
            'mass': 7.0,
        },
        
        # 登坂能力
        'capabilities': {
            'max_step_height': 0.15,   # 車輪直径の1.5倍近くまで登れる可能性あり
            'max_slope': 35,
            'target_tier': 'Tier 2',
        },
        
        # 制御パラメータ
        'control': {
            'action_space_dim': 2,     # [left_drive, right_drive] (6輪を左右でグループ化)
            'max_torque': 40.0,
            'max_speed': 1.0,
        },
        
        # 開始位置
        'start_positions': {
            'flat': {
                'z_offset': 0.15,
            },
            'step': {
                'z_offset': 0.15,
            }
        },
        
        # 報酬パラメータ
        'reward_params': {
            'use_specialized_rewards': False,  # 特殊な姿勢制御なしでも物理的に登れる
        }
    },
    
    'rocker_bogie_large': {
        'name': 'ロッカー・ボギー型（大型）',
        'xml_file': 'robot_rocker_bogie_large.xml',
        'description': '火星探査機スタイルの6輪サスペンションロボット（大型版・段差登坂用）',
        
        # 物理パラメータ
        'physics': {
            'base_size': (0.45, 0.30, 0.12),  # 1.5倍
            'wheel_radius': 0.09,  # 直径18cm（1.5倍）
            'mass': 15.8,  # 約2.25倍（体積比）
        },
        
        # 登坂能力
        'capabilities': {
            'max_step_height': 0.25,   # 車輪直径の1.4倍程度まで登れる
            'max_slope': 35,
            'target_tier': 'Tier 2',
        },
        
        # 制御パラメータ
        'control': {
            'action_space_dim': 2,     # [left_drive, right_drive] (6輪を左右でグループ化)
            'max_torque': 120.0,        # 大型化に伴いトルク増加
            'max_speed': 0.9,          # 大型化で少し遅く
        },
        
        # 開始位置
        'start_positions': {
            'flat': {
                'z_offset': 0.225,  # 1.5倍
            },
            'step': {
                'z_offset': 0.225,
            }
        },
        
        # 報酬パラメータ
        'reward_params': {
            'use_specialized_rewards': False,  # 特殊な姿勢制御なしでも物理的に登れる
        }
    },
}

def get_robot_config(robot_type):
    """
    ロボットタイプから設定を取得
    
    Args:
        robot_type (str): ロボットタイプ ('standard', 'tristar', 'tristar_large')
        
    Returns:
        dict: ロボット設定
    """
    if robot_type not in ROBOT_CONFIGS:
        raise ValueError(f"Unknown robot type: {robot_type}. Available: {list(ROBOT_CONFIGS.keys())}")
    
    return ROBOT_CONFIGS[robot_type]

def get_start_height(robot_type, env_type='flat'):
    """
    ロボットタイプと環境タイプから開始高さを取得
    
    Args:
        robot_type (str): ロボットタイプ
        env_type (str): 環境タイプ ('flat', 'step')
        
    Returns:
        float: 開始高さ (m)
    """
    config = get_robot_config(robot_type)
    return config['start_positions'][env_type]['z_offset']

def get_max_step_height(robot_type):
    """
    ロボットタイプから最大登坂高さを取得
    
    Args:
        robot_type (str): ロボットタイプ
        
    Returns:
        float: 最大登坂高さ (m)
    """
    config = get_robot_config(robot_type)
    return config['capabilities']['max_step_height']

def list_robots():
    """利用可能なロボットタイプの一覧を表示"""
    print("\n利用可能なロボットタイプ:")
    print("="*70)
    for robot_type, config in ROBOT_CONFIGS.items():
        print(f"\n{robot_type}:")
        print(f"  名前: {config['name']}")
        print(f"  説明: {config['description']}")
        print(f"  最大登坂高さ: {config['capabilities']['max_step_height']*100:.1f}cm")
        print(f"  制御次元: {config['control']['action_space_dim']}D")
    print("="*70)

if __name__ == "__main__":
    # テスト
    list_robots()
    
    print("\n\nTristar Large の詳細:")
    config = get_robot_config('tristar_large')
    print(f"  フレーム半径: {config['physics']['frame_radius']*100:.1f}cm")
    print(f"  ホイール半径: {config['physics']['wheel_radius']*100:.1f}cm")
    print(f"  最大登坂: {config['capabilities']['max_step_height']*100:.1f}cm")
    print(f"  開始高さ(平地): {config['start_positions']['flat']['z_offset']*100:.1f}cm")
