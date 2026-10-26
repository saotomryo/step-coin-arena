import gymnasium as gym
from gymnasium import spaces
import numpy as np
import genesis as gs
from xrobocon.field import XRoboconField
from xrobocon.robot import XRoboconRobot
from xrobocon.game import XRoboconGame

class XRoboconBaseEnv(gym.Env):
    """
    ステップコイン・アリーナ RL Base Environment
    共通の初期化処理とインターフェースを提供します。
    """
    
    def __init__(self, render_mode=None, robot_type='standard', scenario_path=None, backend=None):
        super().__init__()
        
        from xrobocon.scenario import load_scenario
        specification = load_scenario(scenario_path) if scenario_path else None
        if specification is not None:
            robot_type = specification['robot_type']
        self.render_mode = render_mode
        self.visualize = render_mode == "human"
        self.robot_type = robot_type
        
        # Genesis初期化
        import xrobocon.common as common
        common.setup_genesis(backend=backend)
        
        # 描画設定
        self.renderer = gs.renderers.Rasterizer() if self.visualize or render_mode == "rgb_array" else None
        
        # シーン作成
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
            show_viewer=self.visualize, # rgb_arrayの時はFalse
            renderer=self.renderer,
        )
        
        # カメラ (rgb_array用、またはアクセス用)
        self.camera = self.scene.add_camera(
            res=(640, 480),
            pos=(3.0, -3.0, 2.5),
            lookat=(0.0, 0.0, 0.5),
            fov=40,
            GUI=False
        )
        
        # 地面
        self.plane = self.scene.add_entity(gs.morphs.Plane())
        
        # フィールド
        self.field = XRoboconField(specification['field'] if specification else None)
        self.field.build(self.scene)
        
        # ロボット (初期位置はresetで設定)
        self.robot = XRoboconRobot(self.scene, asset_path=specification.get('robot_asset') if specification else None, robot_type=self.robot_type, pos=(5.0, -1.0, 0.0), euler=(0, 0, 90))
        
        # ゲームロジック
        self.game = XRoboconGame(self.field, self.robot, specification)
        self.field.add_coin_spots(self.scene, self.game.spots)
        
        # 目標マーカー（シーンビルド前に追加）
        self.target_marker = None
        if self.render_mode == "human":
            self.target_marker = self.scene.add_entity(
                gs.morphs.Sphere(
                    pos=(0.0, 0.0, -10.0),  # 初期位置は地下（見えない位置）
                    radius=0.15,
                    fixed=True,
                ),
                material=gs.materials.Rigid(),
                surface=gs.surfaces.Default(color=(0.0, 1.0, 0.0))  # 緑色
            )
        
        self.scene.build()
        self.robot.post_build()
        
        # Action Space
        self.max_torque = 20.0
        if self.robot_type == 'tristar_large':
            self.max_torque = 300.0 # Large robot needs much more torque
        elif self.robot_type == 'rocker_bogie':
            self.max_torque = 40.0  # From robot_configs.py
        elif self.robot_type == 'rocker_bogie_large':
            self.max_torque = 90.0  # From robot_configs.py
            
        if self.robot_type == 'tristar':
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        elif self.robot_type == 'tristar_large':
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        elif self.robot_type in ['rocker_bogie', 'rocker_bogie_large']:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        else:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # Observation Space
        # - Robot Pos (3)
        # - Robot Euler (3)
        # - Robot Vel (3)
        # - Robot Ang Vel (3)
        # - Target Vector (3)
        # - Height Map (5x5 = 25)
        # Total: 40
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(40,), dtype=np.float32)
        
        self.current_target = None
        self.prev_dist = 0.0
        self.prev_height = 0.0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 子クラスで実装
        raise NotImplementedError

    def step(self, action):
        # 子クラスで実装
        raise NotImplementedError

    def render(self):
        pass
        
    def close(self):
        pass

    def set_target(self, target_pos):
        """外部からターゲットを指定"""
        self.current_target = {'pos': target_pos, 'tier': 0}
        robot_pos = self.robot.get_pos().cpu().numpy()
        self.prev_dist = np.linalg.norm(robot_pos[:2] - np.array(target_pos)[:2])
        
        # 目標マーカーの位置を更新
        if self.target_marker is not None:
            self.target_marker.set_pos(target_pos)

    def _get_obs(self):
        pos = self.robot.get_pos().cpu().numpy()
        euler = self.robot.get_euler() # numpy array
        vel = self.robot.get_vel().cpu().numpy()
        ang_vel = self.robot.get_ang_vel().cpu().numpy()
        
        # ターゲットベクトル (相対位置)
        target_vec = np.zeros(3)
        if self.current_target:
            target_pos = np.array(self.current_target['pos'])
            target_vec = target_pos - pos
            
        # Height Map (5x5)
        height_map = self._get_height_map(pos, euler[2])
            
        obs = np.concatenate([pos, euler, vel, ang_vel, target_vec, height_map])
        return obs.astype(np.float32)

    def _get_height_map(self, robot_pos, robot_yaw_deg):
        """
        ロボット周辺の地形高さを取得 (5x5グリッド)
        ロボットの向きに合わせて回転させる（ローカル座標系）
        """
        grid_size = 5
        grid_res = 0.2 # 20cm間隔 -> 1m x 1m の範囲
        half_size = (grid_size - 1) / 2
        
        height_map = []
        
        # ロボットのYaw角 (ラジアン)
        yaw_rad = np.radians(robot_yaw_deg)
        cos_yaw = np.cos(yaw_rad)
        sin_yaw = np.sin(yaw_rad)
        
        for i in range(grid_size):
            for j in range(grid_size):
                # グリッドのローカル座標 (ロボット中心)
                # 前方(X)をプラス、左(Y)をプラスとする
                local_x = (i - half_size) * grid_res + 0.5 # 前方に0.5mオフセット (目の前を見るため)
                local_y = (j - half_size) * grid_res
                
                # グローバル座標に変換
                # global_x = robot_x + (local_x * cos - local_y * sin)
                # global_y = robot_y + (local_x * sin + local_y * cos)
                global_x = robot_pos[0] + (local_x * cos_yaw - local_y * sin_yaw)
                global_y = robot_pos[1] + (local_x * sin_yaw + local_y * cos_yaw)
                
                # 地形高さ取得
                h = self.field.get_terrain_height(global_x, global_y)
                
                # ロボットの足元の高さからの相対高さにする
                rel_h = h - robot_pos[2]
                height_map.append(rel_h)
                
        return np.array(height_map, dtype=np.float32)

    def _apply_action(self, action):
        """共通のアクション適用ロジック"""
        scaled_action = action * self.max_torque
        self.robot.set_actions(scaled_action)
        self.scene.step()
        self.game.update(0.01)
