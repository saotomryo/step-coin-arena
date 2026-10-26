import genesis as gs
import torch
import numpy as np
import os
from xrobocon.robot_configs import get_robot_config

class XRoboconRobot:
    """ステップコイン・アリーナ ロボットクラス"""
    
    def __init__(self, scene, pos=(0, 0, 0.1), euler=(0, 0, 0), robot_type='standard', asset_path=None):
        self.scene = scene
        self.robot_type = robot_type
        
        # ロボット設定を取得
        try:
            config = get_robot_config(robot_type)
            robot_filename = config['xml_file']
        except ValueError as e:
            print(f"Warning: {e}, using standard robot")
            robot_filename = 'robot.xml'
        
        # アセットパス
        assets_dir = os.path.join(os.path.dirname(__file__), 'assets')
        robot_path = os.path.join(assets_dir, robot_filename)
        
        if asset_path is not None:
            robot_path = os.path.abspath(asset_path)
            if not os.path.isfile(robot_path):
                raise FileNotFoundError(robot_path)
        self.asset_path = robot_path
        if not os.path.exists(robot_path):
            print(f"Warning: Robot file {robot_path} not found, using standard robot")
            robot_path = os.path.join(assets_dir, 'robot.xml')
            self.asset_path = robot_path
        
        # ロボットエンティティを追加
        from xrobocon.inertia_diagnostics import InertiaFrameFilter,model_inertia_frames
        diagnostic=InertiaFrameFilter(model_inertia_frames(robot_path))
        gs.logger.addFilter(diagnostic)
        try:
            self.entity = self.scene.add_entity(
                gs.morphs.MJCF(file=robot_path,pos=pos,euler=euler)
            )
        finally:
            gs.logger.removeFilter(diagnostic)
        if diagnostic.corrected:
            gs.logger.info(f"Verified {diagnostic.corrected} inertia-frame warnings against the rotated MJCF tensors; mass/inertia unchanged.")

        # Use the imported physical COM for quasi-static climbing assistance.
        import mujoco
        model = mujoco.MjModel.from_xml_path(robot_path)
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.base_com = model.body_ipos[base_id].copy() if base_id >= 0 else np.array([-.03, 0., -.05])
        self.n_dofs = 0 # ビルド後に更新
        
    def post_build(self):
        """シーンビルド後の初期化"""
        self.n_dofs = self.entity.n_dofs
        print(f"Robot ({self.robot_type}) initialized with {self.n_dofs} DOFs")
        
    def set_actions(self, actions):
        """
        アクションを適用
        Standard: actions=[left, right] (2次元)
        Tri-star: actions=[frame_l, frame_r, wheel_l, wheel_r] (4次元)
        """
        if self.n_dofs < 2:
            return
            
        forces = torch.zeros(self.n_dofs, device=gs.device)
        
        if self.robot_type in ['rocker_bogie', 'rocker_bogie_large']:
            # Rocker-Bogie: 6 motors (Left: Front, Mid, Rear / Right: Front, Mid, Rear)
            # アクチュエーターに直接トルクを適用
            
            left_cmd = float(actions[0])
            right_cmd = float(actions[1])
            
            # Genesisのアクチュエーター制御
            # set_dofs_velocityではなく、control_dofs_forceを使用
            # ただし、アクチュエーターのみに適用する必要がある
            
            # 配列の最後の6要素がアクチュエーター
            # XMLの順序: FL, ML, RL, FR, MR, RR
            # Left Motors (最初の3つ)
            forces[-6] = left_cmd  # Left Front
            forces[-5] = left_cmd  # Left Middle
            forces[-4] = left_cmd  # Left Rear
            
            # Right Motors (最後の3つ)
            forces[-3] = right_cmd # Right Front
            forces[-2] = right_cmd # Right Middle
            forces[-1] = right_cmd # Right Rear

        elif self.robot_type == 'tristar' or self.robot_type == 'tristar_large':
            # Tri-star: 14 DOFs total (6 Free + 8 Actuated)
            # 0-5: Free Joint
            # 6: Left Frame
            # 7,8,9: Left Wheels
            # 10: Right Frame
            # 11,12,13: Right Wheels
            
            if len(actions) == 4:
                frame_l, frame_r, wheel_l, wheel_r = actions
            else:
                # Fallback for 2D input
                frame_l, frame_r = 0.0, 0.0
                wheel_l, wheel_r = actions[0], actions[1]
            
            # tristar_largeの場合、フレームとホイールで異なるトルクを適用
            # 注意: base_env._apply_action()で既にmax_torqueが掛けられているので、
            # ここでは追加のスケーリングは不要。actionsはそのまま使う。
            
            # Frame motors
            forces[6] = float(frame_l)
            forces[10] = float(frame_r)
            
            # Wheel motors
            forces[7] = float(wheel_l)
            forces[8] = float(wheel_l)
            forces[9] = float(wheel_l)
            
            forces[11] = float(wheel_r)
            forces[12] = float(wheel_r)
            forces[13] = float(wheel_r)
            
        else:
            # Standard: 2 motors (last 2)
            left, right = actions[0], actions[1]
            forces[-2] = float(left)
            forces[-1] = float(right)
            
        # 力を適用
        self.entity.control_dofs_force(forces)
        
    def set_wheel_torques(self, left, right):
        """互換性のためのラッパー"""
        if self.robot_type == 'tristar':
            # 2D入力の場合はホイールのみ駆動
            self.set_actions([0.0, 0.0, left, right])
        else:
            self.set_actions([left, right])
        
    def set_pose(self, pos, euler_deg):
        """位置と姿勢(オイラー角:度)を設定"""
        # 位置設定
        self.entity.set_pos(pos)
        
        # オイラー角(度) -> クォータニオン変換
        # Genesis/MuJoCo uses [w, x, y, z]
        roll = np.radians(euler_deg[0])
        pitch = np.radians(euler_deg[1])
        yaw = np.radians(euler_deg[2])
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        self.entity.set_quat([w, x, y, z])
        
        # 速度リセット
        self.entity.set_dofs_velocity(torch.zeros(self.n_dofs, device=gs.device))

    def get_pos(self):
        """ロボットの位置を取得"""
        return self.entity.get_pos()
        
    def get_euler(self):
        """ロボットのオイラー角(度)を取得"""
        quat = self.entity.get_quat().cpu().numpy() # [w, x, y, z]
        w, x, y, z = quat
        
        # クォータニオン -> オイラー角変換
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        
        return np.degrees([roll_x, pitch_y, yaw_z])
        
    def get_vel(self):
        """ロボットの線形速度を取得"""
        # 浮遊ベース(freejoint)の場合、DOF速度の最初3つが線形速度
        if self.n_dofs >= 6:
            return self.entity.get_dofs_velocity()[:3]
        return self.entity.get_vel() # フォールバック
        
    def get_ang_vel(self):
        """ロボットの角速度を取得"""
        # 浮遊ベース(freejoint)の場合、DOF速度の次の3つが角速度
        if self.n_dofs >= 6:
            return self.entity.get_dofs_velocity()[3:6]
        return torch.zeros(3, device=gs.device) # フォールバック
    
    def get_frame_angles(self):
        """
        Tri-starロボットのフレーム角度を取得
        
        Returns:
            tuple: (left_frame_angle, right_frame_angle) in degrees
                   None if not a tristar robot
        """
        if self.robot_type not in ['tristar', 'tristar_large']:
            return None
        
        # Tri-starロボットのDOF構成:
        # 0-5: freejoint (base position + orientation)
        # 6: left_tristar_joint
        # 7-9: left_wheel_1/2/3_joint
        # 10: right_tristar_joint
        # 11-13: right_wheel_1/2/3_joint
        
        dof_pos = self.entity.get_dofs_position().cpu().numpy()
        
        if len(dof_pos) >= 14:
            left_frame = np.degrees(dof_pos[6])   # ラジアン -> 度
            right_frame = np.degrees(dof_pos[10])
            return left_frame, right_frame
        
        return 0.0, 0.0  # フォールバック
        
    def get_camera_frame(self):
        """カメラ画像を取得 (未実装: Genesisのレンダリングパイプラインが必要)"""
        # Genesisでのカメラ画像取得はシーンのレンダラー経由で行う必要がある
        # ここではプレースホルダー
        pass

