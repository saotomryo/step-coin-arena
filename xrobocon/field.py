import genesis as gs
import numpy as np

class XRoboconField:
    """ステップコイン・アリーナ 3段フィールド生成クラス"""
    
    def __init__(self, specification=None):
        # フィールド寸法 (単位: メートル)
        # 下段: φ3000mm, 高さ100mm
        # 中段: φ2000mm, 高さ250mm (下段からの相対高さ150mm)
        # 上段: φ1000mm, 高さ250mm (中段からの相対高さ0mm? いや、合計600mmなので...)
        # フィールドの定義 (3段構成) - 画像に基づく寸法修正
        # Tier 1 (Center): 3700mm -> R=1.85m
        # Tier 2 (Middle): 6500mm -> R=3.25m
        # Tier 3 (Outer):  9300mm -> R=4.65m
        # Fence: 13750mm -> R=6.875m
        
        # 地面からの高さ関係を整理
        # Ground (Z=0)
        # Tier 3 Surface: Z=0.1 (仮) -> ここでは相対的な高さとして定義
        # 修正: GenesisのBoxは中心座標指定なので、積み上げ方式で配置
        
        # 実際の配置ロジック (下から順に)
        # Base (Tier 3の下): R=4.65, H=0.1
        # Mid (Tier 2の下): R=3.25, H=0.25 (Tier 3 + 0.25)
        # Top (Tier 1): R=1.85, H=0.1 (Tier 2 + 0.1)
        
        # シンプルにするため、円柱を重ねる
        # Cylinder 1 (Tier 3): R=4.65, H=0.1, Pos Z=0.05
        # Cylinder 2 (Tier 2): R=3.25, H=0.35, Pos Z=0.175
        # Cylinder 3 (Tier 1): R=1.85, H=0.45, Pos Z=0.225
        
        # いや、既存のコードはBoxを使っているようなので、それに合わせるか確認
        # 既存コードは create_tier 関数を使っているはず
        
        # 既存のロジックに合わせてパラメータを調整
        # heightは「厚み」ではなく「地面からの高さ」を意図している？
        # 確認: create_tierの実装を見ると...
        # gs.morphs.Cylinder(height=tier['height']...)
        # なので、heightは円柱の高さ（厚み）。
        
        # Tier 1 (Top): 高さ10cm (0.1m)
        # Tier 2 (Mid): 高さ25cm (0.25m) + Tier 1? いや、段差が重要
        # 画像の段差高さが不明だが、Tier 1が100mmなら、他もそう仮定するか、
        # 以前の設定 (Tier 2=25cm, Tier 3=25cm) を維持するか。
        # とりあえず半径を修正。
        
        # 注: 既存の実装がどうなっているか不明確なので、単純に半径だけ変える
        
        # これだと構造が変わってしまう。既存の値をベースに半径だけ変える。

        # フィールドの定義 (3段構成) - 画像に基づく寸法修正
        # Tier 1 (Center): 3700mm -> R=1.85m
        # Tier 2 (Middle): 6500mm -> R=3.25m
        # Tier 3 (Outer):  9300mm -> R=4.65m
        
        # 高さ設定 (積み上げ)
        # Tier 3 (Base):   高さ 100mm (0.1m) -> Z=0.05 (中心)
        # Tier 2 (Middle): 高さ 350mm (0.35m) -> Z=0.175 (中心)
        # Tier 1 (Top):    高さ 600mm (0.6m) -> Z=0.3 (中心)
        
        self.tiers = [
            # Tier 1 (一番上)
            {'radius': 1.85, 'height': 0.6, 'z': 0.3, 'color': (0.9, 0.9, 0.9)},
            # Tier 2 (真ん中)
            {'radius': 3.25, 'height': 0.35, 'z': 0.175, 'color': (0.6, 0.6, 1.0)}, # 青
            # Tier 3 (一番下)
            {'radius': 4.65, 'height': 0.1, 'z': 0.05, 'color': (1.0, 0.6, 0.6)},   # 赤
        ]

        self.friction = 0.8
        if specification is not None:
            self.tiers = [dict(t, z=t['height']/2, color=(0.7, 0.7, 0.7))
                          for t in specification['tiers']]
            self.friction = specification['friction']

    def build(self, scene, collision_mesh_dir=None):
        """シーンにフィールドエンティティを追加"""
        entities = []
        
        for i, tier in enumerate(self.tiers):
            if collision_mesh_dir is not None:
                from pathlib import Path
                import trimesh
                folder=Path(collision_mesh_dir)
                folder.mkdir(parents=True,exist_ok=True)
                path=folder/f'tier-{i+1}.obj'
                trimesh.creation.cylinder(radius=tier['radius'],height=tier['height'],sections=256).export(path)
                morph=gs.morphs.Mesh(file=str(path.resolve()),pos=(0,0,tier['z']),fixed=True,
                                     convexify=True,decimate=False)
            else:
                morph=gs.morphs.Cylinder(pos=(0,0,tier['z']),height=tier['height'],
                                         radius=tier['radius'],fixed=True)
            # 円筒形の土俵を追加（静的オブジェクトとして固定）
            entity = scene.add_entity(
                morph,
                material=gs.materials.Rigid(
                    friction=self.friction,
                )
            )
            entities.append(entity)
            print(f"Tier {i+1} created: Radius={tier['radius']}m, Height={tier['height']}m, Z={tier['z']}m")
            
        # スロープの追加 (保留 - 難易度が高すぎるため撤去)
        # ramp_width = 0.8
        # ramp_thickness = 0.1
        # def create_ramp(scene, name, start_h, end_h, target_r, target_angle_deg, length):
        #     # 目標地点 (円周上の接点)
        #     rad = np.radians(target_angle_deg)
        #     end_x = target_r * np.cos(rad)
        #     end_y = target_r * np.sin(rad)
            
        #     # 接線方向 (反時計回りなら +90度)
        #     tangent_deg = target_angle_deg + 90
        #     tangent_rad = np.radians(tangent_deg)
        #     dir_x = np.cos(tangent_rad)
        #     dir_y = np.sin(tangent_rad)
            
        #     # 傾斜角
        #     dh = end_h - start_h
        #     slope_rad = np.arcsin(dh / length)
        #     slope_deg = np.degrees(slope_rad)
            
        #     # 中心位置 (XY)
        #     # Endから接線方向逆向きに Length/2 戻る
        #     center_x = end_x - (dir_x * length / 2)
        #     center_y = end_y - (dir_y * length / 2)
            
        #     # 中心位置 (Z)
        #     # Z_pos = (H_top + H_bottom)/2 - (Thickness/2)*cos(slope)
        #     center_z = (start_h + end_h) / 2 - (ramp_thickness / 2) * np.cos(slope_rad)
            
        #     # Euler角 (Genesis: XYZ順と仮定)
        #     # 1. Y軸回転でSlopeをつける (X軸正方向を持ち上げるには -回転)
        #     #    Right-hand rule around Y: Z -> X. Positive rot moves X to -Z (down).
        #     #    So to move X up (Z+), we need Negative Y rotation.
        #     #    euler_y = -slope_deg
        #     # 2. Z軸回転で向きを合わせる
        #     #    euler_z = tangent_deg
            
        #     # Box作成 (X軸長手)
        #     ramp = scene.add_entity(
        #         gs.morphs.Box(
        #             pos=(center_x, center_y, center_z),
        #             size=(length, ramp_width, ramp_thickness),
        #             euler=(0, -slope_deg, tangent_deg),
        #             fixed=True,
        #         ),
        #         material=gs.materials.Rigid(friction=1.0)
        #     )
        #     return ramp

        # # Ramp 1: Ground(0.0) -> Tier 1(0.1). Target R=1.5, Angle=0.
        # # Length=1.5 (勾配緩やか)
        # entities.append(create_ramp(scene, "ramp1", 0.0, 0.1, 1.5, 0, 1.5))
        
        # # Ramp 2: Tier 1(0.1) -> Tier 2(0.35). Target R=1.0, Angle=120.
        # # Length=1.0 (短くしてTier 1内に収める)
        # # Start R approx 1.41 < 1.5
        # entities.append(create_ramp(scene, "ramp2", 0.1, 0.35, 1.0, 120, 1.0))
        
        # # Ramp 3: Tier 2(0.35) -> Tier 3(0.6). Target R=0.5, Angle=240.
        # # Length=1.0 (短くしてTier 2内に収める)
        # entities.append(create_ramp(scene, "ramp3", 0.35, 0.6, 0.5, 240, 1.0))
            
        return entities

    def add_coin_spots(self, scene, spots):
        """コインスポットを可視化する"""
        spot_entities = []
        for spot in spots:
            # コインスポット: 薄い円柱 (マーカー)
            # 色: 未獲得=黄色, 獲得済み=灰色 (動的に変えるのは難しいので、とりあえず黄色)
            # Tierによって色を変える？
            color = (1.0, 1.0, 0.0) # Yellow
            if spot['tier'] == 3:
                color = (1.0, 0.5, 0.0) # Orange
            elif spot['tier'] == 2:
                color = (1.0, 1.0, 0.0) # Yellow
            else:
                color = (0.8, 0.8, 0.0) # Dark Yellow
                
            entity = scene.add_entity(
                gs.morphs.Cylinder(
                    pos=spot['pos'],
                    height=0.01, # 薄い
                    radius=0.15, # 判定半径より少し小さく
                    fixed=True,
                    collision=False, # 衝突判定なし (通り抜け可能)
                ),
                material=gs.materials.Rigid(), # 材質は適当
                surface=gs.surfaces.Default(
                    color=color,
                )
            )
            spot_entities.append(entity)
        return spot_entities
    def get_terrain_height(self, x, y):
        """
        指定された座標(x, y)の地形高さを返す
        
        Args:
            x (float): X座標
            y (float): Y座標
            
        Returns:
            float: 地形の高さ (Z座標)
        """
        # 中心からの距離
        r = np.sqrt(x**2 + y**2)
        
        # Tier判定 (内側から順に)
        # Tier 1: R <= 1.85, H=0.6 (Z=0.3中心 -> 上面は0.6)
        # Tier 2: R <= 3.25, H=0.35 (Z=0.175中心 -> 上面は0.35)
        # Tier 3: R <= 4.65, H=0.1 (Z=0.05中心 -> 上面は0.1)
        # Ground: R > 4.65, H=0.0
        
        # 注: Cylinderのheightは全高なので、上面の高さは height と一致する (底面が0の場合)
        # しかし、Genesisの配置では pos が中心座標なので、上面 = pos.z + height/2
        
        for tier in self.tiers:
            if r <= tier['radius']:
                return tier['z'] + tier['height']/2
        return 0.0
