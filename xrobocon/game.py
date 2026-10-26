import numpy as np
import time
from xrobocon.competition import CompetitionManager

class XRoboconGame:
    """ステップコイン・アリーナ ゲームロジック管理クラス"""
    
    def __init__(self, field, robot, specification=None, robot_height_offset=0.0):
        if not np.isfinite(robot_height_offset) or robot_height_offset<0:
            raise ValueError('robot height offset must be finite and nonnegative')
        self.robot_height_offset=robot_height_offset
        self.specification = specification
        self.field = field
        self.robot = robot
        
        # ゲーム設定
        self.time_limit = 120.0 # 2分
        self.coin_radius = 0.2  # コインスポットの判定半径
        self.upper_tier_stay_time = 2.0 # 上段での滞在必要時間
        
        if specification is not None:
            for name, value in specification['rules'].items():
                if name in ('time_limit', 'coin_radius', 'upper_tier_stay_time'):
                    setattr(self, name, value)

        # 状態変数
        self.score = 0
        self.elapsed_time = 0.0
        self.start_time = None
        self.is_running = False
        
        # コインスポットの状態
        # {'id': 0, 'pos': (x, y, z), 'tier': 1, 'collected': False, 'stay_timer': 0.0}
        self.spots = []
        self._init_spots()
        
    def _init_spots(self):
        """コインスポットの初期化"""
        self.spots = []
        if self.specification is not None:
            self.spots = [dict(s, id=s.get('id',i), pos=tuple(s['pos']),
                               collected=False, stay_timer=0.0)
                          for i, s in enumerate(self.specification['coins'])]
            return
        spot_id = 0
        
        # 下段 (Tier 3 - Bottom): 8個 (Red)
        # R=4.0m (Tier 3 R=4.65m の内側)
        for i in range(8):
            angle = np.radians(i * (360/8))
            self.spots.append({
                'id': spot_id,
                'pos': (4.0 * np.cos(angle), 4.0 * np.sin(angle), 0.105), # Z=0.1 + margin
                'tier': 3, # Bottom
                'collected': False,
                'stay_timer': 0.0
            })
            spot_id += 1
            
        # 中段 (Tier 2 - Middle): 8個 (Blue)
        # R=2.55m (Tier 2 R=3.25m の内側)
        for i in range(8):
            angle = np.radians(i * (360/8) + 22.5) # 位相をずらす
            self.spots.append({
                'id': spot_id,
                'pos': (2.55 * np.cos(angle), 2.55 * np.sin(angle), 0.355), # Z=0.35 + margin
                'tier': 2, # Middle
                'collected': False,
                'stay_timer': 0.0
            })
            spot_id += 1
            
        # 上段 (Tier 1 - Top): 4個 (Yellow)
        # R=1.0m (Tier 1 R=1.85m の内側)
        for i in range(4):
            angle = np.radians(i * (360/4))
            self.spots.append({
                'id': spot_id,
                'pos': (1.0 * np.cos(angle), 1.0 * np.sin(angle), 0.605), # Z=0.6 + margin
                'tier': 1, # Top
                'collected': False,
                'stay_timer': 0.0
            })
            spot_id += 1
            
    def start(self):
        """Build the authority from current settings; retain the legacy demo API."""
        self.start_time = time.time()
        self._init_spots()
        definitions=[]
        for spot in self.spots:
            coin=dict(spot)
            coin.setdefault('radius',self.coin_radius)
            coin.setdefault('dwell',self.upper_tier_stay_time if coin['tier']==1 else 0)
            definitions.append(coin)
        self.manager=CompetitionManager(definitions,time_limit=self.time_limit)
        self.manager.start()
        self.spots=self.manager.coins
        self.score=0;self.elapsed_time=0.;self.is_running=True
        print("Game Started!")

    def update(self, dt):
        """Adapt robot base position to ground-reference XYZ for the authority."""
        if not self.is_running:return
        pos=self.robot.get_pos() if self.robot is not None else None
        if pos is not None:
            if hasattr(pos,'cpu'):pos=pos.cpu().numpy()
            positions={'self':[float(pos[0]),float(pos[1]),float(pos[2])-self.robot_height_offset]}
        else:positions={}
        previous=len(self.manager.events)
        self.manager.advance(dt,positions)
        self.elapsed_time=self.manager.elapsed
        self.is_running=self.manager.status=='running'
        self.score=self.manager.scores['self']
        for event in self.manager.events[previous:]:
            if event['type']=='coin_collected':
                print(f"Spot Collected! ID={event['coin_id']}, Points={event['points']}, Total Score={event['score']}")
            elif event['type']=='competition_finished':print("Time Up!")

    def get_info(self):
        """表示用情報を返す"""
        return {
            'time': self.time_limit - self.elapsed_time,
            'score': self.score,
            'collected_count': sum(1 for s in self.spots if s['collected']),
            'total_spots': len(self.spots)
        }

    def get_competition_state(self):
        """Detached authoritative state for planning/logging, after start()."""
        if not hasattr(self,'manager'):raise RuntimeError('start competition first')
        return self.manager.snapshot()
