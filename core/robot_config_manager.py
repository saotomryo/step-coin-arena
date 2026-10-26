"""
ロボット設定管理モジュール
各ロボットの設定をJSONファイルで管理します
"""
import json
import os
from typing import Dict, Any

class RobotConfigManager:
    """ロボット設定を管理するクラス"""
    
    def __init__(self, config_dir: str = None):
        if config_dir is None:
            # デフォルトはプロジェクトルート/configs/robots/
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.config_dir = os.path.join(project_root, 'configs', 'robots')
        else:
            self.config_dir = config_dir
        
        # ディレクトリが存在しない場合は作成
        os.makedirs(self.config_dir, exist_ok=True)
    
    def get_config_path(self, robot_name: str) -> str:
        """ロボット設定ファイルのパスを取得"""
        return os.path.join(self.config_dir, f"{robot_name}.json")
    
    def load_config(self, robot_name: str) -> Dict[str, Any]:
        """ロボット設定を読み込む"""
        config_path = self.get_config_path(robot_name)
        
        if os.path.exists(config_path):
            with open(config_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        else:
            # ファイルが存在しない場合はデフォルト設定を返す
            return self.get_default_config(robot_name)
    
    def save_config(self, robot_name: str, config: Dict[str, Any]) -> None:
        """ロボット設定を保存"""
        config_path = self.get_config_path(robot_name)
        
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
    
    def get_default_config(self, robot_name: str) -> Dict[str, Any]:
        """デフォルト設定を取得（robot_configs.pyから）"""
        from xrobocon.robot_configs import get_robot_config
        
        base_config = get_robot_config(robot_name)
        
        # 追加のメタデータ
        config = {
            'robot_name': robot_name,
            'model_file': f'xrobocon_ppo_{robot_name}_step.xml',
            'trained_models': [
                f'models/xrobocon_ppo_{robot_name}_step.zip',
            ],
            'physics': base_config.get('physics', {}),
            'control': base_config.get('control', {}),
            'reward_params': base_config.get('reward_params', {})
        }
        
        return config
    
    def list_robots(self) -> list:
        """設定ファイルが存在するロボットのリストを取得"""
        robots = []
        if os.path.exists(self.config_dir):
            for filename in os.listdir(self.config_dir):
                if filename.endswith('.json'):
                    robots.append(filename[:-5])  # .jsonを除く
        return robots
    
    def initialize_all_configs(self) -> None:
        """全ロボットのデフォルト設定ファイルを作成"""
        from xrobocon.robot_configs import ROBOT_CONFIGS
        
        for robot_name in ROBOT_CONFIGS.keys():
            config_path = self.get_config_path(robot_name)
            if not os.path.exists(config_path):
                default_config = self.get_default_config(robot_name)
                self.save_config(robot_name, default_config)
