"""Select the existing Pixi sidecar or an explicitly sourced system ROS2."""
import os
import sys
from pathlib import Path

def worker_command(root,config_path):
    root=Path(root)
    worker=[str(root/'scripts/ros2_worker.py'),'--role','bridge','--config',str(Path(config_path).resolve())]
    runtime=os.environ.get('XROBOCON_ROS2_RUNTIME','pixi')
    if runtime=='system':return [sys.executable,*worker]
    if runtime!='pixi':raise ValueError('XROBOCON_ROS2_RUNTIME must be pixi or system')
    return [str(root/'.tools/pixi'),'run','--frozen','--no-install','--manifest-path',str(root/'ros2/pixi.toml'),'python',*worker]
