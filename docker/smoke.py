"""Offline container smoke: ROS2 messages, Qt platform and CPU physics."""
import json,os,tempfile
from pathlib import Path
import rclpy
from PySide6.QtWidgets import QApplication
from xrobocon.ros2_runtime import worker_command
from xrobocon.storage import resource_root

def main():
    assert os.environ['XROBOCON_ROS2_RUNTIME']=='system'
    assert worker_command('/app','/tmp/config.json')[0].endswith('python')
    rclpy.init();node=rclpy.create_node('container_smoke');node.destroy_node();rclpy.shutdown()
    os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
    app=QApplication([]);assert app is not None
    from xrobocon.common import setup_genesis
    import genesis as gs
    setup_genesis('cpu')
    scene=gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Box(pos=(0,0,.5),size=(.1,.1,.1)))
    scene.build();scene.step()
    print(json.dumps(dict(success=True,ros2=True,qt=True,genesis_cpu=True,resource_root=str(resource_root()))))
if __name__=='__main__':main()
