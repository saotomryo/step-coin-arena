"""Run with the ROS2 Python interpreter; standard message conversion contracts."""
import unittest
from math import pi
from xrobocon.sensors import SensorSample
from xrobocon.ros2_sensor_messages import to_message,from_message

class StandardMessageTests(unittest.TestCase):
    def test_laser_scan_no_return_and_header(self):
        data=dict(angle_min=-pi,angle_max=pi/2,angle_increment=pi/2,range_min=.05,range_max=12,scan_time=.1,time_increment=0,ranges=[None,1.,2.,None])
        sample=SensorSample(.12,'blue/lidar_link',2,'genesis_sensor',data)
        message=to_message('lidar',sample);stamp,decoded=from_message('lidar',message)
        self.assertEqual(stamp,120000000);self.assertEqual(decoded['ranges'],data['ranges']);self.assertEqual(message.header.frame_id,sample.frame_id)
    def test_imu_marks_orientation_unavailable(self):
        data=dict(linear_acceleration=[0,0,9.8],angular_velocity=[0,0,.1])
        message=to_message('imu',SensorSample(1,'blue/imu_link',0,'genesis_sensor',data))
        self.assertEqual(message.orientation_covariance[0],-1)
        stamp,decoded=from_message('imu',message);self.assertIsNone(decoded['orientation']);self.assertAlmostEqual(decoded['linear_acceleration'][2],9.8)
    def test_joint_state_preserves_names_and_vectors(self):
        data=dict(name=['wheel'],position=[1.],velocity=[2.])
        stamp,decoded=from_message('joint_states',to_message('joint_states',SensorSample(2,'blue/base_link',0,'genesis_sensor',data)))
        self.assertEqual(decoded,data);self.assertEqual(stamp,2000000000)
    def test_contacts_keep_world_coordinates(self):
        data=dict(in_contact=True,coordinate_frame='world',contacts=[{'force':[1,2,3]}])
        stamp,decoded=from_message('contacts',to_message('contacts',SensorSample(3,'world',0,'genesis_sensor',data)))
        self.assertEqual(decoded,data);self.assertEqual(stamp,3000000000)
