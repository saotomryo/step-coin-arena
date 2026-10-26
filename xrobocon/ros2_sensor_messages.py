"""ROS standard sensor message conversions; imported only in the ROS environment."""
from math import isfinite
from sensor_msgs.msg import LaserScan,Imu,JointState
from std_msgs.msg import String
import json

TYPES={'lidar':LaserScan,'imu':Imu,'joint_states':JointState,'contacts':String}
TOPICS={'lidar':'scan','imu':'imu/data','joint_states':'joint_states','contacts':'contacts'}

def stamp_key(timestamp):return round(timestamp*1_000_000_000)
def set_stamp(stamp,timestamp):stamp.sec,stamp.nanosec=divmod(stamp_key(timestamp),1_000_000_000)
def get_stamp(stamp):return stamp.sec*1_000_000_000+stamp.nanosec

def to_message(kind,sample):
    d=sample.data;msg=TYPES[kind]()
    if kind=='contacts':
        msg.data=json.dumps(dict(timestamp=sample.timestamp,frame_id=sample.frame_id,data=d),allow_nan=False);return msg
    msg.header.frame_id=sample.frame_id;set_stamp(msg.header.stamp,sample.timestamp)
    if kind=='lidar':
        for f in ('angle_min','angle_max','angle_increment','range_min','range_max','scan_time','time_increment'):setattr(msg,f,float(d[f]))
        msg.ranges=[float(v) if v is not None else float('inf') for v in d['ranges']]
    elif kind=='imu':
        for field,key in (('linear_acceleration','linear_acceleration'),('angular_velocity','angular_velocity')):
            vector=getattr(msg,field);vector.x,vector.y,vector.z=map(float,d[key])
        msg.orientation_covariance[0]=-1.
    else:
        msg.name=list(d['name']);msg.position=list(map(float,d['position']));msg.velocity=list(map(float,d['velocity']))
    return msg

def from_message(kind,msg):
    if kind=='contacts':
        value=json.loads(msg.data);return stamp_key(value['timestamp']),value['data']
    stamp=get_stamp(msg.header.stamp)
    if kind=='lidar':
        data={f:getattr(msg,f) for f in ('angle_min','angle_max','angle_increment','range_min','range_max','scan_time','time_increment')}
        data['ranges']=[float(v) if isfinite(v) else None for v in msg.ranges]
    elif kind=='imu':
        data={field:[getattr(getattr(msg,field),axis) for axis in ('x','y','z')] for field in ('linear_acceleration','angular_velocity')};data['orientation']=None
    else:data=dict(name=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
    return stamp,data
