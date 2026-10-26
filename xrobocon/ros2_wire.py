"""Explicit JSON boundary for game-specific data, not a DDS replacement."""
from dataclasses import asdict
from math import isfinite
from xrobocon.agent import Observation,PublicCompetition
from xrobocon.navigation import Pose2D,VelocityCommand
from xrobocon.sensors import SensorSample

def encode_observation(obs):
    data=asdict(obs)
    # Standard ROS sensor messages carry the measurements, not this envelope.
    data['sensors']={k:{f:v for f,v in asdict(s).items() if f!='data'} | {'calibration':{f:v for f,v in s.data.items() if f in ('pos_offset','euler_offset','link_idx_local')}} for k,s in obs.sensors.items()}
    return data

def decode_observation(data,samples):
    public=dict(data['competition']);public['coins']=tuple(public['coins'])
    sensors={}
    for kind,meta in data['sensors'].items():
        fields={k:meta[k] for k in ('timestamp','frame_id','sequence','source')}
        measurement=dict(samples[kind]);measurement.update(meta['calibration'])
        sensors[kind]=SensorSample(**fields,data=measurement)
    return Observation(Pose2D(**data['pose']),data['timestamp'],'ros2/'+data['source'],PublicCompetition(**public),data['telemetry'],data['other_robots'],sensors)

def decode_command(data):
    if set(data)!=set(VelocityCommand.__dataclass_fields__):raise ValueError('invalid command fields')
    if any(isinstance(data[k],bool) or not isinstance(data[k],(int,float)) or not isfinite(data[k]) for k in ('linear','angular')):raise ValueError('nonfinite command')
    if type(data['straighten']) is not bool or type(data['reset_drive']) is not bool:raise ValueError('invalid drive flags')
    if data['climb_stage'] is not None and (type(data['climb_stage']) is not int or data['climb_stage'] not in (0,1,2)):raise ValueError('invalid climb stage')
    target=data['frame_target']
    if target is not None and (isinstance(target,bool) or not isinstance(target,(int,float)) or not isfinite(target)):raise ValueError('invalid frame target')
    return VelocityCommand(**data)
