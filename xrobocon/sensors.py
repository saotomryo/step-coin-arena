"""Transport-independent sensor samples and synchronous simulation-time sampling.

No ROS executor, wall-clock timers or agent/game logic belongs here.
"""
from copy import deepcopy
from dataclasses import dataclass
from math import floor,isfinite

DEFAULT_HZ={'lidar':10.,'imu':100.,'joint_states':50.,'contacts':20.}

@dataclass(frozen=True)
class SensorSample:
    timestamp: float
    frame_id: str
    sequence: int
    source: str
    data: dict


def validate_sensor_config(config,participants,physics_hz=100.):
    """Normalize per-robot sensors; reject unsupported settings before scene build."""
    if not isinstance(config,dict) or set(config)-set(participants):
        raise ValueError('sensor participants must belong to this simulator')
    result={}
    common={'enabled','hz','pos_offset','euler_offset','link_idx_local'}
    extra={'lidar':{'rays','min_range','max_range'},'imu':{'acc_noise','gyro_noise'},'joint_states':set(),'contacts':set()}
    for robot,spec in config.items():
        if not isinstance(spec,dict) or set(spec)-set(DEFAULT_HZ):raise ValueError('unknown sensor kind')
        result[robot]={}
        for kind,raw in spec.items():
            if not isinstance(raw,dict):raise ValueError('sensor options must be a mapping')
            allowed=common|extra[kind] if kind in ('imu','lidar') else {'enabled','hz'}
            if set(raw)-allowed:raise ValueError(f'unsupported {kind} sensor option')
            options=deepcopy(raw);enabled=options.pop('enabled',True)
            if not isinstance(enabled,bool):raise ValueError('sensor enabled must be boolean')
            hz=options.setdefault('hz',DEFAULT_HZ[kind])
            if isinstance(hz,bool) or not isinstance(hz,(float,int)) or not isfinite(hz) or not 0<hz<=physics_hz:raise ValueError('sensor hz must be positive and at most physics hz')
            for key in ('pos_offset','euler_offset','acc_noise','gyro_noise'):
                if key in options:
                    value=options[key]
                    if not isinstance(value,(list,tuple)) or len(value)!=3 or not all(isinstance(v,(int,float)) and isfinite(v) for v in value):raise ValueError('sensor vectors require three finite numbers')
                    if key.endswith('noise') and min(value)<0:raise ValueError('noise must be nonnegative')
            if 'link_idx_local' in options and (type(options['link_idx_local']) is not int or options['link_idx_local']<0):raise ValueError('nonnegative sensor link index required')
            if kind=='lidar':
                rays=options.setdefault('rays',72)
                if type(rays) is not int or not 4<=rays<=4096:raise ValueError('lidar rays must be 4..4096')
                low=options.setdefault('min_range',.05);high=options.setdefault('max_range',12.)
                if not all(isinstance(v,(float,int)) and isfinite(v) for v in (low,high)) or not 0<=low<high:raise ValueError('invalid lidar range')
                options.setdefault('pos_offset',[0.,0.,.28])
            if enabled:result[robot][kind]=options
    return result


class SensorSuite:
    """Cache measured samples at configured rates; reads never change time.

    Sampling is quantized to physics ticks. Rates describe exposed samples;
    Genesis native sensors may still compute their internal data every step.
    """
    def __init__(self,config,reader):
        self.config=validate_sensor_config(config,config.keys());self.reader=reader;self.reset()
    def reset(self,participant=None,not_before=0.):
        if participant is None:self.cache={};self.deadlines={};self.sequences={}
        for robot in self.config if participant is None else (participant,):
            self.cache[robot]={}
            for kind in self.config[robot]:
                key=(robot,kind);self.deadlines[key]=not_before;self.sequences[key]=0
    def update(self,timestamp):
        if not isfinite(timestamp) or timestamp<0:raise ValueError('finite nonnegative sensor time required')
        for robot,spec in self.config.items():
            for kind,options in spec.items():
                key=(robot,kind)
                if timestamp+1e-9<self.deadlines[key]:continue
                data=self.reader(robot,kind,options)
                frame='world' if kind=='contacts' else f'{robot}/'+('base_link' if kind=='joint_states' else kind+'_link')
                self.cache[robot][kind]=SensorSample(timestamp,frame,self.sequences[key],'genesis_sensor',deepcopy(data))
                self.sequences[key]+=1
                self.deadlines[key]=(floor((timestamp+1e-9)*options['hz'])+1)/options['hz']
    def read(self,participant):return deepcopy(self.cache.get(participant,{}))
