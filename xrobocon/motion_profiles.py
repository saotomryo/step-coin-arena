"""Selectable motion limits; unchanged baseline and experimental risk profiles."""
from dataclasses import dataclass,asdict,replace
from math import isfinite
@dataclass(frozen=True)
class MotionProfile:
    max_speed:float=.12
    max_turn:float=.3
    climb_speed:float=.12
    align_turn:float=.2
    acceleration:float=.3
    turn_acceleration:float=1.
    settle_time:float=1.

PRESETS={
    'safe':MotionProfile(.10,.25,.10,.15,.20,.7,1.2),
    'fast':MotionProfile(.22,.50,.14,.30,.35,1.2,.5),
}
def resolve_profile(name,max_speed,max_turn,overrides=None):
    if name not in (None,'baseline','safe','fast'):raise ValueError('unknown control profile')
    profile=PRESETS[name] if name in PRESETS else MotionProfile(max_speed=max_speed,max_turn=max_turn,climb_speed=max_speed)
    if overrides:
        if set(overrides)-set(asdict(profile)):raise ValueError('unknown motion limit')
        profile=replace(profile,**overrides)
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) or v<=0 for v in asdict(profile).values()):raise ValueError('positive finite motion limits required')
    return profile
