"""Validated per-robot control settings; no simulator or UI dependency."""
import json
from copy import deepcopy
from math import isfinite
from pathlib import Path
from xrobocon.motion_profiles import resolve_profile
DRIVE_DEFAULTS=dict(kp=.3,ki=.5,kd=0.,steering_kp=12.,steering_kd=.8,steering_torque_limit=2.,frame_kp=20.,frame_kd=2.,frame_torque_limit=3.,turn_gain=1.)

def load_tuning(path):
    value=json.loads(Path(path).read_text())
    if not isinstance(value,dict) or value.get('schema_version')!=1:raise ValueError('unsupported tuning schema')
    drive=value.get('drive',{})
    if not isinstance(drive,dict) or set(drive)-set(DRIVE_DEFAULTS):raise ValueError('unknown drive setting')
    drive=dict(DRIVE_DEFAULTS,**drive)
    for key,v in drive.items():
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) or v<0 or (key in ('steering_kp','steering_torque_limit','frame_kp','frame_torque_limit','turn_gain') and v==0):raise ValueError('finite nonnegative gains and positive steering limits required')
    motion=value.get('motion',{})
    if not isinstance(motion,dict) or set(motion)-{'baseline','safe','fast'}:raise ValueError('unknown motion profile')
    for name,limits in motion.items():
        if not isinstance(limits,dict):raise ValueError('motion overrides must be objects')
        resolve_profile(name,.12,.3,limits)
    result=deepcopy(value);result.update(drive=drive,motion=deepcopy(motion))
    return result

def select_candidate(results):
    """Never recommend a failed, errored or partially evaluated candidate."""
    eligible=[r for r in results if r.get('complete') and r.get('passed') and not r.get('error')]
    return min(eligible,key=lambda r:r['simulation_seconds'])['candidate'] if eligible else None

def tuning_signature(tuning):
    import hashlib
    return hashlib.sha256(json.dumps(tuning,sort_keys=True).encode()).hexdigest()

def verify_asset_binding(tuning,asset):
    """Exported calibrations belong to the exact physical model they tested."""
    import hashlib
    expected=tuning.get('calibration',{}).get('asset',{}).get('sha256')
    if expected and hashlib.sha256(Path(asset).read_bytes()).hexdigest()!=expected:
        raise ValueError('calibration asset differs; reevaluate this physical model')

def verify_comparison(plan,expected):
    if plan['asset']['sha256']!=expected['asset']['sha256'] or plan.get('tuning_signature')!=expected.get('tuning_signature'):
        raise ValueError('model or tuning changed during comparison')

def resolve_locomotion(tuning,scenario):
    """Compile the robot's known map from the selected field, never a second map."""
    settings=deepcopy(tuning.get('locomotion'))
    if settings and settings.get('kind')=='tristar':
        from xrobocon.scenario import load_scenario
        tiers=sorted(load_scenario(scenario)['field']['tiers'],key=lambda t:t['radius'],reverse=True)
        if len(tiers)!=3:raise ValueError('carrier tier navigation requires three tiers')
        settings.update(tiers=[t['radius'] for t in tiers],heights=[t['height'] for t in tiers])
    return settings
