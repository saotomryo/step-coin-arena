"""Validated scenario configuration, separate from legacy GUI/model metadata."""
from copy import deepcopy
import json
from math import isfinite
from pathlib import Path
from xrobocon.robot_configs import get_robot_config


def load_scenario(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema_version') != 1:
        raise ValueError('unsupported schema_version')
    get_robot_config(data['robot_type'])  # fail instead of silently selecting another robot
    if 'robot_asset' in data:
        import xml.etree.ElementTree as ET
        asset = (Path(path).resolve().parent / data['robot_asset']).resolve()
        if not asset.is_file() or asset.suffix.lower() != '.xml':
            raise ValueError('robot_asset must reference an existing XML file')
        if ET.parse(asset).getroot().tag != 'mujoco':
            raise ValueError('robot_asset must be MJCF')
        data['robot_asset'] = str(asset)
    field = data['field']
    tiers = field['tiers']
    previous_radius, previous_height = 0, float('inf')
    for tier in tiers:
        radius, height = tier['radius'], tier['height']
        if not all(isfinite(v) and v > 0 for v in (radius, height)):
            raise ValueError('tier dimensions must be positive finite values')
        if radius <= previous_radius or height >= previous_height:
            raise ValueError('tiers must be ordered top to bottom with increasing radii')
        previous_radius, previous_height = radius, height
    if not isfinite(field['friction']) or field['friction'] < 0:
        raise ValueError('invalid friction')
    for spot in data['coins']:
        if len(spot['pos']) != 3 or not all(isfinite(v) for v in spot['pos']):
            raise ValueError('invalid coin position')
        if not 1 <= spot['tier'] <= len(tiers):
            raise ValueError('coin tier does not exist')
        tier = tiers[spot['tier']-1]
        x, y, z = spot['pos']
        if x*x+y*y > tier['radius']**2 or abs(z-tier['height']) > 0.02:
            raise ValueError('coin must lie on its tier')
        if any(x*x+y*y <= t['radius']**2 for t in tiers[:spot['tier']-1]):
            raise ValueError('coin is covered by a higher tier')
    rules = data['rules']
    for key in ('time_limit', 'coin_radius', 'upper_tier_stay_time'):
        if not isfinite(rules[key]) or rules[key] <= 0:
            raise ValueError('rule values must be positive finite values')
    return deepcopy(data)
