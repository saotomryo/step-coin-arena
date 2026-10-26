"""Structured persistent strategy contract and replaceable Codex plan inference."""
from copy import deepcopy
from math import isfinite
from xrobocon.coin_selector import CodexCoinSelector

GUARDS = {
    'target_coin_unavailable': {'type': 'boolean', 'enum': [True]},
    'max_plan_seconds': {'type': 'number', 'minimum': 5, 'maximum': 120},
    'min_remaining_seconds': {'type': 'number', 'minimum': 0, 'maximum': 60},
    'max_blocked_seconds': {'type': 'number', 'minimum': 1, 'maximum': 20},
    'score_gap_change': {'type': ['number', 'null'], 'minimum': 1, 'maximum': 30},
}
PLAN_SCHEMA = dict(type='object', additionalProperties=False,
    properties=dict(strategy=dict(type='string', minLength=1, maxLength=240),
        coin_ids=dict(type='array', items=dict(type='integer'), minItems=1, maxItems=18),
        stop_conditions=dict(type='object',properties=GUARDS,required=list(GUARDS),additionalProperties=False)),
    required=['strategy','coin_ids','stop_conditions'])


def validate_plan(result, coins, stage):
    if not isinstance(result,dict) or set(result) != {'strategy','coin_ids','stop_conditions'}:
        raise ValueError('strategy, coin_ids and stop_conditions required')
    if not isinstance(result['strategy'],str) or not 1 <= len(result['strategy'].strip()) <= 240:
        raise ValueError('strategy description must be 1..240 characters')
    ids=result['coin_ids'];available={c['id']:c for c in coins if c['status']=='available' and c['tier']<=3-stage}
    if not isinstance(ids,list) or not 1<=len(ids)<=18 or any(type(i) is not int or i not in available for i in ids) or len(ids)!=len(set(ids)):
        raise ValueError('plan requires unique available reachable coin IDs')
    tiers=[available[i]['tier'] for i in ids]
    if any(b>a for a,b in zip(tiers,tiers[1:])):raise ValueError('plan cannot descend to a lower tier')
    guards=result['stop_conditions']
    if not isinstance(guards,dict) or set(guards)!=set(GUARDS):raise ValueError('invalid stop condition fields')
    if guards['target_coin_unavailable'] is not True:raise ValueError('unavailable target guard is mandatory')
    for key,bounds in GUARDS.items():
        if key=='target_coin_unavailable':continue
        value=guards[key]
        if key=='score_gap_change' and value is None:continue
        if type(value) not in (int,float) or not isfinite(value) or not bounds['minimum']<=value<=bounds['maximum']:
            raise ValueError('invalid stop condition: '+key)
    return deepcopy(result)


class CodexPlanSelector(CodexCoinSelector):
    schema=PLAN_SCHEMA
    instruction=('Plan a persistent robot competition strategy. Return a short strategy description, '
        'an ordered nonempty list of available reachable coin_ids, and every stop_conditions field. '
        'Choose the strategy freely to maximize competitive score within remaining time. '
        'Do not list waypoints or motor actions. Coin tiers must be nonincreasing (3 then 2 then 1); no descents. '
        'The rule controller continues the coin sequence without calling you after every acquisition. '
        'target_coin_unavailable must be true for any remaining planned coin. '
        'max_plan_seconds: 5..120 simulation seconds after acceptance. '
        'min_remaining_seconds: 0..60, triggers once when remaining time crosses downward through this threshold. '
        'max_blocked_seconds: 1..20 seconds without movement while attempting to move. '
        'score_gap_change: null to disable, or 1..30 absolute change of own-minus-opponent score from plan acceptance. '
        'Choose thresholds suited to the route and time remaining. Return only schema JSON.')

    def validate_response(self,result,payload):
        validate_plan(result,payload['state']['coins'],payload['movement_rules']['current_stage'])
