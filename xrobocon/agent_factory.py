"""Agent construction shared by direct baselines and ROS2 worker processes."""
from xrobocon.strategic_agent import StrategicTierAgent
from xrobocon.codex_tier_agent import CodexTierAgent
from xrobocon.agent import FixedCoinAgent
from xrobocon.tier_agent import FixedTierAgent
from xrobocon.adaptive_tier_agent import AdaptiveTierAgent

def make_agent(spec):
    options=dict(spec);kind=options.pop('kind','flat');mobility=options.pop('locomotion',None)
    factories={'flat':FixedCoinAgent,'tier':FixedTierAgent,'adaptive_tier':AdaptiveTierAgent,'codex_tier':CodexTierAgent,'strategic_tier':StrategicTierAgent}
    if kind not in factories:raise ValueError('unknown agent kind')
    agent=factories[kind](**options)
    if mobility:
        from xrobocon.cluster_agent import ClusterMobilityAgent
        mobility=dict(mobility)
        if mobility.pop('kind',None)!='tristar':raise ValueError('unknown locomotion adapter')
        agent=ClusterMobilityAgent(agent,**mobility)
    return agent
