import unittest
from dataclasses import replace
from math import pi,cos,sin
from xrobocon.route_planner import RingCoveragePlanner
from xrobocon.adaptive_tier_agent import AdaptiveTierAgent
from xrobocon.agent import Observation,PublicCompetition
from xrobocon.navigation import Pose2D
class RouteStrategyTests(unittest.TestCase):
    def coins(self):
        return tuple(dict(id=i,pos=[r*cos(a),r*sin(a),h],tier=t,status='available',owner=None,points=1,dwell=0,spawn_count=1) for i,(r,a,h,t) in enumerate([(4,0,.105,3),(4,.3,.105,3),(2.6,0,.355,2),(.95,0,.605,1)]))
    def obs(self,coins=None):return Observation(Pose2D(4,0,pi),0,'test',PublicCompetition('running',180,{},coins or self.coins()),dict(speed=0,ground_z=.1,steering=[0]*6))
    def test_budget_maximizes_count_and_removes_collected_coins(self):
        p=RingCoveragePlanner(1);coins=self.coins()[:2]
        self.assertEqual(p.plan(coins,0,4,1,2).coin_ids,(0,1))
        self.assertEqual(p.plan(coins,0,4,1,.5).coin_ids,(0,))
        self.assertEqual(p.plan((dict(coins[0],status='collected'),coins[1]),0,4,1,2).coin_ids,(1,))
    def test_opponent_acquisition_replans_from_actual_pose(self):
        a=AdaptiveTierAgent();o=self.obs();a.act(o,.01);a.phase='coins';a.needs_plan=True;a.act(o,.01)
        coins=list(o.competition.coins);coins[0]=dict(coins[0],status='collected',owner='opponent')
        a.act(replace(o,competition=replace(o.competition,coins=tuple(coins),collection_counts={'opponent':1})),.01)
        self.assertNotIn(0,a.route);self.assertIn(0,a.plan_event['excluded_ids']);self.assertEqual(a.plan_event['reason'],'coin_acquisition')
    def test_crossing_stays_radial_and_gate_is_current_position(self):
        a=AdaptiveTierAgent();o=self.obs();a.act(o,.01);a.phase='climb'
        self.assertEqual(a.act(replace(o,pose=Pose2D(5.2,.02,pi-.02)),.01).angular,0)
        a.phase='coins';coins=[dict(c,status='collected') if c['tier']==3 else c for c in self.coins()]
        o=replace(o,pose=Pose2D(0,4,-pi/2),competition=replace(o.competition,coins=tuple(coins)))
        a.act(o,.01);self.assertEqual(a.stage,1);self.assertAlmostEqual(a.gate_angle,pi/2)
    def test_new_spawn_wakes_waiting_agent(self):
        a=AdaptiveTierAgent();o=self.obs();a.act(o,.01);a.phase='coins';a.stage=2
        empty=tuple(dict(c,status='collected') for c in self.coins());a.act(replace(o,competition=replace(o.competition,coins=empty)),.01)
        self.assertEqual(a.phase,'wait')
        alive=tuple(dict(c,status='available',spawn_count=2) if c['tier']==1 else c for c in empty)
        a.act(replace(o,competition=replace(o.competition,coins=alive)),.01)
        self.assertEqual(a.phase,'coins');self.assertEqual(a.route,(3,))

    def test_far_coin_can_be_skipped_to_collect_more_upstairs(self):
        from xrobocon.route_planner import UpwardCoveragePlanner
        groups=[[dict(id=0,pos=[-4,0,.105],tier=3,status='available')],[dict(id=i,pos=[2.6*cos(a),2.6*sin(a),.355],tier=2,status='available') for i,a in [(1,0),(2,.2)]],[dict(id=3,pos=[.95,0,.605],tier=1,status='collected')]]
        plan=UpwardCoveragePlanner().plan(groups,0,0,.22,.14,40)
        self.assertEqual(plan.current.coin_ids,());self.assertEqual(plan.total_count,2)
        self.assertEqual(set(plan.tier_routes[1].coin_ids),{1,2})

    def test_alignment_anchors_normal_at_actual_pose_before_crossing(self):
        a=AdaptiveTierAgent(control_profile='fast');o=self.obs();a.act(o,.01);a.phase='straighten'
        o=replace(o,pose=Pose2D(0,4,-pi/2))
        for _ in range(50):a.act(o,.01)
        self.assertEqual(a.phase,'climb');self.assertAlmostEqual(a.gate_angle,pi/2)

    def test_upward_search_matches_small_bruteforce_oracle(self):
        import itertools
        from math import atan2,hypot
        from xrobocon.route_planner import UpwardCoveragePlanner
        groups=[[dict(id=i,pos=[r*cos(a),r*sin(a),h],status='available',points=p,dwell=d) for i,a,p,d in items] for r,h,items in [(4,.1,[(0,0,1,0),(1,.4,1,0)]),(2.6,.35,[(2,-.3,2,0),(3,.5,2,0)]),(.95,.6,[(4,0,5,2)])]]
        speed=.22;climb=.14;fraction=.75;budget=70
        def oracle(level,start,remaining):
            best=(0,0,0.)
            for n in range(len(groups[level])+1):
                for route in itertools.permutations(groups[level],n):
                    spent=0.;angle=start;points=0
                    for coin in route:
                        target=atan2(coin['pos'][1],coin['pos'][0]);delta=abs((target-angle+pi)%(2*pi)-pi)
                        distance=(2*hypot(*coin['pos'][:2])*sin(delta/2) if level==2 else hypot(*coin['pos'][:2])*delta)
                        spent+=distance/(speed*fraction)+coin['dwell']+.25;points+=coin['points'];angle=target
                    if spent>remaining:continue
                    rank=(n,points,-spent)
                    if level<2:
                        cross=abs(hypot(*groups[level][0]['pos'][:2])-hypot(*groups[level+1][0]['pos'][:2]))/climb+10
                        if cross<=remaining-spent:
                            future=oracle(level+1,angle,remaining-spent-cross)
                            if future[0]:rank=(n+future[0],points+future[1],-(spent+cross-future[2]))
                    best=max(best,rank)
            return best
        expected=oracle(0,.1,budget);actual=UpwardCoveragePlanner(fraction,top_route='direct').plan(groups,0,.1,speed,climb,budget)
        self.assertEqual((actual.total_count,actual.points),expected[:2]);self.assertAlmostEqual(actual.estimated_seconds,-expected[2])

    def test_top_tier_uses_direct_segment_without_inner_step(self):
        a=AdaptiveTierAgent(top_route='direct');o=self.obs();a.act(o,.01);a.stage=2;a.phase='coins';a.needs_plan=True
        o=replace(o,pose=Pose2D(0,-.95,pi/2),telemetry=dict(speed=0,ground_z=.6,steering=[0]*6))
        a.act(o,.01);self.assertEqual(a.follower.points,((.95,0.),))
