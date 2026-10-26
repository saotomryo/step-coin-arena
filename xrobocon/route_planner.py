"""Budgeted coin coverage on one ring; no physics or ownership assignments."""
from dataclasses import dataclass
from typing import Protocol
from math import atan2,sin,cos,isfinite
@dataclass(frozen=True)
class RoutePlan:
    coin_ids:tuple
    distance:float
    estimated_seconds:float

def angular_distance(a,b):return abs(atan2(sin(a-b),cos(a-b)))
class RingCoveragePlanner:
    """Exact subset DP for small rings: count, points, then minimum modeled time.

    Time is a heuristic travel/dwell model, not a guarantee for real motion.
    """
    def __init__(self,speed_fraction=.75):
        if not isfinite(speed_fraction) or not 0<speed_fraction<=1:raise ValueError('speed fraction must be in (0,1]')
        self.speed_fraction=speed_fraction
    def options(self,coins,start_angle,radius,speed,budget,chord=False):
        if not all(isfinite(x) for x in (start_angle,radius,speed,budget)) or radius<=0 or speed<=0:raise ValueError('finite planner inputs and positive geometry/speed required')
        candidates=sorted([c for c in coins if c['status']=='available'],key=lambda c:str(c['id']))
        n=len(candidates)
        if n>12:raise ValueError('ring DP supports up to 12 available coins')
        if not n or budget<=0:return [RoutePlan((),0.,0.)]
        angles=[atan2(c['pos'][1],c['pos'][0]) for c in candidates]
        velocity=speed*self.speed_fraction
        def travel(a,b):
            angle=angular_distance(a,b)
            return 2*radius*sin(angle/2) if chord else radius*angle
        states={}
        for j,c in enumerate(candidates):
            distance=travel(start_angle,angles[j]);seconds=distance/velocity+c.get('dwell',0)+.25
            if seconds<=budget:states[(1<<j,j)]=(seconds,distance,(j,))
        choices=[RoutePlan((),0.,0.)]
        for mask in range(1,1<<n):
            count=mask.bit_count();points=sum(c.get('points',1) for i,c in enumerate(candidates) if mask>>i&1)
            for last in range(n):
                value=states.get((mask,last))
                if value is None:continue
                seconds,distance,route=value
                choices.append(RoutePlan(tuple(candidates[i]['id'] for i in route),distance,seconds))
                for j,c in enumerate(candidates):
                    if mask>>j&1:continue
                    leg=travel(angles[last],angles[j]);cost=seconds+leg/velocity+c.get('dwell',0)+.25
                    if cost>budget:continue
                    key=(mask|1<<j,j);old=states.get(key)
                    if old is None or cost<old[0]:states[key]=(cost,distance+leg,route+(j,))
        # For a fixed cardinality and endpoint, retain time/points Pareto options.
        points_by_id={c['id']:c.get('points',1) for c in candidates};buckets={}
        for option in choices:
            key=(len(option.coin_ids),option.coin_ids[-1] if option.coin_ids else None)
            score=sum(points_by_id[i] for i in option.coin_ids);bucket=buckets.setdefault(key,[])
            if any(old.estimated_seconds<=option.estimated_seconds and old_score>=score for old,old_score in bucket):continue
            bucket[:]=[(old,old_score) for old,old_score in bucket if not (option.estimated_seconds<=old.estimated_seconds and score>=old_score)]
            bucket.append((option,score))
        return [option for bucket in buckets.values() for option,score in bucket]
    def plan(self,coins,start_angle,radius,speed,budget):
        points={c['id']:c.get('points',1) for c in coins}
        return max(self.options(coins,start_angle,radius,speed,budget),key=lambda p:(len(p.coin_ids),sum(points[i] for i in p.coin_ids),-p.estimated_seconds))

@dataclass(frozen=True)
class UpwardPlan:
    current:RoutePlan
    total_count:int
    points:float
    estimated_seconds:float
    tier_routes:tuple

class RoutePlanner(Protocol):
    def plan(self,groups,stage,start_angle,speed,climb_speed,budget) -> UpwardPlan: ...

class UpwardCoveragePlanner:
    """Compare collecting here vs climbing now; upward-only time-budget search."""
    def __init__(self,speed_fraction=.75,ascent_overhead=10,top_route='arc'):
        if isinstance(ascent_overhead,bool) or not isfinite(ascent_overhead) or ascent_overhead<0:raise ValueError('nonnegative finite ascent overhead required')
        if top_route not in ('arc','direct'):raise ValueError('unknown top-tier route')
        self.top_route=top_route
        self.ring=RingCoveragePlanner(speed_fraction);self.ascent_overhead=ascent_overhead
    def plan(self,groups,stage,start_angle,speed,climb_speed,budget):
        by_id={c['id']:c for group in groups for c in group}
        radii=[(sum(c['pos'][0]**2+c['pos'][1]**2 for c in g)/len(g))**.5 for g in groups]
        options_cache={}
        def options(level,angle):
            key=(level,angle)
            if key not in options_cache:options_cache[key]=self.ring.options(groups[level],angle,radii[level],speed,max(0,budget),chord=level==2 and self.top_route=='direct')
            return options_cache[key]
        def search(level,angle,remaining):
            best=(0,0,0.);result=((),0.,0,0.)
            for route in options(level,angle):
                if route.estimated_seconds>remaining:continue
                score=sum(by_id[i].get('points',1) for i in route.coin_ids);count=len(route.coin_ids);seconds=route.estimated_seconds;routes=(route,)
                end=atan2(by_id[route.coin_ids[-1]]['pos'][1],by_id[route.coin_ids[-1]]['pos'][0]) if route.coin_ids else angle
                if level<2:
                    ascent=abs(radii[level]-radii[level+1])/climb_speed+self.ascent_overhead
                    if remaining-seconds>=ascent:
                        following,spent,n,points=search(level+1,end,remaining-seconds-ascent)
                        if n:routes+=following;seconds+=ascent+spent;count+=n;score+=points
                rank=(count,score,-seconds)
                if rank>best:best=rank;result=(routes,seconds,count,score)
            return result
        routes,seconds,count,score=search(stage,start_angle,max(0,budget))
        return UpwardPlan(routes[0] if routes else RoutePlan((),0.,0.),count,score,seconds,routes)
