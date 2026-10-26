"""Deterministic competition authority; no Genesis, robot control or wall clock.

Positions are ground-reference XYZ samples for the supplied interval. Simultaneous
eligible participants contest a coin: no award and dwell counters reset.
"""
from copy import deepcopy
from math import isfinite,hypot

def number(value,name,minimum=0,strict=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not isfinite(value) or (value<=minimum if strict else value<minimum):
        raise ValueError(f'invalid {name}')
    return value

class CompetitionManager:
    def __init__(self,coins,participants=('self',),time_limit=120):
        self.time_limit=number(time_limit,'time limit',strict=True)
        self.participants=tuple(participants)
        if not self.participants or len(set(self.participants))!=len(self.participants) or any(not isinstance(p,str) or not p for p in self.participants):
            raise ValueError('unique nonempty participant names required')
        self.definitions=[];ids=set()
        for original in coins:
            c=deepcopy(original);cid=c['id']
            if isinstance(cid,bool) or not isinstance(cid,(int,str)) or cid in ids:raise ValueError('unique coin ids required')
            ids.add(cid)
            if len(c['pos'])!=3:raise ValueError('XYZ coin position required')
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) for v in c['pos']):raise ValueError('finite coin position required')
            tier=c['tier']
            if isinstance(tier,bool) or not isinstance(tier,int) or tier<1:raise ValueError('positive tier required')
            defaults=dict(points={1:5,2:2}.get(tier,1),dwell=2 if tier==1 else 0,
                          spawn_at=0,radius=.2,height_tolerance=.05,respawn_delay=None)
            for key,default in defaults.items():c.setdefault(key,default)
            for key in ('points','dwell','spawn_at','radius','height_tolerance'):
                number(c[key],key,strict=key in ('radius','height_tolerance'))
            if c['respawn_delay'] is not None:number(c['respawn_delay'],'respawn delay',strict=True)
            self.definitions.append(c)
        self.reset()

    def reset(self):
        self.elapsed=0.;self.status='ready';self.scores={p:0 for p in self.participants};self.collection_counts={p:0 for p in self.participants};self.events=[]
        self.recovery_counts={p:0 for p in self.participants};self.recovery_blocked_seconds={p:0. for p in self.participants}
        self.coins=[dict(c,status='pending',collected=False,owner=None,spawn_count=0,
                         next_spawn=c['spawn_at'],contested=False,stay_timer=0.,dwell_by={p:0. for p in self.participants}) for c in deepcopy(self.definitions)]

    def _event(self,kind,at=None,**payload):
        self.events.append(dict(sequence=len(self.events),time=self.elapsed if at is None else at,type=kind,**payload))

    def _spawn(self,c,at):
        c.update(status='available',collected=False,owner=None,contested=False,stay_timer=0.,next_spawn=None)
        c['dwell_by']={p:0. for p in self.participants};c['spawn_count']+=1
        self._event('coin_spawned',at,coin_id=c['id'],generation=c['spawn_count'])

    def start(self):
        self.reset();self.status='running';self._event('competition_started')
        for c in self.coins:
            if c['spawn_at']==0:self._spawn(c,0)

    def finish(self,reason='operator_stop'):
        if self.status=='running':
            self.status='finished';self._event('competition_finished',reason=reason)

    def advance(self,dt,positions):
        number(dt,'dt',strict=True)
        for participant,pos in positions.items():
            if participant not in self.scores:raise ValueError('unknown participant')
            if len(pos)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) for v in pos):raise ValueError('finite XYZ position required')
        if self.status!='running':return
        old=self.elapsed;self.elapsed=min(self.time_limit,old+dt)
        if self.elapsed>=self.time_limit:
            self.finish('time_limit');return
        for c in self.coins:
            active_from=old
            if c['next_spawn'] is not None and c['next_spawn']<=self.elapsed:
                active_from=max(old,c['next_spawn']);self._spawn(c,c['next_spawn'])
            if c['status']!='available':continue
            eligible=[p for p,pos in positions.items() if hypot(pos[0]-c['pos'][0],pos[1]-c['pos'][1])<c['radius'] and abs(pos[2]-c['pos'][2])<c['height_tolerance']]
            contested=len(eligible)>1
            if contested and not c['contested']:self._event('coin_contested',coin_id=c['id'],participants=sorted(eligible))
            c['contested']=contested
            for p in self.participants:
                c['dwell_by'][p]=c['dwell_by'][p]+self.elapsed-active_from if len(eligible)==1 and p==eligible[0] else 0.
            c['stay_timer']=max(c['dwell_by'].values())
            if len(eligible)==1 and self.elapsed>active_from and c['stay_timer']+1e-9>=c['dwell']:
                owner=eligible[0];self.scores[owner]+=c['points']
                self.collection_counts[owner]+=1
                c.update(status='collected',collected=True,owner=owner,contested=False)
                c['next_spawn']=self.elapsed+c['respawn_delay'] if c['respawn_delay'] is not None else None
                self._event('coin_collected',coin_id=c['id'],participant=owner,points=c['points'],score=self.scores[owner],generation=c['spawn_count'],position=list(positions[owner]))

    def record_recovery(self,participant,blocked_seconds,from_position,spawn_position):
        if self.status!='running' or participant not in self.scores:raise ValueError('recovery requires a running participant')
        number(blocked_seconds,'blocked seconds',strict=True)
        self.recovery_counts[participant]+=1;self.recovery_blocked_seconds[participant]+=blocked_seconds
        self._event('robot_recovered',participant=participant,reason='stuck',blocked_seconds=blocked_seconds,from_position=list(from_position),spawn_position=list(spawn_position),count=self.recovery_counts[participant])

    def snapshot(self):
        return deepcopy(dict(status=self.status,elapsed=self.elapsed,remaining=max(0,self.time_limit-self.elapsed),scores=self.scores,collection_counts=self.collection_counts,coins=self.coins,events=self.events,recovery_counts=self.recovery_counts,recovery_blocked_seconds=self.recovery_blocked_seconds))
