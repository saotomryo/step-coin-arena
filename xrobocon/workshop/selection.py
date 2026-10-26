"""Conservative same-fixture regression checks, without extra comparison runs."""
from math import isfinite

def _trials(report):
    if not isinstance(report,dict):return {}
    return {(row.get('candidate'),t.get('case')):t for row in report.get('results',[]) for t in row.get('trials',[])}

def _number(value):return isinstance(value,(float,int)) and not isinstance(value,bool) and isfinite(value)

def regression(previous,candidate,suite):
    """Protect passed tasks; otherwise require unopposed measured deterioration.

    Mixed improvements and missing/incomparable evidence do not trigger rollback.
    Error distance has a 2cm deadband to avoid reacting to tiny endpoint changes.
    Joint smoke is deliberately not interpreted as locomotion quality.
    """
    if suite=='joints':return False,'joint smoke has no task-performance ranking'
    before=_trials(previous);after=_trials(candidate)
    if not before:return False,'no comparable baseline trials'
    if isinstance(candidate,dict) and (candidate.get('error') or any(r.get('error') or r.get('complete') is False for r in candidate.get('results',[]))):
        return True,'candidate diagnostic failed or incomplete'
    if set(before)!=set(after):return False,'trial cases differ or are incomplete'
    lost=[case for key,t in before.items() if t.get('passed') is True and after[key].get('passed') is not True for case in [key[1]]]
    if lost:return True,'previously passed tasks failed: '+', '.join(lost)
    better=False;worse=False
    for key,a in before.items():
        b=after[key]
        if b.get('passed') is True and a.get('passed') is not True:better=True
        if b.get('status')=='tipped' and a.get('status')!='tipped':worse=True
        if a.get('status')=='tipped' and b.get('status')!='tipped':better=True
        for metric,direction,deadband in [('final_error',-1,.02),('coins',1,0)]:
            x=a.get(metric);y=b.get(metric)
            if _number(x) and _number(y):
                delta=(y-x)*direction
                better|=delta>deadband;worse|=delta < -deadband
        if a.get('passed') is True and b.get('passed') is True:
            x=a.get('simulation_seconds');y=b.get('simulation_seconds')
            if _number(x) and _number(y):
                deadband=max(.1,.05*x);better|=x-y>deadband;worse|=y-x>deadband
    if worse and not better:return True,'measured task results deteriorated without another task improving'
    return False,'mixed tradeoff' if worse and better else 'not worse or insufficient evidence'
