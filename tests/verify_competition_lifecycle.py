"""Real-physics regression of two rounds, respawn and display projection.

Fixture coins intentionally sit at spawn positions to isolate game/environment
behavior from route planning. No official competition rules are implied.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime
from xrobocon.session import CompetitionSession

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--viewer',action='store_true');args=p.parse_args()
    out=Path(args.out);config,sim,manager,agents,runner=build_runtime('configs/competition_lifecycle_check.json',out,args.viewer)
    saved=[]
    session=CompetitionSession(runner,lambda rid,state,reason:saved.append(dict(round_id=rid,reason=reason,state=state)))
    transitions=[]
    def verify_projection():
        for coin in manager.coins:
            key=coin['id'];available=coin['status']=='available'
            assert sim.coin_display.visible[key]==available
            pos=sim.coin_entities[key].get_pos().cpu().tolist()
            assert abs(pos[2]-(coin['pos'][2] if available else -20))<1e-4,(key,coin['status'],pos)
    try:
        sim.reset();runner._sync_display();verify_projection()
        for round_number in (1,2):
            session.command('start');assert session.round_id==round_number
            assert not any(manager.scores.values());verify_projection()
            hidden_seen=set();restored_seen=set()
            with (out/f'round-{round_number}-trace.jsonl').open('w') as trace:
                while session.status=='running':
                    row=session.tick();verify_projection()
                    trace.write(json.dumps(row)+'\n')
                    for change in row['display_changes']:
                        key=change['coin_id'];transitions.append(dict(round_id=round_number,time=manager.elapsed,**change))
                        if not change['visible']:hidden_seen.add(key)
                        elif key in hidden_seen:restored_seen.add(key)
            assert hidden_seen=={0,1} and restored_seen=={0,1}
            assert all(v>=2 for v in manager.collection_counts.values()),manager.collection_counts
            assert saved[-1]['reason']=='time_limit' and manager.elapsed==manager.time_limit
            assert all(a.done for a in agents.values()),'fixture should finish its placeholder routes'
            session.command('reset');verify_projection()
            assert manager.elapsed==0 and not any(manager.scores.values()) and session.status=='ready'
        result=dict(passed=True,fixture='coins at robot starts; provisional respawn delay',rounds=saved,display_transitions=transitions,checks=['two real-physics robots','collection increments scores','collected entities hidden at Z=-20','respawn restores entity positions','done agents do not finish competition','deadline finishes each round','reset clears scores and coin generations','second round repeats successfully'])
        (out/'verification.json').write_text(json.dumps(result,indent=2));print('Competition lifecycle: 8 checks passed; 2 rounds',flush=True)
    finally:sim.stop()

if __name__=='__main__':main()
