"""Real-physics recovery check with an explicitly immobilized inverted fixture.

The inverted body's state is held by the fixture until recovery, to model a
robot that cannot self-right. This is not a natural fall-success-rate test.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime
from xrobocon.navigation import VelocityCommand
class WaitAgent:
    def reset(self):self.resets=getattr(self,'resets',0)+1
    def act(self,obs,dt):return VelocityCommand(0,0)
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);args=p.parse_args();out=Path(args.out)
    source=Path('configs/competition_lifecycle_check.json').resolve();c=json.loads(source.read_text())
    c['simulation']['scenario']=str((source.parent/c['simulation']['scenario']).resolve());c['competition']['time_limit']=30
    c['competition']['recovery']=dict(enabled=True,timeout=20,min_distance=.05,min_rotation=.15)
    for coin in c['competition']['coins']:coin['respawn_delay']=None
    config_path=out.with_suffix('.config.json');config_path.parent.mkdir(parents=True,exist_ok=True);config_path.write_text(json.dumps(c))
    config,sim,manager,agents,runner=build_runtime(config_path,out,False);runner.agents={name:WaitAgent() for name in agents}
    original=sim.respawn;isolated=[]
    def respawn(participant):
        red=sim.robots['opponent'].entity;before=red.get_qpos().cpu().numpy().copy();velocity=red.get_dofs_velocity().cpu().numpy().copy()
        original(participant)
        np.testing.assert_array_equal(red.get_qpos().cpu().numpy(),before);np.testing.assert_array_equal(red.get_dofs_velocity().cpu().numpy(),velocity);isolated.append(participant)
    sim.respawn=respawn
    try:
        runner.start()
        for _ in range(20):runner.tick()
        scores=dict(manager.scores);assert scores=={'self':1,'opponent':2},scores
        robot=sim.robots['self'];robot.set_pose([-1.2,-.6,.4],[180,0,0]);frozen=robot.entity.get_qpos().clone();started=manager.elapsed;events=[]
        with (out/'trace.jsonl').open('w') as trace:
            for _ in range(2200):
                robot.entity.set_qpos(frozen,zero_velocity=True)
                row=runner.tick()
                if row['recovery_events']:
                    events=row['recovery_events'];trace.write(json.dumps(row)+'\n');break
            else:raise AssertionError('inverted immobilized robot did not recover')
        assert manager.status=='running' and manager.scores==scores
        assert abs(manager.elapsed-started-20)<.011,(manager.elapsed,started)
        assert isolated==['self'] and manager.recovery_counts=={'self':1,'opponent':0}
        assert runner.agents['self'].resets==2 and runner.agents['opponent'].resets==1
        assert max(abs(float(v)) for v in robot.get_euler()[:2])<1
        np.testing.assert_allclose(robot.get_pos().cpu().numpy(),sim.spawns['self']['pos'],atol=1e-5)
        assert all(coin['status']=='collected' for coin in manager.coins)
        for _ in range(30):runner.tick()
        assert manager.recovery_counts['self']==1 and manager.status=='running'
        report=dict(passed=True,checks=['inverted pose does not abort match','recovery after 20 simulation seconds','only blue body and velocity reset','scores and collected coins retained','clock continues','upright start pose restored','agent control state reset','match continues after recovery'],fixture='blue deliberately held inverted and immobile; red intentionally waiting',state=manager.snapshot(),recovery_events=events)
        (out/'verification.json').write_text(json.dumps(report,indent=2));print('Recovery runtime: 8 checks passed',flush=True)
    finally:sim.stop()
if __name__=='__main__':main()
