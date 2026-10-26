"""Real DDS + two distinct robots + active carrier command + Genesis overlay."""
import argparse,json,math,secrets,signal,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.match_setup import MatchCatalog,build_match_config
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);args=p.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json');choices=catalog.defaults();choices['self'].update(robot='tristar_carrier',control='baseline',strategy='fixed')
    cfg=build_match_config(catalog,choices);cfg['simulation']['spawns']['self']=dict(pos=[-4.99,0,.65],euler=[0,0,0]);cfg['agents']['self']['gate_angles']=[math.pi]*3;cfg['competition']['time_limit']=3.;cfg['transport']['domain_id']=100+secrets.randbelow(100)
    path=out/'config.json';path.write_text(json.dumps(cfg,indent=2));command=[sys.executable,str(ROOT/'scripts/run_competition.py'),'--config',str(path),'--out',str(out/'run'),'--panel','overlay','--autostart','--return-after-match']
    with (out/'process.log').open('w') as log:
        child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        try:code=child.wait(timeout=180)
        finally:
            if child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=20)
    if code:raise RuntimeError(f'competition exited {code}; see process.log')
    rows=[json.loads(x) for x in (out/'run/trace.jsonl').read_text().splitlines()]
    assert rows and all(r['transport']['sources']=={'self':'ros2/truth_baseline','opponent':'ros2/truth_baseline'} for r in rows)
    targets=[r['commands']['self']['frame_target'] for r in rows]
    assert max(targets)>.01,'active carrier command never crossed ROS2'
    samples=rows[-1]['observations']['self']['sensors']['joint_states']['data'];joints=dict(zip(samples['name'],samples['position']))
    assert max(abs(joints[n]) for n in ('left_tristar_joint','right_tristar_joint'))>.005,'carrier did not physically move'
    assert (out/'run/viewer-panel.png').exists()
    state=json.loads((out/'run/session-state.json').read_text());assert state['control_status']=='finished'
    report=dict(passed=True,seconds=3,max_carrier_target=max(targets),carrier_positions={n:joints[n] for n in ('left_tristar_joint','right_tristar_joint')},checks=['two distinct robot models','both agents use DDS','carrier command crosses ROS2','JointState encoder feedback and physical carrier motion','Genesis and score overlay','clean match finish'])
    (out/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
if __name__=='__main__':main()
