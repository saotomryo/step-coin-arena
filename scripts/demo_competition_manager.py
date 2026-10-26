"""Scripted authority demo; positions are fixtures, not simulated robot motion."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.competition import CompetitionManager

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',default='configs/competition_manager_demo.json')
    p.add_argument('--out',required=True)
    args=p.parse_args();config=json.loads(Path(args.config).read_text())
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    manager=CompetitionManager(config['coins'],config['participants'],config['time_limit']);manager.start()
    while manager.status=='running':
        poses={}
        for sample in config['position_samples']:
            if sample['from']<=manager.elapsed+1e-9:poses=sample['positions']
        manager.advance(.1,poses)
    snapshot=manager.snapshot()
    (out/'state.json').write_text(json.dumps(snapshot,indent=2))
    (out/'config.json').write_text(json.dumps(config,indent=2))
    print(json.dumps(dict(status=snapshot['status'],scores=snapshot['scores'],events=len(snapshot['events']),output=str(out)),ensure_ascii=False))

if __name__=='__main__':main()
