"""Selected mixed strategies -> real ROS2/Genesis overlay -> results -> ready to replay."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.match_setup import MatchCatalog
from xrobocon.match_launcher import MatchLauncher

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);parser.add_argument('--robot',default='front_bogie_r18');parser.add_argument('--control',default='safe');args=parser.parse_args()
    root=Path(args.out);catalog=MatchCatalog.load('configs/match_catalog.json');launcher=MatchLauncher(catalog,root)
    choices=catalog.defaults();choices['self'].update(robot=args.robot,control=args.control);choices['opponent']['strategy']='fixed'
    trial=launcher.launch(choices,seconds=.3);deadline=time.monotonic()+150
    try:
        while launcher.running:
            if time.monotonic()>deadline:raise TimeoutError('match launch did not finish')
            time.sleep(.1)
        result=launcher.poll();assert not result['error'],result
        assert result['status']=='finished',result
        config=json.loads((trial/'run/config.json').read_text())
        assert config['agents']['self']['control_profile']==args.control and config['agents']['opponent']['kind']=='tier'
        assert config['match_selection']==choices
        assets=json.loads((trial/'run/robot-assets.json').read_text());assert set(assets)=={'self','opponent'}
        rows=list(map(json.loads,(trial/'run/trace.jsonl').read_text().splitlines()))
        assert rows and all(r['transport']['sources']=={'self':'ros2/truth_baseline','opponent':'ros2/truth_baseline'} for r in rows)
        assert (trial/'run/python-panel.png').exists()
        report=dict(passed=True,checks=['mixed color-specific choices saved','selected robot assets exported per color','ROS2 observations received by both selected agents','Qt panel and simulator initialized','match automatically started and finished','results returned to launcher'],trial=str(trial),result=result)
        (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print('Match launcher: 6 runtime checks passed',flush=True)
    finally:
        if launcher.running:launcher.stop()
if __name__=='__main__':main()
