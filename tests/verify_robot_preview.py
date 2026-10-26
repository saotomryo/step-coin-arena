"""Real native preview processes: replace, close, and never integrate physics."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.match_setup import MatchCatalog
from xrobocon.match_launcher import MatchLauncher

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);args=p.parse_args()
    root=Path(args.out);launcher=MatchLauncher(MatchCatalog.load('configs/match_catalog.json'),root)
    try:
        first=launcher.preview('front_bogie_r18');time.sleep(1)
        assert first.poll() is None
        second=launcher.preview('front_bogie_r18');assert first.poll()==0
        time.sleep(1);assert second.poll() is None
        launcher.stop_preview();assert second.poll()==0
        reports=[json.loads(line) for line in (root/'preview.log').read_text().splitlines() if line.startswith('{')]
        assert len(reports)==2 and all(r['physics_time']==0 and r['wheel_count']==6 for r in reports)
        result=dict(passed=True,checks=['native preview stays open','replacement closes previous preview','closing releases native renderer','both previews preserve simulation time zero'],models=reports)
        (root/'verification.json').write_text(json.dumps(result,indent=2));print('Robot preview: 4 runtime checks passed')
    finally:launcher.stop_preview()
if __name__=='__main__':main()
