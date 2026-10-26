"""Display one static competition MJCF; no ROS2 or Genesis world initialization."""
import argparse,sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.robot_preview import show_preview

def main():
    p=argparse.ArgumentParser();p.add_argument('--asset',required=True);p.add_argument('--label',default='Robot');p.add_argument('--screenshot');p.add_argument('--smoke',action='store_true');p.add_argument('--watch',action='store_true');args=p.parse_args()
    print(json.dumps(show_preview(args.asset,args.label,args.screenshot,args.smoke,watch=args.watch)),flush=True)
if __name__=='__main__':main()
