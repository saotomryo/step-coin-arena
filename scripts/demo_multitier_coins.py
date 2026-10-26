"""Play clockwise and counterclockwise six-coin demonstrations."""
import argparse
from datetime import datetime
from pathlib import Path
import subprocess
import sys

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--route',choices=['both','clockwise','counterclockwise'],default='both')
    parser.add_argument('--headless',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    folder=root/'evaluation_results/current'/('multitier-demo-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
    routes=['counterclockwise','clockwise'] if args.route=='both' else [args.route]
    for route in routes:
        print(f'DEMO {route}: two coins on each tier; 360s demonstration limit',flush=True)
        command=[sys.executable,str(root/'scripts/demo_field_first_step.py'),
            '--scenario','configs/scenarios/passive_front_bogie_r18_steering_flat.json',
            '--all-tiers','--tier-route',route,'--mission-time','360','--field-mesh',
            '--collision-method','gjk','--quasistatic-feedforward','--wheel-torque','6',
            '--seconds','360','--out',str(folder/route)]
        if not args.headless:command+=['--viewer','--realtime']
        subprocess.run(command,cwd=root,check=True)
    print(f'Results: {folder}',flush=True)

if __name__=='__main__':main()
