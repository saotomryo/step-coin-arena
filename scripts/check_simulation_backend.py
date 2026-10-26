"""Headless physical-scene smoke check; run each backend in a new process."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['cpu', 'gpu'], default='cpu')
    parser.add_argument('--steps', type=int, default=100)
    parser.add_argument('--scenario', default=str(Path(__file__).resolve().parents[1]/'configs/scenarios/flat_baseline.json'))
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error('--steps must be positive')
    import genesis as gs
    import numpy as np
    from xrobocon.base_env import XRoboconBaseEnv
    started = time.perf_counter()
    env = XRoboconBaseEnv(scenario_path=args.scenario, backend=args.backend)
    build_seconds = time.perf_counter()-started
    env.robot.set_pose((0, 0, 0.3), (0, 0, 0))
    started = time.perf_counter()
    for _ in range(args.steps):
        env.scene.step()
        if not np.isfinite(env.robot.get_pos().cpu().numpy()).all():
            raise RuntimeError('nonfinite robot position')
    print(json.dumps({'requested_backend': args.backend, 'actual_backend': str(gs.backend),
                      'device': str(gs.device), 'steps': args.steps,
                      'build_seconds': build_seconds, 'step_seconds': time.perf_counter()-started,
                      'final_position': env.robot.get_pos().cpu().tolist(), 'status': 'passed'}))

if __name__ == '__main__':
    main()
