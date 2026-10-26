"""Record matched flat-ground PPO trials without changing the training environment.

Pairs original PPO with zero frame torque (not a mechanical frame lock).
CSV records pre-action observations and post-action outcomes separately.
"""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys

os.environ.setdefault('PYTORCH_ENABLE_MPS_FALLBACK', '1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import genesis as gs
from stable_baselines3 import PPO
from xrobocon.env import XRoboconEnv


def heading_error(obs):
    bearing = np.degrees(np.arctan2(obs[13], obs[12]))
    return float((bearing - obs[5] + 180) % 360 - 180)


def run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    from xrobocon.common import setup_genesis
    setup_genesis(backend=args.backend, seed=args.seed, logging_level='warning')
    env = XRoboconEnv(render_mode=None, robot_type='tristar')
    model = PPO.load(args.model, env=env, device='cpu')
    cases = [
        ('short', (8., 0., .08), 0., (9., 0., .08)),
        ('medium', (8., 0., .08), 0., (10., 1., .08)),
        ('long', (8., 0., .08), 90., (8., 3., .08)),
    ]
    summaries = []
    for case, start, yaw, target in cases:
        for variant in ('ppo', 'ppo_zero_frame'):
            env.reset(seed=args.seed)
            env.robot.set_pose(start, (0., 0., yaw))
            env.set_target(target)
            obs = env._get_obs()
            rows = []
            initial_dist = float(np.linalg.norm(obs[12:14]))
            reason = 'step_limit'
            for step in range(args.steps):
                original, _ = model.predict(obs, deterministic=True)
                action = original.copy()
                if variant == 'ppo_zero_frame':
                    action[:2] = 0.
                pre = obs.copy()
                dof_pos = env.robot.entity.get_dofs_position().cpu().numpy()
                obs, reward, terminated, truncated, _ = env.step(action)
                finite = bool(np.isfinite(obs).all() and np.isfinite(reward))
                distance = float(np.linalg.norm(obs[12:14]))
                reached = distance < .5 and abs(float(obs[14])) < .2
                tipped = max(abs(float(obs[3])), abs(float(obs[4]))) > 60
                row = dict(step=step, sim_time=(step+1)*.01,
                           distance_before=float(np.linalg.norm(pre[12:14])),
                           distance_after=distance, heading_before=heading_error(pre),
                           heading_after=heading_error(obs), reward=float(reward),
                           terminated=bool(terminated), truncated=bool(truncated),
                           reached=reached, tipped=tipped, finite=finite,
                           frame_angle_l=float(dof_pos[6]), frame_angle_r=float(dof_pos[10]))
                for prefix, values in [('obs_before',pre),('obs_after',obs),
                                       ('model_action',original),('applied_action',action)]:
                    row.update({f'{prefix}_{i}':float(v) for i,v in enumerate(values)})
                rows.append(row)
                if not finite or terminated or truncated:
                    reason = ('nonfinite' if not finite else 'tipped' if tipped else
                              'fell' if obs[2] < 0 else 'reached' if reached else
                              'timeout' if truncated else 'terminated')
                    break
            path = out / f'{case}-{variant}.csv'
            with path.open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            actions = np.array([[r[f'model_action_{i}'] for i in range(4)] for r in rows])
            distances = np.array([r['distance_after'] for r in rows])
            summary = dict(case=case, variant=variant, seed=args.seed, start=start,
                           target=target, yaw=yaw, steps=len(rows), reason=reason,
                           initial_distance=initial_dist, min_distance=float(distances.min()),
                           final_distance=float(distances[-1]),
                           mean_abs_action=np.abs(actions).mean(axis=0).tolist(),
                           saturated_fraction=(np.abs(actions)>=.99).mean(axis=0).tolist(),
                           receding_fraction=float(np.mean([r['distance_after']>r['distance_before'] for r in rows])),
                           max_abs_roll_pitch=float(max(max(abs(r['obs_after_3']),abs(r['obs_after_4'])) for r in rows)),
                           csv=str(path))
            summaries.append(summary)
            print(json.dumps(summary), flush=True)
            manifest = dict(model=str(Path(args.model).resolve()),
                            model_sha256=hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
                            versions={p:importlib.metadata.version(p) for p in
                                      ['genesis-world','torch','stable-baselines3','numpy']},
                            dt=.01, max_steps=args.steps, trials=summaries,
                            backend=str(gs.backend), device=str(gs.device),
                            note='500 steps is 5 simulated seconds; zero torque is not a frame lock.')
            (out/'summary.json').write_text(json.dumps(manifest, indent=2))
    env.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['cpu', 'gpu'], default=None)
    parser.add_argument('--model', default='xrobocon_ppo_tristar_flat.zip')
    parser.add_argument('--out', required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--steps', type=int, default=500)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error('--steps must be positive')
    run(args)
