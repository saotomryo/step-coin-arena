"""Reusable diagnostic fixtures. They do not change production game rules."""
import json,time,hashlib
from pathlib import Path
from math import hypot
from dataclasses import asdict
from xrobocon.navigation import WaypointFollower,VelocityCommand
from xrobocon.motion_profiles import resolve_profile
ROOT=Path(__file__).resolve().parents[1]

def control_trace(drive):
    """Motor targets/feedback and actual per-joint limits, including generated drives."""
    positions=drive.robot.entity.get_dofs_position().cpu().numpy()
    velocities=drive.robot.entity.get_dofs_velocity().cpu().numpy()
    specs=getattr(drive,'specs',[dict(name=f'dof_{i}',mode='velocity') for i in drive.indices])
    actual=[float(positions[i] if spec['mode']=='position' else velocities[i]) for i,spec in zip(drive.indices,specs)]
    errors=[float(target)-value for target,value in zip(drive.targets,actual)]
    return dict(joint_names=[s['name'] for s in specs],modes=[s['mode'] for s in specs],
        position=[float(positions[i]) for i in drive.indices],velocity=[float(velocities[i]) for i in drive.indices],
        targets=list(drive.targets),tracking_errors=errors,tracking_error_max=max(map(abs,errors),default=0.),
        torque_limits=[c.limit for c in drive.controllers],saturation=[abs(t)>=c.limit*.99 for t,c in zip(drive.torques,drive.controllers)])

def run_flat(config,out,profile,seconds=25,viewer=False):
    from xrobocon.genesis_simulator import GenesisSimulator
    model=config['simulation']['robot_models']['self'];agent=config['agents']['self']
    motion=resolve_profile(profile,.12,.3,agent.get('motion_limits'))
    sim=GenesisSimulator(str(ROOT/'configs/scenarios/passive_front_bogie_r18_steering_flat.json'),viewer=viewer,spawns={'self':dict(pos=[0,0,.65],euler=[0,0,0])},torque_limit=config['simulation']['torque_limit'],robot_models={'self':model})
    results=[]
    try:
        for case,points in [('straight',[(1.,0.)]),('turn',[(0.,1.)]),('route',[(.6,0),(.6,.6),(0,.6)])]:
            sim.reset();follower=WaypointFollower(points,max_speed=motion.max_speed,max_turn=motion.max_turn,acceleration=motion.acceleration,turn_acceleration=motion.turn_acceleration)
            stable=0.;max_tilt=0.;status='timeout';saturated=0;start=time.perf_counter()
            with (out/(case+'.jsonl')).open('w') as trace:
                for step in range(round(seconds/.01)):
                    sim.step({'self':follower.update(sim.read_pose('self'),.01)},.01)
                    pose=sim.read_pose('self');speed=sim.speeds()['self'];angle=sim.robot.get_euler()
                    max_tilt=max(max_tilt,abs(float(angle[0])),abs(float(angle[1])))
                    error=hypot(pose.x-points[-1][0],pose.y-points[-1][1]);drive=sim.drives['self']
                    saturated+=int(any(abs(t)>=c.limit*.99 for t,c in zip(drive.torques,drive.controllers)))
                    stable=stable+.01 if follower.done and error<=.15 and speed<.05 else 0
                    if step%10==0:
                        trace.write(json.dumps(dict(time=(step+1)*.01,pose=asdict(pose),speed=speed,tilt=max_tilt,error=error,torques=drive.torques,**control_trace(drive)),allow_nan=False)+'\n');trace.flush()
                    if stable>=1.:status='reached_and_stopped';break
                    if max_tilt>90:status='tipped';break
            results.append(dict(case=case,passed=status=='reached_and_stopped',status=status,simulation_seconds=(step+1)*.01,wall_seconds=time.perf_counter()-start,final_error=error,max_tilt_degrees=max_tilt,torque_saturated_fraction=saturated/(step+1)))
    finally:
        sim.stop()
        if viewer:sim.scene.viewer.stop()
    return results

def diagnostic_step_config(config):
    """Three isolated perpendicular climbs; fixed targets, no opponent/planner."""
    from copy import deepcopy
    value=deepcopy(config);value.pop('transport',None);value['sensors']={}
    value['simulation']['robot_models']={'self':value['simulation']['robot_models']['self']}
    value['simulation']['spawns']={'self':dict(pos=[-6.1,0,.65],euler=[0,0,0])}
    value['simulation']['show_coins']=False
    value['competition']['participants']=['self'];value['competition']['time_limit']=120
    value['competition']['coins']=[dict(id=i,pos=[-radius,0,height+.005],tier=3-i,points=i+1,dwell=0,spawn_at=0) for i,(radius,height) in enumerate([(4.,.1),(2.7,.35),(1.4,.6)])]
    agent=value['agents']['self'];value['agents']={'self':dict(kind='tier',tier_coin_ids=[[0],[1],[2]],gate_angle=3.141592653589793,control_profile=agent['control_profile'],motion_limits=deepcopy(agent.get('motion_limits',{})))}
    if agent.get('locomotion'):value['agents']['self']['locomotion']=deepcopy(agent['locomotion'])
    return value

def run_steps(config,out,viewer=False):
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from run_framework import build_runtime
    fixture=diagnostic_step_config(config);source=out/'step-config.json';source.write_text(json.dumps(fixture,indent=2))
    config,sim,manager,agents,runner=build_runtime(source,out/'step-runtime',viewer)
    start=time.perf_counter();max_tilt=0.;rows=[];first_tip=None;phase_seconds={};saturated=0;ticks=0
    try:
        runner.start()
        with (out/'steps.jsonl').open('w') as trace:
            while manager.status=='running':
                row=runner.tick();angle=sim.robot.get_euler();tilt=max(abs(float(angle[0])),abs(float(angle[1])));max_tilt=max(max_tilt,tilt)
                phase=f'{agents["self"].stage}:{agents["self"].phase}';phase_seconds[phase]=phase_seconds.get(phase,0)+.01;ticks+=1
                saturated+=int(any(abs(t)>=c.limit*.99 for t,c in zip(sim.drives['self'].torques,sim.drives['self'].controllers)))
                if first_tip is None and tilt>90:first_tip=dict(time=manager.elapsed,phase=phase)
                if round(manager.elapsed/.01)%10==0 or row.get('recovery_events'):
                    record={k:row[k] for k in ('time','positions','scores','agent_states','recovery_events') if k in row}
                    record.update(euler_degrees=list(map(float,sim.robot.get_euler())),wheel_torques=sim.drives['self'].torques,wheel_targets=sim.drives['self'].targets,speed=sim.speeds()['self'],feedforward=sim.drives['self'].torque_feedforward,steering_targets=sim.drives['self'].steering_targets,frames=sim.read_telemetry('self')['frames'],frame_target=sim.drives['self'].frame_target)
                    record.update(control_trace(sim.drives['self']))
                    trace.write(json.dumps(record,allow_nan=False)+'\n');trace.flush()
                if agents['self'].done:break
        state=manager.snapshot();pose=sim.truth_positions()['self'];passed=agents['self'].done and manager.collection_counts['self']==3 and pose[2]>.56
        rows.append(dict(case='three_steps',passed=passed,status='all_three_climbed' if passed else 'timeout_or_unstable',simulation_seconds=manager.elapsed,wall_seconds=time.perf_counter()-start,coins=manager.collection_counts['self'],max_tilt_degrees=max_tilt,recovery_counts=state['recovery_counts'],final_position=pose,checkpoints=agents['self'].checkpoints,phase_seconds=phase_seconds,first_tip=first_tip,torque_saturated_fraction=saturated/max(1,ticks)))
    finally:
        runner.close()
        if viewer:sim.scene.viewer.stop()
    return rows

def asset_fingerprint(config):
    asset=Path(config['simulation']['robot_models']['self']['robot_asset'])
    return dict(path=str(asset),sha256=hashlib.sha256(asset.read_bytes()).hexdigest())
