"""Compare fixed/passive drive on an isolated box or the unchanged outer field tier."""
import argparse
import csv
import json
from math import hypot,isfinite
from pathlib import Path
import sys,time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--viewer',action='store_true')
    p.add_argument('--all-tiers',action='store_true')
    p.add_argument('--field-mesh',action='store_true')
    p.add_argument('--collision-method',choices=['default','gjk'],default='default')
    p.add_argument('--realtime',action='store_true')
    p.add_argument('--start-yaw',type=float,default=0)
    p.add_argument('--out',required=True)
    p.add_argument('--wheel-torque',type=float,default=3)
    p.add_argument('--seconds',type=float,default=40)
    p.add_argument('--telemetry',action='store_true')
    p.add_argument('--quasistatic-feedforward',action='store_true')
    p.add_argument('--scenario',default='configs/scenarios/rigid_sixwheel_flat.json')
    p.add_argument('--box-height',type=float,choices=[.05,.1,.25,.30])
    p.add_argument('--feedback-limit',type=float)
    p.add_argument('--approach-route',action='store_true',help='turn on the floor, align, then climb')
    p.add_argument('--approach-side',type=int,choices=[-1,1],default=1)
    p.add_argument('--round-trip',action='store_true',help='drive on the top tier and descend all tiers')
    p.add_argument('--descent-speed',type=float,default=.08)
    p.add_argument('--descent-kp',type=float,default=.3)
    p.add_argument('--collect-coins',action='store_true',help='collect top-tier spots after ascending')
    p.add_argument('--tier-route',choices=['clockwise','counterclockwise'])
    p.add_argument('--mission-time',type=float,default=180)
    args=p.parse_args()
    if args.tier_route:args.collect_coins=True
    if not isfinite(args.mission_time) or args.mission_time<=0:p.error('positive mission time required')
    if args.approach_route and not args.all_tiers:p.error('approach route requires all tiers')
    if args.round_trip and not args.all_tiers:p.error('round trip requires all tiers')
    if args.collect_coins and (not args.all_tiers or args.round_trip):p.error('coin route requires all tiers and excludes round trip')
    if not isfinite(args.descent_speed) or args.descent_speed<=0:p.error('positive finite descent speed required')
    if not isfinite(args.descent_kp) or args.descent_kp<=0:p.error('positive finite descent kp required')
    if args.all_tiers and (args.box_height or not args.quasistatic_feedforward):p.error('all tiers requires field and quasistatic feedforward')
    if args.quasistatic_feedforward and (args.start_yaw or 'front_bogie_r18' not in args.scenario):p.error('feedforward requires symmetric front_bogie_r18 model')
    if not isfinite(args.start_yaw):p.error("finite start yaw required")
    if not all(isfinite(v) and v>0 for v in (args.wheel_torque,args.seconds)):p.error("positive finite torque and duration required")
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    from xrobocon.common import setup_genesis
    from xrobocon.field import XRoboconField
    from xrobocon.robot import XRoboconRobot
    from xrobocon.scenario import load_scenario
    from xrobocon.pid_drive import DifferentialPIDDrive,GroundTruthPose
    from xrobocon.navigation import WaypointFollower,VelocityCommand
    import genesis as gs
    import numpy as np
    setup_genesis('cpu')
    spec=load_scenario(Path(__file__).resolve().parents[1]/args.scenario)
    import hashlib,shutil
    shutil.copyfile(spec['robot_asset'],out/'robot.xml')
    asset_sha256=hashlib.sha256((out/'robot.xml').read_bytes()).hexdigest()
    spec['robot_asset']=str((out/'robot.xml').resolve())
    (out/'scenario.json').write_text(json.dumps(spec,indent=2))
    scene=gs.Scene(viewer_options=gs.options.ViewerOptions(camera_pos=(9,-11,11) if args.tier_route else (7,-5,4) if args.all_tiers else (6.8,-2.5,1.8),camera_lookat=(0,0,.3) if args.tier_route else (2.7,0,.3) if args.all_tiers else (4.6,0,.12),camera_fov=50 if args.tier_route else 45 if args.all_tiers else 40,refresh_rate=60),
                   rigid_options=gs.options.RigidOptions(dt=.01,gravity=(0,0,-9.8),use_gjk_collision=True if args.collision_method=='gjk' else None),show_viewer=args.viewer)
    scene.add_entity(gs.morphs.Plane())
    field=XRoboconField()
    if args.box_height:
        scene.add_entity(gs.morphs.Box(pos=(3.9,0,args.box_height/2),size=(1.5,2,args.box_height),fixed=True))
    else:field.build(scene,collision_mesh_dir=out/'field-mesh' if args.field_mesh else None)
    if args.collect_coins and args.viewer:
        from xrobocon.game import XRoboconGame
        field.add_coin_spots(scene,XRoboconGame(field,None).spots)
    # Marks never participate in contacts.
    if args.viewer:
        marks=[(5.25,.035,(1,.5,.1))]+([(4,.135,(.2,.4,1)),(2.6,.385,(.2,1,.4)),(1.2,.635,(1,.8,.1))] if args.all_tiers else [(4.,(args.box_height or .1)+.035,(.2,.4,1))])
        for x,z,color in marks:
            scene.add_entity(gs.morphs.Sphere(pos=(x,0,z),radius=.035,fixed=True,collision=False),surface=gs.surfaces.Default(color=color))
    robot=XRoboconRobot(scene,robot_type=spec['robot_type'],asset_path=spec['robot_asset'],pos=(5.8,-args.approach_side,.65) if args.approach_route else (5.25,0,.65),euler=(0,0,90*args.approach_side if args.approach_route else 180+args.start_yaw))
    scene.build();robot.post_build()
    drive=DifferentialPIDDrive(robot,torque_limit=args.wheel_torque,feedback_limit=args.feedback_limit);source=GroundTruthPose(robot)
    if (args.approach_route or args.round_trip) and not drive.steering:p.error('route currently requires steering wheels')
    for _ in range(150):drive.apply_velocity(VelocityCommand(0,0),.01);scene.step()
    telemetry=(out/'contacts.jsonl').open('w') if args.telemetry else None
    base_z=float(robot.get_pos()[2]);follower=WaypointFollower([(4,0)],max_speed=.12)
    game=None;coin_index=0;coin_events=[]
    if args.collect_coins:
        from xrobocon.game import XRoboconGame
        game=XRoboconGame(field,robot,robot_height_offset=base_z);game.time_limit=args.mission_time;game.start()
        coin_targets=[s for s in game.spots if s['tier']==1]
        if args.tier_route:
            from xrobocon.coin_routes import tier_coin_plans,coin_leg,ring_path
            tier_plans=tier_coin_plans(game.spots,args.tier_route)
            coin_targets=tier_plans[0];coin_angle=0
            if args.viewer:
                for plan,color in zip(tier_plans,[(1,.4,.1,1),(.2,.5,1,1),(.2,1,.4,1)]):
                    angle=0;points=[(hypot(*plan[0]['pos'][:2]),0)]
                    for spot in plan:
                        leg,angle=coin_leg(angle,spot,args.tier_route);points+=leg
                    z=plan[0]['pos'][2]+.025
                    for a,b in zip(points,points[1:]):scene.draw_debug_line((*a,z),(*b,z),radius=.012,color=color)
        (out/'effective_mission.json').write_text(json.dumps(dict(field=field.tiers,
            coins=[dict(id=s['id'],pos=s['pos'],tier=s['tier']) for s in game.spots],
            rules=dict(time_limit=game.time_limit,coin_radius=game.coin_radius,upper_tier_stay_time=game.upper_tier_stay_time),
            route_coin_ids=[s['id'] for plan in tier_plans for s in plan] if args.tier_route else [s['id'] for s in coin_targets],robot_height_offset=base_z,
            source='legacy default field and coin layout; robot selected by scenario'),indent=2))
    phase='approach' if args.approach_route else 'climb'
    if args.approach_route:follower=WaypointFollower([(5.8,0),(5.25,0)],max_speed=.12,max_turn=.3)
    phase_results=[];aligned=0
    stage=0;stage_results=[];descent_results=[]
    edges=[4.65,3.25,1.85];heights=[.1,.35,.6];goals=[4.,2.6,1.2]
    passive=[robot.entity.get_joint(f'{side}_{joint}_joint').dofs_idx_local[0] for side in ('left','right') for joint in ('rocker','bogie')] if robot.entity.n_equalities else []
    max_diff_error=0.0
    stalled=0;stable=0;status='timeout';max_tilt=0;min_radius=100
    with (out/'trace.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['step','x','y','z','roll','pitch','speed','command_v','status','passive_angles','wheel_torques','stage','feedforward','phase','yaw','command_w','steering_angles'])
        for step in range(int(args.seconds/.01)):
            started=time.perf_counter();pose=source.read_pose();command=follower.update(pose,.01)
            from math import pi,atan2,sin,cos
            desired_heading=0 if phase.startswith('exit_') else pi
            heading_error=atan2(sin(desired_heading-pose.yaw),cos(desired_heading-pose.yaw))
            if phase in ('align','exit_align'):command=VelocityCommand(0,max(-.2,min(.2,heading_error)))
            elif phase in ('straighten','exit_straighten'):
                command=VelocityCommand(0,0);drive.steering_targets=[0.0]*len(drive.steering)
            drive.torque_feedforward=[0.0]*len(drive.indices)
            if args.quasistatic_feedforward and phase=='climb':
                from xrobocon.passive_feasibility import Geometry,force_feasible,kinematics,surface_contact,cross,wheel_contact
                from math import atan2,radians
                pre=robot.get_pos().cpu().numpy()
                theta=-radians(float(robot.get_euler()[1]))
                beta=-float(robot.entity.get_dofs_position()[passive[1]])
                edge=edges[stage] if args.all_tiers else 4.65
                lower_height=([0,.1,.35][stage] if args.all_tiers else 0)
                rise=([.1,.25,.25][stage] if args.all_tiers else args.box_height or .1)
                state=np.array([edge-pre[0],pre[2]-lower_height,theta,beta])
                offsets=tuple(edge-(edge*edge-y*y)**.5 for y in (.24,.14,.24)) if not args.box_height else (0.,0.,0.)
                geometry=Geometry(wheel_torque=args.wheel_torque,edge_offsets=offsets,
                                  steering_pair_mass=.1 if drive.steering else 0)
                wheels,*_=kinematics(state,geometry)
                valid=max(abs(wheel_contact(w,rise,geometry,i)[0]-geometry.radius) for i,w in enumerate(wheels))<.015
                success,forces=force_feasible(state,geometry,rise) if valid else (False,None)
                ff=[cross(wheel_contact(w,rise,geometry,i)[1]-w,np.array(forces[2*i:2*i+2]))/2 for i,w in enumerate(wheels)] if success else [0.,0.,0.]
                drive.torque_feedforward=ff+ff
            drive.apply_velocity(command,.01);scene.step()
            pos=robot.get_pos().cpu().numpy();euler=robot.get_euler();speed=float(np.linalg.norm(robot.get_vel().cpu().numpy()[:2]))
            radius=hypot(pos[0],pos[1]);tilt=float(max(abs(euler[0]),abs(euler[1])));min_radius=min(min_radius,radius);max_tilt=max(max_tilt,tilt)
            angles=[float(robot.entity.get_dofs_position()[i]) for i in passive]
            if angles:max_diff_error=max(max_diff_error,abs(angles[0]+angles[2]))
            steering_angles=[float(robot.entity.get_dofs_position()[i]) for i in drive.steering]
            w.writerow([step,*pos,*euler[:2],speed,command.linear,status,json.dumps(angles),json.dumps(drive.torques),stage,json.dumps(drive.torque_feedforward),phase,euler[2],command.angular,json.dumps(steering_angles)])
            if telemetry and step%10==0:
                from xrobocon.contact_telemetry import read_contact_telemetry
                telemetry.write(json.dumps(read_contact_telemetry(robot,step*.01))+'\n')
            if not np.isfinite([*pos,*euler,speed]).all():status='nonfinite';break
            if tilt>60:status='abort_posture';break
            if game:
                before={s['id'] for s in game.spots if s['collected']}
                game.update(.01)
                for spot in game.spots:
                    if spot['collected'] and spot['id'] not in before:
                        coin_events.append(dict(id=spot['id'],tier=spot['tier'],seconds=(step+1)*.01,position=pos.tolist(),score=game.score))
                        if args.viewer:scene.draw_debug_sphere(pos=spot['pos'],radius=.16,color=(.35,.35,.35,1))
                if not game.is_running:status='competition_time_limit';break
            if phase=='coins':
                stable=stable+1 if follower.done and coin_targets[coin_index]['collected'] and speed<.05 else 0
                if stable>=100:
                    phase_results.append(dict(phase=phase,coin_id=coin_targets[coin_index]['id'],seconds=(step+1)*.01,position=pos.tolist()))
                    coin_index+=1;stable=0
                    for controller in drive.controllers:controller.integral=0.0
                    if coin_index==len(coin_targets):
                        if args.tier_route and stage<2:
                            phase='return_gate'
                            rr=hypot(*coin_targets[-1]['pos'][:2])
                            follower=WaypointFollower(ring_path(coin_angle,0,rr)+[(goals[stage],0)],max_speed=.12,max_turn=.3)
                            continue
                        status='all_tier_coins_collected_and_stopped' if args.tier_route else 'top_coins_collected_and_stopped';break
                    if args.tier_route:
                        points,coin_angle=coin_leg(coin_angle,coin_targets[coin_index],args.tier_route)
                    else:points=[coin_targets[coin_index]['pos'][:2]]
                    follower=WaypointFollower(points,max_speed=.12,max_turn=.3)
                if args.realtime:time.sleep(max(0,.01-(time.perf_counter()-started)))
                continue
            if phase not in ('climb','descend'):
                ready=(follower.done and speed<.05) if phase in ('approach','top_route','return_gate') else (abs(heading_error)<.03 and speed<.03) if phase in ('align','exit_align') else (max(abs(v) for v in steering_angles)<.03 and speed<.03 and abs(heading_error)<.05)
                aligned=aligned+1 if ready else 0
                if aligned>=100:
                    phase_results.append(dict(phase=phase,seconds=(step+1)*.01,position=pos.tolist(),yaw_deg=float(euler[2]),steering_angles=steering_angles))
                    if phase=='return_gate':stage+=1;coin_index=0;coin_angle=0;coin_targets=tier_plans[stage]
                    phase={'approach':'align','return_gate':'align','align':'straighten','straighten':'climb','top_route':'exit_align','exit_align':'exit_straighten','exit_straighten':'descend'}[phase];aligned=0
                    # Remove accumulated wheel-speed error when changing motion mode.
                    for controller in drive.controllers:controller.integral=0.0
                    if phase=='climb':follower=WaypointFollower([(goals[stage],0)],max_speed=.12)
                    elif phase=='descend':
                        stage=0;stable=0;stalled=0
                        for controller in drive.controllers:controller.kp=args.descent_kp
                        follower=WaypointFollower([(2.6,0)],max_speed=args.descent_speed,max_turn=.2)
                if args.realtime:time.sleep(max(0,.01-(time.perf_counter()-started)))
                continue
            if phase=='descend':
                down_edges=[1.85,3.25,4.65];down_heights=[.35,.1,0];down_goals=[2.6,4.,5.25]
                atop=radius>down_edges[stage]+.35 and abs(pos[2]-base_z-down_heights[stage])<.03
                stable=stable+1 if follower.done and atop and speed<.05 else 0
                if stable>=100:
                    descent_results.append(dict(stage=stage,seconds=(step+1)*.01,position=pos.tolist()))
                    if stage<2:
                        stage+=1;stable=0
                        for controller in drive.controllers:controller.integral=0.0
                        follower=WaypointFollower([(down_goals[stage],0)],max_speed=args.descent_speed,max_turn=.2)
                    else:status='round_trip_completed_and_stopped';break
                if args.realtime:time.sleep(max(0,.01-(time.perf_counter()-started)))
                continue
            stalled=stalled+1 if radius>4.65 and radius<4.95 and command.linear>.04 and speed<.01 else 0
            if stalled>=1000:status='blocked_at_outer_step';break
            atop=radius<(edges[stage]-.3 if args.all_tiers else 4.35) and abs(pos[2]-base_z-(heights[stage] if args.all_tiers else args.box_height or .1))<.03
            stable=stable+1 if follower.done and atop and speed<.05 else 0
            if stable>=100:
                stage_results.append(dict(stage=stage,seconds=(step+1)*.01,position=pos.tolist()))
                if args.tier_route:
                    phase='coins';coin_index=0;stable=0
                    for controller in drive.controllers:controller.integral=0.0
                    points,coin_angle=coin_leg(0,coin_targets[0],args.tier_route)
                    follower=WaypointFollower(points,max_speed=.12,max_turn=.3)
                elif args.all_tiers and stage<2:
                    stage+=1;stable=0;stalled=0
                    follower=WaypointFollower([(goals[stage],0)],max_speed=.12)
                elif args.round_trip:
                    phase='top_route';stable=0
                    for controller in drive.controllers:controller.integral=0.0
                    follower=WaypointFollower([(.8,.4),(.8,-.4),(1.1,0)],max_speed=.08,max_turn=.2)
                elif args.collect_coins:
                    phase='coins';stable=0
                    for controller in drive.controllers:controller.integral=0.0
                    follower=WaypointFollower([coin_targets[0]['pos'][:2]],max_speed=.12,max_turn=.3)
                else:status='climbed_all_tiers_and_stopped' if args.all_tiers else 'climbed_and_stopped';break
            if step%100==0:print(f'DEMO t={step*.01:.1f}s radius={radius:.3f} z={pos[2]:.3f}',flush=True)
            if args.realtime:time.sleep(max(0,.01-(time.perf_counter()-started)))
    if telemetry:telemetry.close()
    result=dict(status=status,step_heights=[.1,.25,.25] if args.all_tiers else [args.box_height or .1],max_step_height=.25 if args.all_tiers else args.box_height or .1,step_height=args.box_height or .1,outer_radius=4.65,wheel_radius=drive.radius,wheel_torque_limit=args.wheel_torque,feedback_limit=args.feedback_limit,
                spawn_height=.65,start_yaw=args.start_yaw,seconds=(step+1)*.01,final_position=pos.tolist(),min_radius=min_radius,max_tilt=max_tilt,
                settled_height=base_z,equalities=robot.entity.n_equalities,max_differential_error_rad=max_diff_error,field_mesh=args.field_mesh,collision_method=args.collision_method,stages=stage_results,quasistatic_feedforward=args.quasistatic_feedforward,asset_sha256=asset_sha256,asset=spec['robot_asset'],note='box test' if args.box_height else 'existing three-tier field; first encountered tier is outer Tier3; truth pose; CPU')
    result.update(approach_route=args.approach_route,approach_side=args.approach_side,final_phase=phase,phase_checkpoints=phase_results)
    result.update(round_trip=args.round_trip,descent_speed=args.descent_speed,descent_kp=args.descent_kp,descent_checkpoints=descent_results)
    result.update(tier_route=args.tier_route,mission_time=args.mission_time)
    if args.tier_route:result['tier_coin_plan_ids']=[[s['id'] for s in plan] for plan in tier_plans]
    if game:result.update(game=game.get_info(),coin_events=coin_events,coin_targets=[s['id'] for s in coin_targets],robot_height_offset=base_z,rules_note='provisional simulator rules, not verified official competition rules')
    result.update(spawn_position=[5.8,-args.approach_side,.65] if args.approach_route else [5.25,0,.65],
                  spawn_yaw_deg=90*args.approach_side if args.approach_route else 180+args.start_yaw)
    (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    if args.viewer:
        # Keep the stalled or reached state visible, actively braking wheels.
        for _ in range(1000):
            started=time.perf_counter();drive.apply_velocity(VelocityCommand(0,0),.01);scene.step()
            if args.realtime:time.sleep(max(0,.01-(time.perf_counter()-started)))

if __name__=='__main__':main()
