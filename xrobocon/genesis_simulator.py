"""Physics-only Genesis adapter. It never creates a game or awards points."""
from math import isfinite
from xrobocon.navigation import VelocityCommand
from xrobocon.program_drive import ProgramDrive

class GenesisSimulator:
    def __init__(self,scenario_path,participant='self',viewer=False,torque_limit=6,spawns=None,field_mesh_dir=None,climb_assist=False,coins=None,show_coins=None,camera=None,sensors=None,robot_models=None):
        import genesis as gs
        from xrobocon.common import setup_genesis
        from xrobocon.scenario import load_scenario
        from xrobocon.field import XRoboconField
        from xrobocon.robot import XRoboconRobot
        from xrobocon.pid_drive import DifferentialPIDDrive,GroundTruthPose
        setup_genesis('cpu');spec=load_scenario(scenario_path)
        self.spawns=spawns or {participant:dict(pos=[0,0,.65],euler=[0,0,0])}
        if not self.spawns:raise ValueError('at least one spawn required')
        for spawn in self.spawns.values():
            if any(len(spawn[key])!=3 or not all(isfinite(v) for v in spawn[key]) for key in ('pos','euler')):raise ValueError('finite XYZ spawns required')
        view=dict(camera_pos=(9,-11,11) if climb_assist else (4,-5,4),camera_lookat=(0,0,.3) if climb_assist else (0,0,0),camera_fov=50)
        if camera:view.update(camera)
        self.scene=gs.Scene(viewer_options=gs.options.ViewerOptions(**view),rigid_options=gs.options.RigidOptions(dt=.01,gravity=(0,0,-9.8),use_gjk_collision=True if climb_assist else None),show_viewer=viewer)
        self.scene.add_entity(gs.morphs.Plane())
        self.field=XRoboconField(spec['field']);self.field.build(self.scene,collision_mesh_dir=field_mesh_dir)
        self.coin_entities={}
        if (viewer if show_coins is None else show_coins) and coins:
            self.coin_entities=dict(zip([c['id'] for c in coins],self.field.add_coin_spots(self.scene,coins)))
        self.coin_display=None
        if self.coin_entities:
            from xrobocon.coin_display import CoinDisplay
            self.coin_display=CoinDisplay({c['id']:c['pos'] for c in coins},lambda key,pos:self.coin_entities[key].set_pos(pos))
        self.climb_assist=climb_assist;self.torque_limit=torque_limit
        models=robot_models or {}
        if set(models)-set(self.spawns):raise ValueError('robot models must match participants')
        self.robots={name:XRoboconRobot(self.scene,robot_type=models.get(name,{}).get('robot_type',spec['robot_type']),asset_path=models.get(name,{}).get('robot_asset',spec.get('robot_asset')),pos=spawn['pos'],euler=spawn['euler']) for name,spawn in self.spawns.items()}
        from xrobocon.sensors import SensorSuite,validate_sensor_config
        from xrobocon.genesis_sensors import GenesisSensorBackend
        sensor_config=validate_sensor_config(sensors or {},self.robots)
        self.sensor_backend=GenesisSensorBackend(self.scene,self.robots,sensor_config)
        self.sensor_suite=SensorSuite(sensor_config,self.sensor_backend.read);self.sensor_time=0.
        self.scene.build()
        for robot in self.robots.values():robot.post_build()
        from xrobocon.program_drive import ProgramDrive
        self.drives={name:(ProgramDrive(robot,models[name]['controller_file'],torque_limit) if models.get(name,{}).get('drive_kind')=='program' else DifferentialPIDDrive(robot,torque_limit=torque_limit,**models.get(name,{}).get('drive_parameters',{}))) for name,robot in self.robots.items()}
        self.pose_sources={name:GroundTruthPose(robot) for name,robot in self.robots.items()}
        self.robot=next(iter(self.robots.values())) # compatibility for asset export
        self.robot_contact_steps=0
        self.initial_qpos={name:robot.entity.get_qpos().clone() for name,robot in self.robots.items()}
        self.robot_markers=None
        if viewer:
            from xrobocon.robot_markers import RobotMarkers
            self.robot_markers=RobotMarkers(self.scene,self.robots)
    def reset(self):
        self.scene.reset();self.robot_contact_steps=0
        if self.coin_display:self.coin_display.invalidate()
        for name,robot in self.robots.items():
            spawn=self.spawns[name];robot.set_pose(spawn['pos'],spawn['euler'])
            drive=self.drives[name]
            if hasattr(drive,'reset'):drive.reset()
            for c in drive.controllers:c.integral=0;c.previous_measurement=None
            drive.steering_targets=[0.]*len(drive.steering);drive.frame_target=0.
            drive.torque_feedforward=[0.]*len(drive.indices)
        for _ in range(150):
            for drive in self.drives.values():drive.apply_velocity(VelocityCommand(0,0),.01)
            self.scene.step()
        self.height_offsets={name:float(robot.get_pos()[2])-self.field.get_terrain_height(*self.spawns[name]['pos'][:2]) for name,robot in self.robots.items()}
        if self.robot_markers:self.robot_markers.sync()
        self.sensor_time=0.;self.sensor_suite.reset();self.sensor_suite.update(0.)
    def read_sensors(self,participant):return self.sensor_suite.read(participant)
    def respawn(self,participant):
        """Reset only one body/controller; never step or reset the shared world."""
        robot=self.robots[participant];spawn=self.spawns[participant]
        robot.entity.set_qpos(self.initial_qpos[participant],zero_velocity=True)
        robot.set_pose(spawn['pos'],spawn['euler'])
        drive=self.drives[participant]
        if hasattr(drive,'reset'):drive.reset()
        for c in drive.controllers:c.integral=0;c.previous_measurement=None
        drive.steering_targets=[0.]*len(drive.steering);drive.frame_target=0.
        drive.torque_feedforward=[0.]*len(drive.indices)
        drive.stop()
        if participant in self.sensor_suite.config:self.sensor_suite.reset(participant,not_before=self.sensor_time+.01)
        if self.robot_markers:self.robot_markers.sync()
    def read_pose(self,participant):return self.pose_sources[participant].read_pose()
    def truth_positions(self):
        result={}
        for name,robot in self.robots.items():
            pos=robot.get_pos().cpu().tolist();pos[2]-=self.height_offsets[name]
            if not all(isfinite(v) for v in pos):raise RuntimeError('nonfinite physics state')
            result[name]=pos
        return result
    def sync_competition_display(self,state):
        return self.coin_display.sync(state['coins']) if self.coin_display else []
    def read_telemetry(self,participant):
        robot=self.robots[participant];drive=self.drives[participant]
        return dict(ground_z=self.truth_positions()[participant][2],speed=self.speeds()[participant],
                    steering=[float(robot.entity.get_dofs_position()[i]) for i in drive.steering],
                    frames=[float(robot.entity.get_dofs_position()[i]) for i in drive.frames],
                    tilt_degrees=max(abs(float(v)) for v in robot.get_euler()[:2]))
    def _feedforward(self,name,stage):
        from math import hypot,radians
        import numpy as np
        from xrobocon.passive_feasibility import Geometry,kinematics,wheel_contact,force_feasible,cross
        robot=self.robots[name];drive=self.drives[name]
        tiers=sorted(self.field.tiers,key=lambda t:t['radius'],reverse=True)
        if stage not in range(len(tiers)):raise ValueError('invalid climb stage')
        edge=tiers[stage]['radius'];lower=0 if stage==0 else tiers[stage-1]['height'];rise=tiers[stage]['height']-lower
        pos=robot.get_pos().cpu().numpy();beta=-float(robot.entity.get_dofs_position()[robot.entity.get_joint('left_bogie_joint').dofs_idx_local[0]])
        state=np.array([edge-hypot(*pos[:2]),pos[2]-lower,-radians(float(robot.get_euler()[1])),beta])
        offsets=tuple(edge-(edge*edge-y*y)**.5 for y in drive.wheel_lateral_offsets[:3])
        geometry=Geometry(radius=drive.radius,com_x=float(robot.base_com[0]),com_z=float(robot.base_com[2]),wheel_torque=self.torque_limit,edge_offsets=offsets,steering_pair_mass=.1)
        wheels,*_=kinematics(state,geometry)
        valid=max(abs(wheel_contact(w,rise,geometry,i)[0]-geometry.radius) for i,w in enumerate(wheels))<.015
        success,forces=force_feasible(state,geometry,rise) if valid else (False,None)
        ff=[cross(wheel_contact(w,rise,geometry,i)[1]-w,np.array(forces[2*i:2*i+2]))/2 for i,w in enumerate(wheels)] if success else [0.,0.,0.]
        return ff+ff
    def step(self,commands,dt):
        if abs(dt-.01)>1e-12:raise ValueError('Genesis adapter requires dt=.01')
        if set(commands)!=set(self.robots):raise ValueError('commands must match robots')
        for name,drive in self.drives.items():
            command=commands[name]
            if command.reset_drive:
                if hasattr(drive,'reset'):drive.reset()
                for controller in drive.controllers:controller.integral=0;controller.previous_measurement=None
            if command.straighten:drive.steering_targets=[0.]*len(drive.steering);drive.frame_target=0.
            drive.torque_feedforward=self._feedforward(name,command.climb_stage) if self.climb_assist and command.climb_stage is not None and not isinstance(drive,ProgramDrive) else [0.]*len(drive.indices)
            if command.frame_target is not None:
                if not drive.frames and not hasattr(drive,'program'):raise ValueError("frame target requires an actuated carrier")
                drive.set_frame_target(command.frame_target)
            drive.apply_velocity(command,dt)
        self.step_physics(dt)
        contact=False
        for name,robot in self.robots.items():
            records=robot.entity.get_contacts(exclude_self_contact=True)
            for a,b in zip(records['link_a'],records['link_b']):
                for other,body in self.robots.items():
                    if other!=name and (body.entity.link_start<=int(a)<body.entity.link_end or body.entity.link_start<=int(b)<body.entity.link_end):contact=True
        self.robot_contact_steps+=int(contact)
    def step_physics(self,dt=.01):
        """Standalone physics and sensor sampling; no agent/game required."""
        if abs(dt-.01)>1e-12:raise ValueError('Genesis adapter requires dt=.01')
        self.scene.step();self.sensor_time+=dt;self.sensor_suite.update(self.sensor_time)
        if self.robot_markers:self.robot_markers.sync()
    def speeds(self):
        import numpy as np
        return {name:float(np.linalg.norm(robot.get_vel().cpu().numpy()[:2])) for name,robot in self.robots.items()}
    def stop(self):
        for drive in self.drives.values():drive.stop()
