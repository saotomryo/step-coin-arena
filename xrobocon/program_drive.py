"""Named-joint drive for generated wheel, leg and auxiliary mechanisms."""
from math import isfinite
from xrobocon.pid_drive import PID
from xrobocon.workshop.generated import ControlProgram, load_bundle

class ProgramDrive:
    """Same body-velocity interface as the existing drive; pure joint programs below.

    Only local joint encoders are read here. Game state/route strategy stays in
    the ROS2 agent; no ground-truth position or opponent data enters this code.
    """
    def __init__(self, robot, controller_file, torque_limit=6):
        import mujoco
        self.robot=robot
        bundle=load_bundle(controller_file,robot.asset_path)
        self.specs=bundle['joints'];self.program=ControlProgram(bundle['code'],self.specs)
        self.indices=[robot.entity.get_joint(s['name']).dofs_idx_local[0] for s in self.specs]
        self.controllers=[PID(s['kp'],0,s['kd'],min(s['torque_limit'],torque_limit)) for s in self.specs]
        model=mujoco.MjModel.from_xml_path(robot.asset_path)
        self.ranges=[]
        for spec in self.specs:
            index=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,spec['name'])
            self.ranges.append(tuple(model.jnt_range[index]) if model.jnt_limited[index] else None)
        # Legacy telemetry consumers can read these empty lists without assuming wheels.
        self.steering=[];self.frames=[];self.steering_targets=[];self.frame_target=0.
        self.torque_feedforward=[0.]*len(self.indices)
        self.targets=[0.]*len(self.indices);self.torques=[0.]*len(self.indices)
        self.lift_override=None
        self.reset()

    def reset(self):
        self.time=0.
        for controller in self.controllers:
            controller.integral=0.;controller.previous_measurement=None

    def set_frame_target(self, angle):
        if not isfinite(angle):raise ValueError('finite frame target required')
        self.frame_target=angle

    def compute(self, command, dt, position, velocity):
        if not isfinite(dt) or dt<=0:raise ValueError('positive dt required')
        state=dict(t=self.time,dt=dt,linear=float(command.linear),angular=float(command.angular),
            lift=float(self.lift_override if self.lift_override is not None else command.climb_stage is not None),
            frame_target=self.frame_target,climb_stage=float(command.climb_stage if command.climb_stage is not None else -1),
            q={s['name']:float(position[i]) for s,i in zip(self.specs,self.indices)},
            qd={s['name']:float(velocity[i]) for s,i in zip(self.specs,self.indices)})
        targets=self.program.evaluate(state)
        if not all(isfinite(v) for v in (*state['q'].values(),*state['qd'].values())):raise ValueError('nonfinite encoder state')
        torques=[];bounded=[]
        for spec,index,controller,joint_range in zip(self.specs,self.indices,self.controllers,self.ranges):
            target=max(-spec['target_limit'],min(spec['target_limit'],targets[spec['name']]))
            if spec['mode']=='position':
                if joint_range:target=max(joint_range[0],min(joint_range[1],target))
                torque=spec['kp']*(target-float(position[index]))-spec['kd']*float(velocity[index])
                torque=max(-controller.limit,min(controller.limit,torque))
            else:torque=controller.update(target,float(velocity[index]),dt)
            torques.append(torque);bounded.append(target)
        self.time+=dt;self.targets=bounded;self.torques=torques
        return torques

    def apply_velocity(self, command, dt):
        import genesis as gs
        import torch
        entity=self.robot.entity
        force=torch.zeros(self.robot.n_dofs,device=gs.device)
        try:
            torques=self.compute(command,dt,entity.get_dofs_position().cpu().numpy(),entity.get_dofs_velocity().cpu().numpy())
            for i,torque in zip(self.indices,torques):force[i]=torque
        except Exception:
            entity.control_dofs_force(force)
            raise
        entity.control_dofs_force(force)

    def stop(self):
        import genesis as gs
        import torch
        self.robot.entity.control_dofs_force(torch.zeros(self.robot.n_dofs,device=gs.device))
        self.torques=[0.]*len(self.indices)
