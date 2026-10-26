"""Genesis-native sensor attachment and conversion, with no game dependencies."""
from math import pi,isfinite

class GenesisSensorBackend:
    def __init__(self,scene,robots,config):
        import genesis as gs
        self.robots=robots;self.handles={}
        for name,spec in config.items():
            robot=robots[name]
            for kind,options in spec.items():
                if kind not in ('lidar','imu'):continue
                mount={key:options[key] for key in ('pos_offset','euler_offset','link_idx_local') if key in options}
                mount['entity_idx']=robot.entity.idx
                if kind=='imu':
                    mount.update({key:options[key] for key in ('acc_noise','gyro_noise') if key in options})
                    handle=scene.add_sensor(gs.sensors.IMU(**mount))
                else:
                    pattern=gs.sensors.SphericalPattern(angles=([-180.+360.*i/options['rays'] for i in range(options['rays'])],[0.]))
                    handle=scene.add_sensor(gs.sensors.Raycaster(**mount,pattern=pattern,min_range=options['min_range'],max_range=options['max_range'],no_hit_value=float('inf'),return_points=False))
                self.handles[name,kind]=handle
    def read(self,name,kind,options):
        entity=self.robots[name].entity
        if kind=='imu':
            value=self.handles[name,kind].read()
            return dict(linear_acceleration=value.lin_acc.reshape(-1).cpu().tolist(),angular_velocity=value.ang_vel.reshape(-1).cpu().tolist(),orientation=None,pos_offset=options.get('pos_offset',[0.,0.,0.]),euler_offset=options.get('euler_offset',[0.,0.,0.]),link_idx_local=options.get('link_idx_local',0))
        if kind=='lidar':
            ranges=self.handles[name,kind].read().distances.reshape(-1).cpu().tolist()
            return dict(angle_min=-pi,angle_max=pi-2*pi/options['rays'],angle_increment=2*pi/options['rays'],range_min=options['min_range'],range_max=options['max_range'],ranges=[float(v) if isfinite(v) else None for v in ranges],scan_time=1/options['hz'],time_increment=0.,pos_offset=options['pos_offset'],euler_offset=options.get('euler_offset',[0.,0.,0.]),link_idx_local=options.get('link_idx_local',0))
        if kind=='joint_states':
            # One-DOF hinges/sliders only: exclude the floating base's true pose.
            joints=[j for j in entity.joints if j.n_dofs==1 and j.n_qs==1]
            indices=[j.dofs_idx_local[0] for j in joints]
            position=entity.get_dofs_position().cpu().tolist();velocity=entity.get_dofs_velocity().cpu().tolist()
            return dict(name=[j.name for j in joints],position=[position[i] for i in indices],velocity=[velocity[i] for i in indices])
        if kind=='contacts':
            contacts=entity.get_contacts(exclude_self_contact=True)
            # Forces are reported on this robot's links, in world coordinates.
            records=[]
            for a,b,pos,fa,fb in zip(contacts['link_a'],contacts['link_b'],contacts['position'],contacts['force_a'],contacts['force_b']):
                own_a=entity.link_start<=int(a)<entity.link_end
                records.append(dict(link_idx_local=int(a if own_a else b)-entity.link_start,position=pos.cpu().tolist(),force=(fa if own_a else fb).cpu().tolist()))
            return dict(in_contact=bool(records),contacts=records,coordinate_frame='world')
        raise ValueError(f'unsupported sensor {kind}')
