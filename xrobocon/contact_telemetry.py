"""Simulator contact diagnostics, not a deployable robot sensor."""
def read_contact_telemetry(robot, seconds):
    import numpy as np
    from scipy.spatial.transform import Rotation
    entity = robot.entity
    positions = entity.get_links_pos(relative=False).cpu().numpy()
    quaternions = entity.get_links_quat(relative=False).cpu().numpy()
    masses = entity.get_links_mass().cpu().numpy()
    offsets = entity.get_links_COM().cpu().numpy()
    # Genesis quaternions use w,x,y,z; scipy expects x,y,z,w.
    centres = positions + Rotation.from_quat(quaternions[:, [1,2,3,0]]).apply(offsets)
    com = np.average(centres, axis=0, weights=masses)
    contacts = entity.get_contacts(exclude_self_contact=True)
    records = []
    for i in range(len(contacts['link_a'])):
        a, b = int(contacts['link_a'][i]), int(contacts['link_b'][i])
        own_a = entity.link_start <= a < entity.link_end
        own = a if own_a else b
        other = b if own_a else a
        records.append(dict(link=robot.scene.rigid_solver.links[own].name,
                            other_link=other,
                            position=contacts['position'][i].cpu().tolist(),
                            force=contacts['force_a' if own_a else 'force_b'][i].cpu().tolist()))
    wheels = {}
    for side in ('left','right'):
        for name in ('front','middle','rear'):
            link = entity.get_link(f'{side}_wheel_{name}')
            wheels[link.name] = positions[link.idx-entity.link_start].tolist()
    return dict(seconds=seconds, com=com.tolist(), wheel_positions=wheels, contacts=records)
