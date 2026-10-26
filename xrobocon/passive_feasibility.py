"""Necessary quasi-static conditions for symmetric front-bogie step climbing.

Three wheel pairs, ideal passive pivots and ideal left/right differential.
No inertia, tyre deformation, lateral forces, link collisions or gear losses.
A feasible result is a candidate, not a complete mechanism proof.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares, linprog

@dataclass(frozen=True)
class Geometry:
    radius: float = .18
    half_span: float = .26
    com_x: float = -.03
    com_z: float = -.05
    friction: float = .8
    wheel_torque: float = 6
    edge_offsets: tuple = (0.,0.,0.)
    bogie_limit_deg: float = 50
    steering_pair_mass: float = 0.0

def rotate(point, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c,-s],[s,c]]) @ np.asarray(point)

def surface_contact(point, height):
    """Signed circle clearance, closest point, outward unit normal."""
    x,z = point
    if x >= 0:
        return z-height, np.array([x,height]), np.array([0.,1.])
    if z <= height:
        distance, location, normal = -x, np.array([0.,z]), np.array([-1.,0.])
    else:
        location = np.array([0.,height])
        vector = point-location
        distance = np.linalg.norm(vector)
        normal = vector/max(distance,1e-12)
    if z < distance:
        return z, np.array([x,0.]), np.array([0.,1.])
    return distance, location, normal

def kinematics(state, geometry):
    origin, theta, beta = state[:2], state[2], state[3]
    half = geometry.half_span
    pivot = origin+rotate([half/2,-.04],theta)
    front = pivot+rotate([half/2,-.06],theta+beta)
    middle = pivot+rotate([-half/2,-.06],theta+beta)
    rear = origin+rotate([-half,-.10],theta)
    wheels = np.array([front,middle,rear])
    bogie_com = pivot+rotate([0,-.06],theta+beta)
    base_com = origin+rotate([geometry.com_x,geometry.com_z],theta)
    rocker_com = origin+rotate([-half/4,-.07],theta)
    pair_mass = 1+geometry.steering_pair_mass
    com = (4*base_com+rocker_com+.6*bogie_com+pair_mass*wheels.sum(axis=0))/(5.6+3*pair_mass)
    return wheels,pivot,bogie_com,com

def cross(a,b):
    return a[0]*b[1]-a[1]*b[0]

def wheel_contact(point,height,geometry,index):
    offset=geometry.edge_offsets[index]
    distance,location,normal=surface_contact(point-np.array([offset,0]),height)
    return distance,location+np.array([offset,0]),normal

def force_feasible(state,geometry,height):
    wheels,pivot,bogie_com,com = kinematics(state,geometry)
    contacts = [wheel_contact(w,height,geometry,i) for i,w in enumerate(wheels)]
    locations = np.array([c[1] for c in contacts])
    equations = np.zeros((4,6))
    equations[0,::2] = 1
    equations[1,1::2] = 1
    for i,location in enumerate(locations):
        offset = location-com
        equations[2,2*i:2*i+2] = [-offset[1],offset[0]]
        if i < 2:
            offset = location-pivot
            equations[3,2*i:2*i+2] = [-offset[1],offset[0]]
    bogie_weight_moment = .6*9.8*(bogie_com[0]-pivot[0])
    bogie_weight_moment += sum((1+geometry.steering_pair_mass)*9.8*(w[0]-pivot[0]) for w in wheels[:2])
    rhs = [0,(8.6+3*geometry.steering_pair_mass)*9.8,0,bogie_weight_moment]
    inequalities,limits = [],[]
    for i,(_,location,normal) in enumerate(contacts):
        tangent = np.array([-normal[1],normal[0]])
        for vector,bound in [(-normal,0),(tangent-geometry.friction*normal,0),
                              (-tangent-geometry.friction*normal,0)]:
            row = np.zeros(6);row[2*i:2*i+2] = vector
            inequalities.append(row);limits.append(bound)
        offset = location-wheels[i]
        for sign in [-1,1]:
            row = np.zeros(6);row[2*i:2*i+2] = sign*np.array([-offset[1],offset[0]])
            inequalities.append(row);limits.append(2*geometry.wheel_torque)
    # Minimise total absolute wheel torque; avoid arbitrary opposing drive forces
    # on flat ground when the force solution is non-unique.
    inequalities = [np.r_[row,np.zeros(3)] for row in inequalities]
    for i,(_,location,_) in enumerate(contacts):
        offset = location-wheels[i]
        for sign in [-1,1]:
            row = np.zeros(9)
            row[2*i:2*i+2] = sign*np.array([-offset[1],offset[0]])
            row[6+i] = -1
            inequalities.append(row);limits.append(0)
    result = linprog(np.r_[np.zeros(6),np.ones(3)],A_ub=inequalities,b_ub=limits,
                     A_eq=np.c_[equations,np.zeros((4,3))],b_eq=rhs,
                     bounds=[(None,None)]*6+[(0,None)]*3,method='highs')
    return bool(result.success), result.x[:6].tolist() if result.success else None

def evaluate(geometry,height,samples=12):
    r = geometry.radius
    floor_end = -np.sqrt(2*r*height-height*height) if height<r else -r
    path = [(x,r) for x in np.linspace(-.8,floor_end,samples)]
    if height > r:
        path += [(-r,z) for z in np.linspace(r,height,samples)[1:]]
    # Corner begins at the floor intersection for steps lower than the radius.
    start = np.arcsin(max(-1,min(1,(r-height)/r))) if height < r else 0
    path += [(-r*np.cos(a),height+r*np.sin(a)) for a in np.linspace(start,np.pi/2,samples)]
    path += [(x,height+r) for x in np.linspace(0,2*geometry.half_span+.3,samples)[1:]]
    guess = np.array([path[0][0]-geometry.half_span,r+.1,0.,0.])
    records=[]
    for front_target in path:
        def residual(state):
            wheels,*_ = kinematics(state,geometry)
            return np.r_[wheels[0]-front_target,
                         [wheel_contact(w,height,geometry,i)[0]-r for i,w in enumerate(wheels) if i>0]]
        solution = least_squares(residual,guess,bounds=([-3,0,-np.pi/3,-np.deg2rad(geometry.bogie_limit_deg)],
                                                       [3,2,np.pi/3,np.deg2rad(geometry.bogie_limit_deg)]),max_nfev=250)
        guess = solution.x
        error = float(np.max(np.abs(residual(guess))))
        feasible,forces = force_feasible(guess,geometry,height) if error<.001 else (False,None)
        records.append(dict(front_target=list(front_target),state=guess.tolist(),
                            geometric_error=error,force_feasible=feasible,forces=forces))
    return dict(height=height,samples=len(records),geometry_pass=all(r['geometric_error']<.001 for r in records),
                force_pass=all(r['force_feasible'] for r in records),records=records)
