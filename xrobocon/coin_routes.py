"""Ring waypoints keep the robot on its current tier between coin spots."""
from math import atan2,cos,sin,hypot,ceil,pi

def ring_path(start_angle,end_angle,radius):
    count=max(1,ceil(abs(end_angle-start_angle)/(pi/16)))
    return [(radius*cos(start_angle+(end_angle-start_angle)*i/count),
             radius*sin(start_angle+(end_angle-start_angle)*i/count)) for i in range(1,count+1)]

def tier_coin_plans(spots,direction):
    if direction not in ('clockwise','counterclockwise'):raise ValueError('unknown route')
    ids=([0,1],[8,9],[16,17]) if direction=='counterclockwise' else ([0,7],[15,14],[16,19])
    return [[spots[i] for i in group] for group in ids]

def coin_leg(previous_angle,spot,direction):
    angle=atan2(spot['pos'][1],spot['pos'][0])
    if direction=='counterclockwise' and angle<0:angle+=2*pi
    if direction=='clockwise' and angle>0:angle-=2*pi
    return ring_path(previous_angle,angle,hypot(*spot['pos'][:2])),angle
