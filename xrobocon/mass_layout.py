"""Aggregate axis-aligned component boxes without extra physical bodies."""
import numpy as np

def aggregate_boxes(components):
    if not components:
        raise ValueError('components required')
    mass=np.asarray([c['mass'] for c in components],dtype=float)
    positions=np.asarray([c['pos'] for c in components],dtype=float)
    sizes=np.asarray([c['size'] for c in components],dtype=float)
    if positions.shape!=(len(mass),3) or sizes.shape!=positions.shape or not all(np.isfinite(a).all() for a in (mass,positions,sizes)) or (mass<=0).any() or (sizes<=0).any():
        raise ValueError('finite positive masses and box sizes required')
    total=float(mass.sum());com=(mass[:,None]*positions).sum(axis=0)/total
    inertia=np.zeros((3,3))
    for m,p,s in zip(mass,positions,sizes):
        # size is full width, not MJCF half-extent.
        diagonal=m/12*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2])
        d=p-com;inertia+=np.diag(diagonal)+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    return total,com,inertia
