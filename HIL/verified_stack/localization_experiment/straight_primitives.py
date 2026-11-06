"""Filter a copied motion library; preserve rotation and geometrically straight edges."""
import copy
import math
import numpy as np

def alignment_primitive(state, pose_yaw, map_yaw, headings, max_angular_speed,
                        tolerance=.025):
    """An in-place correction before a straight edge, in library coordinates.

    The ordinary activator adds measured heading residual to these samples.
    Subtracting that residual at the endpoint makes the executed endpoint match
    the lattice heading without replacing the measured robot orientation.
    """
    nominal=state[2]*2*math.pi/headings
    error=math.atan2(math.sin(pose_yaw-map_yaw-nominal),
                     math.cos(pose_yaw-map_yaw-nominal))
    if abs(error)<=tolerance:return None
    return dict(id=-1,kind='rotate',start_h=state[2],end_h=state[2],dx=0,dy=0,
                length=0.,duration=abs(error)/max_angular_speed,max_curvature=0.,
                samples=[[0.,0.,float(a),0.,0.,0.]
                         for a in np.linspace(nominal,nominal-error,5)],
                heading_alignment=True)

def straight_library(data):
    result=copy.deepcopy(data)
    kept=[]
    for p in result['primitives']:
        a=np.asarray(p['samples'],float)
        if p['kind']=='rotate' or (np.ptp(a[:,2])<1e-6 and np.max(np.abs(a[:,3]))<1e-6):kept.append(p)
    if not any(p['kind']!='rotate' for p in kept):raise ValueError('Library has no straight translations')
    result['primitives']=kept
    return result
