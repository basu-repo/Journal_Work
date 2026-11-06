"""D* local motion controller and collision checks.

Retains velocity rollout, hard obstacle rejection, progress/heading/smoothness
costs, obstacle-side cues and checked reverse/rotation recovery. Geometry and
all operating/tuning limits are supplied by config. No world/truth inputs.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

def rollout(pose,v,w,dt,steps):
    x,y,yaw=pose.x,pose.y,pose.yaw;states=[(x,y,yaw)]
    for _ in range(steps):
        # Exact constant-twist unicycle arc; also samples the initial footprint.
        if abs(w)>1e-9:
            x+=v/w*(math.sin(yaw+w*dt)-math.sin(yaw))
            y-=v/w*(math.cos(yaw+w*dt)-math.cos(yaw))
        else:x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt
        yaw=wrap(yaw+w*dt);states.append((x,y,yaw))
    return np.asarray(states)

def nearest_obstacle_sector(pose,points):
    if not len(points):return None,None
    delta=points-np.array([pose.x,pose.y]);i=np.argmin(np.linalg.norm(delta,axis=1));dx,dy=delta[i]
    front=math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy
    left=-math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
    return ('front_' if front>=0 else 'rear_')+('left' if left>=0 else 'right'),float(np.linalg.norm(delta[i]))

def turn_away_direction(sector):return -1 if sector and sector.endswith('left') else 1

def corridor_obstacle(pose,points,half_width,length):
    """Closest return inside the forward swept corridor, in the robot frame."""
    if not len(points):return None,None
    delta=np.asarray(points)-np.array([pose.x,pose.y])
    forward=delta[:,0]*math.cos(pose.yaw)+delta[:,1]*math.sin(pose.yaw)
    lateral=-delta[:,0]*math.sin(pose.yaw)+delta[:,1]*math.cos(pose.yaw)
    inside=(forward>=0.)&(forward<=length)&(np.abs(lateral)<=half_width)
    if not np.any(inside):return None,None
    indices=np.flatnonzero(inside);i=indices[np.argmin(forward[inside])]
    return ('front_left' if lateral[i]>=0 else 'front_right'),float(forward[i])

def pose_motion_consistent(previous,current,dt,max_linear,max_angular,position_slack,heading_slack):
    """Check map motion against independent local odometry motion and robot limits."""
    map0,odom0=previous;map1,odom1=current
    if not all(math.isfinite(v) for p in (map0,odom0,map1,odom1) for v in (p.x,p.y,p.yaw)):
        return False,'nonfinite_pose'
    if dt<=0:return False,'nonincreasing_time'
    global_delta=np.array([map1.x-map0.x,map1.y-map0.y])
    local_delta=np.array([odom1.x-odom0.x,odom1.y-odom0.y])
    frame_angle=wrap(map0.yaw-odom0.yaw)
    c,s=math.cos(frame_angle),math.sin(frame_angle)
    aligned=np.array([c*local_delta[0]-s*local_delta[1],s*local_delta[0]+c*local_delta[1]])
    if np.linalg.norm(global_delta)>max_linear*dt+position_slack:return False,'global_pose_jump'
    if np.linalg.norm(local_delta)>max_linear*dt+position_slack:return False,'local_pose_jump'
    if np.linalg.norm(global_delta-aligned)>position_slack:return False,'pose_motion_disagreement'
    global_turn=wrap(map1.yaw-map0.yaw);local_turn=wrap(odom1.yaw-odom0.yaw)
    if abs(global_turn)>max_angular*dt+heading_slack or abs(local_turn)>max_angular*dt+heading_slack:
        return False,'heading_jump'
    if abs(wrap(global_turn-local_turn))>heading_slack:return False,'heading_disagreement'
    return True,'stable'

def safety_radius(sigma,base_radius,quantile):return base_radius+quantile*max(0.,sigma)


def physical_radius(robot,settings):
    """Centre clearance enclosing the body plus the configured outside gap.

    Padding already included in robot['radius'] is not counted twice. Keep the
    legacy physical minimum if a smaller outside gap is configured.
    """
    legacy=robot['radius']+settings['local_safety_margin']
    gap=settings.get('obstacle_body_clearance')
    if gap is None:return legacy
    if not math.isfinite(gap) or gap<0:
        raise ValueError('obstacle_body_clearance must be finite and nonnegative')
    body=(max(math.hypot(*p) for p in robot['footprint']) if robot.get('footprint')
          else robot['radius']-robot.get('footprint_padding',0.))
    return max(legacy,body+gap)


def navigation_radius(sigma,robot,settings):
    return max(physical_radius(robot,settings),
               safety_radius(sigma,robot['radius'],settings['chance_quantile']))

def swept_clear(pose,v,w,points,robot,settings):
    states=rollout(pose,v,w,settings['prediction_dt'],settings['prediction_steps'])
    if not len(points):return True,float('inf')
    nearest=float(np.min(cKDTree(points).query(states[:,:2])[0]))
    # Half a sample's travel bounds the unsampled arc between checks.
    bound=physical_radius(robot,settings)+abs(v)*settings['prediction_dt']/2
    return nearest>bound,nearest

def choose_command(pose,goal,points,previous,robot,settings,trajectory_clear=None):
    dt=1/settings['controller_frequency'];vmax=robot['max_linear_speed'];wmax=robot['max_angular_speed']
    dv=robot['linear_acceleration']*dt;dw=robot['angular_acceleration']*dt
    speeds=np.unique(np.r_[0.,np.linspace(max(0.,previous[0]-dv),min(vmax,max(0.,previous[0])+dv),settings['linear_samples'])])
    initial_heading=abs(wrap(math.atan2(goal[1]-pose.y,goal[0]-pose.x)-pose.yaw))
    if initial_heading>math.radians(settings['turn_in_place_heading_degrees']):speeds=np.array([0.])
    turns=np.unique(np.r_[0.,np.linspace(max(-wmax,previous[1]-dw),min(wmax,previous[1]+dw),settings['angular_samples'])])
    tree=cKDTree(points) if len(points) else None;best=None
    sector,sector_distance=corridor_obstacle(pose,points,physical_radius(robot,settings),
                                            settings['obstacle_avoid_distance'])
    for v in speeds:
        for w in turns:
            states=rollout(pose,v,w,settings['prediction_dt'],settings['prediction_steps'])
            if trajectory_clear is not None and not trajectory_clear(states[:,:2]):continue
            nearest=float(np.min(tree.query(states[:,:2])[0])) if tree else float('inf')
            clearance=physical_radius(robot,settings)+abs(v)*settings['prediction_dt']/2
            if nearest<=clearance:continue
            end=states[-1];remaining=math.dist(end[:2],goal)
            heading=abs(wrap(math.atan2(goal[1]-end[1],goal[0]-end[0])-end[2]))
            barrier=settings['obstacle_weight']*math.exp(-settings['obstacle_decay']*max(0.,nearest-clearance))
            smooth=(v-previous[0])**2+settings['angular_smoothness_weight']*(w-previous[1])**2
            cost=settings['goal_weight']*remaining+settings['heading_weight']*heading+barrier+settings['smoothness_weight']*smooth
            if sector and sector_distance<settings['obstacle_avoid_distance'] and ((sector.endswith('left') and w>0) or (sector.endswith('right') and w<0)):
                cost+=settings['turn_toward_obstacle_penalty']*abs(w)/max(wmax,1e-9)
            if v==0:cost+=settings['standstill_penalty']
            if best is None or cost<best[0]:best=(cost,float(v),float(w))
    return (best[1],best[2],'tracking') if best else (0.,0.,'blocked')

def escape_command(pose,points,sigma,robot,trajectory_clear=None):
    cfg=robot['settings'];v=-robot['reverse_speed'];curve=min(cfg['recovery_curve_speed'],robot['max_angular_speed'])
    for w in (0.,curve,-curve):
        if not swept_clear(pose,v,w,points,robot,cfg)[0]:continue
        if trajectory_clear is not None and not trajectory_clear(rollout(pose,v,w,cfg['prediction_dt'],cfg['prediction_steps'])[:,:2]):continue
        return v,w,'escaping'
    return 0.,0.,'recovery_unavailable'

def rotation_command(pose,points,sigma,direction,robot,trajectory_clear=None):
    speed=min(robot['settings']['recovery_turn_speed'],robot['max_angular_speed'])*direction
    cfg=robot['settings']
    if trajectory_clear is not None and not trajectory_clear(rollout(pose,0.,speed,cfg['prediction_dt'],cfg['prediction_steps'])[:,:2]):return 0.,0.,'recovery_unavailable'
    if swept_clear(pose,0.,speed,points,robot,cfg)[0]:return 0.,speed,'rotating'
    return 0.,0.,'recovery_unavailable'
