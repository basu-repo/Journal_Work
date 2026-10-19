"""Shared directional LiDAR and planning-grid clearance for custom navigators."""
import json
import math

import numpy as np
from scipy.spatial import cKDTree
from tf2_ros import TransformException


OFFSETS = {
    'front': 0., 'front_left': math.pi/4, 'left': math.pi/2,
    'rear_left': 3*math.pi/4, 'rear': math.pi,
    'rear_right': -3*math.pi/4, 'right': -math.pi/2,
    'front_right': -math.pi/4,
}


def _map_reach(grid, samples, distances):
    """Find the first blocked ray sample from the grid's cached distance field.

    The sensor grids share the swept-clearance rule in motion_clear().  A
    direction probe previously repeated that KD-tree query for every growing
    prefix of the same ray, which could monopolize the navigator executor.
    Other grid implementations retain the original motion_clear() fallback.
    """
    fields=('origin','rot','lower','res','blocked','distance','radius','hard_radius')
    if not all(hasattr(grid,name) for name in fields):
        for i in range(1,len(samples)):
            if not grid.motion_clear(samples[:i+1]):return float(distances[i-1])
        return float(distances[-1])
    local=(samples-grid.origin)@grid.rot
    cells=np.floor((local-grid.lower)/grid.res).astype(int)
    inside=((cells[:,0]>=0)&(cells[:,1]>=0)&
            (cells[:,0]<grid.blocked.shape[1])&(cells[:,1]<grid.blocked.shape[0]))
    # Grid.distance was already computed from the same map and live obstacle
    # trees when the planning grid was built. Reusing it keeps this diagnostic
    # eight-direction probe bounded even for very large warehouse maps.
    nearest=np.full(len(samples),-float('inf'))
    nearest[inside]=grid.distance[cells[inside,1],cells[inside,0]]
    margin=float(np.linalg.norm(samples[1]-samples[0]))/2
    minimum=np.minimum.accumulate(nearest)
    # Match the inflation-escape exception in Grid.motion_clear(): an
    # inflated start may leave the bubble only while physical clearance and
    # distance from the obstacle both increase.
    escaping=(nearest[0]<=grid.radius and
              np.r_[True,np.logical_and.accumulate(np.diff(nearest)>=-1e-9)].astype(bool))
    clear=(minimum>grid.radius+margin)|(
        escaping&(minimum>grid.hard_radius+margin))
    clear&=np.logical_and.accumulate(inside)
    blocked=np.flatnonzero(~clear[1:])
    return float(distances[int(blocked[0])]) if len(blocked) else float(distances[-1])


def corridors(pose, points, grid, live_radius, horizon, step, heading=None):
    """Measure swept clearance in eight directions from the current pose."""
    if not (0 < step <= horizon and live_radius > 0):
        raise ValueError('Invalid directional probe distance or clearance')
    hits = np.asarray(points, dtype=float).reshape(-1, 2)
    tree = cKDTree(hits) if len(hits) else None
    centre = np.array([pose.x, pose.y], dtype=float)
    base = pose.yaw if heading is None else heading
    distances = np.linspace(0., horizon, max(2, math.ceil(horizon/step)+1))
    margin = float(distances[1]-distances[0])/2
    result = {}
    for name, offset in OFFSETS.items():
        angle = base+offset
        samples = centre+distances[:, None]*[math.cos(angle), math.sin(angle)]
        lidar_reach = horizon
        map_reach = horizon if grid is not None else None
        if tree is not None:
            blocked = np.flatnonzero(tree.query(samples)[0] <= live_radius+margin)
            if len(blocked):
                lidar_reach = float(distances[max(0, int(blocked[0])-1)])
        if grid is not None:
            map_reach=_map_reach(grid,samples,distances)
        known_fraction = None
        if grid is not None and hasattr(grid, 'known'):
            cells = [grid.cell(sample) for sample in samples]
            known_fraction = sum(0 <= y < grid.known.shape[0] and
                                 0 <= x < grid.known.shape[1] and bool(grid.known[y, x])
                                 for x, y in cells)/len(cells)
        reach = min(lidar_reach, map_reach) if map_reach is not None else lidar_reach
        if map_reach is None:
            status = 'map_unknown'
        elif reach >= horizon-1e-9 and known_fraction is not None and known_fraction < 0.6:
            status = 'unobserved'
        elif reach >= horizon-1e-9:
            status = 'clear_to_horizon'
        elif lidar_reach < horizon-1e-9 and (map_reach >= lidar_reach or map_reach is None):
            status = 'lidar_blocked'
        elif map_reach < horizon-1e-9 and lidar_reach >= horizon-1e-9:
            status = 'map_blocked_lidar_clear'
        else:
            status = 'blocked_both'
        result[name] = dict(status=status, clear_m=round(float(reach), 2),
                            lidar_clear_m=round(float(lidar_reach), 2),
                            map_clear_m=round(float(map_reach), 2) if map_reach is not None else None,
                            known_fraction=round(known_fraction, 2) if known_fraction is not None else None)
    return result


def preferred_side(assessment, horizon):
    """Prefer an observed open side only if the forward corridor is blocked."""
    if (assessment is None or assessment['front']['clear_m'] >= horizon-1e-9 or
            assessment['front']['status'] in ('map_unknown', 'unobserved')):
        return None
    def reach(key):
        item = assessment[key]
        return item['clear_m'] if item['status'] not in ('map_unknown', 'unobserved') else 0.
    left = max(reach('front_left'), reach('left'))
    right = max(reach('front_right'), reach('right'))
    if left < horizon/2 and right < horizon/2:
        return None
    if left > right+0.5:
        return 'left'
    if right > left+0.5:
        return 'right'
    return None


class DirectionalCorridors:
    """Mixin for nodes with scan, obstacles(), pose, grid and mission target()."""
    def assess_directions(self, pose, grid, heading=None, points=None):
        scan = self.scan
        if scan is None or not 0 <= self.now()-(scan.header.stamp.sec+
                scan.header.stamp.nanosec*1e-9) < self.cfg['sensor_timeout']:
            return None
        if len(scan.ranges) < 16 or abs(scan.angle_increment)*(len(scan.ranges)-1) < 2*math.pi-0.05:
            return None
        ranges = np.asarray(scan.ranges, dtype=float)
        usable = np.isposinf(ranges) | (np.isfinite(ranges) &
                  (ranges >= scan.range_min) & (ranges <= scan.range_max))
        if np.mean(usable) < 0.95:
            return None
        if points is None:
            try:
                points = self.obstacles()
            except TransformException:
                return None
        return corridors(pose, points, grid, physical_radius_for(self),
                         self.cfg['direction_probe_m'], self.cfg['direction_probe_step_m'], heading)

    def log_directions(self, pose, points):
        now = self.now()
        if (self.grid is None or now-getattr(self, 'last_direction_log', -float('inf')) < 1.
                or self.index >= len(self.cfg['targets'])):
            return
        body = self.assess_directions(pose, self.grid, points=points)
        if body is None:
            return
        goal = self.target()
        bearing = math.atan2(goal[1]-pose.y, goal[0]-pose.x)
        goal_frame = self.assess_directions(pose, self.grid, heading=bearing, points=points)
        self.last_direction_log = now
        with (self.output/'directions.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(sim_time=now, target=self.cfg['targets'][self.index]['name'],
                                         robot_frame=body, goal_frame=goal_frame,
                                         preferred_side=preferred_side(goal_frame, self.cfg['direction_probe_m'])))+'\n')


def physical_radius_for(node):
    robot, settings = node.robot, node.cfg
    legacy = robot['radius'] + settings['local_safety_margin']
    gap = settings.get('obstacle_body_clearance')
    if gap is None:
        return legacy
    if not math.isfinite(gap) or gap < 0:
        raise ValueError('obstacle_body_clearance must be finite and nonnegative')
    body = (max(math.hypot(*p) for p in robot['footprint']) if robot.get('footprint')
            else robot['radius']-robot.get('footprint_padding', 0.))
    return max(legacy, body+gap)
