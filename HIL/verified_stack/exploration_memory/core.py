"""Read-only occupancy analysis. No ROS, simulator, world geometry or commands.

Frontiers are free/unknown boundaries. Their observation positions must lie
in robot-clearance-safe KNOWN FREE space connected to the current pose.
Junctions are provisional angular-opening observations, not confirmed graph
vertices or dead ends. Neither result can authorize a lattice primitive.
"""
from dataclasses import dataclass
from collections import deque
from enum import Enum
import hashlib
import math
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


class MemoryState(str, Enum):
    UNVISITED = 'UNVISITED'
    VISITED = 'VISITED'
    ACTIVE = 'ACTIVE'
    EXPLORED = 'EXPLORED'
    TEMP_BLOCKED = 'TEMP_BLOCKED'
    DEAD_END = 'DEAD_END'
    EXHAUSTED = 'EXHAUSTED'


@dataclass
class MapGrid:
    data: np.ndarray
    resolution: float
    origin: tuple  # x, y, yaw; always in the published map frame

    def __post_init__(self):
        raw = np.asarray(self.data)
        if raw.ndim != 2 or not raw.size:
            raise ValueError('Expected a nonempty 2D occupancy grid')
        if not np.issubdtype(raw.dtype, np.number) or not np.all(np.isfinite(raw)):
            raise ValueError('Occupancy values must be finite numbers')
        if np.any((raw < -1) | (raw > 100)) or np.any(raw != np.rint(raw)):
            raise ValueError('Occupancy values must be -1..100')
        self.data = np.array(raw, dtype=np.int16, copy=True)
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Invalid map resolution')
        if len(self.origin) != 3 or not all(math.isfinite(x) for x in self.origin):
            raise ValueError('Invalid map origin')
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        self.rotation = np.array([[c, -s], [s, c]])
        self.data.flags.writeable = False

    def world(self, cells):
        return (np.asarray(cells) + .5) * self.resolution @ self.rotation.T + self.origin[:2]

    def cell(self, xy):
        return tuple(np.floor((np.asarray(xy)-self.origin[:2]) @ self.rotation /
                              self.resolution).astype(int))

    def inside(self, cell):
        x, y = cell
        return bool(0 <= y < self.data.shape[0] and 0 <= x < self.data.shape[1])

    def revision(self):
        h = hashlib.sha256(self.data.tobytes())
        h.update(repr((self.resolution, self.origin, self.data.shape)).encode())
        return h.hexdigest()[:20]

    def line_in_mask(self, a, b, mask):
        n = max(2, int(math.ceil(np.linalg.norm(np.asarray(a)-b)/self.resolution*2))+1)
        cells = [self.cell(p) for p in np.linspace(a, b, n)]
        return all(self.inside(c) and mask[c[1], c[0]] for c in cells)


def safe_space(grid, clearance, free_threshold):
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('Clearance must be positive and derived from the robot')
    if not 0 <= free_threshold < 65:
        raise ValueError('Free threshold must not classify occupied cells as free')
    free = (grid.data >= 0) & (grid.data <= free_threshold)
    # Unknown/uncertain cells AND map exterior are excluded. Subtract a
    # half-diagonal so cell-centre distances do not overstate clearance.
    distance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
    distance = distance * grid.resolution - grid.resolution * math.sqrt(2)/2
    return free, free & (distance > clearance), distance


def connected_distances(safe, source, resolution):
    distances = np.full(safe.shape, np.inf)
    x, y = source
    if not (0 <= y < safe.shape[0] and 0 <= x < safe.shape[1] and safe[y, x]):
        return distances
    distances[y, x] = 0.
    queue = deque([(x, y)])
    while queue:
        x, y = queue.popleft()
        for nx, ny in ((x-1, y), (x+1, y), (x, y-1), (x, y+1)):
            if (0 <= ny < safe.shape[0] and 0 <= nx < safe.shape[1]
                    and safe[ny, nx] and not math.isfinite(distances[ny, nx])):
                distances[ny, nx] = distances[y, x] + resolution
                queue.append((nx, ny))
    return distances


def angular_openings(grid, safe, xy, cfg):
    step = float(cfg['junction_angle_step_degrees'])
    if not 0 < step <= 30:
        raise ValueError('Invalid angular observation step')
    n = int(math.ceil(360/step))
    angles = np.arange(n) * 2*math.pi/n
    probe = float(cfg['junction_probe_m'])
    if probe <= 0:
        raise ValueError('Invalid probe length')
    radii = np.linspace(0, probe, max(2, int(math.ceil(probe/grid.resolution*2))+1))
    opened = []
    for angle in angles:
        points = np.asarray(xy) + radii[:, None] * [math.cos(angle), math.sin(angle)]
        cells = [grid.cell(p) for p in points]
        opened.append(all(grid.inside(c) and safe[c[1], c[0]] for c in cells))
    if all(opened):
        return {'candidate': False, 'reason': 'open_area', 'openings': []}
    # Rotate the circular signal after a closed ray to merge sectors at 0/360.
    start = (opened.index(False)+1) % n
    sectors = []; current = []
    for i in range(n):
        j = (start+i) % n
        if opened[j]:
            current.append(j)
        elif current:
            sectors.append(current);current = []
    if current:sectors.append(current)
    result = []
    for sector in sectors:
        width = len(sector)*360/n
        if width < cfg['junction_min_sector_degrees']:continue
        mean = math.atan2(np.sin(angles[sector]).mean(), np.cos(angles[sector]).mean())
        result.append({'bearing_rad': mean, 'width_degrees': width})
    return {'candidate': len(result) >= 3,
            'reason': 'multiple_observed_openings' if len(result) >= 3 else 'insufficient_openings',
            'openings': result}


def observe(grid, xy, clearance, cfg, goals=()):
    if grid.data.size > cfg['max_cells']:
        raise ValueError('Map exceeds observer cell budget')
    original = grid.data.tobytes()
    free, safe, distance = safe_space(grid, clearance, cfg['free_threshold'])
    distances = connected_distances(safe, grid.cell(xy), grid.resolution)
    reachable = np.isfinite(distances)
    unknown = grid.data == -1
    adjacent_unknown = ndimage.binary_dilation(unknown, structure=ndimage.generate_binary_structure(2, 1))
    frontiers = free & adjacent_unknown
    labels, count = ndimage.label(frontiers, structure=np.ones((3, 3)))
    ry, rx = np.nonzero(reachable)
    candidates = []; rejected = {'small_cluster': 0, 'no_safe_reachable_viewpoint': 0}
    eligible = []
    for label, region in enumerate(ndimage.find_objects(labels), 1):
        if region is None:continue
        yy, xx = np.nonzero(labels[region] == label)
        yy += region[0].start;xx += region[1].start
        if len(xx) < cfg['min_frontier_cells']:
            rejected['small_cluster'] += 1;continue
        eligible.append((len(xx), label, yy, xx))
    # Phase A caps work by cluster size, not a navigation target utility.
    for size, label, yy, xx in sorted(eligible, key=lambda item: (-item[0], item[1]))[:cfg['max_frontiers']]:
        if not len(rx):
            rejected['no_safe_reachable_viewpoint'] += 1;continue
        separation, nearest = cKDTree(np.column_stack((xx, yy))).query(np.column_stack((rx, ry)))
        separation *= grid.resolution
        choices = np.flatnonzero(separation <= cfg['frontier_view_distance_m'])
        if not len(choices):
            rejected['no_safe_reachable_viewpoint'] += 1;continue
        # A diagnostic representative only: nearest reachable setback to the
        # cluster centroid. Never dispatched as a navigation target in A.
        center = np.array([xx.mean(), yy.mean()])
        ordered = sorted(choices, key=lambda i: (np.linalg.norm([rx[i]-center[0], ry[i]-center[1]]), distances[ry[i], rx[i]]))
        k = next((i for i in ordered if grid.line_in_mask(
            grid.world([rx[i], ry[i]]), grid.world([xx[nearest[i]], yy[nearest[i]]]), free)), None)
        if k is None:
            rejected['no_safe_reachable_viewpoint'] += 1;continue
        viewpoint = grid.world([rx[k], ry[k]]).tolist()
        boundary = grid.world(np.column_stack((xx, yy)))
        candidates.append({'cluster_label': label, 'cell_count': size,
                           'centroid': boundary.mean(axis=0).tolist(),
                           'boundary_sample': boundary[::max(1, len(boundary)//150)].tolist(),
                           'viewpoint': viewpoint, 'known_free_path_length_m': float(distances[ry[k], rx[k]]),
                           'conservative_clearance_m': float(distance[ry[k], rx[k]]),
                           'selected': False})
    goal_connectivity = []
    for goal in goals:
        cell = grid.cell((goal['x'], goal['y']))
        connected = grid.inside(cell) and bool(reachable[cell[1], cell[0]])
        goal_connectivity.append({'name': goal['name'], 'xy': [goal['x'], goal['y']],
                                  'known_free_grid_connected': connected,
                                  'note': 'Connectivity observation, NOT lattice feasibility or arrival'})
    assert original == grid.data.tobytes()
    return {'phase': 'A_OBSERVATION_ONLY', 'map_revision': grid.revision(),
            'robot_xy': list(map(float, xy)), 'clearance_m': clearance,
            'published_map_known_fraction': float(np.mean(grid.data >= 0)),
            'known_free_reachable_cells': int(reachable.sum()),
            'source_safe': bool(reachable.any()), 'frontier_clusters': count,
            'frontiers': candidates, 'rejected': rejected,
            'clusters_deferred_by_budget': max(0, len(eligible)-cfg['max_frontiers']),
            'junction_observation': angular_openings(grid, safe, xy, cfg),
            'mission_goals': goal_connectivity,
            'selected_target': None, 'dead_end_classifications': [],
            'commands_published': 0}
