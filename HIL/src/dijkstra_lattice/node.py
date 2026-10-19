"""Dijkstra search with directional LiDAR clearance and lattice tracking."""
import math
import numpy as np
from scipy.spatial import cKDTree
from .navigator import main
from .core import physical_radius, wrap
from .motion_node import MotionNavigator
from .planner import MotionDijkstra


class DijkstraNavigator(MotionNavigator):
    planner_type = MotionDijkstra
    algorithm = 'dijkstra_state_lattice'
    route_topic = '/dijkstra_lattice/path'

    def immediate_progress_segments(self, source, start, goal):
        """Take the safest available lattice step that approaches the goal.

        Dijkstra remains responsible for detours.  This fast path only avoids
        a full heading-lattice search when an existing motion primitive is
        collision free and its endpoint measurably reduces straight-line
        distance to the active waypoint.
        """
        current = math.dist(start, goal)
        minimum = float(self.cfg.get('lattice_min_goal_progress_m', .2))
        candidates = []
        for primitive in self.planner.outgoing[source[2]]:
            if primitive['kind'] != 'curve':
                continue
            successor = (source[0] + primitive['dx'],
                         source[1] + primitive['dy'], primitive['end_h'])
            edge_cost = self.planner.cost(source, successor)
            if not math.isfinite(edge_cost):
                continue
            samples = self.planner.world_samples(source, primitive, start)
            endpoint = tuple(map(float, samples[-1]))
            remaining = math.dist(endpoint, goal)
            progress = current - remaining
            if progress < minimum:
                continue
            heading_change = abs(float(primitive['samples'][-1][2] -
                                       primitive['samples'][0][2]))
            # Minimum remaining distance is the primary objective.  Prefer a
            # straighter and longer safe step only when progress is tied.
            candidates.append((remaining, heading_change,
                               -float(primitive['length']), successor, primitive,
                               progress))
        if not candidates:
            return None
        _, _, _, successor, primitive, progress = min(candidates,
                                                       key=lambda item:item[:3])
        self.log('direct_progress_primitive_selected',
                 primitive_id=primitive['id'], progress=float(progress),
                 nominal_length=float(primitive['length']))
        return [(source, successor, primitive)]

    def braking_distance(self, remaining, points):
        # A validated curve can hand off to the next curve without braking to
        # a near stop at the arbitrary lattice boundary. A rotation, missing
        # segment, or newly blocked continuation still requires braking.
        if (self.active is None or self.active_samples is None or
                len(self.segments) < 2 or self.grid is None):
            return remaining
        state, _, primitive = self.segments[1]
        if primitive['kind'] != 'curve':
            return remaining
        nominal = (state[2]*2*math.pi/self.cfg['lattice_headings'] +
                   math.atan2(self.grid.rot[1, 0], self.grid.rot[0, 0]))
        end = self.active_samples[-1]
        residual = wrap(end[2]-nominal)
        samples = np.asarray(primitive['samples'], float)
        c, s = math.cos(residual), math.sin(residual)
        rotation = np.array([[c, -s], [s, c]])
        xy = end[:2] + samples[:, :2] @ self.grid.rot.T @ rotation.T
        if not self.grid.motion_clear(xy, self.grid.radius):
            return remaining
        if len(points) and np.min(cKDTree(points).query(xy)[0]) <= physical_radius(self.robot, self.cfg):
            return remaining
        return remaining + min(float(primitive['length']), self.cfg['lattice_handoff_lookahead_m'])

    def _activate(self, pose, goal, now):
        rejected_subgoal = self.macro_subgoal
        super()._activate(pose, goal, now)
        if self.active is None and self.route_reason == 'primitive_start_blocked':
            # A route checked at a quantized lattice heading can fail after
            # anchoring to the measured pose. Do not repeat the same branch.
            if rejected_subgoal is not None:
                self.failed_subgoals.append(rejected_subgoal)
                self.failed_subgoals = self.failed_subgoals[-8:]
            self.macro_subgoal = None

if __name__ == '__main__':
    raise SystemExit(main(DijkstraNavigator))
