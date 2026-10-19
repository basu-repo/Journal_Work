"""Forward Dijkstra search over the directed motion-primitive graph.

Search starts at the robot's single measured heading and stops at the first
settled goal-cell heading. It uses no heuristic or incremental route repair.
"""
import heapq
import math
import time

from .motion_graph import GOAL, MotionGraph


class MotionDijkstra(MotionGraph):
    algorithm = 'dijkstra_state_lattice'

    def __init__(self):
        super().__init__()
        self.settled = set()
        self.reached_goal = None

    def update(self, grid, start, goal, start_xy, goal_xy):
        if self.library is None:
            raise ValueError('Motion library has not been loaded')
        self.grid = grid
        self.start = start
        self.goal = goal
        self.goal_cell = goal
        self.start_xy = tuple(start_xy)
        self.goal_xy = tuple(goal_xy)
        self.open = [(0.0, 0, start)]
        self.h = {start: 0.0}
        self.parent = {}
        self.settled.clear()
        self.edge_cache.clear()
        self.serial = 0
        self.reached_goal = None
        return 'reset'

    def compute(self, limit, seconds):
        deadline = time.monotonic() + seconds
        expanded = 0
        while self.open and self.reached_goal is None:
            if expanded >= limit or time.monotonic() >= deadline:
                self.expansions += expanded
                return False
            distance, _, state = heapq.heappop(self.open)
            if state in self.settled or distance != self.h.get(state, math.inf):
                continue
            self.settled.add(state)
            expanded += 1
            # A motion lattice can only finish on one of its discrete primitive
            # endpoints.  A geometrically free goal cell is therefore not
            # necessarily an exact lattice endpoint.  Treat the local target as
            # a small goal region so Dijkstra can settle the closest safe,
            # kinematically reachable state instead of exhausting the search in
            # otherwise open space.
            tolerance = float(self.grid.cfg.get('lattice_goal_tolerance_m', 0.0))
            dx = (state[0] - self.goal_cell[0]) * self.grid.res
            dy = (state[1] - self.goal_cell[1]) * self.grid.res
            if math.hypot(dx, dy) <= tolerance + 1e-9:
                self.reached_goal = state
                break
            for successor in self.successors(state):
                if successor == GOAL or successor in self.settled or not self.grid.free(successor[:2]):
                    continue
                edge = self.cost(state, successor)
                candidate = distance + edge
                if candidate < self.h.get(successor, math.inf):
                    self.h[successor] = candidate
                    self.parent[successor] = state
                    self.serial += 1
                    heapq.heappush(self.open, (candidate, self.serial, successor))
        self.expansions += expanded
        return True

    def path(self):
        if not self.grid.free(self.goal_cell):
            return [], 'goal_occupied'
        if not self.grid.free(self.start[:2]):
            return [], 'start_occupied'
        if self.reached_goal is None:
            return [], 'no_route'
        state = self.reached_goal
        seen = {state}
        states = [state]
        for _ in range(self.grid.blocked.size * self.library['headings']):
            if state == self.start:
                states.reverse()
                segments = []
                for a, b in zip(states, states[1:]):
                    primitive = self.primitive(a, b)
                    if primitive is None or not math.isfinite(self.cost(a, b)):
                        return [], 'invalid_transition'
                    segments.append((a, b, primitive))
                return segments, self.algorithm
            previous = self.parent.get(state)
            if previous is None or previous in seen:
                return [], 'no_route'
            states.append(previous)
            state = previous
            seen.add(state)
        return [], 'route_cycle'
