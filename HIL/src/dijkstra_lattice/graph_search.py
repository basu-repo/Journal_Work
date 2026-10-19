"""Reusable weighted-grid graph operations used by the motion-lattice search."""
import heapq
import math
import time

import numpy as np

INF = float('inf')
OFFSETS = ((-1, 0), (1, 0), (0, -1), (0, 1),
           (-1, -1), (-1, 1), (1, -1), (1, 1))


class GraphSearchBase:
    def __init__(self):
        self.grid = None
        self.start = None
        self.goal = None
        self.start_xy = None
        self.goal_xy = None
        self.open = []
        self.edge_cache = {}
        self.expansions = 0
        self.repairs = 0
        self.h = {}
        self.parent = {}
        self.tag = {}
        self.keys = {}
        self.serial = 0

    def cost(self, a, b):
        edge = (a, b)
        if edge not in self.edge_cache:
            self.edge_cache[edge] = self._cost(a, b)
        return self.edge_cache[edge]

    def _cost(self, a, b):
        grid = self.grid
        if not grid.free(a) or not grid.free(b):
            return INF
        dx, dy = b[0] - a[0], b[1] - a[1]
        if abs(dx) > 1 or abs(dy) > 1 or (dx == 0 and dy == 0):
            return INF
        if dx and dy and (not grid.free((a[0] + dx, a[1])) or
                          not grid.free((a[0], a[1] + dy))):
            return INF
        if a == self.start and not grid.line_clear(self.start_xy, grid.xy(b)):
            return INF
        if b == self.goal and not grid.line_clear(grid.xy(a), self.goal_xy):
            return INF
        if min(grid.distance[a[1], a[0]], grid.distance[b[1], b[0]]) <= grid.radius + math.hypot(dx, dy) * grid.res / 2:
            if not grid.line_clear(grid.xy(a), grid.xy(b)):
                return INF
        weight = 1.0 if grid.known[b[1], b[0]] else grid.cfg['unknown_cost']
        weight += grid.cfg['clearance_cost'] / max(grid.res, grid.distance[b[1], b[0]])
        return math.hypot(dx, dy) * weight

    def same_geometry(self, grid):
        old = self.grid
        return (old is not None and old.blocked.shape == grid.blocked.shape
                and math.isclose(old.res, grid.res)
                and np.allclose(old.origin, grid.origin)
                and np.allclose(old.lower, grid.lower)
                and np.allclose(old.rot, grid.rot))

    def adjacent(self, cell):
        x, y = cell
        height, width = self.grid.blocked.shape
        for dx, dy in OFFSETS:
            n = (x + dx, y + dy)
            if 0 <= n[0] < width and 0 <= n[1] < height:
                yield n

    def insert(self, cell, value):
        state = self.tag.get(cell, 'NEW')
        if state == 'NEW':
            key = value
        elif state == 'OPEN':
            key = min(self.keys[cell], value)
        else:
            key = min(self.h[cell], value)
        if not math.isfinite(key):
            return
        self.h[cell] = value
        self.tag[cell] = 'OPEN'
        self.keys[cell] = key
        self.serial += 1
        heapq.heappush(self.open, (key, self.serial, cell))

    def top(self):
        while self.open:
            key, _, cell = self.open[0]
            if self.tag.get(cell) == 'OPEN' and self.keys.get(cell) == key:
                return key
            heapq.heappop(self.open)
        return INF

    def pop(self):
        while self.open:
            key, _, cell = heapq.heappop(self.open)
            if self.tag.get(cell) == 'OPEN' and self.keys.get(cell) == key:
                self.keys.pop(cell, None)
                self.tag[cell] = 'CLOSED'
                return key, cell
        return None

    def reset(self, grid, start, goal, start_xy, goal_xy):
        self.grid = grid
        self.start, self.goal = start, goal
        self.start_xy, self.goal_xy = tuple(start_xy), tuple(goal_xy)
        self.h.clear(); self.parent.clear(); self.tag.clear(); self.keys.clear()
        self.open.clear(); self.edge_cache.clear()
        self.serial = 0
        self.insert(goal, 0.)

    def update(self, grid, start, goal, start_xy=None, goal_xy=None):
        start_xy = grid.xy(start) if start_xy is None else start_xy
        goal_xy = grid.xy(goal) if goal_xy is None else goal_xy
        if (goal != self.goal or not self.same_geometry(grid) or
                self.goal_xy is None or
                math.dist(self.goal_xy, goal_xy) > grid.res / 2):
            self.reset(grid, start, goal, start_xy, goal_xy)
            return 'reset'
        old = self.grid
        old_start = self.start
        old_start_xy = self.start_xy
        old_weight = np.where(old.known, 1., old.cfg['unknown_cost']) + old.cfg['clearance_cost'] / np.maximum(old.res, old.distance)
        new_weight = np.where(grid.known, 1., grid.cfg['unknown_cost']) + grid.cfg['clearance_cost'] / np.maximum(grid.res, grid.distance)
        changed = np.argwhere((old.blocked != grid.blocked) |
                              (np.abs(old_weight - new_weight) > 1e-3))
        if len(changed) > grid.blocked.size * grid.cfg['dstar_reset_fraction']:
            self.reset(grid, start, goal, start_xy, goal_xy)
            return f'reset_broad_change:{len(changed)}'
        self.grid = grid
        self.start = start
        self.start_xy = tuple(start_xy)
        self.goal_xy = tuple(goal_xy)
        self.edge_cache.clear()
        affected = {(int(x), int(y)) for y, x in changed}
        if old_start != start or old_start_xy != self.start_xy:
            affected.update((old_start, start))
        # MODIFY-COST: enter endpoints of changed arcs into OPEN. Closed
        # states retain their old h as the key; processing propagates RAISE
        # and LOWER waves only where the changed arcs affect the tree.
        if affected:
            frontier = set(affected)
            for cell in tuple(affected):
                frontier.update(self.adjacent(cell))
            for cell in frontier:
                if self.tag.get(cell) == 'CLOSED':
                    self.insert(cell, self.h[cell])
            if len(changed):
                self.repairs += 1
        return f'updated:{len(changed)}'

    def process_state(self):
        item = self.pop()
        if item is None:
            return INF
        k_old, x = item
        hx = self.h[x]
        if k_old < hx:
            for y in self.adjacent(x):
                hy = self.h.get(y, INF)
                candidate = hy + self.cost(x, y)
                if hy <= k_old and hx > candidate:
                    self.parent[x] = y
                    self.h[x] = hx = candidate
        if k_old == hx:
            for y in self.adjacent(x):
                candidate = hx + self.cost(y, x)
                if not math.isfinite(candidate):
                    if self.parent.get(y) == x and self.tag.get(y) != 'NEW':
                        self.insert(y, INF)
                    continue
                hy = self.h.get(y, INF)
                if (self.tag.get(y, 'NEW') == 'NEW' or
                    (self.parent.get(y) == x and not math.isclose(hy, candidate)) or
                    (self.parent.get(y) != x and hy > candidate)):
                    self.parent[y] = x
                    self.insert(y, candidate)
        else:
            for y in self.adjacent(x):
                candidate = hx + self.cost(y, x)
                hy = self.h.get(y, INF)
                if self.tag.get(y, 'NEW') == 'NEW' or self.parent.get(y) == x:
                    if self.tag.get(y) != 'NEW' and math.isclose(hy, candidate):
                        continue
                    self.parent[y] = x
                    self.insert(y, candidate)
                elif hy > candidate:
                    self.insert(x, hx)
                elif (hx > hy + self.cost(x, y) and
                      self.tag.get(y) == 'CLOSED' and hy > k_old):
                    self.insert(y, hy)
        return self.top()

    def compute(self, limit, seconds):
        deadline = time.monotonic() + seconds
        expanded = 0
        while self.top() < INF:
            # Incremental search stops once the robot state is closed and no open
            # state can still lower its current route cost.
            if (self.tag.get(self.start) == 'CLOSED' and
                    self.top() >= self.h.get(self.start, INF)):
                break
            if expanded >= limit or time.monotonic() >= deadline:
                self.expansions += expanded
                return False
            self.process_state()
            expanded += 1
        self.expansions += expanded
        return True

    def path(self, start_xy, goal_xy):
        grid = self.grid
        if not grid.free(self.goal):
            return [], 'goal_occupied'
        if not grid.free(self.start):
            return [], 'start_occupied'
        if not math.isfinite(self.h.get(self.start, INF)):
            return [], 'no_route'
        cells = [self.start]
        seen = {self.start}
        while cells[-1] != self.goal:
            current = cells[-1]
            nxt = self.parent.get(current)
            if (nxt is None or nxt in seen or not math.isfinite(self.cost(current, nxt))):
                return [], 'no_route'
            cells.append(nxt)
            seen.add(nxt)
            if len(cells) > grid.blocked.size:
                return [], 'no_route'
        points = [tuple(start_xy)] + [grid.xy(c) for c in cells[1:-1]] + [tuple(goal_xy)]
        if not all(grid.line_clear(a, b) for a, b in zip(points, points[1:])):
            return [], 'edge_blocked'
        compressed = [points[0]]
        i = 0
        while i < len(points) - 1:
            far = i + 1
            for j in range(i + 2, len(points)):
                if grid.line_clear(points[i], points[j]):
                    far = j
                else:
                    break
            compressed.append(points[far])
            i = far
        return compressed, 'dstar_original'
