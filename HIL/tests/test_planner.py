"""Check Dijkstra lattice paths against an independent forward cost search."""
import heapq
import math
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import numpy as np

from dijkstra_lattice.primitives import compile_library
from tests.fixtures import ROBOT, CFG, Grid
from dijkstra_lattice.planner import GOAL, MotionDijkstra
from dijkstra_lattice.node import DijkstraNavigator


def forward_shortest_cost(planner):
    queue = [(0., planner.start)]
    distance = {planner.start: 0.}
    while queue:
        cost, state = heapq.heappop(queue)
        if cost > distance[state]:
            continue
        if state == GOAL:
            return cost
        for nxt in planner.successors(state):
            candidate = cost + planner.cost(state, nxt)
            if candidate < distance.get(nxt, math.inf):
                distance[nxt] = candidate
                heapq.heappush(queue, (candidate, nxt))
    return math.inf


class DijkstraLatticeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.library = compile_library(ROBOT, CFG, Path(cls.tmp.name)/'primitives.json')

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_shortest_paths_with_new_obstacle(self):
        self.assertEqual(len(self.library['primitives']), 192)
        planner = MotionDijkstra().configure(self.library)
        cells = np.zeros((13, 13), bool)
        start, goal = (2, 6, 0), (10, 6)
        for blocked in (False, True):
            if blocked:
                cells[6, 6] = True
            grid = Grid(cells.copy())
            self.assertEqual(planner.update(grid, start, goal, grid.xy(start[:2]), grid.xy(goal)), 'reset')
            for _ in range(300):
                if planner.compute(3000, .25):
                    break
            else:
                self.fail('Dijkstra search remained pending')
            expected = forward_shortest_cost(planner)
            self.assertIsNotNone(planner.reached_goal)
            self.assertTrue(math.isclose(planner.h[planner.reached_goal], expected, abs_tol=1e-6))
            segments, reason = planner.path()
            self.assertEqual(reason, 'dijkstra_state_lattice')
            self.assertTrue(segments)
            self.assertEqual(planner.repairs, 0)

    def test_curve_handoff_requires_clear_next_sweep(self):
        navigator = DijkstraNavigator.__new__(DijkstraNavigator)
        navigator.cfg = dict(lattice_headings=16, lattice_handoff_lookahead_m=1.)
        navigator.robot = dict(width=.7, length=1.)
        navigator.grid = Grid(np.zeros((12, 12), bool))
        navigator.grid.radius = .5
        navigator.active = ((2, 2, 0), dict(kind='curve'))
        navigator.active_samples = np.array([[1., 1., 0.], [2., 1., 0.]])
        next_curve = dict(kind='curve', length=1., samples=[[0., 0.], [.5, 0.], [1., 0.]])
        navigator.segments = [(None, None, navigator.active[1]), ((4, 2, 0), None, next_curve)]
        with mock.patch('dijkstra_lattice.node.physical_radius', return_value=.5):
            self.assertAlmostEqual(navigator.braking_distance(.25, np.empty((0, 2))), 1.25)
            self.assertAlmostEqual(navigator.braking_distance(.25, np.array([[2.5, 1.]])), .25)
        navigator.segments[1] = ((4, 2, 0), None, dict(kind='rotate'))
        self.assertAlmostEqual(navigator.braking_distance(.25, np.empty((0, 2))), .25)

    def test_open_space_selects_immediate_progress_primitive(self):
        navigator = DijkstraNavigator.__new__(DijkstraNavigator)
        navigator.cfg = dict(lattice_min_goal_progress_m=.2)
        navigator.planner = MotionDijkstra().configure(self.library)
        navigator.log = mock.Mock()
        grid = Grid(np.zeros((20, 20), bool))
        source = (2, 10, 0)
        start = grid.xy(source[:2])
        goal = (9., start[1])
        navigator.planner.update(grid, source, grid.cell(goal) if hasattr(grid, 'cell') else (18, 10),
                                 start, goal)
        segments = navigator.immediate_progress_segments(source, start, goal)
        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0][0], source)
        self.assertEqual(segments[0][2]['kind'], 'curve')
        endpoint = navigator.planner.world_samples(source, segments[0][2], start)[-1]
        self.assertLess(math.dist(endpoint, goal), math.dist(start, goal))
        navigator.log.assert_called_once()


if __name__ == '__main__':
    unittest.main()
