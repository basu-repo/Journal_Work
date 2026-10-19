"""Directional clearance must separate lateral hits from a blocked route."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

from dijkstra_lattice.directional_corridors import corridors, preferred_side, physical_radius_for, _map_reach
from dijkstra_lattice.core import physical_radius


POSE = SimpleNamespace(x=0., y=0., yaw=0.)


class OpenGrid:
    def __init__(self, observed=True):
        self.known = np.full((41, 41), observed, dtype=bool)

    def cell(self, point):
        return int(round(point[0]*4))+20, int(round(point[1]*4))+20

    def motion_clear(self, points):
        return True


class DirectionTests(unittest.TestCase):
    def test_fast_grid_probe_matches_swept_prefix_checks(self):
        class DistanceGrid:
            def __init__(self, obstacle):
                self.origin=np.zeros(2);self.rot=np.eye(2)
                self.lower=np.array([-5.,-5.]);self.res=.1
                self.blocked=np.zeros((100,100),bool)
                self.radius=.8;self.hard_radius=.5
                self.obstacle=np.asarray(obstacle)
                self.queries=0
                yy,xx=np.indices(self.blocked.shape)
                centres=self.lower+(np.stack((xx,yy),axis=-1)+.5)*self.res
                self.distance=np.linalg.norm(centres-self.obstacle,axis=2)
            def nearest_distance(self,local):
                self.queries+=1
                return np.linalg.norm(local-self.obstacle,axis=1)
            def motion_clear(self,points):
                local=(points-self.origin)@self.rot
                cells=np.floor((local-self.lower)/self.res).astype(int)
                if ((cells<0).any() or (cells[:,0]>=100).any() or
                        (cells[:,1]>=100).any()):return False
                values=self.distance[cells[:,1],cells[:,0]]
                margin=np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))/2
                if np.min(values)>self.radius+margin:return True
                return (values[0]<=self.radius and
                        np.min(values)>self.hard_radius+margin and
                        np.all(np.diff(values)>=-1e-9))
        lengths=np.linspace(0.,3.,13)
        for obstacle in ((1.4,0.),(0.,0.),(-.3,0.),(4.,0.)):
            for angle in (0.,math.pi/2,math.pi):
                grid=DistanceGrid(obstacle)
                samples=lengths[:,None]*[math.cos(angle),math.sin(angle)]
                expected=3.
                for i in range(1,len(samples)):
                    if not grid.motion_clear(samples[:i+1]):
                        expected=float(lengths[i-1]);break
                grid.queries=0
                self.assertAlmostEqual(_map_reach(grid,samples,lengths),expected)
                self.assertEqual(grid.queries,0)

    def test_shared_probe_uses_controller_body_clearance(self):
        robot = dict(radius=.60, footprint=[(.55, .20)], footprint_padding=.05)
        for settings in ({'local_safety_margin': .08},
                         {'local_safety_margin': .08, 'obstacle_body_clearance': .5}):
            node = SimpleNamespace(robot=robot, cfg=settings)
            self.assertEqual(physical_radius_for(node), physical_radius(robot, settings))

    def test_blocked_front_and_right_selects_left(self):
        hits = np.array([[1.3, 0.], [0., -1.1], [1.3, -1.3]])
        assessment = corridors(POSE, hits, OpenGrid(), .6, 3., .25)
        self.assertLess(assessment['front']['clear_m'], 3.)
        self.assertEqual(assessment['left']['status'], 'clear_to_horizon')
        self.assertEqual(preferred_side(assessment, 3.), 'left')

    def test_side_hit_does_not_block_forward_corridor(self):
        assessment = corridors(POSE, np.array([[0., 1.2]]), OpenGrid(), .6, 3., .25)
        self.assertEqual(assessment['front']['status'], 'clear_to_horizon')
        self.assertLess(assessment['left']['clear_m'], 3.)
        self.assertIsNone(preferred_side(assessment, 3.))

    def test_unobserved_direction_is_not_declared_free(self):
        assessment = corridors(POSE, np.empty((0, 2)), OpenGrid(False), .6, 3., .25)
        self.assertEqual(assessment['front']['status'], 'unobserved')
        self.assertIsNone(preferred_side(assessment, 3.))


if __name__ == '__main__':
    unittest.main()
