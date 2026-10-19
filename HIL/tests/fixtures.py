import math
import numpy as np

ROBOT = dict(max_linear_speed=.8, max_angular_speed=.6,
             linear_acceleration=.5, angular_acceleration=.8)
CFG = dict(planning_resolution=.5, lattice_headings=16,
           lattice_cruise_speed=.7, lattice_min_curve_speed=.35,
           lattice_lateral_acceleration=.6, lattice_max_curvature=1.5,
           lattice_sample_spacing=.08,
           lattice_primitive_lengths=[1., 1.5, 2., 2.5, 3.])


class Grid:
    def __init__(self, blocked):
        self.blocked = np.asarray(blocked, bool)
        self.known = np.ones_like(self.blocked, bool)
        self.distance = np.full(self.blocked.shape, math.inf)
        self.origin = np.zeros(2)
        self.lower = np.zeros(2)
        self.rot = np.eye(2)
        self.res = .5
        self.radius = self.hard_radius = 0.
        self.cfg = dict(unknown_cost=2., clearance_cost=0.,
                        dstar_reset_fraction=1., lattice_comfort_distance_m=0.,
                        lattice_comfort_weight=0., lattice_turn_cost=0.)

    def free(self, cell):
        x, y = cell
        return (0 <= x < self.blocked.shape[1] and
                0 <= y < self.blocked.shape[0] and not self.blocked[y, x])

    def xy(self, cell):
        return (cell[0]+.5)*self.res, (cell[1]+.5)*self.res

    def motion_clear(self, points, radius=None):
        return all(self.free((int(x/self.res), int(y/self.res))) for x, y in points)
