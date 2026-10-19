import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from dijkstra_lattice.primitives import compile_library
from tests.fixtures import ROBOT, CFG


class PrimitiveTests(unittest.TestCase):
    def test_library_respects_motion_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            library = compile_library(ROBOT, CFG, Path(directory)/'primitives.json')
        self.assertEqual(len(library['primitives']), 192)
        for primitive in library['primitives']:
            samples = np.asarray(primitive['samples'])
            self.assertAlmostEqual(samples[-1, 0], primitive['dx']*.5, places=6)
            self.assertAlmostEqual(samples[-1, 1], primitive['dy']*.5, places=6)
            if primitive['kind'] == 'curve':
                self.assertLessEqual(np.max(samples[:, 4]), ROBOT['max_linear_speed']+1e-9)
                self.assertLessEqual(np.max(abs(samples[:, 3]*samples[:, 4])),
                                     ROBOT['max_angular_speed']+1e-9)
                self.assertLessEqual(np.max(abs(samples[:, 3]*samples[:, 4]**2)),
                                     CFG['lattice_lateral_acceleration']+1e-9)
