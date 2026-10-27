import unittest
import numpy as np
from localization.geometry import base_pose


class AdapterGeometryTests(unittest.TestCase):
    def test_body_offset_rotates_with_heading(self):
        xyz, q, cov = base_pose([1,2,0], [0,0,2**-.5,2**-.5], np.eye(6).ravel(),
                                [1,0,0], [0,0,0,1])
        np.testing.assert_allclose(xyz, [1,3,0], atol=1e-12)
        self.assertAlmostEqual(sum(q*q), 1.)
        self.assertGreater(np.asarray(cov).reshape(6,6)[0,0], 1.)

    def test_identity_calibration_preserves_pose_covariance(self):
        xyz, q, cov = base_pose([1,2,3], [0,0,0,1], np.eye(6).ravel(),
                                [0,0,0], [0,0,0,1])
        np.testing.assert_allclose(xyz, [1,2,3])
        np.testing.assert_allclose(q, [0,0,0,1])
        np.testing.assert_allclose(cov, np.eye(6).ravel())
