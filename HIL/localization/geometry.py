"""Rigid body pose conversion independent of ROS; quaternions are x,y,z,w."""
import numpy as np
from scipy.spatial.transform import Rotation


def base_pose(position, quaternion, covariance, translation, rotation):
    body = Rotation.from_quat(quaternion)
    lever = body.apply(translation)
    xyz = np.asarray(position) + lever
    quat = (body * Rotation.from_quat(rotation)).as_quat()
    # ROS pose orientation covariance is expressed about fixed/world axes.
    x, y, z = lever
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    jacobian = np.eye(6)
    jacobian[:3, 3:] = -skew
    cov = jacobian @ np.asarray(covariance).reshape(6, 6) @ jacobian.T
    return xyz, quat, cov.ravel().tolist()
