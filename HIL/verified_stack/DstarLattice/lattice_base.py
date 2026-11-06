"""Shared rolling-map and path-publication base for curved lattice planners.

This module deliberately contains no search algorithm.  Dijkstra, D*, and
D* Lite provide their own planners while sharing the same ROS path publisher
and rolling occupancy-map crop.
"""
import copy
import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import DurabilityPolicy, QoSProfile

from Dstar.node import Navigator


class LatticeNavigatorBase(Navigator):
    """Common ROS state required by the active curved-lattice controllers."""

    route_topic = "/lattice/path"

    def __init__(self, cfg, robot, output):
        super().__init__(cfg, robot, output)
        self.states = []
        self.turn_heading = None
        self.pending_goal = None
        self.crop_center = None
        self.macro_subgoal = None
        self.macro_goal = None
        self.destroy_publisher(self.path_pub)
        self.path_pub = self.create_publisher(
            PathMsg,
            self.route_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def publish_path(self):
        msg = PathMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def local_map(self, msg, start):
        """Crop the live occupancy grid around the moving robot."""
        half = self.cfg.get("lattice_window_half_m", 10.0)
        resolution = msg.info.resolution
        origin = np.array(
            [msg.info.origin.position.x, msg.info.origin.position.y]
        )
        quaternion = msg.info.origin.orientation
        yaw = math.atan2(
            2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        rotation = np.array(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        )
        if self.crop_center is None or math.dist(start, self.crop_center) > half * 0.5:
            self.crop_center = tuple(start)
        local = rotation.T @ (np.asarray(self.crop_center) - origin)
        center = np.floor(local / resolution).astype(int)
        width, height = msg.info.width, msg.info.height
        radius = max(4, int(math.ceil(half / resolution)))
        x0 = max(0, int(center[0] - radius))
        x1 = min(width, int(center[0] + radius + 1))
        y0 = max(0, int(center[1] - radius))
        y1 = min(height, int(center[1] + radius + 1))
        if x1 <= x0 or y1 <= y0:
            return msg

        cropped = copy.deepcopy(msg)
        data = np.asarray(msg.data, dtype=np.int8).reshape(height, width)
        cropped.data = data[y0:y1, x0:x1].ravel().tolist()
        cropped.info.width = x1 - x0
        cropped.info.height = y1 - y0
        shifted = origin + rotation @ np.array([x0 * resolution, y0 * resolution])
        cropped.info.origin.position.x = float(shifted[0])
        cropped.info.origin.position.y = float(shifted[1])
        return cropped

