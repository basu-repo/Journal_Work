"""Draw live navigation diagnostics in Gazebo without changing the saved world.

Blue: current planned route. Red: obstacle points from the 3D LiDAR.
Magenta: estimated pose. The rendered robot remains Gazebo ground truth.
"""

import argparse
import math

import rclpy
import yaml
from nav_msgs.msg import Path
from rclpy.node import Node as RosNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.transport13 import Node as GazeboNode


def rotate_xy(x, y, yaw, origin):
    return origin[0] + math.cos(yaw) * x - math.sin(yaw) * y, origin[1] + math.sin(yaw) * x + math.cos(yaw) * y


def rotate_xyz(xyz, q):
    # Rotate a 3D point by the TF quaternion; LiDAR points need pitch and roll.
    x, y, z = xyz
    tx = 2 * (q.y * z - q.z * y)
    ty = 2 * (q.z * x - q.x * z)
    tz = 2 * (q.x * y - q.y * x)
    return (x + q.w * tx + q.y * tz - q.z * ty,
            y + q.w * ty + q.z * tx - q.x * tz,
            z + q.w * tz + q.x * ty - q.y * tx)


class Overlays(RosNode):
    def __init__(self, config, spawn):
        super().__init__('gazebo_navigation_overlays')
        self.spawn = spawn
        self.gazebo = GazeboNode()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.path = None
        self.cloud = None
        self.cloud_received_at = None
        self.base_frame = config['base_frame']
        self.route_topic = next(iter(config['path_topics'].values()))
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, self.route_topic, self.on_path, path_qos)
        self.create_subscription(PointCloud2, '/lidar/obstacles', self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.5, self.publish)
        self.last_warning = 0.0

    def on_path(self, msg):
        self.path = msg

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_received_at = self.get_clock().now()

    def send(self, marker):
        ok, _ = self.gazebo.request('/marker', marker, Marker, Boolean, 300)
        if not ok:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_warning > 10:
                self.get_logger().warning('Gazebo /marker service unavailable; check MarkerManager GUI plugin')
                self.last_warning = now

    def marker(self, number, kind, rgb, scale):
        msg = Marker()
        msg.ns = 'navigation_overlay'
        msg.id = number
        msg.action = Marker.ADD_MODIFY
        msg.type = kind
        msg.visibility = Marker.GUI
        msg.scale.x = scale
        msg.scale.y = scale
        msg.scale.z = scale
        msg.material.diffuse.r, msg.material.diffuse.g, msg.material.diffuse.b = rgb
        msg.material.diffuse.a = 1.0
        return msg

    def world_point(self, point, frame='map'):
        if frame != 'map':
            transform = self.tf.lookup_transform('map', frame, rclpy.time.Time()).transform
            x, y, z = rotate_xyz(point, transform.rotation)
            x += transform.translation.x
            y += transform.translation.y
            z += transform.translation.z
        else:
            x, y, z = point
        wx, wy = rotate_xy(x, y, self.spawn[5], self.spawn)
        return wx, wy, z + self.spawn[2]

    def add_point(self, marker, xyz):
        point = marker.point.add()
        point.x, point.y, point.z = xyz

    def publish(self):
        if self.path is not None:
            route = self.marker(1, Marker.LINE_STRIP, (0.05, 0.35, 1.0), 0.12)
            for pose in self.path.poses:
                try:
                    x, y, z = self.world_point((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                                               self.path.header.frame_id or 'map')
                    self.add_point(route, (x, y, max(z, self.spawn[2] + 0.3)))
                except Exception:
                    break
            if route.point:
                self.send(route)
            else:
                route.action = Marker.DELETE_MARKER
                self.send(route)
        try:
            estimated = self.world_point((0, 0, 0.15), self.base_frame)
            pose = self.marker(2, Marker.SPHERE, (1.0, 0.05, 0.75), 0.35)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = estimated
            self.send(pose)
        except Exception:
            pass
        if self.cloud is not None and self.cloud_received_at is not None:
            age = (self.get_clock().now() - self.cloud_received_at).nanoseconds / 1e9
            if age > 2.0:
                stale = self.marker(3, Marker.POINTS, (1.0, 0.12, 0.05), 0.12)
                stale.action = Marker.DELETE_MARKER
                self.send(stale)
                return
            obstacles = self.marker(3, Marker.POINTS, (1.0, 0.12, 0.05), 0.12)
            try:
                for index, p in enumerate(point_cloud2.read_points(self.cloud, field_names=('x', 'y', 'z'), skip_nans=True)):
                    if index % 20:
                        continue
                    xyz = self.world_point((float(p[0]), float(p[1]), float(p[2])), self.cloud.header.frame_id)
                    self.add_point(obstacles, xyz)
                    if len(obstacles.point) >= 300:
                        break
                if obstacles.point:
                    self.send(obstacles)
                else:
                    obstacles.action = Marker.DELETE_MARKER
                    self.send(obstacles)
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--spawn', nargs=6, type=float, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    with open(args.config) as stream:
        config = yaml.safe_load(stream)
    node = Overlays(config, args.spawn)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
