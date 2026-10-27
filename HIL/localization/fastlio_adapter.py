#!/usr/bin/env python3
"""Publish a calibrated IMU/body pose as base_link odometry and TF."""
import argparse
import copy
import math
from pathlib import Path
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
try:
    from .geometry import base_pose
except ImportError:
    from geometry import base_pose


class Adapter(Node):
    def __init__(self, robot):
        super().__init__('hil_fastlio_adapter')
        self.robot = robot
        self.cfg = robot['localization']
        self.last_stamp = -1
        self.pub = self.create_publisher(Odometry, robot['odom_topic'], qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.cfg['raw_odom_topic'], self.on_odom,
                                 qos_profile_sensor_data)

    def on_odom(self, msg):
        if (msg.header.frame_id != self.cfg['source_frame'] or
                msg.child_frame_id != self.cfg['source_body_frame']):
            self.get_logger().error('Unexpected FAST-LIO odometry frames; refusing pose')
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.last_stamp:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *msg.pose.covariance]
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[3:7]) < .5:
            return
        xyz, quat, cov = base_pose(values[:3], values[3:7], msg.pose.covariance,
                                   self.cfg['body_to_base_translation'],
                                   self.cfg['body_to_base_quaternion'])
        out = Odometry()
        out.header = copy.deepcopy(msg.header)
        # camera_init and odom_lidar denote the same initial local world here;
        # this is a frame alias, with no live ground-truth or GNSS alignment.
        out.header.frame_id = self.robot['odom_frame']
        out.child_frame_id = self.robot['base_frame']
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, xyz)
        q = out.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, quat)
        out.pose.covariance = cov
        # FAST-LIO does not populate a usable twist in this version. Mark it
        # unknown rather than implying that zero is a measured velocity.
        out.twist.covariance = [1e6 if i % 7 == 0 else 0. for i in range(36)]
        self.pub.publish(out)
        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = out.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        tf.transform.rotation = q
        self.tf.sendTransform(tf)
        self.last_stamp = stamp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=Path, required=True)
    args, ros = parser.parse_known_args()
    robot = yaml.safe_load(args.robot.read_text())
    if not robot['localization'].get('calibration_confirmed'):
        parser.error('Measure extrinsics and confirm calibration in the robot profile first')
    rclpy.init(args=ros)
    node = Adapter(robot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
