#!/usr/bin/env python3
"""Fail-closed localization readiness from stamped odometry, map and TF."""
import argparse
import math
import time
from pathlib import Path
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


class Health(Node):
    def __init__(self, robot):
        super().__init__('hil_localization_health')
        self.robot, self.cfg = robot, robot['localization']
        self.odom = self.map = None
        self.odom_received = self.map_received = 0.
        self.stable_since = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, '/localization/ready', qos)
        self.create_subscription(Odometry, robot['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, robot['map_topic'], self.on_map, qos)
        self.pub.publish(Bool(data=False))
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def on_map(self, msg):
        self.map, self.map_received = msg, time.monotonic()

    def tick(self):
        now, wall = self.get_clock().now().nanoseconds * 1e-9, time.monotonic()
        ok = self.odom is not None and self.map is not None
        if ok:
            p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
            stamp = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
            ok = (all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                                *self.odom.pose.covariance)) and
                  .9 < sum(v*v for v in (q.x, q.y, q.z, q.w)) < 1.1 and
                  self.odom.header.frame_id == self.robot['odom_frame'] and
                  self.odom.child_frame_id == self.robot['base_frame'] and
                  self.map.header.frame_id == self.robot['map_frame'] and
                  self.map.info.width > 0 and self.map.info.height > 0 and
                  len(self.map.data) == self.map.info.width * self.map.info.height and
                  -.05 <= now-stamp <= self.cfg['sensor_timeout'] and
                  wall-self.odom_received <= self.cfg['sensor_timeout'] and
                  wall-self.map_received <= self.cfg['map_timeout'])
        if ok:
            try:
                tf = self.tf.lookup_transform(self.robot['map_frame'], self.robot['base_frame'], Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                ok = -.05 <= now-stamp <= self.cfg['tf_timeout']
            except TransformException:
                ok = False
        if not ok:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = wall
        ready = ok and wall-self.stable_since >= self.cfg['stable_seconds']
        self.pub.publish(Bool(data=bool(ready)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=Path, required=True)
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    node = Health(yaml.safe_load(args.robot.read_text()))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
