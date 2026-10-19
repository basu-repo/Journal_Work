#!/usr/bin/env python3
"""Fail-closed bridge from planner velocity to the physical base command."""
import argparse
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Watchdog(Node):
    def __init__(self, cfg, armed):
        super().__init__('hil_command_watchdog')
        self.cfg = cfg
        self.armed = armed
        self.enabled = False
        self.command = Twist()
        self.last_command = self.last_scan = self.last_odom = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/hil/navigation_enabled', self.on_enable, qos)
        self.create_subscription(Twist, cfg['planner_command_topic'], self.on_command, 10)
        self.create_subscription(LaserScan, cfg['scan_topic'], self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, cfg['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, cfg['hardware_command_topic'], 10)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_enable(self, msg): self.enabled = bool(msg.data)
    def on_command(self, msg): self.command, self.last_command = msg, time.monotonic()
    def on_scan(self, msg):
        if msg.ranges: self.last_scan = time.monotonic()
    def on_odom(self, _msg): self.last_odom = time.monotonic()

    def stop(self): self.publisher.publish(Twist())

    def tick(self):
        now = time.monotonic()
        out = Twist()
        finite = all(math.isfinite(v) for v in
                     (self.command.linear.x, self.command.angular.z))
        fresh = (now - self.last_command < self.cfg['command_timeout'] and
                 now - self.last_scan < 1.0 and now - self.last_odom < 1.0)
        if self.armed and self.enabled and finite and fresh:
            out.linear.x = max(-self.cfg['reverse_speed'],
                               min(self.cfg['max_linear_speed'], self.command.linear.x))
            out.angular.z = max(-self.cfg['max_angular_speed'],
                                min(self.cfg['max_angular_speed'], self.command.angular.z))
        self.publisher.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--arm', action='store_true')
    args, ros = parser.parse_known_args()
    cfg = yaml.safe_load(args.config.read_text())
    rclpy.init(args=ros)
    node = Watchdog(cfg, args.arm)
    print('HIL watchdog:', 'ARMED' if args.arm else 'DISARMED (zero velocity only)', flush=True)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__': main()
