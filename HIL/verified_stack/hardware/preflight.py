#!/usr/bin/env python3
"""Read-only receipt, timestamp, frame and TF checks; publishes no commands."""
import argparse
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener, TransformException
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=Path, required=True)
    parser.add_argument('--sensors-only', action='store_true')
    parser.add_argument('--seconds', type=float, default=8.)
    args = parser.parse_args()
    robot = yaml.safe_load(args.robot.read_text())
    cfg = robot['localization']
    rclpy.init()
    node = Node('hil_preflight', parameter_overrides=[
        Parameter('use_sim_time', value=robot.get('use_sim_time', False))])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples, observed = {}, {}
    required = {cfg['pointcloud_topic']: PointCloud2, cfg['imu_topic']: Imu}
    if not args.sensors_only:
        required.update({robot['scan_topic']: LaserScan, robot['map_topic']: OccupancyGrid,
                         robot['odom_topic']: Odometry})
    def receive(topic, msg):
        samples.setdefault(topic, []).append(time.monotonic())
        observed[topic] = msg
    qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, typ in required.items():
        node.create_subscription(typ, topic, lambda msg, t=topic: receive(t, msg),
                                 qos_map if typ is OccupancyGrid else qos_profile_sensor_data)
    begin, failed = time.monotonic(), False
    try:
        while time.monotonic()-begin < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        now = node.get_clock().now().nanoseconds * 1e-9
        for topic, typ in required.items():
            msg = observed.get(topic)
            if msg is None:
                print('FAIL', topic, 'no live', typ.__name__, 'received')
                failed = True
                continue
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age, times = now-stamp, samples[topic]
            hz = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.
            limit = cfg['map_timeout'] if typ is OccupancyGrid else cfg['sensor_timeout']
            ok = -.05 <= age <= limit
            print('OK' if ok else 'FAIL', topic, 'samples=', len(times),
                  f'wall_rate={hz:.2f} Hz age={age:.3f}s frame={msg.header.frame_id}')
            failed |= not ok
        cloud = observed.get(cfg['pointcloud_topic'])
        if cloud is not None:
            fields = {field.name for field in cloud.fields}
            print('Cloud fields:', ', '.join(sorted(fields)))
            fast = (args.robot.parent/cfg['fastlio_config']).resolve()
            if fast.exists():
                mode = yaml.safe_load(fast.read_text())['/**']['ros__parameters']['preprocess']['lidar_type']
                expected = {2: {'x','y','z','intensity','ring','time'},
                            3: {'x','y','z','intensity','ring','t','reflectivity','ambient','range'},
                            5: {'x','y','z','intensity'}}.get(mode)
                if expected is None or not expected <= fields:
                    print('FAIL unsupported or missing FAST-LIO point fields for lidar_type', mode)
                    failed = True
            else:
                print('FAIL missing selected FAST-LIO config:', fast)
                failed = True
            try:
                buffer.lookup_transform(robot['base_frame'], cloud.header.frame_id, Time())
            except TransformException as exc:
                print('FAIL LiDAR mounting TF:', str(exc))
                failed = True
        imu = observed.get(cfg['imu_topic'])
        if imu is not None:
            a, w = imu.linear_acceleration, imu.angular_velocity
            vals = (a.x,a.y,a.z,w.x,w.y,w.z)
            ok = all(math.isfinite(v) for v in vals)
            print('IMU accel (m/s^2), gyro (rad/s):', vals)
            failed |= not ok
            try:
                buffer.lookup_transform(robot['base_frame'], imu.header.frame_id, Time())
            except TransformException as exc:
                print('FAIL IMU mounting TF:', str(exc))
                failed = True
        if not args.sensors_only:
            try:
                tf = buffer.lookup_transform(robot['map_frame'], robot['base_frame'], Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
                if not -.05 <= now-stamp <= cfg['tf_timeout']:
                    raise ValueError('map-to-base TF is stale or from a different clock')
                print('OK map -> base TF')
            except (TransformException, ValueError) as exc:
                print('FAIL map -> base TF:', str(exc))
                failed = True
        return int(failed)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
