#!/usr/bin/env python3
"""Read-only ROS topic/type and TF preflight for the physical robot."""
import argparse
import os
import subprocess
import sys
from pathlib import Path
import yaml


def run(*args, timeout=8):
    env = os.environ.copy()
    env.setdefault('ROS_LOG_DIR', '/tmp/hil_ros_logs')
    return subprocess.run(args, text=True, capture_output=True, timeout=timeout, env=env)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', type=Path, required=True)
    args = p.parse_args()
    robot = yaml.safe_load(args.robot.read_text())
    required = {
        robot['scan_topic']: 'sensor_msgs/msg/LaserScan',
        robot['map_topic']: 'nav_msgs/msg/OccupancyGrid',
        robot['odom_topic']: 'nav_msgs/msg/Odometry',
        '/tf': 'tf2_msgs/msg/TFMessage',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
    }
    result = run('ros2', 'topic', 'list', '-t')
    if result.returncode:
        print(result.stderr.strip()); return 2
    observed = {}
    for line in result.stdout.splitlines():
        if ' [' in line: observed[line.split(' [', 1)[0]] = line.split('[', 1)[1].rstrip(']')
    failed = False
    for topic, expected in required.items():
        actual = observed.get(topic)
        ok = actual == expected
        print(('OK  ' if ok else 'FAIL'), topic, actual or 'MISSING', 'expected', expected)
        failed |= not ok
    tf = run('timeout', '5', 'ros2', 'run', 'tf2_ros', 'tf2_echo',
             robot.get('map_frame', 'map'), robot['base_frame'], timeout=8)
    tf_ok = 'Translation:' in tf.stdout or 'translation:' in tf.stdout
    print(('OK  ' if tf_ok else 'FAIL'), 'TF', robot.get('map_frame', 'map'),
          '->', robot['base_frame'])
    return 1 if failed or not tf_ok else 0


if __name__ == '__main__': sys.exit(main())
