#!/usr/bin/env python3
"""Create an isolated Gazebo bridge for the packaged HIL experiment."""
import argparse
from pathlib import Path
import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'verified_stack/robot/a300/bridge.yaml'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'runs/hil_bridge.yaml')
    args = parser.parse_args()
    bridge = yaml.safe_load(SOURCE.read_text())
    output = []
    for item in bridge:
        item = dict(item)
        # FAST-LIO owns odom_lidar -> base_link; wheel TF would create a
        # competing dynamic parent. GNSS is excluded from this denied profile.
        if item['ros_topic_name'] in ('/tf', '/gps/fix'):
            continue
        if item['direction'] == 'ROS_TO_GZ' and item['ros_topic_name'] == '/cmd_vel':
            item['ros_topic_name'] = '/sim/husky/cmd_vel'
        output.append(item)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(output, sort_keys=False))
    print(args.output.resolve())


if __name__ == '__main__':
    main()
