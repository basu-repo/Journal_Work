#!/usr/bin/env python3
"""Build a run-scoped planner configuration and motion primitive library."""
import argparse
from pathlib import Path
import yaml


ROOT = Path(__file__).parents[1]


def settings(path):
    data = yaml.safe_load(path.read_text()) or {}
    parent = data.pop('extends', None)
    if parent:
        base = settings((path.parent / parent).resolve())
        base.update(data)
        return base
    return data


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', type=Path, default=ROOT/'config/robot.example.yaml')
    p.add_argument('--mission', type=Path, default=ROOT/'config/mission.example.yaml')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    robot = yaml.safe_load(args.robot.read_text())
    planner_name = 'dijkstra_lattice'
    mission = settings(ROOT/'src'/planner_name/'settings.yaml')
    mission.update(yaml.safe_load(args.mission.read_text()))
    mission.update(localization_mode='slam', gnss_mode='denied', odom_topic=robot['odom_topic'],
                   base_frame=robot['base_frame'])
    primitive = args.output/'motion_primitives.json'
    mission['primitive_file'] = str(primitive)
    from dijkstra_lattice.primitives import compile_library
    compile_library(robot, mission, primitive)
    (args.output/'robot.yaml').write_text(yaml.safe_dump(robot, sort_keys=False))
    (args.output/'mission.yaml').write_text(yaml.safe_dump(mission, sort_keys=False))
    print(args.output)


if __name__ == '__main__': main()
