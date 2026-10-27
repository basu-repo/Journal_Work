#!/usr/bin/env python3
"""Launch bundled FAST-LIO2 and online mapping against team-owned sensors/TF."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]


def commands(robot, profile, output):
    cfg = robot['localization']
    if not cfg.get('calibration_confirmed'):
        raise ValueError('calibration_confirmed must be set after checking real extrinsics and clocks')
    source = (profile.parent / cfg['fastlio_config']).resolve()
    fastlio = yaml.safe_load(source.read_text())
    params = fastlio['/**']['ros__parameters']
    params['use_sim_time'] = robot.get('use_sim_time', False)
    params['common']['lid_topic'] = cfg['pointcloud_topic']
    params['common']['imu_topic'] = cfg['imu_topic']
    if params['preprocess']['lidar_type'] == 5 and not robot.get('use_sim_time', False):
        raise ValueError('Snapshot type 5 is reserved for validated Gazebo snapshot clouds')
    output.mkdir(parents=True, exist_ok=True)
    fast = output/'fastlio.yaml'
    fast.write_text(yaml.safe_dump(fastlio))
    clock = robot.get('use_sim_time', False)
    slam = yaml.safe_load((Path(get_package_share_directory('slam_toolbox')) /
                           'config/mapper_params_online_async.yaml').read_text())
    slam['slam_toolbox']['ros__parameters'].update(
        use_sim_time=clock, base_frame=robot['base_frame'], odom_frame=robot['odom_frame'],
        map_frame=robot['map_frame'], scan_topic=robot['scan_topic'], resolution=.1,
        map_update_interval=1., minimum_travel_distance=.2, minimum_travel_heading=.15,
        minimum_time_interval=.15, max_laser_range=float(cfg['range_max']),
        transform_timeout=.5, enable_interactive_mode=False,
        use_scan_matching=True, do_loop_closing=True)
    slam_path = output/'slam.yaml'
    slam_path.write_text(yaml.safe_dump(slam))
    scan = dict(use_sim_time=clock, target_frame=robot['base_frame'], transform_tolerance=.5,
                min_height=float(cfg['min_height']), max_height=float(cfg['max_height']),
                angle_min=-3.141592653589793, angle_max=3.141592653589793,
                angle_increment=float(cfg['scan_angle_increment']), scan_time=float(cfg['scan_period']),
                range_min=float(cfg['range_min']), range_max=float(cfg['range_max']),
                use_inf=True, inf_epsilon=1., concurrency_level=1)
    scan_path = output/'scan.yaml'
    scan_path.write_text(yaml.safe_dump({'pointcloud_to_laserscan': {'ros__parameters': scan}}))
    obstacles = {'use_sim_time': clock, 'frame_id': robot['base_frame'],
                 'Grid/NormalsSegmentation': 'true', 'Grid/RangeMin': str(cfg['range_min']),
                 'Grid/RangeMax': str(cfg['range_max']), 'Grid/CellSize': '0.1',
                 'Grid/MaxObstacleHeight': str(cfg['max_height'])}
    obstacle_path = output/'obstacles.yaml'
    obstacle_path.write_text(yaml.safe_dump({'obstacles_detection': {'ros__parameters': obstacles}}))
    obstacles_topic = '/hil/lidar_obstacles'
    return [
        ['ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args', '--params-file', str(fast),
         '-r', '/Odometry:='+cfg['raw_odom_topic'], '-r', '/tf:=/fastlio/internal_tf',
         '-r', '/tf_static:=/fastlio/internal_tf_static', '-r', '/path:=/fastlio/path'],
        [sys.executable, str(ROOT/'localization/fastlio_adapter.py'), '--robot', str(profile),
         '--ros-args', '-p', 'use_sim_time:='+str(clock).lower()],
        ['ros2', 'run', 'rtabmap_util', 'obstacles_detection', '--ros-args',
         '-r', '__node:=obstacles_detection', '--params-file', str(obstacle_path),
         '-r', 'cloud:='+cfg['pointcloud_topic'], '-r', 'obstacles:='+obstacles_topic,
         '-r', 'ground:=/hil/lidar_ground'],
        ['ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', '--ros-args',
         '-r', '__node:=pointcloud_to_laserscan', '--params-file', str(scan_path),
         '-r', 'cloud_in:='+obstacles_topic, '-r', 'scan:='+robot['scan_topic']],
        ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
         'use_sim_time:='+str(clock).lower(), 'slam_params_file:='+str(slam_path)],
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'runs/localization')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    profile = args.robot.resolve()
    robot = yaml.safe_load(profile.read_text())
    if robot['map_topic'] != '/map' or robot['map_frame'] != 'map':
        parser.error('Included SLAM launcher requires map_topic: /map and map_frame: map')
    try:
        cmds = commands(robot, profile, args.output.resolve())
    except (ValueError, FileNotFoundError, KeyError) as exc:
        parser.error(str(exc))
    (args.output/'commands.yaml').write_text(yaml.safe_dump(cmds))
    if args.prepare_only:
        return 0
    check = subprocess.run([sys.executable, str(ROOT/'scripts/preflight.py'),
                            '--robot', str(profile), '--sensors-only'])
    if check.returncode:
        return check.returncode
    processes = []
    logs = []
    try:
        for i, cmd in enumerate(cmds):
            log = (args.output/f'process_{i}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        print('Localization running; logs:', args.output, flush=True)
        while all(p.poll() is None for p in processes):
            time.sleep(.2)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for log in logs:
            log.close()


if __name__ == '__main__':
    raise SystemExit(main())
