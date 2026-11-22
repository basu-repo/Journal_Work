#!/usr/bin/env python3
"""Start a planner and fail-closed hardware watchdog; no simulator dependencies."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent
VERIFIED_ROOT = ROOT/'verified_stack'


def recording_topics(robot):
    """Return the configured physical inputs and HIL outputs needed for diagnosis."""
    localization = robot.get('localization', {})
    topics = [
        robot.get('tf_source_topic', '/tf'),
        robot.get('tf_static_topic', '/tf_static'),
        robot.get('tf_topic', '/tf'),
        localization.get('pointcloud_topic'),
        localization.get('imu_topic'),
        robot.get('wheel_odom_topic'),
        localization.get('raw_odom_topic', '/fastlio/odometry'),
        robot.get('odom_topic', '/fastlio/odometry_base'),
        '/cloud_registered',
        robot.get('scan_topic', '/scan'),
        robot.get('map_topic', '/map'),
        '/localization/ready',
        '/dijkstra_lattice/path',
        '/navigation/mission_targets',
        robot.get('planner_command_topic', '/hil/planner_cmd_vel'),
        robot.get('hardware_command_topic', '/cmd_vel'),
        '/rosout',
    ]
    topics.extend(robot.get('additional_record_topics', []))
    # Preserve order while avoiding duplicate TF or command topics.
    return list(dict.fromkeys(topic for topic in topics if topic))


def verified_planner_command(robot, output):
    """Launch the navigator and options used by the verified Bayland run."""
    return [
        sys.executable, str(VERIFIED_ROOT/'localization_experiment/navigator.py'),
        '--adaptive-horizon', '--straight-only', '--boundary-follow',
        '--robot', str(output/'robot.yaml'),
        '--mission', str(output/'mission.yaml'), '--output', str(output),
        '--ros-args',
        '-r', '/scan:='+robot['scan_topic'],
        '-r', '/map:='+robot['map_topic'],
        '-r', '/cmd_vel:='+robot['planner_command_topic'],
        '-r', '/navigation/enabled:=/hil/navigation_enabled',
        '-r', '/tf:='+robot.get('tf_topic', '/tf'),
        '-r', '/tf_static:='+robot.get('tf_static_topic', '/tf_static'),
        '-p', 'use_sim_time:='+str(robot.get('use_sim_time', False)).lower(),
    ]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', type=Path, default=ROOT/'config/robot.example.yaml')
    p.add_argument('--mission', type=Path, required=True)
    p.add_argument('--arm', action='store_true', help='Allow watchdog to forward bounded velocity commands')
    p.add_argument('--twist-stamped-output', action='store_true',
                   help='Publish TwistStamped on the configured hardware command topic')
    p.add_argument('--startup-delay', type=float, default=15.0,
                   help='Additional delay after readiness before physical motion (default: 15 s)')
    p.add_argument('--record-bag', action='store_true',
                   help='Automatically record configured physical and HIL diagnostic topics')
    p.add_argument('--skip-preflight', action='store_true')
    args = p.parse_args()
    if not 0.0 <= args.startup_delay <= 300.0:
        p.error('--startup-delay must be between 0 and 300 seconds')
    robot = yaml.safe_load(args.robot.read_text())
    if args.arm and not robot.get('verified_profile_armed', False):
        p.error('--arm is locked for the verified profile; complete simulation and disarmed validation, then set verified_profile_armed: true in '+str(args.robot))
    env = os.environ.copy()
    # Keep the full event log on disk while showing only operator-relevant
    # navigation transitions in the physical-run terminal.
    env['HIL_CONCISE_CONSOLE'] = '1'
    env['PYTHONPATH'] = (str(VERIFIED_ROOT) + os.pathsep + str(ROOT/'src') +
                         os.pathsep + env.get('PYTHONPATH', ''))
    output = ROOT/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    prep = [sys.executable, str(ROOT/'scripts/prepare_run.py'),
            '--robot', str(args.robot), '--mission', str(args.mission), '--output', str(output)]
    subprocess.run(prep, check=True, env=env)
    prepared_mission_path = output/'mission.yaml'
    prepared_mission = yaml.safe_load(prepared_mission_path.read_text())
    prepared_mission['startup_delay'] = float(args.startup_delay)
    prepared_mission_path.write_text(yaml.safe_dump(prepared_mission, sort_keys=False))
    if not args.skip_preflight:
        code = subprocess.run([
            sys.executable, str(ROOT/'scripts/preflight.py'), '--robot', str(args.robot),
            '--ros-args', '-r', '/tf:='+robot.get('tf_topic', '/tf'),
            '-r', '/tf_static:='+robot.get('tf_static_topic', '/tf_static')],
            env=env).returncode
        if code: return code
    watchdog = [sys.executable, str(ROOT/'scripts/hardware_watchdog.py'),
                '--config', str(output/'robot.yaml')]
    if args.arm: watchdog.append('--arm')
    if args.twist_stamped_output: watchdog.append('--twist-stamped-output')
    clock_args = [
        '--ros-args',
        '-p', 'use_sim_time:='+str(robot.get('use_sim_time', False)).lower(),
        '-r', '/tf:='+robot.get('tf_topic', '/tf'),
        '-r', '/tf_static:='+robot.get('tf_static_topic', '/tf_static')]
    watchdog += clock_args
    health = [sys.executable, str(ROOT/'localization/health.py'),
              '--robot', str(output/'robot.yaml')] + clock_args
    planner = verified_planner_command(robot, output)
    topics = recording_topics(robot) if args.record_bag else []
    bag_directory = output/'rosbag' if args.record_bag else None
    (output/'navigation_profile.json').write_text(json.dumps({
        'profile': 'verified_bayland',
        'planner': 'dijkstra_state_lattice',
        'global_guidance': True,
        'boundary_follow': True,
        'straight_only': True,
        'adaptive_horizon': True,
        'base_horizon_m': 5.0,
        'persistent_exploration_memory': False,
        'armed': bool(args.arm),
        'hardware_command_type': ('TwistStamped' if args.twist_stamped_output else 'Twist'),
        'bag_recording': bool(args.record_bag),
        'bag_directory': str(bag_directory) if bag_directory else None,
        'bag_topics': topics,
    }, indent=2))
    bag_process = None
    bag_log = None
    if args.record_bag:
        bag_log = (output/'bag_record.log').open('w')
        bag_process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', str(bag_directory), *topics],
            env=env, start_new_session=True, stdout=bag_log,
            stderr=subprocess.STDOUT)
        time.sleep(1.0)
        if bag_process.poll() is not None:
            bag_log.close()
            print('ERROR: rosbag recorder failed; see', output/'bag_record.log', file=sys.stderr)
            return bag_process.returncode or 1
        print('Bag recording:', bag_directory, flush=True)
    processes = [subprocess.Popen(health, env=env, start_new_session=True),
                 subprocess.Popen(watchdog, env=env, start_new_session=True),
                 subprocess.Popen(planner, env=env, start_new_session=True)]
    print('Run:', output, '|', 'ARMED' if args.arm else 'DISARMED', flush=True)
    print('Motion remains inhibited until preflight, localization readiness, and the',
          f"{float(prepared_mission.get('startup_delay', 0.0)):.0f} s startup delay complete.",
          flush=True)
    try:
        while all(x.poll() is None for x in processes): time.sleep(0.2)
    except KeyboardInterrupt: pass
    finally:
        for proc in processes:
            if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGTERM)
        if bag_process is not None and bag_process.poll() is None:
            os.killpg(bag_process.pid, signal.SIGINT)
            try: bag_process.wait(timeout=15)
            except subprocess.TimeoutExpired: os.killpg(bag_process.pid, signal.SIGTERM)
        if bag_log is not None: bag_log.close()
    return next((x.returncode for x in processes if x.returncode not in (None, 0)), 0)


if __name__ == '__main__': raise SystemExit(main())
