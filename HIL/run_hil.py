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
        '-p', 'use_sim_time:='+str(robot.get('use_sim_time', False)).lower(),
    ]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', type=Path, default=ROOT/'config/robot.example.yaml')
    p.add_argument('--mission', type=Path, required=True)
    p.add_argument('--arm', action='store_true', help='Allow watchdog to forward bounded velocity commands')
    p.add_argument('--skip-preflight', action='store_true')
    args = p.parse_args()
    robot = yaml.safe_load(args.robot.read_text())
    if args.arm and not robot.get('verified_profile_armed', False):
        p.error('--arm is locked for the verified profile; complete simulation and disarmed validation, then set verified_profile_armed: true in robot.site.yaml')
    env = os.environ.copy()
    env['PYTHONPATH'] = (str(VERIFIED_ROOT) + os.pathsep + str(ROOT/'src') +
                         os.pathsep + env.get('PYTHONPATH', ''))
    output = ROOT/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    prep = [sys.executable, str(ROOT/'scripts/prepare_run.py'),
            '--robot', str(args.robot), '--mission', str(args.mission), '--output', str(output)]
    subprocess.run(prep, check=True, env=env)
    if not args.skip_preflight:
        code = subprocess.run([sys.executable, str(ROOT/'scripts/preflight.py'),
                               '--robot', str(args.robot)], env=env).returncode
        if code: return code
    watchdog = [sys.executable, str(ROOT/'scripts/hardware_watchdog.py'),
                '--config', str(output/'robot.yaml')]
    if args.arm: watchdog.append('--arm')
    clock_args = ['--ros-args', '-p', 'use_sim_time:='+str(robot.get('use_sim_time', False)).lower()]
    watchdog += clock_args
    health = [sys.executable, str(ROOT/'localization/health.py'),
              '--robot', str(output/'robot.yaml')] + clock_args
    planner = verified_planner_command(robot, output)
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
    }, indent=2))
    processes = [subprocess.Popen(health, env=env, start_new_session=True),
                 subprocess.Popen(watchdog, env=env, start_new_session=True),
                 subprocess.Popen(planner, env=env, start_new_session=True)]
    print('Run:', output, '|', 'ARMED' if args.arm else 'DISARMED', flush=True)
    try:
        while all(x.poll() is None for x in processes): time.sleep(0.2)
    except KeyboardInterrupt: pass
    finally:
        for proc in processes:
            if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGTERM)
    return next((x.returncode for x in processes if x.returncode not in (None, 0)), 0)


if __name__ == '__main__': raise SystemExit(main())
