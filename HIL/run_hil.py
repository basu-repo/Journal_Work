#!/usr/bin/env python3
"""Start a planner and fail-closed hardware watchdog; no simulator dependencies."""
import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', type=Path, default=ROOT/'config/robot.example.yaml')
    p.add_argument('--mission', type=Path, required=True)
    p.add_argument('--arm', action='store_true', help='Allow watchdog to forward bounded velocity commands')
    p.add_argument('--skip-preflight', action='store_true')
    args = p.parse_args()
    robot = yaml.safe_load(args.robot.read_text())
    env = os.environ.copy()
    env['PYTHONPATH'] = str(ROOT/'src') + os.pathsep + env.get('PYTHONPATH', '')
    output = ROOT/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    planner_name = 'dijkstra_lattice'
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
    planner = [sys.executable, '-m', planner_name+'.node', '--robot', str(output/'robot.yaml'),
               '--mission', str(output/'mission.yaml'), '--output', str(output), '--ros-args',
               '-r', '/scan:='+robot['scan_topic'],
               '-r', '/map:='+robot['map_topic'],
               '-r', '/cmd_vel:='+robot['planner_command_topic'],
               '-r', '/navigation/enabled:=/hil/navigation_enabled',
               '-p', 'use_sim_time:='+str(robot.get('use_sim_time', False)).lower()]
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
