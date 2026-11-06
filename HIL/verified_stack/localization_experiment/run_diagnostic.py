"""Launch the verified GNSS-denied FAST-LIO2 and Dijkstra-lattice stack."""
import json
import os
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PACKAGE_ROOT))

import run_simulation as runner

for option in ('--explore', '--boundary-follow', '--straight-only',
               '--adaptive-horizon'):
    if option in sys.argv:
        sys.argv.remove(option)

HORIZON_M = 5.0
if '--experimental-horizon-m' in sys.argv:
    index = sys.argv.index('--experimental-horizon-m')
    HORIZON_M = float(sys.argv[index + 1])
    del sys.argv[index:index + 2]
if not 0.5 <= HORIZON_M <= 10.0:
    raise ValueError('Planning horizon must be between 0.5 and 10 m')

original_popen = subprocess.Popen


def verified_popen(command, *args, **kwargs):
    """Replace only the normal Dijkstra node with verified global guidance."""
    if isinstance(command, list) and 'DijkstraLattice.node' in command:
        command = list(command)
        index = command.index('-m')
        command[index:index + 2] = [str(EXPERIMENT_ROOT/'navigator.py')]
        command[2:2] = ['--adaptive-horizon', '--straight-only',
                        '--boundary-follow']
        output = Path(command[command.index('--output') + 1])
        (output/'exploration_experiment.json').write_text(json.dumps({
            'variant': 'known_free_global_guidance_frontiers',
            'boundary_follow': True,
            'straight_only': True,
            'adaptive_horizon': True,
            'base_horizon_m': HORIZON_M,
            'baseline_collision_checks': True,
            'persistent_exploration_memory': False,
        }, indent=2))
    return original_popen(command, *args, **kwargs)


subprocess.Popen = verified_popen
original_settings = runner.model_settings


def settings(path, seen=()):
    config = original_settings(path, seen)
    if path.resolve() == (PACKAGE_ROOT/'DijkstraLattice/settings.yaml').resolve():
        config['readiness_timeout'] = float('inf')
        config['waypoint_timeout'] = float('inf')
        config['lattice_horizon_m'] = HORIZON_M
    return config


runner.model_settings = settings
original_run_navigation = runner.run_navigation


def run_navigation(args, *rest):
    output = EXPERIMENT_ROOT/'result'/datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True)
    variant = {
        'explore': True,
        'boundary_follow': True,
        'straight_only': True,
        'adaptive_horizon': True,
        'experimental_horizon_m': HORIZON_M,
        'slam_scan_matching': args.slam_scan_matching,
        'slam_loop_closing': args.slam_loop_closing,
        'pose_backend': args.pose_backend,
        'fastlio_imu_noise_profile': args.fastlio_imu_noise_profile,
    }
    (output/'variant.json').write_text(json.dumps(variant, indent=2))
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(args.ros_domain)
    recorder_log = (output/'recorder.log').open('w')
    recorder = original_popen(
        [sys.executable, str(EXPERIMENT_ROOT/'record_motion.py'),
         str(output/'sensors.jsonl')],
        env=environment, stdout=recorder_log, stderr=recorder_log,
        start_new_session=True)
    print('Read-only diagnostics:', output, flush=True)
    try:
        return original_run_navigation(args, *rest)
    finally:
        if recorder.poll() is None:
            os.killpg(recorder.pid, signal.SIGINT)
        recorder.wait(timeout=15)
        recorder_log.close()


runner.run_navigation = run_navigation


if __name__ == '__main__':
    runner.main()
