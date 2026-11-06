"""Ensure HIL launches the frozen Bayland navigation profile."""
import sys
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from run_hil import VERIFIED_ROOT, verified_planner_command
from scripts.prepare_run import settings


class VerifiedProfileTests(unittest.TestCase):
    def test_hil_uses_verified_navigator_and_options(self):
        robot = yaml.safe_load((ROOT/'config/robot.example.yaml').read_text())
        output = ROOT/'runs/test_profile'
        command = verified_planner_command(robot, output)
        self.assertEqual(command[1], str(VERIFIED_ROOT/'localization_experiment/navigator.py'))
        for option in ('--adaptive-horizon', '--straight-only', '--boundary-follow'):
            self.assertIn(option, command)
        self.assertIn('/cmd_vel:='+robot['planner_command_topic'], command)

    def test_packaged_and_verified_settings_match(self):
        packaged = settings(ROOT/'src/dijkstra_lattice/settings.yaml')
        verified = settings(ROOT/'verified_stack/DijkstraLattice/settings.yaml')
        self.assertEqual(packaged, verified)
        self.assertEqual(packaged['lattice_horizon_m'], 5.0)

    def test_localization_uses_verified_slam_policy(self):
        source = (ROOT/'scripts/start_localization.py').read_text()
        self.assertIn('use_scan_matching=False', source)
        self.assertIn('do_loop_closing=False', source)


if __name__ == '__main__':
    unittest.main()
