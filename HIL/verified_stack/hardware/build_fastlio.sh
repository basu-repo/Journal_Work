#!/usr/bin/env bash
set -euo pipefail
stack_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_DISTRO:-}" != jazzy ]]; then
  echo 'Source /opt/ros/jazzy/setup.bash first.' >&2
  exit 2
fi
cd "$stack_root/FASTLIO2Experiment"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --symlink-install --packages-select fast_lio --executor sequential --parallel-workers 1
