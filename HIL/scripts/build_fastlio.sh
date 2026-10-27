#!/usr/bin/env bash
set -euo pipefail
hil_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_DISTRO:-}" != jazzy ]]; then
  echo 'Source /opt/ros/jazzy/setup.bash first.' >&2
  exit 2
fi
mkdir -p "$hil_root/fastlio_ws/src"
ln -sfn "$hil_root/vendor/FAST_LIO" "$hil_root/fastlio_ws/src/FAST_LIO"
cd "$hil_root/fastlio_ws"
colcon build --symlink-install --packages-select fast_lio --executor sequential --parallel-workers 1
