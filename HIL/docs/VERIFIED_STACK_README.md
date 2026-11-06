# Verified GNSS-denied Husky stack

This folder is a portable copy of the current FAST-LIO2, SLAM Toolbox and
Dijkstra-lattice runtime from the successful Work configuration. The original
files under `/home/basudeo/Documents/Work` are not modified by this copy.

## Build FAST-LIO2

```bash
source /opt/ros/jazzy/setup.bash
bash hardware/build_fastlio.sh
source FASTLIO2Experiment/install/setup.bash
```

## Check the packaged Bayland configuration

```bash
source /opt/ros/jazzy/setup.bash
python3 localization_experiment/run_diagnostic.py \
  --map bayland \
  --robot a300 \
  --model DijkstraLattice \
  --localization slam \
  --slam-backend slam_toolbox \
  --pose-backend fast_lio2 \
  --no-slam-scan-matching \
  --no-slam-loop-closing \
  --no-record-bag \
  --check
```

Remove `--check` to run the visible simulation. Add `--headless` for a
headless run. The packaged launcher fixes the verified exploration settings:
boundary preference, straight primitives, adaptive horizon and a 5 m base
horizon. It does not enable persistent exploration memory.

## Hardware transition

The files under `hardware/` provide sensor/TF preflight checks, a fail-closed
command watchdog and configuration templates. They have not yet completed a
physical-Husky validation. Configure measured topics, frames, extrinsics,
footprint and motion limits before connecting motor commands. Start disarmed.

Gazebo truth and contact topics are simulation evaluation inputs only. They are
never localization or planner inputs.
