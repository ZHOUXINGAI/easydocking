#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START_MODE="${START_MODE:-auto}"
set +u
source "$ROOT_DIR/scripts/setup_local_env.sh"
set -u

if [ "$START_MODE" = "auto" ]; then
  python3 "$ROOT_DIR/scripts/wait_for_docking_window.py" \
    --ros-args \
    -p alignment_min:="${ALIGNMENT_MIN:-0.65}" \
    -p tca_min_sec:="${TCA_MIN_SEC:-4.6}" \
    -p tca_max_sec:="${TCA_MAX_SEC:-5.4}" \
    -p relative_z_min_m:="${RELATIVE_Z_MIN_M:-20.0}" \
    -p relative_distance_min_m:="${RELATIVE_DISTANCE_MIN_M:-35.0}" \
    -p relative_distance_max_m:="${RELATIVE_DISTANCE_MAX_M:-95.0}" \
    -p relative_speed_min_mps:="${RELATIVE_SPEED_MIN_MPS:-8.0}" \
    -p hold_count:="${WINDOW_HOLD_COUNT:-4}" \
    -p timeout_sec:="${WINDOW_TIMEOUT_SEC:-60.0}" \
    -p publish_repeats:="${PUBLISH_REPEATS:-2}" \
    -p fallback_immediate:="${FALLBACK_IMMEDIATE:-true}"
else
  ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}"
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
    /docking/command_latched easydocking_msgs/msg/DockingCommand "{command: 'START'}"
  sleep 1
  ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}"
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
    /docking/command_latched easydocking_msgs/msg/DockingCommand "{command: 'START'}"
fi
