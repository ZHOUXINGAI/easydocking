#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EASYDOCKING_SETUP="$ROOT_DIR/install/setup.bash"
UXRCE_SETUP="/home/hw/uxrce_agent_ws/install/local_setup.bash"
PX4_DIR="/home/hw/PX4-Autopilot"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "$EASYDOCKING_SETUP" ]; then
  source "$EASYDOCKING_SETUP"
fi

if [ -f "$UXRCE_SETUP" ]; then
  source "$UXRCE_SETUP"
fi

if [ -x "/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent" ]; then
  export PATH="/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin:$PATH"
fi

if [ -d "$PX4_DIR" ]; then
  export PX4_AUTOPILOT_PATH="$PX4_DIR"
fi

echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "PX4_AUTOPILOT_PATH=${PX4_AUTOPILOT_PATH:-missing}"
