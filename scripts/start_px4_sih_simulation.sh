#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="/home/hw/PX4-Autopilot"
RUN_DIR="$ROOT_DIR/results/live_px4_sih_$(date +%Y%m%d_%H%M%S)"
START_RVIZ="${START_RVIZ:-true}"
CARRIER_OFFSET_X="${CARRIER_OFFSET_X:-54.0}"
CARRIER_OFFSET_Y="${CARRIER_OFFSET_Y:--30.0}"
CARRIER_OFFSET_Z="${CARRIER_OFFSET_Z:-0.0}"
MINI_TAKEOFF_ALTITUDE="${MINI_TAKEOFF_ALTITUDE:-30.0}"
MINI_ORBIT_CENTER_X="${MINI_ORBIT_CENTER_X:-10.0}"
MINI_ORBIT_CENTER_Y="${MINI_ORBIT_CENTER_Y:--6.0}"
MINI_ORBIT_RADIUS="${MINI_ORBIT_RADIUS:-80.0}"
MINI_ORBIT_SPEED="${MINI_ORBIT_SPEED:-12.0}"
MINI_LOITER_SPEED_COMMAND="${MINI_LOITER_SPEED_COMMAND:-12.0}"
MINI_ORBIT_HOLD_READY_ALTITUDE="${MINI_ORBIT_HOLD_READY_ALTITUDE:-29.0}"
MINI_ORBIT_HOLD_ENABLE_DELAY_SEC="${MINI_ORBIT_HOLD_ENABLE_DELAY_SEC:-4.0}"
MINI_USE_OFFBOARD_ORBIT_HOLD="${MINI_USE_OFFBOARD_ORBIT_HOLD:-true}"
MINI_ALLOW_ORBIT_RECENTERING="${MINI_ALLOW_ORBIT_RECENTERING:-true}"
KEEP_VERBOSE_PX4_TEXT_LOGS="${KEEP_VERBOSE_PX4_TEXT_LOGS:-false}"
CLEAR_PX4_ROOTFS_LOGS_ON_START="${CLEAR_PX4_ROOTFS_LOGS_ON_START:-true}"
mkdir -p "$RUN_DIR"

set +u
source "$ROOT_DIR/scripts/setup_local_env.sh"
set -u

cleanup_existing() {
  pkill -9 -f "ros2 launch easydocking_control" 2>/dev/null || true
  pkill -9 -f docking_controller_node 2>/dev/null || true
  pkill -9 -f simple_dual_uav_sim.py 2>/dev/null || true
  pkill -9 -f px4_odom_bridge.py 2>/dev/null || true
  pkill -9 -f px4_offboard_bridge.py 2>/dev/null || true
  pkill -9 -f px4_fixed_wing_bridge.py 2>/dev/null || true
  pkill -9 -f rviz_visualizer.py 2>/dev/null || true
  pkill -9 -f rviz2 2>/dev/null || true
  pkill -9 -f MicroXRCEAgent 2>/dev/null || true
  pkill -9 -f "$PX4_DIR/build/px4_sitl_default/bin/px4 -i 1" 2>/dev/null || true
  pkill -9 -f "$PX4_DIR/build/px4_sitl_default/bin/px4 -i 2" 2>/dev/null || true
}

clear_px4_rootfs_logs() {
  python3 - <<'PY'
from pathlib import Path
import shutil
for base in [
    Path('/home/hw/PX4-Autopilot/build/px4_sitl_default/rootfs/1/log'),
    Path('/home/hw/PX4-Autopilot/build/px4_sitl_default/rootfs/2/log'),
]:
    if not base.exists():
        continue
    for child in base.iterdir():
        if child.is_dir():
            shutil.rmtree(child, ignore_errors=True)
        else:
            child.unlink(missing_ok=True)
PY
}

echo "[0/4] Cleaning up old processes"
cleanup_existing
sleep 1

if [ "$CLEAR_PX4_ROOTFS_LOGS_ON_START" = "true" ]; then
  clear_px4_rootfs_logs
fi

PX4_CARRIER_LOG="$RUN_DIR/px4_carrier.log"
PX4_MINI_LOG="$RUN_DIR/px4_mini.log"
if [ "$KEEP_VERBOSE_PX4_TEXT_LOGS" != "true" ]; then
  PX4_CARRIER_LOG="/dev/null"
  PX4_MINI_LOG="/dev/null"
fi

echo "[1/4] Starting MicroXRCEAgent"
/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent udp4 -p 8888 \
  >"$RUN_DIR/microxrce_agent.log" 2>&1 &
AGENT_PID=$!

echo "[2/4] Starting dual PX4 SIH"
(
  cd "$PX4_DIR"
  PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=quadx PX4_SIMULATOR=sihsim \
    "$PX4_DIR/build/px4_sitl_default/bin/px4" -i 1
) >"$PX4_CARRIER_LOG" 2>&1 &
PX4_CARRIER_PID=$!

(
  cd "$PX4_DIR"
  PX4_SYS_AUTOSTART=10041 PX4_SIM_MODEL=airplane PX4_SIMULATOR=sihsim \
    "$PX4_DIR/build/px4_sitl_default/bin/px4" -i 2
) >"$PX4_MINI_LOG" 2>&1 &
PX4_MINI_PID=$!

sleep 8

echo "[3/4] Starting ROS 2 docking stack"
ros2 launch easydocking_control simulation.launch.py \
  use_sim_time:=false \
  start_agent:=false \
  start_rviz:="$START_RVIZ" \
  start_px4_bridge:=true \
  use_mock_sim:=false \
  use_px4_odom_bridge:=true \
  carrier_offset_x:="$CARRIER_OFFSET_X" \
  carrier_offset_y:="$CARRIER_OFFSET_Y" \
  carrier_offset_z:="$CARRIER_OFFSET_Z" \
  carrier_activate_on_launch:=false \
  mini_use_offboard_orbit_hold:="$MINI_USE_OFFBOARD_ORBIT_HOLD" \
  mini_takeoff_altitude:="$MINI_TAKEOFF_ALTITUDE" \
  mini_orbit_center_x:="$MINI_ORBIT_CENTER_X" \
  mini_orbit_center_y:="$MINI_ORBIT_CENTER_Y" \
  mini_orbit_radius:="$MINI_ORBIT_RADIUS" \
  mini_orbit_speed:="$MINI_ORBIT_SPEED" \
  mini_loiter_speed_command:="$MINI_LOITER_SPEED_COMMAND" \
  mini_orbit_hold_ready_altitude:="$MINI_ORBIT_HOLD_READY_ALTITUDE" \
  mini_orbit_hold_enable_delay_sec:="$MINI_ORBIT_HOLD_ENABLE_DELAY_SEC" \
  mini_allow_orbit_recentering:="$MINI_ALLOW_ORBIT_RECENTERING" \
  >"$RUN_DIR/ros_stack.log" 2>&1 &
STACK_PID=$!

echo "[4/4] Ready"
echo "PX4 SIH is running. Carrier stays idle until START; mini auto-takes off, climbs to hold altitude, then enters a waiting orbit."
echo "Carrier offset: (${CARRIER_OFFSET_X}, ${CARRIER_OFFSET_Y}, ${CARRIER_OFFSET_Z})"
echo "Mini orbit center: (${MINI_ORBIT_CENTER_X}, ${MINI_ORBIT_CENTER_Y}), radius=${MINI_ORBIT_RADIUS} m, altitude=${MINI_TAKEOFF_ALTITUDE} m"
echo "Mini offboard orbit hold: ${MINI_USE_OFFBOARD_ORBIT_HOLD}"
echo "Mini orbit recentering: ${MINI_ALLOW_ORBIT_RECENTERING}"
echo "Runtime logs: $RUN_DIR"
echo "Start docking with:"
echo "./scripts/start_docking_command.sh"

cleanup() {
  kill $STACK_PID $PX4_CARRIER_PID $PX4_MINI_PID $AGENT_PID 2>/dev/null || true
}

trap cleanup EXIT INT TERM
wait
