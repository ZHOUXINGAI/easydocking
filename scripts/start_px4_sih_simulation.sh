#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="/home/hw/PX4-Autopilot"
RUN_DIR="$ROOT_DIR/results/live_px4_sih_$(date +%Y%m%d_%H%M%S)"
START_RVIZ="${START_RVIZ:-true}"
MINI_TAKEOFF_ALTITUDE="${MINI_TAKEOFF_ALTITUDE:-30.0}"
MINI_ORBIT_CENTER_X="${MINI_ORBIT_CENTER_X:-10.0}"
MINI_ORBIT_CENTER_Y="${MINI_ORBIT_CENTER_Y:--6.0}"
MINI_ORBIT_RADIUS="${MINI_ORBIT_RADIUS:-80.0}"
CARRIER_OUTSIDE_MARGIN="${CARRIER_OUTSIDE_MARGIN:-10.0}"
CARRIER_OUTSIDE_ANGLE_DEG="${CARRIER_OUTSIDE_ANGLE_DEG:--135.0}"
CARRIER_ENFORCE_OUTSIDE_ORBIT="${CARRIER_ENFORCE_OUTSIDE_ORBIT:-true}"

resolve_carrier_offset() {
  local resolved
  resolved="$(python3 - <<PY
import math

center_x = float("${MINI_ORBIT_CENTER_X}")
center_y = float("${MINI_ORBIT_CENTER_Y}")
radius = float("${MINI_ORBIT_RADIUS}")
margin = float("${CARRIER_OUTSIDE_MARGIN}")
angle_deg = float("${CARRIER_OUTSIDE_ANGLE_DEG}")
enforce = "${CARRIER_ENFORCE_OUTSIDE_ORBIT}".lower() in {"1", "true", "yes", "on"}
ring = radius + margin
raw_x = """${CARRIER_OFFSET_X-}""".strip() or None
raw_y = """${CARRIER_OFFSET_Y-}""".strip() or None
reason = "explicit"
if raw_x is None or raw_y is None:
    theta = math.radians(angle_deg)
    x = center_x + ring * math.cos(theta)
    y = center_y + ring * math.sin(theta)
    reason = "default_outside_ring"
else:
    x = float(raw_x)
    y = float(raw_y)
    dist = math.hypot(x - center_x, y - center_y)
    if enforce and dist < ring - 1e-6:
        theta = math.radians(angle_deg)
        x = center_x + ring * math.cos(theta)
        y = center_y + ring * math.sin(theta)
        reason = "adjusted_to_default_outside_ring"
print(f"{x:.1f} {y:.1f} {reason}")
PY
)"
  read -r CARRIER_OFFSET_X CARRIER_OFFSET_Y CARRIER_OFFSET_REASON <<<"$resolved"
}

resolve_carrier_offset
CARRIER_OFFSET_Z="${CARRIER_OFFSET_Z:-0.0}"
MINI_ORBIT_SPEED="${MINI_ORBIT_SPEED:-10.0}"
MINI_LOITER_SPEED_COMMAND="${MINI_LOITER_SPEED_COMMAND:-10.5}"
MINI_GLIDE_SPEED_COMMAND="${MINI_GLIDE_SPEED_COMMAND:-10.0}"
MINI_GLIDE_RELEASE_ENABLED="${MINI_GLIDE_RELEASE_ENABLED:-true}"
MINI_TRACKING_SPEED_COMMAND="${MINI_TRACKING_SPEED_COMMAND:-9.2}"
MINI_DOCKING_SPEED_COMMAND="${MINI_DOCKING_SPEED_COMMAND:-8.0}"
MINI_CAPTURE_SPEED_COMMAND="${MINI_CAPTURE_SPEED_COMMAND:-6.0}"
MINI_GLIDE_TRIGGER_DISTANCE="${MINI_GLIDE_TRIGGER_DISTANCE:-10.0}"
MINI_CAPTURE_DISTANCE="${MINI_CAPTURE_DISTANCE:-1.6}"
MINI_TERMINAL_SLOWDOWN_START_DISTANCE="${MINI_TERMINAL_SLOWDOWN_START_DISTANCE:-4.2}"
MINI_TERMINAL_SLOWDOWN_FINISH_DISTANCE="${MINI_TERMINAL_SLOWDOWN_FINISH_DISTANCE:-0.8}"
MINI_TERMINAL_SLOWDOWN_MAX_ABS_Y="${MINI_TERMINAL_SLOWDOWN_MAX_ABS_Y:-1.0}"
MINI_ORBIT_HOLD_READY_ALTITUDE="${MINI_ORBIT_HOLD_READY_ALTITUDE:-29.0}"
MINI_ORBIT_HOLD_ENABLE_DELAY_SEC="${MINI_ORBIT_HOLD_ENABLE_DELAY_SEC:-4.0}"
MINI_USE_OFFBOARD_ORBIT_HOLD="${MINI_USE_OFFBOARD_ORBIT_HOLD:-false}"
MINI_ALLOW_ORBIT_RECENTERING="${MINI_ALLOW_ORBIT_RECENTERING:-true}"
CARRIER_APPROACH_SPEED_LIMIT="${CARRIER_APPROACH_SPEED_LIMIT:-12.0}"
CARRIER_TRACKING_SPEED_LIMIT="${CARRIER_TRACKING_SPEED_LIMIT:-11.4}"
CARRIER_DOCKING_SPEED_LIMIT="${CARRIER_DOCKING_SPEED_LIMIT:-9.8}"
CARRIER_MAX_ACCEL="${CARRIER_MAX_ACCEL:-2.5}"
CARRIER_USE_POSITION_SETPOINT="${CARRIER_USE_POSITION_SETPOINT:-true}"
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
  carrier_offset_auto:=false \
  carrier_offset_x:="$CARRIER_OFFSET_X" \
  carrier_offset_y:="$CARRIER_OFFSET_Y" \
  carrier_offset_z:="$CARRIER_OFFSET_Z" \
  carrier_activate_on_launch:=false \
  carrier_use_position_setpoint:="$CARRIER_USE_POSITION_SETPOINT" \
  carrier_approach_speed_limit:="$CARRIER_APPROACH_SPEED_LIMIT" \
  carrier_tracking_speed_limit:="$CARRIER_TRACKING_SPEED_LIMIT" \
  carrier_docking_speed_limit:="$CARRIER_DOCKING_SPEED_LIMIT" \
  carrier_max_accel:="$CARRIER_MAX_ACCEL" \
  mini_use_offboard_orbit_hold:="$MINI_USE_OFFBOARD_ORBIT_HOLD" \
  mini_takeoff_altitude:="$MINI_TAKEOFF_ALTITUDE" \
  mini_orbit_center_x:="$MINI_ORBIT_CENTER_X" \
  mini_orbit_center_y:="$MINI_ORBIT_CENTER_Y" \
  mini_orbit_radius:="$MINI_ORBIT_RADIUS" \
  mini_orbit_speed:="$MINI_ORBIT_SPEED" \
  mini_loiter_speed_command:="$MINI_LOITER_SPEED_COMMAND" \
  mini_glide_speed_command:="$MINI_GLIDE_SPEED_COMMAND" \
  mini_glide_release_enabled:="$MINI_GLIDE_RELEASE_ENABLED" \
  mini_tracking_speed_command:="$MINI_TRACKING_SPEED_COMMAND" \
  mini_docking_speed_command:="$MINI_DOCKING_SPEED_COMMAND" \
  mini_capture_speed_command:="$MINI_CAPTURE_SPEED_COMMAND" \
  mini_glide_trigger_distance:="$MINI_GLIDE_TRIGGER_DISTANCE" \
  mini_capture_distance:="$MINI_CAPTURE_DISTANCE" \
  mini_terminal_slowdown_start_distance:="$MINI_TERMINAL_SLOWDOWN_START_DISTANCE" \
  mini_terminal_slowdown_finish_distance:="$MINI_TERMINAL_SLOWDOWN_FINISH_DISTANCE" \
  mini_terminal_slowdown_max_abs_y:="$MINI_TERMINAL_SLOWDOWN_MAX_ABS_Y" \
  mini_orbit_hold_ready_altitude:="$MINI_ORBIT_HOLD_READY_ALTITUDE" \
  mini_orbit_hold_enable_delay_sec:="$MINI_ORBIT_HOLD_ENABLE_DELAY_SEC" \
  mini_allow_orbit_recentering:="$MINI_ALLOW_ORBIT_RECENTERING" \
  >"$RUN_DIR/ros_stack.log" 2>&1 &
STACK_PID=$!

echo "[4/4] Ready"
echo "PX4 SIH is running. Carrier stays idle until START; mini auto-takes off, climbs to hold altitude, then enters a waiting orbit."
echo "Carrier offset: (${CARRIER_OFFSET_X}, ${CARRIER_OFFSET_Y}, ${CARRIER_OFFSET_Z})"
echo "Carrier offset reason: ${CARRIER_OFFSET_REASON}"
echo "Mini orbit center: (${MINI_ORBIT_CENTER_X}, ${MINI_ORBIT_CENTER_Y}), radius=${MINI_ORBIT_RADIUS} m, altitude=${MINI_TAKEOFF_ALTITUDE} m"
echo "Mini offboard orbit hold: ${MINI_USE_OFFBOARD_ORBIT_HOLD}"
echo "Mini orbit recentering: ${MINI_ALLOW_ORBIT_RECENTERING}"
echo "Mini speeds: loiter=${MINI_LOITER_SPEED_COMMAND} tracking=${MINI_TRACKING_SPEED_COMMAND} docking=${MINI_DOCKING_SPEED_COMMAND} capture=${MINI_CAPTURE_SPEED_COMMAND}"
echo "Runtime logs: $RUN_DIR"
echo "Start docking with:"
echo "./scripts/start_docking_command.sh"

cleanup() {
  kill $STACK_PID $PX4_CARRIER_PID $PX4_MINI_PID $AGENT_PID 2>/dev/null || true
}

trap cleanup EXIT INT TERM
wait
