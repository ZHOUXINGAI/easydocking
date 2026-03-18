#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="/home/hw/PX4-Autopilot"
RESULTS_DIR="$ROOT_DIR/results/$(date +%Y%m%d_%H%M%S)_px4_sih"
START_RVIZ="${START_RVIZ:-false}"
START_DELAY="${START_DELAY:-auto}"
START_RELAY_DELAY="${START_RELAY_DELAY:-2.4}"
CARRIER_OFFSET_X="${CARRIER_OFFSET_X:-54.0}"
CARRIER_OFFSET_Y="${CARRIER_OFFSET_Y:--30.0}"
CARRIER_OFFSET_Z="${CARRIER_OFFSET_Z:-0.0}"
MINI_TAKEOFF_ALTITUDE="${MINI_TAKEOFF_ALTITUDE:-30.0}"
MINI_ORBIT_CENTER_X="${MINI_ORBIT_CENTER_X:-10.0}"
MINI_ORBIT_CENTER_Y="${MINI_ORBIT_CENTER_Y:--6.0}"
MINI_ORBIT_START_PHASE_DEG="${MINI_ORBIT_START_PHASE_DEG:-180.0}"
MINI_ORBIT_RADIUS="${MINI_ORBIT_RADIUS:-80.0}"
MINI_ORBIT_SPEED="${MINI_ORBIT_SPEED:-12.0}"
MINI_LOITER_SPEED_COMMAND="${MINI_LOITER_SPEED_COMMAND:-12.0}"
MINI_ORBIT_HOLD_READY_ALTITUDE="${MINI_ORBIT_HOLD_READY_ALTITUDE:-29.0}"
MINI_ORBIT_HOLD_ENABLE_DELAY_SEC="${MINI_ORBIT_HOLD_ENABLE_DELAY_SEC:-4.0}"
MINI_USE_OFFBOARD_ORBIT_HOLD="${MINI_USE_OFFBOARD_ORBIT_HOLD:-true}"
AUTO_START_MIN_DISTANCE="${AUTO_START_MIN_DISTANCE:-10.0}"
AUTO_START_MAX_DISTANCE="${AUTO_START_MAX_DISTANCE:-20.0}"
AUTO_START_STABLE_SAMPLES="${AUTO_START_STABLE_SAMPLES:-2}"
AUTO_START_TIMEOUT_SEC="${AUTO_START_TIMEOUT_SEC:-85.0}"
AUTO_START_MIN_WAIT_SEC="${AUTO_START_MIN_WAIT_SEC:-18.0}"
AUTO_START_MIN_REL_Z="${AUTO_START_MIN_REL_Z:--2.0}"
AUTO_START_MAX_REL_Z="${AUTO_START_MAX_REL_Z:-2.0}"
AUTO_START_USE_LOCAL_MIN_TRIGGER="${AUTO_START_USE_LOCAL_MIN_TRIGGER:-false}"
AUTO_START_LOCAL_MIN_INCREASE_MARGIN="${AUTO_START_LOCAL_MIN_INCREASE_MARGIN:-0.08}"
AUTO_START_LOCAL_MIN_MAX_DISTANCE="${AUTO_START_LOCAL_MIN_MAX_DISTANCE:-6.0}"
AUTO_START_LOCAL_MIN_MIN_SAMPLES="${AUTO_START_LOCAL_MIN_MIN_SAMPLES:-6}"
AUTO_START_LOCAL_MIN_MAX_ABS_Y="${AUTO_START_LOCAL_MIN_MAX_ABS_Y:-6.0}"
AUTO_START_LOCAL_MIN_WINDOW_START_SEC="${AUTO_START_LOCAL_MIN_WINDOW_START_SEC:-0.0}"
AUTO_START_LOCAL_MIN_WINDOW_END_SEC="${AUTO_START_LOCAL_MIN_WINDOW_END_SEC:--1.0}"
AUTO_START_USE_STATE_MACHINE_TRIGGER="${AUTO_START_USE_STATE_MACHINE_TRIGGER:-true}"
AUTO_START_SM_WINDOW_START_SEC="${AUTO_START_SM_WINDOW_START_SEC:-18.0}"
AUTO_START_SM_WINDOW_END_SEC="${AUTO_START_SM_WINDOW_END_SEC:--1.0}"
AUTO_START_SM_OBSERVE_DISTANCE="${AUTO_START_SM_OBSERVE_DISTANCE:-24.0}"
AUTO_START_SM_OBSERVE_ABS_Y="${AUTO_START_SM_OBSERVE_ABS_Y:-16.0}"
AUTO_START_SM_TRIGGER_DISTANCE="${AUTO_START_SM_TRIGGER_DISTANCE:-12.0}"
AUTO_START_SM_TRIGGER_ABS_Y="${AUTO_START_SM_TRIGGER_ABS_Y:-8.0}"
AUTO_START_SM_TRIGGER_ABS_X="${AUTO_START_SM_TRIGGER_ABS_X:-10.0}"
AUTO_START_SM_MIN_CONVERGING_SAMPLES="${AUTO_START_SM_MIN_CONVERGING_SAMPLES:-6}"
AUTO_START_SM_ENTER_MAX_CLOSING_RATE="${AUTO_START_SM_ENTER_MAX_CLOSING_RATE:--0.15}"
AUTO_START_SM_STAY_MAX_CLOSING_RATE="${AUTO_START_SM_STAY_MAX_CLOSING_RATE:-0.25}"
AUTO_START_SM_REBOUND_MARGIN="${AUTO_START_SM_REBOUND_MARGIN:-0.08}"
AUTO_START_SM_MAX_RELATIVE_SPEED="${AUTO_START_SM_MAX_RELATIVE_SPEED:-9.5}"
AUTO_START_SM_STAGNATION_SAMPLES="${AUTO_START_SM_STAGNATION_SAMPLES:-4}"
AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE="${AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE:--0.25}"
AUTO_START_SM_TRIGGER_MAX_ABS_VY="${AUTO_START_SM_TRIGGER_MAX_ABS_VY:-5.0}"
AUTO_START_SM_TRIGGER_PROJECTED_ABS_Y="${AUTO_START_SM_TRIGGER_PROJECTED_ABS_Y:-2.2}"
AUTO_START_SM_ENABLE_SEARCH_TRIGGER="${AUTO_START_SM_ENABLE_SEARCH_TRIGGER:-true}"
AUTO_START_SM_SEARCH_TRIGGER_MIN_DISTANCE="${AUTO_START_SM_SEARCH_TRIGGER_MIN_DISTANCE:-16.0}"
AUTO_START_SM_SEARCH_TRIGGER_MAX_DISTANCE="${AUTO_START_SM_SEARCH_TRIGGER_MAX_DISTANCE:-20.5}"
AUTO_START_SM_SEARCH_TRIGGER_ABS_Y="${AUTO_START_SM_SEARCH_TRIGGER_ABS_Y:-14.0}"
AUTO_START_SM_SEARCH_TRIGGER_MAX_CLOSING_RATE="${AUTO_START_SM_SEARCH_TRIGGER_MAX_CLOSING_RATE:-2.0}"
AUTO_START_SM_SEARCH_TRIGGER_PROJECTED_ABS_Y="${AUTO_START_SM_SEARCH_TRIGGER_PROJECTED_ABS_Y:-1.2}"
AUTO_START_SM_SEARCH_MIN_SAMPLES="${AUTO_START_SM_SEARCH_MIN_SAMPLES:-4}"
AUTO_START_REPUBLISH_PERIOD_SEC="${AUTO_START_REPUBLISH_PERIOD_SEC:-1.2}"
AUTO_START_MAX_REPUBLISH_COUNT="${AUTO_START_MAX_REPUBLISH_COUNT:-5}"
EXPERIMENT_DURATION_SEC="${EXPERIMENT_DURATION_SEC:-85.0}"
KEEP_VERBOSE_PX4_TEXT_LOGS="${KEEP_VERBOSE_PX4_TEXT_LOGS:-false}"
KEEP_PX4_ULOG_COPY="${KEEP_PX4_ULOG_COPY:-false}"
CLEAR_PX4_ROOTFS_LOGS_ON_START="${CLEAR_PX4_ROOTFS_LOGS_ON_START:-true}"
CLEAR_PX4_ROOTFS_LOGS_ON_EXIT="${CLEAR_PX4_ROOTFS_LOGS_ON_EXIT:-true}"
mkdir -p "$RESULTS_DIR"

set +u
source "$ROOT_DIR/scripts/setup_local_env.sh" >/dev/null
set -u

export ROS_HOME="$ROOT_DIR/.ros"
export ROS_LOG_DIR="$ROOT_DIR/.ros/log"
mkdir -p "$ROS_LOG_DIR"

cleanup() {
  pkill -9 -f "ros2 launch easydocking_control" 2>/dev/null || true
  pkill -9 -f docking_controller_node 2>/dev/null || true
  pkill -9 -f simple_dual_uav_sim.py 2>/dev/null || true
  pkill -9 -f px4_odom_bridge.py 2>/dev/null || true
  pkill -9 -f px4_offboard_bridge.py 2>/dev/null || true
  pkill -9 -f px4_fixed_wing_bridge.py 2>/dev/null || true
  pkill -9 -f rviz_visualizer.py 2>/dev/null || true
  pkill -9 -f experiment_logger.py 2>/dev/null || true
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

copy_latest_ulog() {
  local instance="$1"
  local log_dir="$PX4_DIR/build/px4_sitl_default/rootfs/$instance/log"
  local latest
  latest="$(find "$log_dir" -type f -name '*.ulg' 2>/dev/null | sort | tail -n 1 || true)"
  if [ -n "$latest" ] && [ -f "$latest" ]; then
    cp "$latest" "$RESULTS_DIR/px4_instance_${instance}.ulg"
  fi
}

cleanup

if [ "$CLEAR_PX4_ROOTFS_LOGS_ON_START" = "true" ]; then
  clear_px4_rootfs_logs
fi

PX4_CARRIER_LOG="$RESULTS_DIR/px4_carrier.log"
PX4_MINI_LOG="$RESULTS_DIR/px4_mini.log"
if [ "$KEEP_VERBOSE_PX4_TEXT_LOGS" != "true" ]; then
  PX4_CARRIER_LOG="/dev/null"
  PX4_MINI_LOG="/dev/null"
fi

/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent udp4 -p 8888 \
  >"$RESULTS_DIR/microxrce_agent.log" 2>&1 &
AGENT_PID=$!

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
  mini_orbit_start_phase_deg:="$MINI_ORBIT_START_PHASE_DEG" \
  mini_orbit_radius:="$MINI_ORBIT_RADIUS" \
  mini_orbit_speed:="$MINI_ORBIT_SPEED" \
  mini_loiter_speed_command:="$MINI_LOITER_SPEED_COMMAND" \
  mini_orbit_hold_ready_altitude:="$MINI_ORBIT_HOLD_READY_ALTITUDE" \
  mini_orbit_hold_enable_delay_sec:="$MINI_ORBIT_HOLD_ENABLE_DELAY_SEC" \
  >"$RESULTS_DIR/launch.log" 2>&1 &
STACK_PID=$!

sleep 12

ros2 run easydocking_control experiment_logger.py \
  --ros-args -p output_dir:="$RESULTS_DIR" -p duration_sec:="$EXPERIMENT_DURATION_SEC" \
  >"$RESULTS_DIR/logger.log" 2>&1 &
LOGGER_PID=$!

if [ "$START_DELAY" = "auto" ]; then
  ros2 run easydocking_control auto_start_docking.py \
    --ros-args \
    -p min_distance:="$AUTO_START_MIN_DISTANCE" \
    -p max_distance:="$AUTO_START_MAX_DISTANCE" \
    -p stable_samples:="$AUTO_START_STABLE_SAMPLES" \
    -p timeout_sec:="$AUTO_START_TIMEOUT_SEC" \
    -p min_wait_sec:="$AUTO_START_MIN_WAIT_SEC" \
    -p min_rel_x:=-100.0 \
    -p max_rel_x:=100.0 \
    -p min_rel_y:=-100.0 \
    -p max_rel_y:=100.0 \
    -p min_rel_z:="$AUTO_START_MIN_REL_Z" \
    -p max_rel_z:="$AUTO_START_MAX_REL_Z" \
    -p use_local_min_trigger:="$AUTO_START_USE_LOCAL_MIN_TRIGGER" \
    -p local_min_increase_margin:="$AUTO_START_LOCAL_MIN_INCREASE_MARGIN" \
    -p local_min_max_distance:="$AUTO_START_LOCAL_MIN_MAX_DISTANCE" \
    -p local_min_min_samples:="$AUTO_START_LOCAL_MIN_MIN_SAMPLES" \
    -p local_min_max_abs_y:="$AUTO_START_LOCAL_MIN_MAX_ABS_Y" \
    -p local_min_window_start_sec:="$AUTO_START_LOCAL_MIN_WINDOW_START_SEC" \
    -p local_min_window_end_sec:="$AUTO_START_LOCAL_MIN_WINDOW_END_SEC" \
    -p use_state_machine_trigger:="$AUTO_START_USE_STATE_MACHINE_TRIGGER" \
    -p sm_window_start_sec:="$AUTO_START_SM_WINDOW_START_SEC" \
    -p sm_window_end_sec:="$AUTO_START_SM_WINDOW_END_SEC" \
    -p sm_observe_distance:="$AUTO_START_SM_OBSERVE_DISTANCE" \
    -p sm_observe_abs_y:="$AUTO_START_SM_OBSERVE_ABS_Y" \
    -p sm_trigger_distance:="$AUTO_START_SM_TRIGGER_DISTANCE" \
    -p sm_trigger_abs_y:="$AUTO_START_SM_TRIGGER_ABS_Y" \
    -p sm_trigger_abs_x:="$AUTO_START_SM_TRIGGER_ABS_X" \
    -p sm_min_converging_samples:="$AUTO_START_SM_MIN_CONVERGING_SAMPLES" \
    -p sm_enter_max_closing_rate:="$AUTO_START_SM_ENTER_MAX_CLOSING_RATE" \
    -p sm_stay_max_closing_rate:="$AUTO_START_SM_STAY_MAX_CLOSING_RATE" \
    -p sm_rebound_margin:="$AUTO_START_SM_REBOUND_MARGIN" \
    -p sm_max_relative_speed:="$AUTO_START_SM_MAX_RELATIVE_SPEED" \
    -p sm_stagnation_samples:="$AUTO_START_SM_STAGNATION_SAMPLES" \
    -p sm_trigger_min_closing_rate:="$AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE" \
    -p sm_trigger_max_abs_vy:="$AUTO_START_SM_TRIGGER_MAX_ABS_VY" \
    -p sm_trigger_projected_abs_y:="$AUTO_START_SM_TRIGGER_PROJECTED_ABS_Y" \
    -p sm_enable_search_trigger:="$AUTO_START_SM_ENABLE_SEARCH_TRIGGER" \
    -p sm_search_trigger_min_distance:="$AUTO_START_SM_SEARCH_TRIGGER_MIN_DISTANCE" \
    -p sm_search_trigger_max_distance:="$AUTO_START_SM_SEARCH_TRIGGER_MAX_DISTANCE" \
    -p sm_search_trigger_abs_y:="$AUTO_START_SM_SEARCH_TRIGGER_ABS_Y" \
    -p sm_search_trigger_max_closing_rate:="$AUTO_START_SM_SEARCH_TRIGGER_MAX_CLOSING_RATE" \
    -p sm_search_trigger_projected_abs_y:="$AUTO_START_SM_SEARCH_TRIGGER_PROJECTED_ABS_Y" \
    -p sm_search_min_samples:="$AUTO_START_SM_SEARCH_MIN_SAMPLES" \
    -p republish_period_sec:="$AUTO_START_REPUBLISH_PERIOD_SEC" \
    -p max_republish_count:="$AUTO_START_MAX_REPUBLISH_COUNT" \
    >"$RESULTS_DIR/start_command.log" 2>&1 || true
else
  sleep "$START_DELAY"
  ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}" \
    >"$RESULTS_DIR/start_command.log" 2>&1
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
    /docking/command_latched easydocking_msgs/msg/DockingCommand "{command: 'START'}" \
    >>"$RESULTS_DIR/start_command.log" 2>&1
  sleep "$START_RELAY_DELAY"
  ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}" \
    >>"$RESULTS_DIR/start_command.log" 2>&1
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
    /docking/command_latched easydocking_msgs/msg/DockingCommand "{command: 'START'}" \
    >>"$RESULTS_DIR/start_command.log" 2>&1
fi

wait $LOGGER_PID || true

python3 "$ROOT_DIR/scripts/generate_report.py" "$RESULTS_DIR" \
  >"$RESULTS_DIR/report.log" 2>&1 || true

kill $STACK_PID $PX4_CARRIER_PID $PX4_MINI_PID $AGENT_PID 2>/dev/null || true
sleep 2

if [ "$KEEP_PX4_ULOG_COPY" = "true" ]; then
  copy_latest_ulog 1
  copy_latest_ulog 2
fi

if [ "$CLEAR_PX4_ROOTFS_LOGS_ON_EXIT" = "true" ]; then
  clear_px4_rootfs_logs
fi

{
  echo "backend=px4_sih"
  echo "px4_carrier_namespace=/px4_1"
  echo "px4_mini_namespace=/px4_2"
  echo "carrier_world_offset=${CARRIER_OFFSET_X},${CARRIER_OFFSET_Y},${CARRIER_OFFSET_Z}"
  echo "mini_orbit_center=${MINI_ORBIT_CENTER_X},${MINI_ORBIT_CENTER_Y}"
  echo "mini_takeoff_altitude=${MINI_TAKEOFF_ALTITUDE}"
  echo "mini_orbit_radius=${MINI_ORBIT_RADIUS}"
  echo "mini_orbit_speed=${MINI_ORBIT_SPEED}"
  echo "mini_use_offboard_orbit_hold=${MINI_USE_OFFBOARD_ORBIT_HOLD}"
  echo "mini_world_offset=0.0,0.0,0.0"
  echo "mini_orbit_start_phase_deg=${MINI_ORBIT_START_PHASE_DEG}"
  echo "mini_vehicle_type=fixed_wing_sih"
  echo "auto_start_local_min_max_distance=${AUTO_START_LOCAL_MIN_MAX_DISTANCE}"
  echo "auto_start_local_min_max_abs_y=${AUTO_START_LOCAL_MIN_MAX_ABS_Y}"
  echo "auto_start_local_min_window_start_sec=${AUTO_START_LOCAL_MIN_WINDOW_START_SEC}"
  echo "auto_start_local_min_window_end_sec=${AUTO_START_LOCAL_MIN_WINDOW_END_SEC}"
  echo "auto_start_use_state_machine_trigger=${AUTO_START_USE_STATE_MACHINE_TRIGGER}"
  echo "auto_start_sm_window_start_sec=${AUTO_START_SM_WINDOW_START_SEC}"
  echo "auto_start_sm_window_end_sec=${AUTO_START_SM_WINDOW_END_SEC}"
  echo "auto_start_sm_trigger_distance=${AUTO_START_SM_TRIGGER_DISTANCE}"
  echo "auto_start_sm_trigger_abs_y=${AUTO_START_SM_TRIGGER_ABS_Y}"
  echo "auto_start_sm_enable_search_trigger=${AUTO_START_SM_ENABLE_SEARCH_TRIGGER}"
  echo "auto_start_sm_search_trigger_min_distance=${AUTO_START_SM_SEARCH_TRIGGER_MIN_DISTANCE}"
  echo "auto_start_sm_search_trigger_max_distance=${AUTO_START_SM_SEARCH_TRIGGER_MAX_DISTANCE}"
  echo "auto_start_sm_search_trigger_abs_y=${AUTO_START_SM_SEARCH_TRIGGER_ABS_Y}"
  echo "auto_start_sm_search_trigger_projected_abs_y=${AUTO_START_SM_SEARCH_TRIGGER_PROJECTED_ABS_Y}"
} >>"$RESULTS_DIR/metadata.txt"

echo "$RESULTS_DIR"
