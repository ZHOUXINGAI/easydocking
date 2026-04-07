#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS_DIR="$ROOT_DIR/results/$(date +%Y%m%d_%H%M%S)_mock_fixed_wing"
START_RVIZ="${START_RVIZ:-false}"
START_MODE="${START_MODE:-auto}"
DEFAULT_START_DELAY="30.0"
if [ "$START_MODE" = "auto" ]; then
  DEFAULT_START_DELAY="4.0"
fi
START_DELAY="${START_DELAY:-$DEFAULT_START_DELAY}"
EXPERIMENT_DURATION_SEC="${EXPERIMENT_DURATION_SEC:-90.0}"
DURATION_SEC_FMT="$(printf '%.3f' "$EXPERIMENT_DURATION_SEC")"
LOGGER_POST_COMPLETED_SETTLE_SEC="${LOGGER_POST_COMPLETED_SETTLE_SEC:-0.6}"
LOGGER_POST_COMPLETED_ROWS="${LOGGER_POST_COMPLETED_ROWS:-12}"
CARRIER_OFFSET_X="${CARRIER_OFFSET_X:-54.0}"
CARRIER_OFFSET_Y="${CARRIER_OFFSET_Y:--30.0}"
CARRIER_OFFSET_Z="${CARRIER_OFFSET_Z:-0.0}"
MINI_TAKEOFF_ALTITUDE="${MINI_TAKEOFF_ALTITUDE:-30.0}"
MINI_ORBIT_CENTER_X="${MINI_ORBIT_CENTER_X:-10.0}"
MINI_ORBIT_CENTER_Y="${MINI_ORBIT_CENTER_Y:--6.0}"
MINI_ORBIT_RADIUS="${MINI_ORBIT_RADIUS:-55.0}"
MINI_ORBIT_SPEED="${MINI_ORBIT_SPEED:-10.0}"
MINI_TRACKING_SPEED="${MINI_TRACKING_SPEED:-9.7}"
MINI_DOCKING_SPEED="${MINI_DOCKING_SPEED:-6.6}"
MINI_CAPTURE_SPEED="${MINI_CAPTURE_SPEED:-4.8}"
MINI_SLOWDOWN_START_DISTANCE="${MINI_SLOWDOWN_START_DISTANCE:-5.0}"
MINI_SLOWDOWN_FINISH_DISTANCE="${MINI_SLOWDOWN_FINISH_DISTANCE:-2.0}"
CARRIER_APPROACH_SPEED_LIMIT="${CARRIER_APPROACH_SPEED_LIMIT:-11.6}"
CARRIER_TRACKING_SPEED_LIMIT="${CARRIER_TRACKING_SPEED_LIMIT:-10.8}"
CARRIER_DOCKING_SPEED_LIMIT="${CARRIER_DOCKING_SPEED_LIMIT:-9.4}"
CARRIER_MAX_SPEED_XY="${CARRIER_MAX_SPEED_XY:-11.8}"
CARRIER_MAX_SPEED_Z="${CARRIER_MAX_SPEED_Z:-2.2}"
CARRIER_MAX_ACCEL="${CARRIER_MAX_ACCEL:-2.5}"
MINI_MAX_ACCEL="${MINI_MAX_ACCEL:-2.0}"
ATTACH_DISTANCE="${ATTACH_DISTANCE:-0.24}"
ATTACH_SPEED_THRESHOLD="${ATTACH_SPEED_THRESHOLD:-0.8}"
SOFT_ATTACH_DISTANCE="${SOFT_ATTACH_DISTANCE:-0.34}"
SOFT_ATTACH_XY_TOLERANCE="${SOFT_ATTACH_XY_TOLERANCE:-0.16}"
SOFT_ATTACH_Z_MIN="${SOFT_ATTACH_Z_MIN:-0.12}"
SOFT_ATTACH_Z_MAX="${SOFT_ATTACH_Z_MAX:-0.32}"
SOFT_ATTACH_VX_THRESHOLD="${SOFT_ATTACH_VX_THRESHOLD:-0.9}"
SOFT_ATTACH_VY_THRESHOLD="${SOFT_ATTACH_VY_THRESHOLD:-0.9}"
SOFT_ATTACH_VZ_THRESHOLD="${SOFT_ATTACH_VZ_THRESHOLD:-0.35}"

mkdir -p "$RESULTS_DIR"

set +u
source "$ROOT_DIR/scripts/setup_local_env.sh" >/dev/null
set -u

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
}

cleanup

ros2 launch easydocking_control simulation.launch.py \
  use_sim_time:=false \
  start_agent:=false \
  start_rviz:="$START_RVIZ" \
  start_px4_bridge:=false \
  use_mock_sim:=true \
  use_px4_odom_bridge:=false \
  carrier_activate_on_launch:=false \
  carrier_approach_speed_limit:="$CARRIER_APPROACH_SPEED_LIMIT" \
  carrier_tracking_speed_limit:="$CARRIER_TRACKING_SPEED_LIMIT" \
  carrier_docking_speed_limit:="$CARRIER_DOCKING_SPEED_LIMIT" \
  carrier_offset_x:="$CARRIER_OFFSET_X" \
  carrier_offset_y:="$CARRIER_OFFSET_Y" \
  carrier_offset_z:="$CARRIER_OFFSET_Z" \
  mini_takeoff_altitude:="$MINI_TAKEOFF_ALTITUDE" \
  mini_orbit_center_x:="$MINI_ORBIT_CENTER_X" \
  mini_orbit_center_y:="$MINI_ORBIT_CENTER_Y" \
  mini_orbit_radius:="$MINI_ORBIT_RADIUS" \
  mini_orbit_speed:="$MINI_ORBIT_SPEED" \
  mini_tracking_speed:="$MINI_TRACKING_SPEED" \
  mini_docking_speed:="$MINI_DOCKING_SPEED" \
  mini_capture_speed:="$MINI_CAPTURE_SPEED" \
  mini_slowdown_start_distance:="$MINI_SLOWDOWN_START_DISTANCE" \
  mini_slowdown_finish_distance:="$MINI_SLOWDOWN_FINISH_DISTANCE" \
  mini_max_accel:="$MINI_MAX_ACCEL" \
  carrier_max_accel:="$CARRIER_MAX_ACCEL" \
  carrier_max_speed_xy:="$CARRIER_MAX_SPEED_XY" \
  carrier_max_speed_z:="$CARRIER_MAX_SPEED_Z" \
  attach_distance:="$ATTACH_DISTANCE" \
  attach_speed_threshold:="$ATTACH_SPEED_THRESHOLD" \
  soft_attach_distance:="$SOFT_ATTACH_DISTANCE" \
  soft_attach_xy_tolerance:="$SOFT_ATTACH_XY_TOLERANCE" \
  soft_attach_z_min:="$SOFT_ATTACH_Z_MIN" \
  soft_attach_z_max:="$SOFT_ATTACH_Z_MAX" \
  soft_attach_vx_threshold:="$SOFT_ATTACH_VX_THRESHOLD" \
  soft_attach_vy_threshold:="$SOFT_ATTACH_VY_THRESHOLD" \
  soft_attach_vz_threshold:="$SOFT_ATTACH_VZ_THRESHOLD" \
  >"$RESULTS_DIR/launch.log" 2>&1 &
STACK_PID=$!

sleep 4

ros2 run easydocking_control experiment_logger.py \
  --ros-args \
  -p output_dir:="$RESULTS_DIR" \
  -p duration_sec:="$DURATION_SEC_FMT" \
  -p post_completed_settle_sec:="$LOGGER_POST_COMPLETED_SETTLE_SEC" \
  -p post_completed_rows:="$LOGGER_POST_COMPLETED_ROWS" \
  >"$RESULTS_DIR/logger.log" 2>&1 &
LOGGER_PID=$!

sleep "$START_DELAY"
START_MODE="$START_MODE" \
WINDOW_TIMEOUT_SEC="${WINDOW_TIMEOUT_SEC:-60.0}" \
ALIGNMENT_MIN="${ALIGNMENT_MIN:-0.65}" \
TCA_MIN_SEC="${TCA_MIN_SEC:-4.6}" \
TCA_MAX_SEC="${TCA_MAX_SEC:-5.4}" \
RELATIVE_Z_MIN_M="${RELATIVE_Z_MIN_M:-20.0}" \
RELATIVE_DISTANCE_MIN_M="${RELATIVE_DISTANCE_MIN_M:-35.0}" \
RELATIVE_DISTANCE_MAX_M="${RELATIVE_DISTANCE_MAX_M:-95.0}" \
RELATIVE_SPEED_MIN_MPS="${RELATIVE_SPEED_MIN_MPS:-8.0}" \
WINDOW_HOLD_COUNT="${WINDOW_HOLD_COUNT:-4}" \
PUBLISH_REPEATS="${PUBLISH_REPEATS:-2}" \
FALLBACK_IMMEDIATE="${FALLBACK_IMMEDIATE:-true}" \
./scripts/start_docking_command.sh >"$RESULTS_DIR/start_command.log" 2>&1

wait $LOGGER_PID || true

python3 "$ROOT_DIR/scripts/generate_report.py" "$RESULTS_DIR" \
  >"$RESULTS_DIR/report.log" 2>&1 || true
python3 "$ROOT_DIR/scripts/generate_animation.py" "$RESULTS_DIR" \
  >"$RESULTS_DIR/animation.log" 2>&1 || true

kill $STACK_PID 2>/dev/null || true

{
  echo "backend=mock_fixed_wing"
  echo "start_mode=${START_MODE}"
  echo "start_delay=${START_DELAY}"
  echo "experiment_duration_sec=${DURATION_SEC_FMT}"
  echo "carrier_offset=${CARRIER_OFFSET_X},${CARRIER_OFFSET_Y},${CARRIER_OFFSET_Z}"
  echo "mini_orbit_center=${MINI_ORBIT_CENTER_X},${MINI_ORBIT_CENTER_Y}"
  echo "mini_takeoff_altitude=${MINI_TAKEOFF_ALTITUDE}"
  echo "mini_orbit_radius=${MINI_ORBIT_RADIUS}"
  echo "mini_orbit_speed=${MINI_ORBIT_SPEED}"
  echo "carrier_max_speed_xy=${CARRIER_MAX_SPEED_XY}"
} >>"$RESULTS_DIR/metadata.txt"

echo "$RESULTS_DIR"
