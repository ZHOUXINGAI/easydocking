#!/bin/bash
set -eo pipefail

# Clean up leftover processes from previous runs
pkill -9 -f "simple_dual_uav_sim" 2>/dev/null || true
pkill -9 -f "docking_controller_node" 2>/dev/null || true
pkill -9 -f "rviz_visualizer" 2>/dev/null || true
pkill -9 -f "experiment_logger" 2>/dev/null || true
pkill -9 -f "fixed_wing_bridge" 2>/dev/null || true
pkill -9 -f "window_starter" 2>/dev/null || true
pkill -9 -f "odom_bridge" 2>/dev/null || true
pkill -9 -f "offboard_bridge" 2>/dev/null || true
sleep 1

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS_DIR="$ROOT_DIR/result_sim/$(date +%Y%m%d_%H%M%S)_mock"
mkdir -p "$RESULTS_DIR"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash" 2>/dev/null || source "$ROOT_DIR/install/local_setup.bash"

DURATION="${1:-120}"
TIMEOUT="${2:-150}"

CARRIER_APPROACH_SPEED="${CARRIER_APPROACH_SPEED:-8.0}"
CARRIER_TRACKING_SPEED="${CARRIER_TRACKING_SPEED:-8.0}"
CARRIER_DOCKING_SPEED="${CARRIER_DOCKING_SPEED:-8.0}"
CARRIER_MAX_ACCEL="${CARRIER_MAX_ACCEL:-2.5}"
HOLD_FRACTION="${HOLD_FRACTION:-0.30}"

echo "[mock] launching simulation (no PX4)..."
ros2 launch easydocking_control simulation.launch.py \
  use_sim_time:=false \
  start_agent:=false \
  start_rviz:=false \
  start_px4_bridge:=false \
  use_mock_sim:=true \
  use_px4_odom_bridge:=false \
  carrier_activate_on_launch:=true \
  carrier_idle_hover_altitude:=30.0 \
  carrier_approach_speed_limit:="$CARRIER_APPROACH_SPEED" \
  carrier_tracking_speed_limit:="$CARRIER_TRACKING_SPEED" \
  carrier_docking_speed_limit:="$CARRIER_DOCKING_SPEED" \
  carrier_max_accel:="$CARRIER_MAX_ACCEL" \
  carrier_departure_min_orbit_fraction:="$HOLD_FRACTION" \
  mini_takeoff_altitude:=30.0 \
  mini_orbit_center_x:=10.0 \
  mini_orbit_center_y:=-6.0 \
  mini_orbit_center:="[10.0,-6.0,30.0]" \
  mini_orbit_radius:=80.0 \
  mini_orbit_speed:=10.0 \
  carrier_outside_margin:=140.0 \
  >"$RESULTS_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

sleep 4

echo "[mock] starting logger (${DURATION}s)..."
ros2 run easydocking_control experiment_logger.py \
  --ros-args -p output_dir:="$RESULTS_DIR" -p duration_sec:="${DURATION}.0" \
  >"$RESULTS_DIR/logger.log" 2>&1 &
LOGGER_PID=$!

sleep 2

echo "[mock] sending START..."
ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}" \
  >"$RESULTS_DIR/start.log" 2>&1 || true

echo "[mock] waiting for logger (max ${TIMEOUT}s)..."
timeout "$TIMEOUT" tail --pid="$LOGGER_PID" -f /dev/null 2>/dev/null || true
wait $LOGGER_PID 2>/dev/null || true

echo "[mock] generating plots..."
# Dynamic docking thresholds: both aircraft flying ≥8 m/s, formation flight
export FINAL_PASS_V1_DISTANCE_MAX_M=25.0
export FINAL_PASS_V1_REL_SPEED_MAX_MPS=2.0
export FINAL_PASS_V1_XY_ABS_MAX_M=25.0
export FINAL_PASS_V1_Z_MIN_M=-30.0
export FINAL_PASS_V1_Z_MAX_M=30.0
export FINAL_PASS_LOOSE_DISTANCE_MAX_M=3.0
export FINAL_PASS_LOOSE_REL_SPEED_MAX_MPS=2.0
export FINAL_PASS_LOOSE_XY_ABS_MAX_M=3.0
export FINAL_PASS_LOOSE_Z_MIN_M=-30.0
export FINAL_PASS_LOOSE_Z_MAX_M=30.0
python3 "$ROOT_DIR/scripts/generate_report.py" --mock "$RESULTS_DIR" \
  >"$RESULTS_DIR/report.log" 2>&1 || true
echo "[mock] generating planner diagnostic..."
python3 "$ROOT_DIR/scripts/plot_docking_planner.py" "$RESULTS_DIR" \
  >>"$RESULTS_DIR/report.log" 2>&1 || true

kill $LAUNCH_PID 2>/dev/null || true

echo "[mock] done: $RESULTS_DIR"
ls "$RESULTS_DIR"/*.png "$RESULTS_DIR"/*.gif 2>/dev/null || echo "WARNING: some plots missing"
