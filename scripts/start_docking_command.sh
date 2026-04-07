#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START_MODE="${START_MODE:-auto}"
set +u
source "$ROOT_DIR/scripts/setup_local_env.sh"
set -u

if [ "$START_MODE" = "auto" ] || [ "$START_MODE" = "window" ]; then
  python3 "$ROOT_DIR/scripts/wait_for_docking_window.py" \
    --ros-args \
    -p alignment_min:="${ALIGNMENT_MIN:-0.82}" \
    -p tca_min_sec:="${TCA_MIN_SEC:-5.8}" \
    -p tca_max_sec:="${TCA_MAX_SEC:-7.2}" \
    -p relative_z_min_m:="${RELATIVE_Z_MIN_M:-20.0}" \
    -p relative_distance_min_m:="${RELATIVE_DISTANCE_MIN_M:-75.0}" \
    -p relative_distance_max_m:="${RELATIVE_DISTANCE_MAX_M:-120.0}" \
    -p relative_speed_min_mps:="${RELATIVE_SPEED_MIN_MPS:-9.0}" \
    -p hold_count:="${WINDOW_HOLD_COUNT:-4}" \
    -p timeout_sec:="${WINDOW_TIMEOUT_SEC:-120.0}" \
    -p publish_repeats:="${PUBLISH_REPEATS:-2}" \
    -p fallback_immediate:="${FALLBACK_IMMEDIATE:-false}" \
    -p require_orbit_completion_before_start:="${REQUIRE_ORBIT_COMPLETION_BEFORE_START:-true}" \
    -p orbit_center_x:="${MINI_ORBIT_CENTER_X:-10.0}" \
    -p orbit_center_y:="${MINI_ORBIT_CENTER_Y:--6.0}" \
    -p orbit_radius:="${MINI_ORBIT_RADIUS:-80.0}" \
    -p orbit_gate_ready_altitude:="${ORBIT_GATE_READY_ALTITUDE:-29.0}" \
    -p orbit_gate_ready_radius_tolerance:="${ORBIT_GATE_READY_RADIUS_TOLERANCE:-30.0}" \
    -p orbit_gate_min_valid_samples:="${ORBIT_GATE_MIN_VALID_SAMPLES:-8}" \
    -p orbit_gate_required_laps:="${ORBIT_GATE_REQUIRED_LAPS:-1.0}" \
    -p orbit_gate_min_accumulated_angle_deg:="${ORBIT_GATE_MIN_ACCUMULATED_ANGLE_DEG:-330.0}" \
    -p orbit_gate_return_tolerance_deg:="${ORBIT_GATE_RETURN_TOLERANCE_DEG:-45.0}" \
    -p enable_geometry_cluster_gate:="${ENABLE_GEOMETRY_CLUSTER_GATE:-false}" \
    -p geometry_cluster_score_threshold:="${GEOMETRY_CLUSTER_SCORE_THRESHOLD:-12.0}" \
    -p geometry_cluster_min_rel_z:="${GEOMETRY_CLUSTER_MIN_REL_Z:--40.0}" \
    -p geometry_cluster_max_rel_z:="${GEOMETRY_CLUSTER_MAX_REL_Z:-40.0}" \
    -p geometry_cluster_a_center:="${GEOMETRY_CLUSTER_A_CENTER:=[-4.166384,105.1457845,-6.485333,-10.2991625]}" \
    -p geometry_cluster_a_spread:="${GEOMETRY_CLUSTER_A_SPREAD:=[1.6,1.2,0.25,0.25]}" \
    -p geometry_cluster_b_center:="${GEOMETRY_CLUSTER_B_CENTER:=[-5.395847,102.9623125,-6.2016955,-10.510685]}" \
    -p geometry_cluster_b_spread:="${GEOMETRY_CLUSTER_B_SPREAD:=[0.9,1.2,0.22,0.18]}" \
    -p require_mini_energy_healthy:="${REQUIRE_MINI_ENERGY_HEALTHY:-false}" \
    -p mini_px4_namespace:="${MINI_PX4_NAMESPACE:-/px4_2}" \
    -p health_min_true_airspeed_mps:="${HEALTH_MIN_TRUE_AIRSPEED_MPS:-7.0}" \
    -p health_max_underspeed_ratio:="${HEALTH_MAX_UNDERSPEED_RATIO:-0.35}" \
    -p health_min_samples:="${HEALTH_MIN_SAMPLES:-5}"
elif [ "$START_MODE" = "state_machine" ]; then
  ros2 run easydocking_control auto_start_docking.py \
    --ros-args \
    -p min_distance:="${MIN_DISTANCE:-24.0}" \
    -p max_distance:="${MAX_DISTANCE:-34.0}" \
    -p stable_samples:="${STABLE_SAMPLES:-3}" \
    -p timeout_sec:="${WINDOW_TIMEOUT_SEC:-120.0}" \
    -p min_wait_sec:="${MIN_WAIT_SEC:-0.0}" \
    -p min_rel_z:="${MIN_REL_Z:--40.0}" \
    -p max_rel_z:="${MAX_REL_Z:-40.0}" \
    -p min_rel_x:="${MIN_REL_X:--1000.0}" \
    -p max_rel_x:="${MAX_REL_X:-1000.0}" \
    -p min_rel_y:="${MIN_REL_Y:--1000.0}" \
    -p max_rel_y:="${MAX_REL_Y:-1000.0}" \
    -p republish_period_sec:="${REPUBLISH_PERIOD_SEC:-1.2}" \
    -p max_republish_count:="${MAX_REPUBLISH_COUNT:-5}" \
    -p use_local_min_trigger:="${USE_LOCAL_MIN_TRIGGER:-false}" \
    -p local_min_increase_margin:="${LOCAL_MIN_INCREASE_MARGIN:-0.08}" \
    -p local_min_max_distance:="${LOCAL_MIN_MAX_DISTANCE:-6.0}" \
    -p local_min_min_samples:="${LOCAL_MIN_MIN_SAMPLES:-6}" \
    -p local_min_max_abs_y:="${LOCAL_MIN_MAX_ABS_Y:-6.0}" \
    -p local_min_window_start_sec:="${LOCAL_MIN_WINDOW_START_SEC:-0.0}" \
    -p local_min_window_end_sec:="${LOCAL_MIN_WINDOW_END_SEC:--1.0}" \
    -p use_state_machine_trigger:="${USE_STATE_MACHINE_TRIGGER:-true}" \
    -p sm_window_start_sec:="${SM_WINDOW_START_SEC:-25.0}" \
    -p sm_window_end_sec:="${SM_WINDOW_END_SEC:--1.0}" \
    -p sm_observe_distance:="${SM_OBSERVE_DISTANCE:-60.0}" \
    -p sm_observe_abs_y:="${SM_OBSERVE_ABS_Y:-20.0}" \
    -p sm_trigger_distance:="${SM_TRIGGER_DISTANCE:-12.0}" \
    -p sm_trigger_abs_y:="${SM_TRIGGER_ABS_Y:-8.0}" \
    -p sm_trigger_abs_x:="${SM_TRIGGER_ABS_X:-10.0}" \
    -p sm_min_converging_samples:="${SM_MIN_CONVERGING_SAMPLES:-6}" \
    -p sm_enter_max_closing_rate:="${SM_ENTER_MAX_CLOSING_RATE:--0.15}" \
    -p sm_stay_max_closing_rate:="${SM_STAY_MAX_CLOSING_RATE:-0.25}" \
    -p sm_rebound_margin:="${SM_REBOUND_MARGIN:-0.08}" \
    -p sm_max_relative_speed:="${SM_MAX_RELATIVE_SPEED:-14.5}" \
    -p sm_stagnation_samples:="${SM_STAGNATION_SAMPLES:-4}" \
    -p sm_trigger_min_closing_rate:="${SM_TRIGGER_MIN_CLOSING_RATE:--0.25}" \
    -p sm_trigger_max_abs_vy:="${SM_TRIGGER_MAX_ABS_VY:-5.0}" \
    -p sm_trigger_projected_abs_y:="${SM_TRIGGER_PROJECTED_ABS_Y:-2.2}" \
    -p sm_enable_search_trigger:="${SM_ENABLE_SEARCH_TRIGGER:-true}" \
    -p sm_search_trigger_min_distance:="${SM_SEARCH_TRIGGER_MIN_DISTANCE:-35.0}" \
    -p sm_search_trigger_max_distance:="${SM_SEARCH_TRIGGER_MAX_DISTANCE:-60.0}" \
    -p sm_search_trigger_abs_y:="${SM_SEARCH_TRIGGER_ABS_Y:-20.0}" \
    -p sm_search_trigger_max_closing_rate:="${SM_SEARCH_TRIGGER_MAX_CLOSING_RATE:-5.0}" \
    -p sm_search_trigger_projected_abs_y:="${SM_SEARCH_TRIGGER_PROJECTED_ABS_Y:-4.0}" \
    -p sm_search_min_samples:="${SM_SEARCH_MIN_SAMPLES:-4}" \
    -p sm_enable_orbit_phase_trigger:="${SM_ENABLE_ORBIT_PHASE_TRIGGER:-true}" \
    -p sm_orbit_trigger_min_distance:="${SM_ORBIT_TRIGGER_MIN_DISTANCE:-118.0}" \
    -p sm_orbit_trigger_max_distance:="${SM_ORBIT_TRIGGER_MAX_DISTANCE:-138.0}" \
    -p sm_orbit_trigger_min_rel_x:="${SM_ORBIT_TRIGGER_MIN_REL_X:-10.0}" \
    -p sm_orbit_trigger_max_rel_x:="${SM_ORBIT_TRIGGER_MAX_REL_X:-22.0}" \
    -p sm_orbit_trigger_min_rel_y:="${SM_ORBIT_TRIGGER_MIN_REL_Y:--135.0}" \
    -p sm_orbit_trigger_max_rel_y:="${SM_ORBIT_TRIGGER_MAX_REL_Y:--118.0}" \
    -p sm_orbit_trigger_min_rel_vx:="${SM_ORBIT_TRIGGER_MIN_REL_VX:-10.0}" \
    -p sm_orbit_trigger_max_abs_vy:="${SM_ORBIT_TRIGGER_MAX_ABS_VY:-2.5}" \
    -p sm_orbit_trigger_max_abs_vz:="${SM_ORBIT_TRIGGER_MAX_ABS_VZ:-0.30}" \
    -p sm_orbit_trigger_min_samples:="${SM_ORBIT_TRIGGER_MIN_SAMPLES:-3}" \
    -p sm_orbit_trigger_stagnation_samples:="${SM_ORBIT_TRIGGER_STAGNATION_SAMPLES:-2}" \
    -p sm_orbit_score_target_distance:="${SM_ORBIT_SCORE_TARGET_DISTANCE:-134.0}" \
    -p sm_orbit_score_target_rel_x:="${SM_ORBIT_SCORE_TARGET_REL_X:-15.0}" \
    -p sm_orbit_score_target_rel_y:="${SM_ORBIT_SCORE_TARGET_REL_Y:--129.0}" \
    -p sm_enable_idle_orbit_trigger:="${SM_ENABLE_IDLE_ORBIT_TRIGGER:-true}" \
    -p sm_idle_orbit_trigger_min_distance:="${SM_IDLE_ORBIT_TRIGGER_MIN_DISTANCE:-38.0}" \
    -p sm_idle_orbit_trigger_max_distance:="${SM_IDLE_ORBIT_TRIGGER_MAX_DISTANCE:-50.0}" \
    -p sm_idle_orbit_trigger_min_rel_x:="${SM_IDLE_ORBIT_TRIGGER_MIN_REL_X:-20.0}" \
    -p sm_idle_orbit_trigger_max_rel_x:="${SM_IDLE_ORBIT_TRIGGER_MAX_REL_X:-35.0}" \
    -p sm_idle_orbit_trigger_min_rel_y:="${SM_IDLE_ORBIT_TRIGGER_MIN_REL_Y:--20.0}" \
    -p sm_idle_orbit_trigger_max_rel_y:="${SM_IDLE_ORBIT_TRIGGER_MAX_REL_Y:--8.0}" \
    -p sm_idle_orbit_trigger_min_rel_vx:="${SM_IDLE_ORBIT_TRIGGER_MIN_REL_VX:-4.0}" \
    -p sm_idle_orbit_trigger_max_rel_vx:="${SM_IDLE_ORBIT_TRIGGER_MAX_REL_VX:-8.0}" \
    -p sm_idle_orbit_trigger_min_rel_vy:="${SM_IDLE_ORBIT_TRIGGER_MIN_REL_VY:-8.0}" \
    -p sm_idle_orbit_trigger_max_rel_vy:="${SM_IDLE_ORBIT_TRIGGER_MAX_REL_VY:-13.0}" \
    -p sm_idle_orbit_trigger_max_abs_vz:="${SM_IDLE_ORBIT_TRIGGER_MAX_ABS_VZ:-1.0}" \
    -p sm_idle_orbit_trigger_min_samples:="${SM_IDLE_ORBIT_TRIGGER_MIN_SAMPLES:-8}" \
    -p require_orbit_completion_before_start:="${REQUIRE_ORBIT_COMPLETION_BEFORE_START:-true}" \
    -p orbit_center_x:="${MINI_ORBIT_CENTER_X:-10.0}" \
    -p orbit_center_y:="${MINI_ORBIT_CENTER_Y:--6.0}" \
    -p orbit_radius:="${MINI_ORBIT_RADIUS:-80.0}" \
    -p orbit_gate_ready_altitude:="${ORBIT_GATE_READY_ALTITUDE:-29.0}" \
    -p orbit_gate_ready_radius_tolerance:="${ORBIT_GATE_READY_RADIUS_TOLERANCE:-30.0}" \
    -p orbit_gate_min_valid_samples:="${ORBIT_GATE_MIN_VALID_SAMPLES:-8}" \
    -p orbit_gate_required_laps:="${ORBIT_GATE_REQUIRED_LAPS:-1.0}" \
    -p orbit_gate_min_accumulated_angle_deg:="${ORBIT_GATE_MIN_ACCUMULATED_ANGLE_DEG:-330.0}" \
    -p orbit_gate_return_tolerance_deg:="${ORBIT_GATE_RETURN_TOLERANCE_DEG:-45.0}" \
    -p require_mini_energy_healthy:="${REQUIRE_MINI_ENERGY_HEALTHY:-false}" \
    -p mini_px4_namespace:="${MINI_PX4_NAMESPACE:-/px4_2}" \
    -p health_min_true_airspeed_mps:="${HEALTH_MIN_TRUE_AIRSPEED_MPS:-7.0}" \
    -p health_max_underspeed_ratio:="${HEALTH_MAX_UNDERSPEED_RATIO:-0.35}" \
    -p health_min_samples:="${HEALTH_MIN_SAMPLES:-5}"
else
  ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}"
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
    /docking/command_latched easydocking_msgs/msg/DockingCommand "{command: 'START'}"
  sleep 1
  ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}"
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
    /docking/command_latched easydocking_msgs/msg/DockingCommand "{command: 'START'}"
fi
