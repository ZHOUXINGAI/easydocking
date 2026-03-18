#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PHASES="${PHASES:-120 180}"
CARRIER_OFFSET_X="${CARRIER_OFFSET_X:-54.0}"
CARRIER_OFFSET_Y="${CARRIER_OFFSET_Y:--30.0}"
EXPERIMENT_DURATION_SEC="${EXPERIMENT_DURATION_SEC:-90.0}"

run_phase() {
  local phase="$1"
  local result_dir
  result_dir="$(
    set +u
    source "$ROOT_DIR/scripts/setup_local_env.sh" >/dev/null
    set -u
    CARRIER_OFFSET_X="$CARRIER_OFFSET_X" \
    CARRIER_OFFSET_Y="$CARRIER_OFFSET_Y" \
    MINI_ORBIT_START_PHASE_DEG="$phase" \
    EXPERIMENT_DURATION_SEC="$EXPERIMENT_DURATION_SEC" \
    AUTO_START_USE_STATE_MACHINE_TRIGGER=true \
    AUTO_START_USE_LOCAL_MIN_TRIGGER=false \
    AUTO_START_TIMEOUT_SEC=85.0 \
    AUTO_START_MIN_WAIT_SEC=25.0 \
    AUTO_START_SM_WINDOW_START_SEC=27.0 \
    AUTO_START_SM_WINDOW_END_SEC=-1.0 \
    AUTO_START_SM_OBSERVE_DISTANCE=24.0 \
    AUTO_START_SM_OBSERVE_ABS_Y=16.0 \
    AUTO_START_SM_TRIGGER_DISTANCE=12.0 \
    AUTO_START_SM_TRIGGER_ABS_Y=8.0 \
    AUTO_START_SM_TRIGGER_ABS_X=10.0 \
    AUTO_START_SM_MIN_CONVERGING_SAMPLES=6 \
    AUTO_START_SM_ENTER_MAX_CLOSING_RATE=-0.15 \
    AUTO_START_SM_STAY_MAX_CLOSING_RATE=0.25 \
    AUTO_START_SM_REBOUND_MARGIN=0.08 \
    AUTO_START_SM_MAX_RELATIVE_SPEED=9.5 \
    AUTO_START_SM_STAGNATION_SAMPLES=4 \
    AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE=-0.25 \
    AUTO_START_SM_TRIGGER_MAX_ABS_VY=5.0 \
    AUTO_START_SM_TRIGGER_PROJECTED_ABS_Y=2.2 \
    bash "$ROOT_DIR/scripts/run_px4_sih_docking_experiment.sh"
  )"

  local summary_file="$result_dir/summary.txt"
  local start_log="$result_dir/start_command.log"
  local min_distance final_phase best_distance best_speed trigger_line

  min_distance="$(awk -F= '$1=="min_distance_m"{print $2}' "$summary_file")"
  final_phase="$(awk -F= '$1=="final_phase"{print $2}' "$summary_file")"
  best_distance="$(awk -F= '$1=="best_window_distance_m"{print $2}' "$summary_file")"
  best_speed="$(awk -F= '$1=="best_window_rel_speed_mps"{print $2}' "$summary_file")"
  trigger_line="$(grep -E 'Auto START sent|Auto START timed out' "$start_log" | tail -n 1 || true)"

  printf 'phase=%s min_distance=%s best_window_distance=%s best_window_speed=%s final_phase=%s\n' \
    "$phase" "$min_distance" "$best_distance" "$best_speed" "$final_phase"
  if [ -n "$trigger_line" ]; then
    printf '  trigger: %s\n' "$trigger_line"
  fi
  printf '  result_dir=%s\n' "$result_dir"
}

for phase in $PHASES; do
  run_phase "$phase"
done
