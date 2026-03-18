#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPEATS="${REPEATS:-5}"
CARRIER_OFFSET_X="${CARRIER_OFFSET_X:-54.0}"
CARRIER_OFFSET_Y="${CARRIER_OFFSET_Y:--28.0}"
MINI_ORBIT_START_PHASE_DEG="${MINI_ORBIT_START_PHASE_DEG:-150.0}"

AUTO_START_USE_STATE_MACHINE_TRIGGER="${AUTO_START_USE_STATE_MACHINE_TRIGGER:-true}"
AUTO_START_USE_LOCAL_MIN_TRIGGER="${AUTO_START_USE_LOCAL_MIN_TRIGGER:-false}"
AUTO_START_TIMEOUT_SEC="${AUTO_START_TIMEOUT_SEC:-68.0}"
AUTO_START_MIN_WAIT_SEC="${AUTO_START_MIN_WAIT_SEC:-10.0}"
AUTO_START_SM_WINDOW_START_SEC="${AUTO_START_SM_WINDOW_START_SEC:-20.0}"
AUTO_START_SM_WINDOW_END_SEC="${AUTO_START_SM_WINDOW_END_SEC:--1.0}"
AUTO_START_SM_OBSERVE_DISTANCE="${AUTO_START_SM_OBSERVE_DISTANCE:-22.0}"
AUTO_START_SM_OBSERVE_ABS_Y="${AUTO_START_SM_OBSERVE_ABS_Y:-14.0}"
AUTO_START_SM_TRIGGER_DISTANCE="${AUTO_START_SM_TRIGGER_DISTANCE:-10.0}"
AUTO_START_SM_TRIGGER_ABS_Y="${AUTO_START_SM_TRIGGER_ABS_Y:-7.0}"
AUTO_START_SM_TRIGGER_ABS_X="${AUTO_START_SM_TRIGGER_ABS_X:-8.5}"
AUTO_START_SM_MIN_CONVERGING_SAMPLES="${AUTO_START_SM_MIN_CONVERGING_SAMPLES:-6}"
AUTO_START_SM_ENTER_MAX_CLOSING_RATE="${AUTO_START_SM_ENTER_MAX_CLOSING_RATE:--0.2}"
AUTO_START_SM_STAY_MAX_CLOSING_RATE="${AUTO_START_SM_STAY_MAX_CLOSING_RATE:-0.2}"
AUTO_START_SM_REBOUND_MARGIN="${AUTO_START_SM_REBOUND_MARGIN:-0.08}"
AUTO_START_SM_MAX_RELATIVE_SPEED="${AUTO_START_SM_MAX_RELATIVE_SPEED:-9.5}"
AUTO_START_SM_STAGNATION_SAMPLES="${AUTO_START_SM_STAGNATION_SAMPLES:-4}"
AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE="${AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE:--0.4}"

trigger_count=0
best_min_distance=""
best_dir=""

for rep in $(seq 1 "$REPEATS"); do
  echo "[repeat $rep/$REPEATS] x=$CARRIER_OFFSET_X y=$CARRIER_OFFSET_Y phase=$MINI_ORBIT_START_PHASE_DEG"
  result_dir="$(
    CARRIER_OFFSET_X="$CARRIER_OFFSET_X" \
    CARRIER_OFFSET_Y="$CARRIER_OFFSET_Y" \
    MINI_ORBIT_START_PHASE_DEG="$MINI_ORBIT_START_PHASE_DEG" \
    AUTO_START_USE_STATE_MACHINE_TRIGGER="$AUTO_START_USE_STATE_MACHINE_TRIGGER" \
    AUTO_START_USE_LOCAL_MIN_TRIGGER="$AUTO_START_USE_LOCAL_MIN_TRIGGER" \
    AUTO_START_TIMEOUT_SEC="$AUTO_START_TIMEOUT_SEC" \
    AUTO_START_MIN_WAIT_SEC="$AUTO_START_MIN_WAIT_SEC" \
    AUTO_START_SM_WINDOW_START_SEC="$AUTO_START_SM_WINDOW_START_SEC" \
    AUTO_START_SM_WINDOW_END_SEC="$AUTO_START_SM_WINDOW_END_SEC" \
    AUTO_START_SM_OBSERVE_DISTANCE="$AUTO_START_SM_OBSERVE_DISTANCE" \
    AUTO_START_SM_OBSERVE_ABS_Y="$AUTO_START_SM_OBSERVE_ABS_Y" \
    AUTO_START_SM_TRIGGER_DISTANCE="$AUTO_START_SM_TRIGGER_DISTANCE" \
    AUTO_START_SM_TRIGGER_ABS_Y="$AUTO_START_SM_TRIGGER_ABS_Y" \
    AUTO_START_SM_TRIGGER_ABS_X="$AUTO_START_SM_TRIGGER_ABS_X" \
    AUTO_START_SM_MIN_CONVERGING_SAMPLES="$AUTO_START_SM_MIN_CONVERGING_SAMPLES" \
    AUTO_START_SM_ENTER_MAX_CLOSING_RATE="$AUTO_START_SM_ENTER_MAX_CLOSING_RATE" \
    AUTO_START_SM_STAY_MAX_CLOSING_RATE="$AUTO_START_SM_STAY_MAX_CLOSING_RATE" \
    AUTO_START_SM_REBOUND_MARGIN="$AUTO_START_SM_REBOUND_MARGIN" \
    AUTO_START_SM_MAX_RELATIVE_SPEED="$AUTO_START_SM_MAX_RELATIVE_SPEED" \
    AUTO_START_SM_STAGNATION_SAMPLES="$AUTO_START_SM_STAGNATION_SAMPLES" \
    AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE="$AUTO_START_SM_TRIGGER_MIN_CLOSING_RATE" \
    "$ROOT_DIR/scripts/run_px4_sih_docking_experiment.sh" | tail -n 1
  )"

  python3 - <<PY
import os
result_dir = "$result_dir"
summary = {}
with open(os.path.join(result_dir, "summary.txt")) as f:
    for line in f:
        if "=" in line:
            k, v = line.strip().split("=", 1)
            summary[k] = v
start_log = open(os.path.join(result_dir, "start_command.log")).read()
trigger = "timeout"
if "state-machine-current" in start_log:
    trigger = "state-machine-current"
elif "state-machine-stagnation" in start_log:
    trigger = "state-machine-stagnation"
elif "state-machine distance=" in start_log:
    trigger = "state-machine"
elif "Auto START sent" in start_log:
    trigger = "other"
print(f"result_dir={result_dir}")
print(f"trigger={trigger}")
print(f"final_phase={summary.get('final_phase', '?')}")
print(f"min_distance_m={summary.get('min_distance_m', '?')}")
PY

  trigger_type="$(python3 - <<PY
import os
text = open(os.path.join("$result_dir", "start_command.log")).read()
if "Auto START sent" in text:
    print("triggered")
else:
    print("timeout")
PY
)"
  min_distance="$(awk -F= '$1=="min_distance_m"{print $2}' "$result_dir/summary.txt")"

  if [ "$trigger_type" = "triggered" ]; then
    trigger_count=$((trigger_count + 1))
  fi

  if [ -z "$best_min_distance" ] || python3 - <<PY
best = float("${best_min_distance:-1e9}")
current = float("$min_distance")
raise SystemExit(0 if current < best else 1)
PY
  then
    best_min_distance="$min_distance"
    best_dir="$result_dir"
  fi
done

echo "TRIGGER_COUNT=$trigger_count"
echo "REPEATS=$REPEATS"
echo "BEST_MIN_DISTANCE=$best_min_distance"
echo "BEST_DIR=$best_dir"
