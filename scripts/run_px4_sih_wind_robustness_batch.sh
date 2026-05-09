#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNS_PER_LEVEL="${RUNS_PER_LEVEL:-2}"
EXPERIMENT_DURATION_SEC="${EXPERIMENT_DURATION_SEC:-95.0}"

sample_direction() {
  local speed="$1"
  local seed="$2"
  python3 - <<PY
import math
import random
rng = random.Random(${seed})
theta = rng.uniform(0.0, 2.0 * math.pi)
speed = float(${speed})
print(f"{speed * math.cos(theta):.3f} {speed * math.sin(theta):.3f}")
PY
}

run_one() {
  local label="$1"
  local speed="$2"
  local gust_enable="$3"
  local gust_amp="$4"
  local step_events="$5"
  local run_index="$6"

  local wind_n wind_e
  local dir_seed=$((1701 + run_index * 31))
  read -r wind_n wind_e <<<"$(sample_direction "$speed" "$dir_seed")"

  echo "=== level=${label} run=${run_index} base=(${wind_n},${wind_e}) gust=${gust_enable}/${gust_amp} step='${step_events}'"
  (
    cd "$ROOT_DIR"
    ENABLE_WIND_PROFILE=true \
    WIND_BASE_N="$wind_n" \
    WIND_BASE_E="$wind_e" \
    WIND_GUST_ENABLE="$gust_enable" \
    WIND_GUST_AMP="$gust_amp" \
    WIND_GUST_CHANGE_SEC="${WIND_GUST_CHANGE_SEC:-1.5}" \
    WIND_STEP_EVENTS="$step_events" \
    WIND_UPDATE_PERIOD_SEC="${WIND_UPDATE_PERIOD_SEC:-0.5}" \
    WIND_SEED="$((2400 + run_index))" \
    EXPERIMENT_DURATION_SEC="$EXPERIMENT_DURATION_SEC" \
    ./scripts/run_px4_sih_docking_experiment.sh
  )
}

echo "[wind-batch] runs_per_level=${RUNS_PER_LEVEL}"

for idx in $(seq 1 "$RUNS_PER_LEVEL"); do
  run_one "steady_lvl1" "2.0" "false" "0.0" "" "$idx"
done

for idx in $(seq 1 "$RUNS_PER_LEVEL"); do
  run_one "gust_lvl2" "3.0" "true" "1.5" "" "$((100 + idx))"
done

for idx in $(seq 1 "$RUNS_PER_LEVEL"); do
  run_one "step_gust_lvl3" "3.0" "true" "2.0" "35:2.0,-1.5;60:-2.5,2.0" "$((200 + idx))"
done

echo "[wind-batch] evaluate latest $((RUNS_PER_LEVEL * 3)) windy runs"
python3 "$ROOT_DIR/scripts/evaluate_wind_robustness.py" --latest "$((RUNS_PER_LEVEL * 3))"
