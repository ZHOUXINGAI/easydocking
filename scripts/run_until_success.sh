#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAX_ATTEMPTS="${1:-5}"
SUCCESS_MIN_DISTANCE="${SUCCESS_MIN_DISTANCE:-3.0}"
SUCCESS_MAX_ERROR="${SUCCESS_MAX_ERROR:-0.8}"
SUCCESS_MAX_SPEED="${SUCCESS_MAX_SPEED:-1.5}"
SUCCESS_MAX_VY="${SUCCESS_MAX_VY:-1.0}"

best_dir=""
best_score=""

for ((attempt=1; attempt<=MAX_ATTEMPTS; attempt++)); do
  echo "[attempt $attempt/$MAX_ATTEMPTS] running experiment"
  result_dir="$("$ROOT_DIR/scripts/run_px4_sih_docking_experiment.sh")"
  summary_file="$result_dir/summary.txt"
  log_file="$result_dir/docking_log.csv"

  min_distance="$(awk -F= '$1=="min_distance_m"{print $2}' "$summary_file")"
  final_phase="$(awk -F= '$1=="final_phase"{print $2}' "$summary_file")"
  metrics="$(python3 - <<PY
import csv, math
path = "$log_file"
best = None
with open(path) as f:
    for row in csv.DictReader(f):
        err = math.sqrt(float(row['rel_x'])**2 + float(row['rel_y'])**2 + (float(row['rel_z']) - 0.8)**2)
        spd = math.sqrt(float(row['rel_vx'])**2 + float(row['rel_vy'])**2 + float(row['rel_vz'])**2)
        vy = abs(float(row['rel_vy']))
        score = err + 0.25 * spd + 0.12 * vy
        if best is None or score < best[0]:
            best = (score, err, spd, vy, row['t'], row['phase'], row['relative_distance'])
print(f"{best[0]:.6f} {best[1]:.6f} {best[2]:.6f} {best[3]:.6f} {best[4]} {best[5]} {best[6]}")
PY
)"
  read -r best_attempt_score best_error best_speed best_vy best_t best_phase best_dist_at_score <<<"$metrics"

  echo "result_dir=$result_dir"
  echo "min_distance_m=$min_distance"
  echo "final_phase=$final_phase"
  echo "best_score=$best_attempt_score"
  echo "best_error=$best_error"
  echo "best_speed=$best_speed"
  echo "best_vy=$best_vy"
  echo "best_t=$best_t"
  echo "best_phase=$best_phase"
  echo "best_dist_at_score=$best_dist_at_score"

  if [ -z "$best_score" ] || python3 - <<PY
best = float("${best_score:-1e9}")
current = float("$best_attempt_score")
raise SystemExit(0 if current < best else 1)
PY
  then
    best_score="$best_attempt_score"
    best_dir="$result_dir"
  fi

  if python3 - <<PY
min_distance = float("$min_distance")
final_phase = "$final_phase".strip().upper()
best_error = float("$best_error")
best_speed = float("$best_speed")
best_vy = float("$best_vy")
success_distance = float("$SUCCESS_MIN_DISTANCE")
success_error = float("$SUCCESS_MAX_ERROR")
success_speed = float("$SUCCESS_MAX_SPEED")
success_vy = float("$SUCCESS_MAX_VY")
strict_success = best_error <= success_error and best_speed <= success_speed and best_vy <= success_vy
legacy_success = final_phase == "COMPLETED" or min_distance <= success_distance
raise SystemExit(0 if (strict_success or legacy_success) else 1)
PY
  then
    echo "SUCCESS $result_dir"
    exit 0
  fi
done

echo "BEST $best_dir"
echo "BEST_SCORE $best_score"
exit 1
