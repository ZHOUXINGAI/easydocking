#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CARRIER_OFFSET_X="${CARRIER_OFFSET_X:-58.0}"
CARRIER_OFFSET_Z="${CARRIER_OFFSET_Z:-0.0}"
OFFSET_X_VALUES=(${OFFSET_X_VALUES:-$CARRIER_OFFSET_X})
OFFSET_Y_VALUES=(${OFFSET_Y_VALUES:--27.0 -25.0 -23.0 -21.0 -19.0})
PHASE_VALUES=(${PHASE_VALUES:-140 160 180 200 220})
MAX_RUNS="${MAX_RUNS:-0}"
SCORE_PHASE_FILTER="${SCORE_PHASE_FILTER:-IDLE}"

best_dir=""
best_score=""
run_count=0

score_result() {
  local result_dir="$1"
  python3 - <<PY
import csv, math, os
path = os.path.join("$result_dir", "docking_log.csv")
phase_filter = "${SCORE_PHASE_FILTER}"
best = None
with open(path) as f:
    for row in csv.DictReader(f):
        if phase_filter != "ANY" and row["phase"] != phase_filter:
            continue
        err = math.sqrt(float(row['rel_x'])**2 + float(row['rel_y'])**2 + (float(row['rel_z']) - 0.8)**2)
        spd = math.sqrt(float(row['rel_vx'])**2 + float(row['rel_vy'])**2 + float(row['rel_vz'])**2)
        vx = abs(float(row['rel_vx']))
        vy = abs(float(row['rel_vy']))
        score = err + 0.25 * spd + 0.18 * vx + 0.12 * vy
        if best is None or score < best[0]:
            best = (score, err, spd, vx, vy, row['t'], row['phase'], row['relative_distance'])
if best is None:
    raise SystemExit("no rows matched score filter")
print(f"{best[0]:.6f} {best[1]:.6f} {best[2]:.6f} {best[3]:.6f} {best[4]:.6f} {best[5]} {best[6]} {best[7]}")
PY
}

for offset_x in "${OFFSET_X_VALUES[@]}"; do
  for offset_y in "${OFFSET_Y_VALUES[@]}"; do
    for phase in "${PHASE_VALUES[@]}"; do
      run_count=$((run_count + 1))
      if [ "$MAX_RUNS" -gt 0 ] && [ "$run_count" -gt "$MAX_RUNS" ]; then
        break 3
      fi

      echo "[sweep $run_count] carrier_offset_x=$offset_x carrier_offset_y=$offset_y mini_orbit_start_phase_deg=$phase"
      result_dir="$(
        CARRIER_OFFSET_X="$offset_x" \
        CARRIER_OFFSET_Y="$offset_y" \
        CARRIER_OFFSET_Z="$CARRIER_OFFSET_Z" \
        MINI_ORBIT_START_PHASE_DEG="$phase" \
        "$ROOT_DIR/scripts/run_px4_sih_docking_experiment.sh" | tail -n 1
      )"

      summary_file="$result_dir/summary.txt"
      min_distance="$(awk -F= '$1=="min_distance_m"{print $2}' "$summary_file")"
      final_phase="$(awk -F= '$1=="final_phase"{print $2}' "$summary_file")"
      metrics="$(score_result "$result_dir")"
      read -r score err spd vx vy best_t best_phase best_dist <<<"$metrics"

      echo "result_dir=$result_dir"
      echo "min_distance_m=$min_distance"
      echo "final_phase=$final_phase"
      echo "best_score=$score"
      echo "best_error=$err"
      echo "best_speed=$spd"
      echo "best_vx=$vx"
      echo "best_vy=$vy"
      echo "best_t=$best_t"
      echo "best_phase=$best_phase"
      echo "best_dist_at_score=$best_dist"

      if [ -z "$best_score" ] || python3 - <<PY
best = float("${best_score:-1e9}")
current = float("$score")
raise SystemExit(0 if current < best else 1)
PY
      then
        best_score="$score"
        best_dir="$result_dir"
      fi
    done
  done
done

echo "BEST_DIR=$best_dir"
echo "BEST_SCORE=$best_score"
