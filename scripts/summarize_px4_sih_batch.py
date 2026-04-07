#!/usr/bin/env python3

import argparse
import csv
import glob
import math
from pathlib import Path

from classify_px4_sih_result import (
    best_distance_row,
    classify_result,
    compute_start_cluster_scores,
    compute_start_metrics,
    first_non_idle_row,
    first_phase_row,
    load_summary,
    post_start_energy_stats,
    safe_float,
)

HOLD_Z_MIN_M = 0.25
HOLD_Z_MAX_M = 0.95
TERMINAL_EVAL_MAX_DISTANCE_M = 10.0
MIN_LAT_EVAL_Z_MIN_M = 0.20
MIN_LAT_EVAL_Z_MAX_M = 1.20


def _safe_phase(row: dict[str, str] | None) -> str:
    if not row:
        return ""
    return (row.get("phase") or "").strip()


def _finite(value: float) -> bool:
    return math.isfinite(value)


def compute_min_terminal_lateral_error(rows: list[dict[str, str]]) -> float:
    values: list[float] = []
    for row in rows:
        if _safe_phase(row) == "IDLE":
            continue
        distance = safe_float(row, "relative_distance")
        rel_z = safe_float(row, "rel_z")
        if not (_finite(distance) and _finite(rel_z)):
            continue
        # Avoid counting coincidental small lateral error when the vehicles are still far apart
        # or at irrelevant altitude offsets (e.g. in early APPROACH).
        if distance > TERMINAL_EVAL_MAX_DISTANCE_M:
            continue
        if not (MIN_LAT_EVAL_Z_MIN_M <= rel_z <= MIN_LAT_EVAL_Z_MAX_M):
            continue
        lateral = safe_float(row, "controller_terminal_lateral_error")
        if _finite(lateral):
            values.append(abs(lateral))
    if not values:
        return math.nan
    return min(values)


def compute_hold_time_sec(
    rows: list[dict[str, str]],
    *,
    lateral_threshold_m: float,
    z_min_m: float = HOLD_Z_MIN_M,
    z_max_m: float = HOLD_Z_MAX_M,
) -> float:
    """Longest continuous time span satisfying abs(lat)<=threshold AND z-band."""
    best = 0.0
    current = 0.0
    last_t = math.nan
    last_ok = False

    for row in rows:
        phase = _safe_phase(row)
        t = safe_float(row, "t")
        lat = safe_float(row, "controller_terminal_lateral_error")
        z = safe_float(row, "rel_z")
        distance = safe_float(row, "relative_distance")

        ok = (
            phase != "IDLE" and
            _finite(t) and
            _finite(lat) and
            _finite(z) and
            _finite(distance) and
            distance <= TERMINAL_EVAL_MAX_DISTANCE_M and
            abs(lat) <= lateral_threshold_m and
            z_min_m <= z <= z_max_m
        )

        if ok and last_ok and _finite(last_t):
            dt = t - last_t
            if dt > 0.0 and dt < 1.0:  # guard against timestamp resets/spikes
                current += dt
        elif ok:
            current = 0.0
        else:
            current = 0.0

        best = max(best, current)
        last_t = t
        last_ok = ok

    return best


def compute_docking_entry_metrics(rows: list[dict[str, str]]) -> tuple[int, float, float]:
    """Return (docking_entry_count, first_entry_lat_abs_m, second_entry_lat_abs_m)."""
    entry_lats: list[float] = []
    prev_phase = ""
    for row in rows:
        phase = _safe_phase(row)
        if phase == "DOCKING" and prev_phase != "DOCKING":
            lateral = safe_float(row, "controller_terminal_lateral_error")
            if _finite(lateral):
                entry_lats.append(abs(lateral))
            else:
                entry_lats.append(math.nan)
        if phase:
            prev_phase = phase

    count = len(entry_lats)
    first = entry_lats[0] if count >= 1 else math.nan
    second = entry_lats[1] if count >= 2 else math.nan
    return count, first, second


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--latest",
        type=int,
        default=0,
        help="Only process the newest N result dirs (0 = all).",
    )
    parser.add_argument(
        "--pattern",
        default="results/*_px4_sih",
        help="Glob pattern for result dirs.",
    )
    args = parser.parse_args()

    results = sorted(glob.glob(args.pattern))
    if args.latest and args.latest > 0:
        results = results[-args.latest :]
    out_path = Path("results") / "px4_sih_batch_summary.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "run",
            "classification",
            "classification_reasons",
            "final_phase",
            "min_distance_m",
            "start_phase",
            "start_t",
            "start_rel_x",
            "start_rel_y",
            "start_rel_z",
            "start_rel_vx",
            "start_rel_vy",
            "start_rel_vz",
            "start_distance_xy_m",
            "start_speed_xy_mps",
            "start_alignment",
            "start_tca_sec",
            "start_cluster_a_score",
            "start_cluster_b_score",
            "start_cluster_min_score",
            "first_docking_t",
            "first_docking_phase",
            "first_docking_distance_m",
            "first_docking_rel_x",
            "first_docking_rel_y",
            "first_docking_rel_z",
            "first_docking_rel_speed_mps",
            "first_docking_along_error",
            "first_docking_lateral_error",
            "first_docking_vertical_error",
            "best_t",
            "best_phase",
            "best_distance_m",
            "best_rel_speed_mps",
            "best_along_error",
            "best_lateral_error",
            "best_vertical_error",
            "best_active_t",
            "best_active_phase",
            "best_active_distance_m",
            "best_active_rel_speed_mps",
            "best_active_along_error",
            "best_active_lateral_error",
            "best_active_vertical_error",
            "post_start_energy_bad_rows",
            "post_start_energy_bad_streak",
            "post_start_energy_first_bad_t",
            "post_start_energy_first_bad_phase",
            "post_start_energy_first_bad_tas_mps",
            "post_start_energy_first_bad_underspeed_ratio",
            "best_energy_guard_active",
            "max_underspeed_ratio",
            "min_tas_mps",
            "best_lateral_m",
            "min_terminal_lateral_error_m",
            "hold_lat_0p35_zband_sec",
            "hold_lat_0p2_zband_sec",
            "docking_entry_count",
            "first_docking_entry_lat_abs_m",
            "second_docking_entry_lat_abs_m",
            "second_docking_entry_lateral_improvement_m",
            "second_docking_entry_lateral_improved",
        ])

        for result_dir_str in results:
            result_dir = Path(result_dir_str)
            summary = load_summary(result_dir / "summary.txt")
            rows_path = result_dir / "docking_log.csv"
            if not rows_path.exists():
                continue
            with rows_path.open() as rf:
                rows = list(csv.DictReader(rf))
            if not rows:
                continue

            classification, reasons = classify_result(result_dir)
            start = first_non_idle_row(rows)
            start_metrics = compute_start_metrics(start)
            start_cluster_scores = compute_start_cluster_scores(start)
            first_docking = first_phase_row(rows, "DOCKING")
            best = best_distance_row(rows, active_only=False)
            best_active = best_distance_row(rows, active_only=True)
            energy_stats = post_start_energy_stats(rows)
            first_bad_energy = energy_stats["first_bad_row"]
            min_terminal_lateral_error_m = compute_min_terminal_lateral_error(rows)
            hold_lat_0p35_zband_sec = compute_hold_time_sec(rows, lateral_threshold_m=0.35)
            hold_lat_0p2_zband_sec = compute_hold_time_sec(rows, lateral_threshold_m=0.20)
            docking_entry_count, first_entry_lat, second_entry_lat = compute_docking_entry_metrics(rows)
            second_entry_improvement_m = math.nan
            second_entry_improved = math.nan
            if _finite(first_entry_lat) and _finite(second_entry_lat):
                second_entry_improvement_m = first_entry_lat - second_entry_lat
                second_entry_improved = 1.0 if second_entry_lat < first_entry_lat else 0.0

            writer.writerow([
                result_dir.name,
                classification,
                " | ".join(reasons),
                summary.get("final_phase", ""),
                summary.get("min_distance_m", ""),
                (start or {}).get("phase", ""),
                safe_float(start or {}, "t"),
                safe_float(start or {}, "rel_x"),
                safe_float(start or {}, "rel_y"),
                safe_float(start or {}, "rel_z"),
                safe_float(start or {}, "rel_vx"),
                safe_float(start or {}, "rel_vy"),
                safe_float(start or {}, "rel_vz"),
                start_metrics["distance_xy"],
                start_metrics["speed_xy"],
                start_metrics["alignment"],
                start_metrics["tca_sec"],
                start_cluster_scores["cluster_a_score"],
                start_cluster_scores["cluster_b_score"],
                start_cluster_scores["cluster_min_score"],
                safe_float(first_docking or {}, "t"),
                (first_docking or {}).get("phase", ""),
                safe_float(first_docking or {}, "relative_distance"),
                safe_float(first_docking or {}, "rel_x"),
                safe_float(first_docking or {}, "rel_y"),
                safe_float(first_docking or {}, "rel_z"),
                safe_float(first_docking or {}, "controller_relative_speed"),
                safe_float(first_docking or {}, "controller_terminal_along_error"),
                safe_float(first_docking or {}, "controller_terminal_lateral_error"),
                safe_float(first_docking or {}, "controller_terminal_vertical_error"),
                safe_float(best or {}, "t"),
                (best or {}).get("phase", ""),
                safe_float(best or {}, "relative_distance"),
                safe_float(best or {}, "controller_relative_speed"),
                safe_float(best or {}, "controller_terminal_along_error"),
                safe_float(best or {}, "controller_terminal_lateral_error"),
                safe_float(best or {}, "controller_terminal_vertical_error"),
                safe_float(best_active or {}, "t"),
                (best_active or {}).get("phase", ""),
                safe_float(best_active or {}, "relative_distance"),
                safe_float(best_active or {}, "controller_relative_speed"),
                safe_float(best_active or {}, "controller_terminal_along_error"),
                safe_float(best_active or {}, "controller_terminal_lateral_error"),
                safe_float(best_active or {}, "controller_terminal_vertical_error"),
                int(energy_stats["bad_rows"]),
                int(energy_stats["max_bad_streak"]),
                safe_float(first_bad_energy or {}, "t"),
                (first_bad_energy or {}).get("phase", ""),
                safe_float(first_bad_energy or {}, "mini_energy_guard_true_airspeed_mps"),
                safe_float(first_bad_energy or {}, "mini_energy_guard_underspeed_ratio"),
                safe_float(best or {}, "mini_energy_guard_active"),
                summary.get("mini_tecs_max_underspeed_ratio", ""),
                summary.get("mini_tecs_min_tas_mps", ""),
                min_terminal_lateral_error_m,
                min_terminal_lateral_error_m,
                hold_lat_0p35_zband_sec,
                hold_lat_0p2_zband_sec,
                docking_entry_count,
                first_entry_lat,
                second_entry_lat,
                second_entry_improvement_m,
                second_entry_improved,
            ])

    rows = list(csv.DictReader(out_path.open()))
    counts: dict[str, int] = {}
    for row in rows:
        counts[row["classification"]] = counts.get(row["classification"], 0) + 1
    counts_str = " ".join(f"{key}={counts[key]}" for key in sorted(counts))
    print(f"{out_path} {counts_str}".strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
