#!/usr/bin/env python3

import argparse
import csv
import glob
import io
import math
from pathlib import Path

from classify_px4_sih_result import (
    best_distance_row,
    classify_result,
    compute_tracking_entry_window_metrics,
    compute_final_pass_metrics,
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
            "final_pass",
            "final_pass_hold_sec",
            "final_pass_loose",
            "final_pass_hold_sec_loose",
            "final_rel_x_m",
            "final_rel_y_m",
            "final_rel_z_m",
            "final_distance_m",
            "final_rel_speed_mps",
            "final_abs_xy_max_m",
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
            "start_carrier_z_m",
            "start_carrier_vz_mps",
            "start_prehold_required",
            "start_prehold_min_altitude_m",
            "start_prehold_max_abs_vz_mps",
            "start_prehold_ready",
            "start_cluster_a_score",
            "start_cluster_b_score",
            "start_cluster_min_score",
            "start_to_completed_sec",
            "time_to_first_tracking_sec",
            "time_to_first_docking_sec",
            "time_to_corridor_sec",
            "post_start_path_length_m",
            "post_start_10s_path_length_m",
            "post_start_10s_net_displacement_m",
            "post_start_10s_path_efficiency_ratio",
            "post_start_20s_path_length_m",
            "post_start_20s_net_displacement_m",
            "post_start_20s_path_efficiency_ratio",
            "docking_path_length_m",
            "start_prediction_horizon_sec",
            "start_prediction_carrier_speed_mps",
            "start_prediction_tangent_weight",
            "start_intercept_target_t_sec",
            "start_intercept_mini_pred_x_m",
            "start_intercept_mini_pred_y_m",
            "start_intercept_tangent_heading_deg",
            "start_intercept_guard_heading_deg",
            "start_intercept_pred_along_m",
            "start_intercept_pred_lat_m",
            "start_intercept_pred_score",
            "start_intercept_gate_feasibility_margin_m",
            "accepted_window_dist_xy_m",
            "accepted_window_rel_z_m",
            "accepted_window_carrier_z_m",
            "accepted_window_carrier_vz_mps",
            "accepted_window_prehold_ok",
            "accepted_window_speed_xy_mps",
            "accepted_window_align",
            "accepted_window_tca_sec",
            "accepted_window_ahead_m",
            "accepted_window_phase_err_deg",
            "accepted_window_pred_along_m",
            "accepted_window_pred_lat_m",
            "accepted_window_pred_score",
            "accepted_window_int_tau_sec",
            "accepted_window_int_route_m",
            "accepted_window_int_cos",
            "accepted_window_int_margin_mps",
            "accepted_window_int_front_ok",
            "accepted_window_diag_tau_sec",
            "accepted_window_diag_route_m",
            "accepted_window_diag_cos",
            "accepted_window_diag_margin_mps",
            "accepted_window_diag_lat_m",
            "accepted_window_diag_score",
            "accepted_window_diag_path_ratio",
            "accepted_window_diag_route_ok",
            "accepted_window_diag_speed_ok",
            "accepted_window_diag_cand_count",
            "accepted_window_diag_route_rej_count",
            "accepted_window_diag_speed_rej_count",
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
            "tracking_nearest_t_sec",
            "tracking_nearest_terminal_distance_m",
            "tracking_nearest_along_error_m",
            "tracking_nearest_lateral_error_m",
            "tracking_nearest_lateral_abs_m",
            "tracking_nearest_rel_z_m",
            "tracking_behind_rows",
            "tracking_behind_best_t_sec",
            "tracking_behind_best_terminal_distance_m",
            "tracking_behind_best_along_error_m",
            "tracking_behind_best_lateral_error_m",
            "tracking_behind_best_lateral_abs_m",
            "tracking_behind_best_rel_z_m",
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
            raw_text = rows_path.read_text(encoding="utf-8", errors="ignore")
            if "\x00" in raw_text:
                raw_text = raw_text.replace("\x00", "")
            rows = list(csv.DictReader(io.StringIO(raw_text)))
            if not rows:
                continue

            classification, reasons = classify_result(result_dir)
            final_metrics = compute_final_pass_metrics(rows, summary)
            start = first_non_idle_row(rows)
            start_metrics = compute_start_metrics(start)
            start_cluster_scores = compute_start_cluster_scores(start)
            first_docking = first_phase_row(rows, "DOCKING")
            best = best_distance_row(rows, active_only=False)
            best_active = best_distance_row(rows, active_only=True)
            energy_stats = post_start_energy_stats(rows)
            tracking_entry = compute_tracking_entry_window_metrics(rows)
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
                final_metrics.get("final_pass", math.nan),
                final_metrics.get("final_pass_hold_sec", math.nan),
                final_metrics.get("final_pass_loose", math.nan),
                final_metrics.get("final_pass_hold_sec_loose", math.nan),
                final_metrics.get("final_rel_x_m", math.nan),
                final_metrics.get("final_rel_y_m", math.nan),
                final_metrics.get("final_rel_z_m", math.nan),
                final_metrics.get("final_distance_m", math.nan),
                final_metrics.get("final_rel_speed_mps", math.nan),
                final_metrics.get("final_abs_xy_max_m", math.nan),
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
                summary.get("start_carrier_z_m", ""),
                summary.get("start_carrier_vz_mps", ""),
                summary.get("start_prehold_required", ""),
                summary.get("start_prehold_min_altitude_m", ""),
                summary.get("start_prehold_max_abs_vz_mps", ""),
                summary.get("start_prehold_ready", ""),
                start_cluster_scores["cluster_a_score"],
                start_cluster_scores["cluster_b_score"],
                start_cluster_scores["cluster_min_score"],
                summary.get("start_to_completed_sec", ""),
                summary.get("time_to_first_tracking_sec", ""),
                summary.get("time_to_first_docking_sec", ""),
                summary.get("time_to_corridor_sec", ""),
                summary.get("post_start_path_length_m", ""),
                summary.get("post_start_10s_path_length_m", ""),
                summary.get("post_start_10s_net_displacement_m", ""),
                summary.get("post_start_10s_path_efficiency_ratio", ""),
                summary.get("post_start_20s_path_length_m", ""),
                summary.get("post_start_20s_net_displacement_m", ""),
                summary.get("post_start_20s_path_efficiency_ratio", ""),
                summary.get("docking_path_length_m", ""),
                summary.get("start_prediction_horizon_sec", ""),
                summary.get("start_prediction_carrier_speed_mps", ""),
                summary.get("start_prediction_tangent_weight", ""),
                summary.get("start_intercept_target_t_sec", ""),
                summary.get("start_intercept_mini_pred_x_m", ""),
                summary.get("start_intercept_mini_pred_y_m", ""),
                summary.get("start_intercept_tangent_heading_deg", ""),
                summary.get("start_intercept_guard_heading_deg", ""),
                summary.get("start_intercept_pred_along_m", ""),
                summary.get("start_intercept_pred_lat_m", ""),
                summary.get("start_intercept_pred_score", ""),
                summary.get("start_intercept_gate_feasibility_margin_m", ""),
                summary.get("accepted_window_dist_xy", ""),
                summary.get("accepted_window_rel_z", ""),
                summary.get("accepted_window_carrier_z", ""),
                summary.get("accepted_window_carrier_vz", ""),
                summary.get("accepted_window_prehold_ok", ""),
                summary.get("accepted_window_speed_xy", ""),
                summary.get("accepted_window_align", ""),
                summary.get("accepted_window_tca", ""),
                summary.get("accepted_window_ahead", ""),
                summary.get("accepted_window_phase_err_deg", ""),
                summary.get("accepted_window_pred_along", ""),
                summary.get("accepted_window_pred_lat", ""),
                summary.get("accepted_window_pred_score", ""),
                summary.get("accepted_window_int_tau", ""),
                summary.get("accepted_window_int_route", ""),
                summary.get("accepted_window_int_cos", ""),
                summary.get("accepted_window_int_margin", ""),
                summary.get("accepted_window_int_front_ok", ""),
                summary.get("accepted_window_diag_tau", ""),
                summary.get("accepted_window_diag_route", ""),
                summary.get("accepted_window_diag_cos", ""),
                summary.get("accepted_window_diag_margin", ""),
                summary.get("accepted_window_diag_lat", ""),
                summary.get("accepted_window_diag_score", ""),
                summary.get("accepted_window_diag_path", ""),
                summary.get("accepted_window_diag_route_ok", ""),
                summary.get("accepted_window_diag_speed_ok", ""),
                summary.get("accepted_window_diag_cand", ""),
                summary.get("accepted_window_diag_route_rej", ""),
                summary.get("accepted_window_diag_speed_rej", ""),
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
                tracking_entry["tracking_nearest_t_sec"],
                tracking_entry["tracking_nearest_terminal_distance_m"],
                tracking_entry["tracking_nearest_along_error_m"],
                tracking_entry["tracking_nearest_lateral_error_m"],
                tracking_entry["tracking_nearest_lateral_abs_m"],
                tracking_entry["tracking_nearest_rel_z_m"],
                tracking_entry["tracking_behind_rows"],
                tracking_entry["tracking_behind_best_t_sec"],
                tracking_entry["tracking_behind_best_terminal_distance_m"],
                tracking_entry["tracking_behind_best_along_error_m"],
                tracking_entry["tracking_behind_best_lateral_error_m"],
                tracking_entry["tracking_behind_best_lateral_abs_m"],
                tracking_entry["tracking_behind_best_rel_z_m"],
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
