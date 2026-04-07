#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path


START_CLUSTER_A_CENTER = (-4.166384, 105.1457845, -6.485333, -10.2991625)
START_CLUSTER_A_SPREAD = (1.6, 1.2, 0.25, 0.25)
START_CLUSTER_B_CENTER = (-5.395847, 102.9623125, -6.2016955, -10.510685)
START_CLUSTER_B_SPREAD = (0.9, 1.2, 0.22, 0.18)


def load_summary(path: Path) -> dict[str, str]:
    summary: dict[str, str] = {}
    if not path.exists():
        return summary
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        summary[key.strip()] = value.strip()
    return summary


def load_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def safe_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def first_non_idle_row(rows: list[dict[str, str]]) -> dict[str, str] | None:
    return next((row for row in rows if row.get("phase") and row.get("phase") != "IDLE"), None)


def first_phase_row(rows: list[dict[str, str]], phase: str) -> dict[str, str] | None:
    return next((row for row in rows if row.get("phase") == phase), None)


def best_distance_row(
    rows: list[dict[str, str]],
    *,
    active_only: bool = False,
) -> dict[str, str] | None:
    candidates = rows
    if active_only:
        candidates = [row for row in rows if row.get("phase") and row.get("phase") != "IDLE"]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda row: safe_float(row, "relative_distance", 1e9),
    )


def compute_start_metrics(row: dict[str, str] | None) -> dict[str, float]:
    if row is None:
        return {
            "distance_xy": math.nan,
            "speed_xy": math.nan,
            "alignment": math.nan,
            "tca_sec": math.nan,
        }

    rel_x = safe_float(row, "rel_x")
    rel_y = safe_float(row, "rel_y")
    rel_vx = safe_float(row, "rel_vx")
    rel_vy = safe_float(row, "rel_vy")
    distance_xy = math.hypot(rel_x, rel_y)
    speed_xy = math.hypot(rel_vx, rel_vy)
    if distance_xy < 1e-6 or speed_xy < 1e-6:
        alignment = math.nan
        tca_sec = math.nan
    else:
        alignment = (-rel_x * rel_vx - rel_y * rel_vy) / (distance_xy * speed_xy)
        tca_sec = -(rel_x * rel_vx + rel_y * rel_vy) / max(speed_xy ** 2, 1e-6)
    return {
        "distance_xy": distance_xy,
        "speed_xy": speed_xy,
        "alignment": alignment,
        "tca_sec": tca_sec,
    }


def _normalized_cluster_score(
    values: tuple[float, float, float, float],
    center: tuple[float, float, float, float],
    spread: tuple[float, float, float, float],
) -> float:
    score = 0.0
    for value, target, scale in zip(values, center, spread):
        if not math.isfinite(value):
            return math.inf
        score += ((value - target) / max(scale, 1e-6)) ** 2
    return score


def compute_start_cluster_scores(row: dict[str, str] | None) -> dict[str, float]:
    if row is None:
        return {
            "cluster_a_score": math.nan,
            "cluster_b_score": math.nan,
            "cluster_min_score": math.nan,
        }

    sample = (
        safe_float(row, "rel_x"),
        safe_float(row, "rel_y"),
        safe_float(row, "rel_vx"),
        safe_float(row, "rel_vy"),
    )
    score_a = _normalized_cluster_score(sample, START_CLUSTER_A_CENTER, START_CLUSTER_A_SPREAD)
    score_b = _normalized_cluster_score(sample, START_CLUSTER_B_CENTER, START_CLUSTER_B_SPREAD)
    return {
        "cluster_a_score": score_a,
        "cluster_b_score": score_b,
        "cluster_min_score": min(score_a, score_b),
    }


def _energy_guard_bad_row(row: dict[str, str]) -> bool:
    return (
        safe_float(row, "mini_energy_guard_bad_airspeed", 0.0) >= 0.5 or
        safe_float(row, "mini_energy_guard_bad_underspeed", 0.0) >= 0.5
    )


def post_start_energy_stats(rows: list[dict[str, str]]) -> dict[str, object]:
    active_rows = [row for row in rows if row.get("phase") and row.get("phase") != "IDLE"]
    bad_rows = [row for row in active_rows if _energy_guard_bad_row(row)]
    max_bad_streak = 0
    current_streak = 0
    for row in active_rows:
        if _energy_guard_bad_row(row):
            current_streak += 1
            max_bad_streak = max(max_bad_streak, current_streak)
        else:
            current_streak = 0

    worst_row = None
    if bad_rows:
        worst_row = max(
            bad_rows,
            key=lambda row: (
                safe_float(row, "mini_energy_guard_recover_time_remaining_sec", 0.0),
                safe_float(row, "mini_energy_guard_underspeed_ratio", 0.0),
                -safe_float(row, "mini_energy_guard_true_airspeed_mps", math.inf),
            ),
        )

    return {
        "active_rows": len(active_rows),
        "bad_rows": len(bad_rows),
        "max_bad_streak": max_bad_streak,
        "first_bad_row": bad_rows[0] if bad_rows else None,
        "worst_bad_row": worst_row,
    }


def classify_result(result_dir: Path) -> tuple[str, list[str]]:
    summary = load_summary(result_dir / "summary.txt")
    rows = load_rows(result_dir / "docking_log.csv")
    start_log = (result_dir / "start_command.log").read_text() if (result_dir / "start_command.log").exists() else ""

    reasons: list[str] = []
    final_phase = summary.get("final_phase", "")
    min_distance = float(summary.get("min_distance_m", "nan"))

    if final_phase == "COMPLETED":
        return "completed", [f"min_distance_m={min_distance:.3f}"]

    if "Timeout waiting for docking window" in start_log or "Timeout waiting for mini-energy health" in start_log:
        reasons.append("starter_timeout")
        if "mini-energy health" in start_log:
            reasons.append("prestart_energy_health_blocked")
        return "start-window-fail", reasons

    start = first_non_idle_row(rows)
    if start is None:
        reasons.append("never_left_idle")
        if "Auto START sent" in start_log:
            reasons.append("start_sent_but_no_active_phase")
        return "start-window-fail", reasons

    energy_stats = post_start_energy_stats(rows)
    worst_energy_row = energy_stats["worst_bad_row"]
    if worst_energy_row is not None:
        reasons.append(
            "post_start_energy_guard "
            f"rows={int(energy_stats['bad_rows'])} "
            f"streak={int(energy_stats['max_bad_streak'])} "
            f"t={safe_float(worst_energy_row, 't', math.nan):.3f} "
            f"phase={worst_energy_row.get('phase', '')} "
            f"tas={safe_float(worst_energy_row, 'mini_energy_guard_true_airspeed_mps', math.nan):.2f} "
            f"underspeed={safe_float(worst_energy_row, 'mini_energy_guard_underspeed_ratio', math.nan):.3f}"
        )
        return "energy-fail", reasons

    start_metrics = compute_start_metrics(start)
    start_scores = compute_start_cluster_scores(start)
    reasons.append(
        "start "
        f"cluster={start_scores['cluster_min_score']:.3f} "
        f"dist_xy={start_metrics['distance_xy']:.3f} "
        f"speed_xy={start_metrics['speed_xy']:.3f} "
        f"align={start_metrics['alignment']:.3f} "
        f"tca={start_metrics['tca_sec']:.3f}"
    )

    first_docking = first_phase_row(rows, "DOCKING")
    if first_docking is not None:
        reasons.append(
            "first_docking "
            f"distance={safe_float(first_docking, 'relative_distance', math.nan):.3f} "
            f"along={safe_float(first_docking, 'controller_terminal_along_error', math.nan):.3f} "
            f"lat={safe_float(first_docking, 'controller_terminal_lateral_error', math.nan):.3f} "
            f"vz={safe_float(first_docking, 'rel_vz', math.nan):.3f}"
        )

    best_active = best_distance_row(rows, active_only=True)
    if best_active is not None:
        reasons.append(
            "best_active "
            f"distance={safe_float(best_active, 'relative_distance', math.nan):.3f} "
            f"phase={best_active.get('phase', '')} "
            f"along={safe_float(best_active, 'controller_terminal_along_error', math.nan):.3f} "
            f"lat={safe_float(best_active, 'controller_terminal_lateral_error', math.nan):.3f} "
            f"speed={safe_float(best_active, 'controller_relative_speed', math.nan):.3f}"
        )
    elif rows:
        best = best_distance_row(rows, active_only=False)
        reasons.append(
            "best "
            f"distance={safe_float(best or {}, 'relative_distance', math.nan):.3f} "
            f"phase={(best or {}).get('phase', '')}"
        )

    return "geometry-fail", reasons


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: classify_px4_sih_result.py <result_dir>", file=sys.stderr)
        return 2

    result_dir = Path(sys.argv[1]).resolve()
    classification, reasons = classify_result(result_dir)
    print(f"classification={classification}")
    for reason in reasons:
        print(f"reason={reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
