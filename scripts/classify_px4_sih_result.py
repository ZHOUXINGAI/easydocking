#!/usr/bin/env python3

import csv
import math
import os
import sys
from pathlib import Path


START_CLUSTER_A_CENTER = (-4.166384, 105.1457845, -6.485333, -10.2991625)
START_CLUSTER_A_SPREAD = (1.6, 1.2, 0.25, 0.25)
START_CLUSTER_B_CENTER = (-5.395847, 102.9623125, -6.2016955, -10.510685)
START_CLUSTER_B_SPREAD = (0.9, 1.2, 0.22, 0.18)


def _env_float(name: str, default: float) -> float:
    value = (os.getenv(name) or "").strip()
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


FINAL_PASS_PROFILE = (os.getenv("FINAL_PASS_PROFILE") or "v1").strip().lower()

if FINAL_PASS_PROFILE == "loose":
    _DEFAULT_FINAL_PASS_XY_ABS_MAX_M = 0.15
    _DEFAULT_FINAL_PASS_Z_MIN_M = 0.15
    _DEFAULT_FINAL_PASS_Z_MAX_M = 0.55
    _DEFAULT_FINAL_PASS_DISTANCE_MAX_M = 0.40
    _DEFAULT_FINAL_PASS_REL_SPEED_MAX_MPS = 0.60
    _DEFAULT_FINAL_PASS_HOLD_MIN_SEC = 0.30
else:
    _DEFAULT_FINAL_PASS_XY_ABS_MAX_M = 0.10
    _DEFAULT_FINAL_PASS_Z_MIN_M = 0.15
    _DEFAULT_FINAL_PASS_Z_MAX_M = 0.45
    _DEFAULT_FINAL_PASS_DISTANCE_MAX_M = 0.30
    _DEFAULT_FINAL_PASS_REL_SPEED_MAX_MPS = 0.40
    _DEFAULT_FINAL_PASS_HOLD_MIN_SEC = 0.30

FINAL_PASS_XY_ABS_MAX_M = _env_float("FINAL_PASS_XY_ABS_MAX_M", _DEFAULT_FINAL_PASS_XY_ABS_MAX_M)
FINAL_PASS_Z_MIN_M = _env_float("FINAL_PASS_Z_MIN_M", _DEFAULT_FINAL_PASS_Z_MIN_M)
FINAL_PASS_Z_MAX_M = _env_float("FINAL_PASS_Z_MAX_M", _DEFAULT_FINAL_PASS_Z_MAX_M)
FINAL_PASS_DISTANCE_MAX_M = _env_float("FINAL_PASS_DISTANCE_MAX_M", _DEFAULT_FINAL_PASS_DISTANCE_MAX_M)
FINAL_PASS_REL_SPEED_MAX_MPS = _env_float("FINAL_PASS_REL_SPEED_MAX_MPS", _DEFAULT_FINAL_PASS_REL_SPEED_MAX_MPS)
FINAL_PASS_HOLD_MIN_SEC = _env_float("FINAL_PASS_HOLD_MIN_SEC", _DEFAULT_FINAL_PASS_HOLD_MIN_SEC)
FINAL_PASS_PHASES = {"DOCKING", "COMPLETED"}


def _summary_float(summary: dict[str, str] | None, key: str) -> float:
    if not summary:
        return math.nan
    try:
        return float(str(summary.get(key, "")).strip())
    except ValueError:
        return math.nan


def _final_pass_config_from_summary(summary: dict[str, str] | None) -> tuple[dict[str, float | str], str]:
    """Return (cfg, source). Source is 'summary' when cfg keys exist, else 'env'."""
    cfg: dict[str, float | str] = {
        "profile": FINAL_PASS_PROFILE,
        "xy_abs_max_m": FINAL_PASS_XY_ABS_MAX_M,
        "z_min_m": FINAL_PASS_Z_MIN_M,
        "z_max_m": FINAL_PASS_Z_MAX_M,
        "distance_max_m": FINAL_PASS_DISTANCE_MAX_M,
        "rel_speed_max_mps": FINAL_PASS_REL_SPEED_MAX_MPS,
        "hold_min_sec": FINAL_PASS_HOLD_MIN_SEC,
    }

    profile = (summary or {}).get("final_pass_profile")
    if profile:
        cfg["profile"] = str(profile).strip()

    xy = _summary_float(summary, "final_pass_xy_abs_max_m_cfg")
    z_min = _summary_float(summary, "final_pass_z_min_m_cfg")
    z_max = _summary_float(summary, "final_pass_z_max_m_cfg")
    d_max = _summary_float(summary, "final_pass_distance_max_m_cfg")
    v_max = _summary_float(summary, "final_pass_rel_speed_max_mps_cfg")
    hold_min = _summary_float(summary, "final_pass_hold_min_sec_cfg")

    if all(_finite(v) for v in [xy, z_min, z_max, d_max, v_max, hold_min]):
        cfg["xy_abs_max_m"] = float(xy)
        cfg["z_min_m"] = float(z_min)
        cfg["z_max_m"] = float(z_max)
        cfg["distance_max_m"] = float(d_max)
        cfg["rel_speed_max_mps"] = float(v_max)
        cfg["hold_min_sec"] = float(hold_min)
        return cfg, "summary"

    return cfg, "env"


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


def _finite(value: float) -> bool:
    return math.isfinite(value)


def first_non_idle_row(rows: list[dict[str, str]]) -> dict[str, str] | None:
    return next((row for row in rows if row.get("phase") and row.get("phase") != "IDLE"), None)


def last_non_idle_row(rows: list[dict[str, str]]) -> dict[str, str] | None:
    for row in reversed(rows):
        if row.get("phase") and row.get("phase") != "IDLE":
            return row
    return None


def first_phase_row(rows: list[dict[str, str]], phase: str) -> dict[str, str] | None:
    return next((row for row in rows if row.get("phase") == phase), None)


def _row_relative_speed_mps(row: dict[str, str]) -> float:
    speed = safe_float(row, "controller_relative_speed")
    if _finite(speed):
        return speed
    rel_vx = safe_float(row, "rel_vx")
    rel_vy = safe_float(row, "rel_vy")
    rel_vz = safe_float(row, "rel_vz")
    if _finite(rel_vx) and _finite(rel_vy) and _finite(rel_vz):
        return math.sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz)
    return math.nan


def compute_final_pass_hold_time_sec(
    rows: list[dict[str, str]],
    *,
    xy_abs_max_m: float,
    z_min_m: float,
    z_max_m: float,
    distance_max_m: float,
    rel_speed_max_mps: float,
) -> float:
    """Longest continuous time span satisfying FINAL_PASS constraints."""
    best = 0.0
    current = 0.0
    last_t = math.nan
    last_ok = False

    for row in rows:
        phase = (row.get("phase") or "").strip()
        t = safe_float(row, "t")
        rel_x = safe_float(row, "rel_x")
        rel_y = safe_float(row, "rel_y")
        rel_z = safe_float(row, "rel_z")
        distance = safe_float(row, "relative_distance")
        rel_speed = _row_relative_speed_mps(row)

        ok = (
            phase in FINAL_PASS_PHASES and
            _finite(t) and
            _finite(rel_x) and
            _finite(rel_y) and
            _finite(rel_z) and
            _finite(distance) and
            _finite(rel_speed) and
            abs(rel_x) <= xy_abs_max_m and
            abs(rel_y) <= xy_abs_max_m and
            z_min_m <= rel_z <= z_max_m and
            distance <= distance_max_m and
            rel_speed <= rel_speed_max_mps
        )

        if ok and last_ok and _finite(last_t):
            dt = t - last_t
            if dt > 0.0 and dt < 1.0:
                current += dt
        elif ok:
            current = 0.0
        else:
            current = 0.0

        best = max(best, current)
        last_t = t
        last_ok = ok

    return best


def compute_final_pass_metrics(
    rows: list[dict[str, str]],
    summary: dict[str, str] | None = None,
) -> dict[str, object]:
    final_phase = ""
    if summary is not None:
        final_phase = (summary.get("final_phase") or "").strip()

    last_row = last_non_idle_row(rows) or (rows[-1] if rows else {})
    final_rel_x = safe_float(last_row, "rel_x")
    final_rel_y = safe_float(last_row, "rel_y")
    final_rel_z = safe_float(last_row, "rel_z")
    final_distance = safe_float(last_row, "relative_distance")
    final_rel_speed = _row_relative_speed_mps(last_row)
    final_abs_xy_max = (
        max(abs(final_rel_x), abs(final_rel_y))
        if _finite(final_rel_x) and _finite(final_rel_y)
        else math.nan
    )

    cfg, cfg_source = _final_pass_config_from_summary(summary)
    xy_abs_max_m = float(cfg["xy_abs_max_m"])  # type: ignore[arg-type]
    z_min_m = float(cfg["z_min_m"])  # type: ignore[arg-type]
    z_max_m = float(cfg["z_max_m"])  # type: ignore[arg-type]
    distance_max_m = float(cfg["distance_max_m"])  # type: ignore[arg-type]
    rel_speed_max_mps = float(cfg["rel_speed_max_mps"])  # type: ignore[arg-type]
    hold_min_sec = float(cfg["hold_min_sec"])  # type: ignore[arg-type]
    profile = str(cfg["profile"])

    hold_sec = compute_final_pass_hold_time_sec(
        rows,
        xy_abs_max_m=xy_abs_max_m,
        z_min_m=z_min_m,
        z_max_m=z_max_m,
        distance_max_m=distance_max_m,
        rel_speed_max_mps=rel_speed_max_mps,
    )
    final_pass = (
        final_phase == "COMPLETED" and
        _finite(hold_sec) and
        hold_sec >= hold_min_sec
    )

    reasons: list[str] = []
    reasons.append(f"final_profile={profile}")
    reasons.append(f"final_cfg_source={cfg_source}")
    reasons.append(
        "final_cfg "
        f"xy={xy_abs_max_m:.3f} "
        f"z=[{z_min_m:.3f},{z_max_m:.3f}] "
        f"d={distance_max_m:.3f} "
        f"v={rel_speed_max_mps:.3f} "
        f"hold={hold_min_sec:.3f}"
    )
    if final_phase != "COMPLETED":
        reasons.append(f"final_phase={final_phase or 'UNKNOWN'}")
    reasons.append(f"final_hold_sec={hold_sec:.3f}")
    if _finite(final_abs_xy_max):
        reasons.append(f"final_abs_xy_max_m={final_abs_xy_max:.3f}")
    if _finite(final_rel_z):
        reasons.append(f"final_rel_z_m={final_rel_z:.3f}")
    if _finite(final_rel_speed):
        reasons.append(f"final_rel_speed_mps={final_rel_speed:.3f}")
    if _finite(final_distance):
        reasons.append(f"final_distance_m={final_distance:.3f}")

    return {
        "final_pass": 1.0 if final_pass else 0.0,
        "final_pass_hold_sec": hold_sec,
        "final_rel_x_m": final_rel_x,
        "final_rel_y_m": final_rel_y,
        "final_rel_z_m": final_rel_z,
        "final_distance_m": final_distance,
        "final_rel_speed_mps": final_rel_speed,
        "final_abs_xy_max_m": final_abs_xy_max,
        "final_pass_reasons": reasons,
    }


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
        final_metrics = compute_final_pass_metrics(rows, summary)
        final_pass = float(final_metrics.get("final_pass", 0.0)) >= 0.5
        final_reasons = list(final_metrics.get("final_pass_reasons", []))
        if final_pass:
            return "final-pass", [f"min_distance_m={min_distance:.3f}", *final_reasons]
        return "completed-but-not-final", [f"min_distance_m={min_distance:.3f}", *final_reasons]

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
