#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="ignore") as file:
        return list(csv.DictReader(file))


def read_kv(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not path.exists():
        return data
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def parse_log_times(text: str) -> dict[str, float | str]:
    armed = re.search(
        r"\[([0-9.]+)\].*tangent exit armed anchor=\(([-0-9.]+),([-0-9.]+),",
        text,
    )
    cleared = re.search(r"\[([0-9.]+)\].*tangent exit cleared reason=([^\n]+)", text)
    release = re.search(
        r"\[([0-9.]+)\].*glide (?:score|close-tracking|docking) release accepted .*distance=([0-9.]+) "
        r"rel=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)",
        text,
    )
    window = re.search(
        r"\[([0-9.]+)\].*glide window accepted .*phase=([A-Z]+) distance=([0-9.]+) "
        r"rel=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)",
        text,
    )
    data: dict[str, float | str] = {}
    if armed:
        data["armed_stamp"] = float(armed.group(1))
    if cleared:
        data["cleared_stamp"] = float(cleared.group(1))
        data["clear_reason"] = cleared.group(2).strip()
    if release:
        data["release_stamp"] = float(release.group(1))
        data["release_distance"] = float(release.group(2))
        data["release_rel_x"] = float(release.group(3))
        data["release_rel_y"] = float(release.group(4))
        data["release_rel_z"] = float(release.group(5))
    if window:
        data["window_phase"] = window.group(2).strip().upper()
    return data


def find_release_row_index(rows: list[dict[str, str]], log_data: dict[str, float | str]) -> int | None:
    if "release_distance" not in log_data:
        return None
    target_distance = float(log_data["release_distance"])
    target_rel_x = float(log_data["release_rel_x"])
    target_rel_y = float(log_data["release_rel_y"])
    target_rel_z = float(log_data["release_rel_z"])
    return min(
        range(len(rows)),
        key=lambda index: (
            abs(float(rows[index]["relative_distance"]) - target_distance) +
            abs(float(rows[index]["rel_x"]) - target_rel_x) +
            abs(float(rows[index]["rel_y"]) - target_rel_y) +
            abs(float(rows[index]["rel_z"]) - target_rel_z)
        ),
    )


def orbit_radius(row: dict[str, str], orbit_center_x: float, orbit_center_y: float) -> float:
    return math.hypot(float(row["mini_x"]) - orbit_center_x, float(row["mini_y"]) - orbit_center_y)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check tangent-release semantics for one result dir.")
    parser.add_argument("result_dir")
    parser.add_argument("--radius-window-sec", type=float, default=1.2)
    parser.add_argument("--max-radius-drop", type=float, default=0.5)
    parser.add_argument("--min-tangent-duration-sec", type=float, default=0.5)
    parser.add_argument("--max-handoff-min-distance", type=float, default=12.0)
    parser.add_argument("--post-release-window-sec", type=float, default=3.0)
    parser.add_argument("--max-post-release-radius-drop", type=float, default=1.0)
    parser.add_argument("--max-post-release-angle-sweep-deg", type=float, default=30.0)
    parser.add_argument("--max-pre-docking-line-lateral-deviation-m", type=float, default=5.0)
    parser.add_argument("--max-pre-docking-heading-deviation-deg", type=float, default=20.0)
    parser.add_argument("--max-full-line-lateral-deviation-m", type=float, default=8.0)
    parser.add_argument("--max-full-carrier-line-lateral-deviation-m", type=float, default=6.0)
    parser.add_argument("--max-full-line-backslide-m", type=float, default=5.0)
    parser.add_argument("--max-full-carrier-line-backslide-m", type=float, default=15.0)
    args = parser.parse_args()

    result_dir = Path(args.result_dir)
    rows = read_rows(result_dir / "docking_log.csv")
    meta = read_kv(result_dir / "metadata.txt")
    summary = read_kv(result_dir / "summary.txt")
    log_text = (result_dir / "launch.log").read_text(encoding="utf-8", errors="ignore")
    log_data = parse_log_times(log_text)

    if "armed_stamp" not in log_data:
        print("semantic_pass=false")
        print("reason=no_tangent_release")
        return 1

    release_index = find_release_row_index(rows, log_data)
    if release_index is None:
        print("semantic_pass=false")
        print("reason=no_release_row")
        return 1

    orbit_center_x, orbit_center_y = map(float, meta.get("mini_orbit_center", "10.0,-6.0").split(",")[:2])
    release_t = float(rows[release_index]["t"])
    radius_samples: list[tuple[float, float]] = []
    for row in rows[release_index:]:
        dt = float(row["t"]) - release_t
        if dt > args.radius_window_sec:
            break
        radius_samples.append((dt, orbit_radius(row, orbit_center_x, orbit_center_y)))

    radius_at_release = radius_samples[0][1]
    radius_min = min(radius for _, radius in radius_samples)
    radius_drop = radius_at_release - radius_min

    release_x = float(rows[release_index]["mini_x"])
    release_y = float(rows[release_index]["mini_y"])
    carrier_release_x = float(rows[release_index]["carrier_x"])
    carrier_release_y = float(rows[release_index]["carrier_y"])
    release_axis_x = 1.0
    release_axis_y = 0.0
    if 0 < release_index < len(rows) - 1:
        dt = max(float(rows[release_index + 1]["t"]) - float(rows[release_index - 1]["t"]), 1e-6)
        release_vx = (
            float(rows[release_index + 1]["mini_x"]) - float(rows[release_index - 1]["mini_x"])
        ) / dt
        release_vy = (
            float(rows[release_index + 1]["mini_y"]) - float(rows[release_index - 1]["mini_y"])
        ) / dt
        release_speed = math.hypot(release_vx, release_vy)
        if release_speed > 1e-6:
            release_axis_x = release_vx / release_speed
            release_axis_y = release_vy / release_speed
    release_lateral_x = -release_axis_y
    release_lateral_y = release_axis_x

    post_release_samples: list[tuple[float, float, float]] = []
    pre_docking_max_lateral_deviation = 0.0
    pre_docking_max_heading_deviation_deg = 0.0
    full_max_lateral_deviation = 0.0
    full_carrier_max_lateral_deviation = 0.0
    full_max_forward_progress = -math.inf
    full_max_backslide = 0.0
    full_carrier_max_forward_progress = -math.inf
    full_carrier_max_backslide = 0.0
    previous_angle = None
    unwrapped_angle = 0.0
    for index, row in enumerate(rows[release_index:], start=release_index):
        dt = float(row["t"]) - release_t
        dx = float(row["mini_x"]) - orbit_center_x
        dy = float(row["mini_y"]) - orbit_center_y
        radius = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        if dt <= args.post_release_window_sec:
            if previous_angle is None:
                unwrapped_angle = angle
            else:
                delta = angle - previous_angle
                while delta > math.pi:
                    delta -= 2.0 * math.pi
                while delta < -math.pi:
                    delta += 2.0 * math.pi
                unwrapped_angle += delta
            previous_angle = angle
            post_release_samples.append((dt, radius, unwrapped_angle))

        if dt > 0.0 and row["phase"].upper() == "DOCKING":
            break

        if dt >= 0.0:
            line_dx = float(row["mini_x"]) - release_x
            line_dy = float(row["mini_y"]) - release_y
            lateral_error = abs(line_dx * release_lateral_x + line_dy * release_lateral_y)
            full_max_lateral_deviation = max(full_max_lateral_deviation, lateral_error)
            forward_progress = line_dx * release_axis_x + line_dy * release_axis_y
            full_max_forward_progress = max(full_max_forward_progress, forward_progress)
            full_max_backslide = max(
                full_max_backslide,
                full_max_forward_progress - forward_progress,
            )
            carrier_line_dx = float(row["carrier_x"]) - carrier_release_x
            carrier_line_dy = float(row["carrier_y"]) - carrier_release_y
            carrier_lateral_error = abs(
                carrier_line_dx * release_lateral_x + carrier_line_dy * release_lateral_y
            )
            full_carrier_max_lateral_deviation = max(
                full_carrier_max_lateral_deviation,
                carrier_lateral_error,
            )
            carrier_forward_progress = (
                carrier_line_dx * release_axis_x + carrier_line_dy * release_axis_y
            )
            full_carrier_max_forward_progress = max(
                full_carrier_max_forward_progress,
                carrier_forward_progress,
            )
            full_carrier_max_backslide = max(
                full_carrier_max_backslide,
                full_carrier_max_forward_progress - carrier_forward_progress,
            )
            pre_docking_max_lateral_deviation = max(
                pre_docking_max_lateral_deviation,
                lateral_error,
            )

            if 0 < index < len(rows) - 1:
                velocity_dt = max(float(rows[index + 1]["t"]) - float(rows[index - 1]["t"]), 1e-6)
                velocity_x = (
                    float(rows[index + 1]["mini_x"]) - float(rows[index - 1]["mini_x"])
                ) / velocity_dt
                velocity_y = (
                    float(rows[index + 1]["mini_y"]) - float(rows[index - 1]["mini_y"])
                ) / velocity_dt
                velocity_norm = math.hypot(velocity_x, velocity_y)
                if velocity_norm > 1e-6:
                    dot = max(
                        -1.0,
                        min(
                            1.0,
                            (velocity_x / velocity_norm) * release_axis_x +
                            (velocity_y / velocity_norm) * release_axis_y,
                        ),
                    )
                    heading_deviation_deg = math.degrees(math.acos(dot))
                    pre_docking_max_heading_deviation_deg = max(
                        pre_docking_max_heading_deviation_deg,
                        heading_deviation_deg,
                    )

    post_release_radius_min = min(radius for _, radius, _ in post_release_samples)
    post_release_radius_drop = radius_at_release - post_release_radius_min
    angle_sweep_deg = abs(post_release_samples[-1][2] - post_release_samples[0][2]) * 180.0 / math.pi

    tangent_duration = math.inf
    clear_reason = "not_cleared"
    if "cleared_stamp" in log_data:
        tangent_duration = float(log_data["cleared_stamp"]) - float(log_data["armed_stamp"])
        clear_reason = str(log_data.get("clear_reason", "unknown"))

    release_phase = str(log_data.get("window_phase", rows[release_index]["phase"])).upper()
    tangent_release_pass = (
        release_phase in {"TRACKING", "DOCKING"} and
        radius_drop <= args.max_radius_drop and
        tangent_duration >= args.min_tangent_duration_sec and
        post_release_radius_drop <= args.max_post_release_radius_drop and
        angle_sweep_deg <= args.max_post_release_angle_sweep_deg and
        pre_docking_max_lateral_deviation <= args.max_pre_docking_line_lateral_deviation_m and
        pre_docking_max_heading_deviation_deg <= args.max_pre_docking_heading_deviation_deg and
        full_max_lateral_deviation <= args.max_full_line_lateral_deviation_m and
        full_carrier_max_lateral_deviation <= args.max_full_carrier_line_lateral_deviation_m and
        full_max_backslide <= args.max_full_line_backslide_m and
        full_carrier_max_backslide <= args.max_full_carrier_line_backslide_m
    )
    final_phase = summary.get("final_phase", "IDLE").upper()
    try:
        min_distance = float(summary.get("min_distance_m", "inf"))
    except ValueError:
        min_distance = math.inf
    handoff_progress_pass = (
        final_phase in {"TRACKING", "DOCKING", "COMPLETED"} and
        min_distance <= args.max_handoff_min_distance and
        clear_reason != "not_cleared"
    )
    semantic_pass = tangent_release_pass and handoff_progress_pass

    print(f"semantic_pass={'true' if semantic_pass else 'false'}")
    print(f"tangent_release_pass={'true' if tangent_release_pass else 'false'}")
    print(f"handoff_progress_pass={'true' if handoff_progress_pass else 'false'}")
    print(f"release_phase={release_phase}")
    print(f"release_t={release_t:.3f}")
    print(f"tangent_duration_sec={tangent_duration:.3f}")
    print(f"tangent_clear_reason={clear_reason}")
    print(f"final_phase={final_phase}")
    print(f"min_distance_m={min_distance:.3f}")
    print(f"radius_at_release_m={radius_at_release:.3f}")
    print(f"radius_min_window_m={radius_min:.3f}")
    print(f"radius_drop_window_m={radius_drop:.3f}")
    print(f"post_release_radius_drop_m={post_release_radius_drop:.3f}")
    print(f"post_release_angle_sweep_deg={angle_sweep_deg:.3f}")
    print(f"pre_docking_max_line_lateral_deviation_m={pre_docking_max_lateral_deviation:.3f}")
    print(f"pre_docking_max_heading_deviation_deg={pre_docking_max_heading_deviation_deg:.3f}")
    print(f"full_max_line_lateral_deviation_m={full_max_lateral_deviation:.3f}")
    print(f"full_carrier_max_line_lateral_deviation_m={full_carrier_max_lateral_deviation:.3f}")
    print(f"full_max_line_backslide_m={full_max_backslide:.3f}")
    print(f"full_carrier_max_line_backslide_m={full_carrier_max_backslide:.3f}")

    return 0 if semantic_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
