#!/usr/bin/env python3

import csv
import io
import math
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def parse_corridor_plan_from_log(log_path: Path):
    """Extract CorridorPlan data from launch.log (old or new format)."""
    if not log_path.exists():
        return None
    text = log_path.read_text(encoding="utf-8", errors="ignore")
    # New format with arc params
    m = re.search(
        r"CorridorPlan published: T=\(([^)]+)\) dir=\(([^)]+)\) "
        r"hold=([\d.]+)s spd=([\d.]+)m/s "
        r"arc_M=\(([^)]+)\) arc_r=([\d.]+) arc_phi0=([\d.-]+) arc_dphi=([\d.-]+) "
        r"trigger_phase=([\d.]+)deg",
        text,
    )
    if m:
        tx, ty = [float(v) for v in m.group(1).split(",")]
        dx, dy = [float(v) for v in m.group(2).split(",")]
        mx, my = [float(v) for v in m.group(5).split(",")]
        return {
            "T": (tx, ty), "dir": (dx, dy),
            "hold_sec": float(m.group(3)), "spd": float(m.group(4)),
            "arc_M": (mx, my), "arc_r": float(m.group(6)),
            "arc_phi0": float(m.group(7)), "arc_dphi": float(m.group(8)),
            "trigger_phase_deg": float(m.group(9)),
        }
    # Old format (no arc)
    m = re.search(
        r"CorridorPlan published: T=\(([^)]+)\) dir=\(([^)]+)\) "
        r"hold=([\d.]+)s trigger_phase=([\d.]+)deg",
        text,
    )
    if not m:
        return None
    tx, ty = [float(v) for v in m.group(1).split(",")]
    dx, dy = [float(v) for v in m.group(2).split(",")]
    return {
        "T": (tx, ty), "dir": (dx, dy),
        "hold_sec": float(m.group(3)),
        "trigger_phase_deg": float(m.group(4)),
    }


def load_rows(csv_path: Path):
    rows = []
    raw_text = csv_path.read_text(encoding="utf-8", errors="ignore")
    if "\x00" in raw_text:
        raw_text = raw_text.replace("\x00", "")
    reader = csv.DictReader(io.StringIO(raw_text))
    for row in reader:
        parsed = {}
        for key, value in row.items():
            if key == "phase":
                parsed[key] = value
            else:
                parsed[key] = float(value)
        rows.append(parsed)
    return rows


def load_metadata(path: Path):
    metadata = {}
    if not path.exists():
        return metadata
    with path.open("r", encoding="utf-8") as file:
        for line in file:
            if "=" not in line:
                continue
            key, value = line.strip().split("=", 1)
            metadata[key] = value
    return metadata


def parse_start_window_accept_metrics(output_dir: Path):
    log_path = output_dir / "start_command.log"
    if not log_path.exists():
        return {}

    accepted_line = None
    for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "Window accepted " in line:
            accepted_line = line
            break
    if accepted_line is None:
        return {}

    payload = accepted_line.split("Window accepted ", 1)[-1]
    metrics = {}
    for token in payload.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        value = value.strip().rstrip(",")
        metric_key = f"accepted_window_{key}"
        try:
            numeric = float(value)
        except ValueError:
            metrics[metric_key] = value
            continue
        if math.isfinite(numeric):
            metrics[metric_key] = numeric
        else:
            metrics[metric_key] = value
    return metrics


def measured_rel_position(row):
    if "measured_rel_x" in row:
        return row["measured_rel_x"], row["measured_rel_y"], row["measured_rel_z"]
    return (
        row["mini_x"] - row["carrier_x"],
        row["mini_y"] - row["carrier_y"],
        row["mini_z"] - row["carrier_z"],
    )


def measured_rel_distance(row):
    if "measured_rel_distance" in row:
        return row["measured_rel_distance"]
    x, y, z = measured_rel_position(row)
    return math.sqrt(x * x + y * y + z * z)


def measured_rel_velocity(row):
    if "measured_rel_vx" in row:
        return row["measured_rel_vx"], row["measured_rel_vy"], row["measured_rel_vz"]
    return row["rel_vx"], row["rel_vy"], row["rel_vz"]


def measured_rel_speed(row):
    if "measured_rel_speed" in row:
        return row["measured_rel_speed"]
    vx, vy, vz = measured_rel_velocity(row)
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def finite_values(rows, key):
    return [r[key] for r in rows if key in r and math.isfinite(r[key])]


def parse_pair(value: str):
    parts = [p.strip() for p in value.split(",")]
    if len(parts) < 2:
        raise ValueError(f"invalid pair: {value}")
    return float(parts[0]), float(parts[1])


def metadata_float(metadata: dict[str, str], key: str, default: float) -> float:
    value = str(metadata.get(key, "")).strip()
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def metadata_bool(metadata: dict[str, str], key: str, default: bool = False) -> bool:
    value = str(metadata.get(key, "")).strip().lower()
    if not value:
        return default
    return value in {"1", "true", "yes", "on"}


def wrap_angle_deg(angle: float) -> float:
    wrapped = (angle + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def estimate_xy_velocity(rows, index):
    if not rows:
        return 0.0, 0.0
    if len(rows) == 1:
        return 0.0, 0.0
    if 0 < index < len(rows) - 1:
        previous = rows[index - 1]
        current = rows[index]
        following = rows[index + 1]
        dt = max(following["t"] - previous["t"], 1e-6)
        return (
            (following["mini_x"] - previous["mini_x"]) / dt,
            (following["mini_y"] - previous["mini_y"]) / dt,
        )
    if index == 0:
        current = rows[0]
        following = rows[1]
        dt = max(following["t"] - current["t"], 1e-6)
        return (
            (following["mini_x"] - current["mini_x"]) / dt,
            (following["mini_y"] - current["mini_y"]) / dt,
        )
    previous = rows[index - 1]
    current = rows[index]
    dt = max(current["t"] - previous["t"], 1e-6)
    return (
        (current["mini_x"] - previous["mini_x"]) / dt,
        (current["mini_y"] - previous["mini_y"]) / dt,
    )


def heading_deg_from_rows(rows, index):
    vx, vy = estimate_xy_velocity(rows, index)
    if math.hypot(vx, vy) <= 1e-6:
        return math.nan
    return math.degrees(math.atan2(vy, vx))


def find_nearest_row_index(rows, target_t):
    return min(range(len(rows)), key=lambda idx: abs(rows[idx]["t"] - target_t))


def find_release_row_index(rows, distance, rel_x, rel_y, rel_z):
    return min(
        range(len(rows)),
        key=lambda idx: (
            abs(rows[idx]["relative_distance"] - distance) +
            abs(rows[idx]["rel_x"] - rel_x) +
            abs(rows[idx]["rel_y"] - rel_y) +
            abs(rows[idx]["rel_z"] - rel_z)
        ),
    )


def first_active_row_index(rows):
    for index, row in enumerate(rows):
        if row.get("phase") and row.get("phase") != "IDLE":
            return index
    return None


def first_phase_row_index(rows, phase: str, start_index: int = 0):
    for index in range(max(0, start_index), len(rows)):
        if rows[index].get("phase") == phase:
            return index
    return None


def first_corridor_active_index(rows, start_index: int = 0):
    for index in range(max(0, start_index), len(rows)):
        value = rows[index].get("controller_rendezvous_corridor_active", math.nan)
        if math.isfinite(value) and value >= 0.5:
            return index
    return None


def carrier_xy_path_length(rows, start_index: int, end_index: int | None = None) -> float:
    if start_index is None or start_index < 0 or start_index >= len(rows):
        return math.nan
    last_index = len(rows) - 1 if end_index is None else max(start_index, min(end_index, len(rows) - 1))
    total = 0.0
    previous = rows[start_index]
    for index in range(start_index + 1, last_index + 1):
        current = rows[index]
        dx = current["carrier_x"] - previous["carrier_x"]
        dy = current["carrier_y"] - previous["carrier_y"]
        step = math.hypot(dx, dy)
        if math.isfinite(step):
            total += step
        previous = current
    return total


def carrier_xy_net_displacement(rows, start_index: int, end_index: int | None = None) -> float:
    if start_index is None or start_index < 0 or start_index >= len(rows):
        return math.nan
    last_index = len(rows) - 1 if end_index is None else max(start_index, min(end_index, len(rows) - 1))
    start_row = rows[start_index]
    end_row = rows[last_index]
    return math.hypot(
        end_row["carrier_x"] - start_row["carrier_x"],
        end_row["carrier_y"] - start_row["carrier_y"],
    )


def row_index_at_or_after_t(rows, target_t: float, start_index: int) -> int | None:
    if start_index is None or start_index < 0 or start_index >= len(rows):
        return None
    for index in range(start_index, len(rows)):
        if rows[index]["t"] >= target_t:
            return index
    return len(rows) - 1 if rows else None


def compute_path_efficiency_metrics(rows):
    metrics = {}
    start_index = first_active_row_index(rows)
    if start_index is None:
        return metrics

    start_row = rows[start_index]
    start_t = start_row["t"]
    metrics["start_t_sec"] = start_t

    first_tracking_index = first_phase_row_index(rows, "TRACKING", start_index)
    if first_tracking_index is not None:
        metrics["time_to_first_tracking_sec"] = rows[first_tracking_index]["t"] - start_t

    first_docking_index = first_phase_row_index(rows, "DOCKING", start_index)
    if first_docking_index is not None:
        metrics["time_to_first_docking_sec"] = rows[first_docking_index]["t"] - start_t

    corridor_index = first_corridor_active_index(rows, start_index)
    if corridor_index is not None:
        metrics["time_to_corridor_sec"] = rows[corridor_index]["t"] - start_t

    completed_index = first_phase_row_index(rows, "COMPLETED", start_index)
    path_end_index = completed_index if completed_index is not None else len(rows) - 1
    metrics["post_start_path_length_m"] = carrier_xy_path_length(rows, start_index, path_end_index)

    if completed_index is not None:
        metrics["start_to_completed_sec"] = rows[completed_index]["t"] - start_t

    if first_docking_index is not None:
        metrics["docking_path_length_m"] = carrier_xy_path_length(rows, first_docking_index, path_end_index)

    for horizon_sec in (10.0, 20.0):
        horizon_end_index = row_index_at_or_after_t(rows, start_t + horizon_sec, start_index)
        if horizon_end_index is None:
            continue
        horizon_label = str(int(horizon_sec))
        path_length = carrier_xy_path_length(rows, start_index, horizon_end_index)
        net_displacement = carrier_xy_net_displacement(rows, start_index, horizon_end_index)
        metrics[f"post_start_{horizon_label}s_path_length_m"] = path_length
        metrics[f"post_start_{horizon_label}s_net_displacement_m"] = net_displacement
        if math.isfinite(path_length) and math.isfinite(net_displacement) and net_displacement > 1e-6:
            metrics[f"post_start_{horizon_label}s_path_efficiency_ratio"] = path_length / net_displacement

    return metrics


def compute_prehold_start_metrics(rows, metadata):
    metrics = {}
    start_index = first_active_row_index(rows)
    if start_index is None:
        return metrics

    start_row = rows[start_index]
    carrier_z = start_row.get("carrier_z", math.nan)
    carrier_vz = start_row.get("carrier_vz", math.nan)
    metrics["start_carrier_z_m"] = carrier_z
    metrics["start_carrier_vz_mps"] = carrier_vz

    prehold_required = metadata_bool(metadata, "auto_start_carrier_prehold_required", False)
    min_altitude = metadata_float(metadata, "auto_start_carrier_prehold_min_altitude_m", 0.0)
    max_abs_vz = metadata_float(metadata, "auto_start_carrier_prehold_max_abs_vz_mps", 2.5)
    metrics["start_prehold_required"] = 1.0 if prehold_required else 0.0
    metrics["start_prehold_min_altitude_m"] = min_altitude
    metrics["start_prehold_max_abs_vz_mps"] = max_abs_vz
    if not prehold_required:
        metrics["start_prehold_ready"] = 1.0
    else:
        ready = (
            math.isfinite(carrier_z) and
            math.isfinite(carrier_vz) and
            carrier_z >= min_altitude and
            abs(carrier_vz) <= max_abs_vz
        )
        metrics["start_prehold_ready"] = 1.0 if ready else 0.0
        if math.isfinite(carrier_z):
            metrics["start_prehold_alt_margin_m"] = carrier_z - min_altitude
    return metrics


def compute_start_intercept_metrics(rows, metadata):
    metrics = {}
    start_index = first_active_row_index(rows)
    if start_index is None:
        return metrics

    row = rows[start_index]
    mini_vx = row["mini_vx"]
    mini_vy = row["mini_vy"]
    mini_vz = row["mini_vz"]
    mini_speed_xy = math.hypot(mini_vx, mini_vy)
    if mini_speed_xy <= 1e-6:
        return metrics

    horizon = metadata_float(metadata, "auto_start_rear_entry_prediction_horizon_sec", 6.0)
    carrier_speed = metadata_float(metadata, "auto_start_rear_entry_prediction_carrier_speed_mps", 9.5)
    tangent_weight = metadata_float(metadata, "auto_start_rear_entry_prediction_tangent_weight", 0.65)
    lateral_max = metadata_float(metadata, "auto_start_rear_entry_prediction_lateral_max_m", 20.0)
    along_min_cfg = metadata_float(metadata, "auto_start_rear_entry_prediction_along_min_m", 10.0)
    along_max_cfg = metadata_float(metadata, "auto_start_rear_entry_prediction_along_max_m", 70.0)
    distance_min = metadata_float(metadata, "auto_start_rear_entry_prediction_distance_min_m", 60.0)
    distance_max = metadata_float(metadata, "auto_start_rear_entry_prediction_distance_max_m", 110.0)
    score_threshold = metadata_float(metadata, "auto_start_rear_entry_prediction_score_threshold", 2.6)
    along_target_cfg = metadata_float(metadata, "auto_start_rear_entry_prediction_score_along_target_m", 24.0)
    require_carrier_behind = metadata_bool(metadata, "auto_start_rear_entry_require_carrier_behind", False)

    mini_dir_x = mini_vx / mini_speed_xy
    mini_dir_y = mini_vy / mini_speed_xy
    mini_lat_x = -mini_dir_y
    mini_lat_y = mini_dir_x

    mini_pred_x = row["mini_x"] + mini_vx * horizon
    mini_pred_y = row["mini_y"] + mini_vy * horizon
    mini_pred_z = row["mini_z"] + mini_vz * horizon
    to_pred_x = mini_pred_x - row["carrier_x"]
    to_pred_y = mini_pred_y - row["carrier_y"]
    to_pred_norm = math.hypot(to_pred_x, to_pred_y)
    if to_pred_norm > 1e-6:
        to_pred_x /= to_pred_norm
        to_pred_y /= to_pred_norm
    else:
        to_pred_x = mini_dir_x
        to_pred_y = mini_dir_y

    blend_x = tangent_weight * mini_dir_x + (1.0 - tangent_weight) * to_pred_x
    blend_y = tangent_weight * mini_dir_y + (1.0 - tangent_weight) * to_pred_y
    blend_norm = math.hypot(blend_x, blend_y)
    if blend_norm > 1e-6:
        blend_x /= blend_norm
        blend_y /= blend_norm
    else:
        blend_x = mini_dir_x
        blend_y = mini_dir_y

    carrier_pred_x = row["carrier_x"] + blend_x * carrier_speed * horizon
    carrier_pred_y = row["carrier_y"] + blend_y * carrier_speed * horizon

    prediction_dx = mini_pred_x - carrier_pred_x
    prediction_dy = mini_pred_y - carrier_pred_y
    prediction_along_m = prediction_dx * mini_dir_x + prediction_dy * mini_dir_y
    prediction_lateral_m = prediction_dx * mini_lat_x + prediction_dy * mini_lat_y

    along_target_m = along_target_cfg if require_carrier_behind else -along_target_cfg
    prediction_along_min_m = along_min_cfg if require_carrier_behind else -along_max_cfg
    prediction_along_max_m = along_max_cfg if require_carrier_behind else -along_min_cfg
    along_scale = max(
        abs(prediction_along_max_m - along_target_m),
        abs(along_target_m - prediction_along_min_m),
        1e-6,
    )
    prediction_score = (
        abs(prediction_lateral_m) / max(lateral_max, 1e-6) +
        abs(prediction_along_m - along_target_m) / along_scale
    )
    rel_distance_xy = math.hypot(row["rel_x"], row["rel_y"])
    lateral_margin = lateral_max - abs(prediction_lateral_m)
    along_margin = min(
        prediction_along_m - prediction_along_min_m,
        prediction_along_max_m - prediction_along_m,
    )
    distance_margin = min(rel_distance_xy - distance_min, distance_max - rel_distance_xy)
    score_margin = score_threshold - prediction_score
    gate_feasibility_margin = min(lateral_margin, along_margin, distance_margin, score_margin)

    metrics.update({
        "start_prediction_horizon_sec": horizon,
        "start_prediction_carrier_speed_mps": carrier_speed,
        "start_prediction_tangent_weight": tangent_weight,
        "start_intercept_target_t_sec": row["t"] + horizon,
        "start_intercept_mini_pred_x_m": mini_pred_x,
        "start_intercept_mini_pred_y_m": mini_pred_y,
        "start_intercept_mini_pred_z_m": mini_pred_z,
        "start_intercept_tangent_heading_deg": math.degrees(math.atan2(mini_dir_y, mini_dir_x)),
        "start_intercept_guard_heading_deg": math.degrees(math.atan2(blend_y, blend_x)),
        "start_intercept_guard_dir_x": blend_x,
        "start_intercept_guard_dir_y": blend_y,
        "start_intercept_carrier_ref_x_m": carrier_pred_x,
        "start_intercept_carrier_ref_y_m": carrier_pred_y,
        "start_intercept_pred_along_m": prediction_along_m,
        "start_intercept_pred_lat_m": prediction_lateral_m,
        "start_intercept_pred_score": prediction_score,
        "start_intercept_score_margin": score_margin,
        "start_intercept_lateral_margin_m": lateral_margin,
        "start_intercept_along_margin_m": along_margin,
        "start_intercept_distance_margin_m": distance_margin,
        "start_intercept_gate_feasibility_margin_m": gate_feasibility_margin,
    })
    return metrics


def save_intercept_diagnostics(rows, output_dir: Path, metadata):
    metrics = compute_start_intercept_metrics(rows, metadata)
    if not metrics:
        return None
    output_path = output_dir / "intercept_diagnostics.txt"
    with output_path.open("w", encoding="utf-8") as file:
        for key, value in metrics.items():
            if isinstance(value, str):
                file.write(f"{key}={value}\n")
            elif math.isfinite(value):
                file.write(f"{key}={value:.6f}\n")
    return output_path


def parse_tangent_events(output_dir: Path):
    log_path = output_dir / "launch.log"
    if not log_path.exists():
        return {}

    events = {}
    release_pattern = re.compile(
        r"\[(?P<stamp>[0-9.]+)\].*fixed-wing: glide score release accepted .*"
        r"distance=(?P<distance>[-0-9.]+) rel=\((?P<rel_x>[-0-9.]+),(?P<rel_y>[-0-9.]+),(?P<rel_z>[-0-9.]+)\)"
    )
    arm_pattern = re.compile(
        r"\[(?P<stamp>[0-9.]+)\].*fixed-wing: tangent exit armed .*"
        r"axis=\((?P<axis_x>[-0-9.]+),(?P<axis_y>[-0-9.]+)\)"
    )
    clear_pattern = re.compile(
        r"\[(?P<stamp>[0-9.]+)\].*fixed-wing: tangent exit cleared reason=(?P<reason>.+)$"
    )

    for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "glide score release accepted" in line:
            match = release_pattern.search(line)
            if match:
                events["release"] = {
                    "stamp": float(match.group("stamp")),
                    "distance": float(match.group("distance")),
                    "rel_x": float(match.group("rel_x")),
                    "rel_y": float(match.group("rel_y")),
                    "rel_z": float(match.group("rel_z")),
                    "line": line.strip(),
                }
        elif "tangent exit armed" in line:
            match = arm_pattern.search(line)
            if match:
                events["arm"] = {
                    "stamp": float(match.group("stamp")),
                    "axis_x": float(match.group("axis_x")),
                    "axis_y": float(match.group("axis_y")),
                    "line": line.strip(),
                }
        elif "tangent exit cleared" in line:
            match = clear_pattern.search(line)
            if match:
                events["clear"] = {
                    "stamp": float(match.group("stamp")),
                    "reason": match.group("reason").strip(),
                    "line": line.strip(),
                }

    return events


def compute_tangent_exit_metrics(rows, metadata, output_dir: Path):
    events = parse_tangent_events(output_dir)
    release_event = events.get("release")
    if not release_event or not rows:
        return {}

    release_index = find_release_row_index(
        rows,
        release_event["distance"],
        release_event["rel_x"],
        release_event["rel_y"],
        release_event["rel_z"],
    )
    release_row = rows[release_index]
    release_t = release_row["t"]

    metrics = {
        "release_t_sec": release_t,
        "release_phase": release_row["phase"],
        "release_distance_m": release_row["relative_distance"],
        "release_rel_x_m": release_row["rel_x"],
        "release_rel_y_m": release_row["rel_y"],
        "release_rel_z_m": release_row["rel_z"],
    }

    release_heading_deg = heading_deg_from_rows(rows, release_index)
    if math.isfinite(release_heading_deg):
        metrics["release_heading_deg"] = release_heading_deg

    release_vx, release_vy = estimate_xy_velocity(rows, release_index)
    release_speed = math.hypot(release_vx, release_vy)
    if math.isfinite(release_speed):
        metrics["release_speed_mps"] = release_speed

    if "mini_orbit_center" in metadata:
        try:
            center_x, center_y = parse_pair(metadata["mini_orbit_center"])
            release_radius = math.hypot(
                release_row["mini_x"] - center_x,
                release_row["mini_y"] - center_y,
            )
            metrics["release_radius_m"] = release_radius

            for horizon_sec in (1.0, 2.0):
                horizon_rows = [
                    row for row in rows[release_index:]
                    if (row["t"] - release_t) <= horizon_sec
                ]
                if horizon_rows:
                    radius_values = [
                        math.hypot(row["mini_x"] - center_x, row["mini_y"] - center_y)
                        for row in horizon_rows
                    ]
                    min_radius = min(radius_values)
                    metrics[f"post_release_radius_min_{int(horizon_sec)}s_m"] = min_radius
                    metrics[f"post_release_radius_drop_{int(horizon_sec)}s_m"] = (
                        release_radius - min_radius
                    )
        except ValueError:
            pass

    heading_1s_index = find_nearest_row_index(rows, release_t + 1.0)
    heading_1s_deg = heading_deg_from_rows(rows, heading_1s_index)
    if math.isfinite(release_heading_deg) and math.isfinite(heading_1s_deg):
        metrics["post_release_heading_change_1s_deg"] = abs(
            wrap_angle_deg(heading_1s_deg - release_heading_deg)
        )

    axis_x = None
    axis_y = None
    if "arm" in events:
        axis_x = events["arm"]["axis_x"]
        axis_y = events["arm"]["axis_y"]
    elif release_speed > 1e-6:
        axis_x = release_vx / release_speed
        axis_y = release_vy / release_speed

    if axis_x is not None and axis_y is not None:
        lateral_x = -axis_y
        lateral_y = axis_x
        deck_x = release_row["carrier_x"]
        deck_y = release_row["carrier_y"]
        delta_x = deck_x - release_row["mini_x"]
        delta_y = deck_y - release_row["mini_y"]
        metrics["post_release_forward_progress_to_deck_m"] = (
            delta_x * axis_x + delta_y * axis_y
        )
        metrics["post_release_lateral_miss_to_deck_m"] = (
            delta_x * lateral_x + delta_y * lateral_y
        )

    arm_event = events.get("arm")
    clear_event = events.get("clear")
    if arm_event:
        metrics["tangent_exit_arm_stamp_sec"] = arm_event["stamp"]
    if clear_event:
        metrics["tangent_exit_clear_stamp_sec"] = clear_event["stamp"]
        metrics["tangent_exit_clear_reason"] = clear_event["reason"]
    if arm_event and clear_event:
        metrics["tangent_exit_duration_sec"] = clear_event["stamp"] - arm_event["stamp"]

    return metrics


def wait_orbit_rows(rows, metadata):
    if not rows:
        return []
    filtered = rows
    for idx, row in enumerate(rows):
        if row["phase"] in {"DOCKING", "COMPLETED"}:
            filtered = rows[:idx]
            break

    if not filtered:
        filtered = rows

    target_alt = None
    if "mini_takeoff_altitude" in metadata:
        try:
            target_alt = float(metadata["mini_takeoff_altitude"])
        except ValueError:
            target_alt = None

    if target_alt is None:
        return filtered

    ready_altitude = max(target_alt * 0.88, target_alt - 2.0)
    for idx, row in enumerate(filtered):
        if row["mini_z"] >= ready_altitude:
            return filtered[idx:]
    return filtered


def save_distance_plot(rows, output_dir: Path):
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=160)
    time_axis = [r["t"] for r in rows]
    status_distance = [r["relative_distance"] for r in rows]
    measured_distance = [measured_rel_distance(r) for r in rows]
    ax.plot(time_axis, status_distance, color="#0b84a5", lw=2.2, label="Status")
    if any(abs(a - b) > 1e-3 for a, b in zip(status_distance, measured_distance)):
        ax.plot(
            time_axis,
            measured_distance,
            color="#f28e2b",
            lw=1.8,
            ls="--",
            alpha=0.9,
            label="Measured (async)",
        )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Relative distance [m]")
    ax.set_title("Docking Distance Convergence")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    path = output_dir / "distance_convergence.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_signed_front_distance_plot(rows, output_dir: Path):
    time_axis = []
    signed_front_distance = []

    last_axis = (1.0, 0.0)
    for index, row in enumerate(rows):
        mini_vx = row.get("mini_vx", math.nan)
        mini_vy = row.get("mini_vy", math.nan)
        if not (math.isfinite(mini_vx) and math.isfinite(mini_vy)):
            mini_vx, mini_vy = estimate_xy_velocity(rows, index)
        speed = math.hypot(mini_vx, mini_vy)
        if speed > 1e-6:
            axis = (mini_vx / speed, mini_vy / speed)
            last_axis = axis
        else:
            axis = last_axis

        carrier_minus_mini_x = row["carrier_x"] - row["mini_x"]
        carrier_minus_mini_y = row["carrier_y"] - row["mini_y"]
        signed_distance = (
            carrier_minus_mini_x * axis[0] +
            carrier_minus_mini_y * axis[1]
        )

        time_axis.append(row["t"])
        signed_front_distance.append(signed_distance)

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=160)
    ax.plot(
        time_axis,
        signed_front_distance,
        color="#6a3d9a",
        lw=2.0,
        label="Signed front/back distance",
    )
    ax.axhline(0.0, color="#555555", lw=1.3, ls="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Signed distance [m]")
    ax.set_title("Carrier Front/Back Signed Distance ( + front / - back )")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    path = output_dir / "carrier_front_back_signed_distance.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_xy_plot(rows, output_dir: Path, corridor_plan=None):
    if not rows:
        raise ValueError("no rows available for XY plot")

    best_index = min(range(len(rows)), key=lambda idx: rows[idx]["relative_distance"])
    focus_end_index = min(len(rows) - 1, best_index + 45)
    for idx, row in enumerate(rows):
        phase = row["phase"]
        distance = row["relative_distance"]
        rel_speed = measured_rel_speed(row)
        if phase in {"DOCKING", "COMPLETED"} and distance <= 1.25 and rel_speed <= 0.45:
            focus_end_index = min(len(rows) - 1, idx + 30)
            break
        if phase in {"DOCKING", "COMPLETED"} and distance <= 2.0 and rel_speed <= 0.80:
            focus_end_index = min(len(rows) - 1, idx + 36)
            break

    focused_rows = rows[: focus_end_index + 1]

    fig, ax = plt.subplots(figsize=(6.5, 6.0), dpi=160)
    ax.plot(
        [r["carrier_x"] for r in focused_rows],
        [r["carrier_y"] for r in focused_rows],
        color="#4daf4a",
        lw=2.5,
        label="Carrier",
    )
    ax.plot(
        [r["mini_x"] for r in focused_rows],
        [r["mini_y"] for r in focused_rows],
        color="#e41a1c",
        lw=2.0,
        label="Mini",
    )
    ax.scatter(focused_rows[0]["carrier_x"], focused_rows[0]["carrier_y"], color="#984ea3", s=42, label="Carrier start")
    ax.scatter(focused_rows[-1]["carrier_x"], focused_rows[-1]["carrier_y"], color="#a65628", s=42, label="Carrier end")
    ax.scatter(focused_rows[0]["mini_x"], focused_rows[0]["mini_y"], color="#ff7f00", s=40, label="Mini start")
    ax.scatter(focused_rows[-1]["mini_x"], focused_rows[-1]["mini_y"], color="#377eb8", s=40, label="Mini end")
    # Helper: draw corridor plan (arc or tangent) on an axis
    def draw_corridor_plan(ax_):
        if not corridor_plan:
            return
        T = corridor_plan["T"]
        d = corridor_plan["dir"]
        if "arc_M" in corridor_plan:
            # Draw the circular arc trajectory (the actual plan)
            Mx, My = corridor_plan["arc_M"]
            r = corridor_plan["arc_r"]
            phi0 = corridor_plan["arc_phi0"]
            dphi = corridor_plan["arc_dphi"]
            N = 80
            phis = [phi0 + dphi * i / N for i in range(N + 1)]
            ax_.plot(
                [Mx + r * math.cos(p) for p in phis],
                [My + r * math.sin(p) for p in phis],
                "m--", lw=2.0, alpha=0.8, label="CorridorPlan arc"
            )
        else:
            # Fallback: just the tangent line
            L, A = 60.0, 15.0
            ax_.plot(
                [T[0] - d[0] * A, T[0] + d[0] * L],
                [T[1] - d[1] * A, T[1] + d[1] * L],
                "m--", lw=1.5, alpha=0.7, label="CorridorPlan (tangent)"
            )
        ax_.scatter([T[0]], [T[1]], marker="s", color="magenta", s=36, zorder=5, label="Rendezvous T")

    draw_corridor_plan(ax)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Planar Docking Trajectory (Focused)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6)
    fig.tight_layout()
    path = output_dir / "trajectory_xy.png"
    fig.savefig(path)
    plt.close(fig)

    fig_full, ax_full = plt.subplots(figsize=(6.5, 6.0), dpi=160)
    ax_full.plot([r["carrier_x"] for r in rows], [r["carrier_y"] for r in rows], color="#4daf4a", lw=2.5, label="Carrier")
    ax_full.plot([r["mini_x"] for r in rows], [r["mini_y"] for r in rows], color="#e41a1c", lw=2.0, label="Mini")
    ax_full.scatter(rows[0]["carrier_x"], rows[0]["carrier_y"], color="#984ea3", s=42, label="Carrier start")
    ax_full.scatter(rows[-1]["carrier_x"], rows[-1]["carrier_y"], color="#a65628", s=42, label="Carrier end")
    ax_full.scatter(rows[0]["mini_x"], rows[0]["mini_y"], color="#ff7f00", s=40, label="Mini start")
    ax_full.scatter(rows[-1]["mini_x"], rows[-1]["mini_y"], color="#377eb8", s=40, label="Mini end")
    draw_corridor_plan(ax_full)
    ax_full.set_xlabel("X [m]")
    ax_full.set_ylabel("Y [m]")
    ax_full.set_title("Planar Docking Trajectory (Full)")
    ax_full.axis("equal")
    ax_full.grid(True, alpha=0.3)
    ax_full.legend(fontsize=7)
    fig_full.tight_layout()
    fig_full.savefig(output_dir / "trajectory_xy_full.png")
    plt.close(fig_full)
    return path


def save_xy_animation_gif(rows, output_dir: Path):
    if not rows:
        return None

    sample_count = len(rows)
    step = max(2, sample_count // 150)
    sampled = rows[::step]
    if sampled[-1] is not rows[-1]:
        sampled.append(rows[-1])

    carrier_x = [r["carrier_x"] for r in rows]
    carrier_y = [r["carrier_y"] for r in rows]
    mini_x = [r["mini_x"] for r in rows]
    mini_y = [r["mini_y"] for r in rows]
    all_x = carrier_x + mini_x
    all_y = carrier_y + mini_y
    if not all_x or not all_y:
        return None

    margin = 10.0
    fig, ax = plt.subplots(figsize=(6.5, 6.0), dpi=140)
    ax.set_title("XY Trajectory (animated)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)

    # Draw corridor plan arc on animation if available
    corridor_plan = parse_corridor_plan_from_log(output_dir / "launch.log")
    if corridor_plan and "arc_M" in corridor_plan:
        Mx, My = corridor_plan["arc_M"]
        r = corridor_plan["arc_r"]
        phi0 = corridor_plan["arc_phi0"]
        dphi = corridor_plan["arc_delta_phi"] if "arc_delta_phi" in corridor_plan else corridor_plan["arc_dphi"]
        N = 80
        phis = [phi0 + dphi * i / N for i in range(N + 1)]
        ax.plot(
            [Mx + r * math.cos(p) for p in phis],
            [My + r * math.sin(p) for p in phis],
            "m--", lw=1.5, alpha=0.7, label="CorridorPlan arc"
        )
        ax.scatter([corridor_plan["T"][0]], [corridor_plan["T"][1]],
                   marker="s", color="magenta", s=28, zorder=5, label="Rendezvous T")
    elif corridor_plan:
        T = corridor_plan["T"]
        d = corridor_plan["dir"]
        ax.plot([T[0]-d[0]*15, T[0]+d[0]*60], [T[1]-d[1]*15, T[1]+d[1]*60],
                "m--", lw=1.5, alpha=0.7, label="CorridorPlan")
        ax.scatter([T[0]], [T[1]], marker="s", color="magenta", s=28, zorder=5)

    carrier_line, = ax.plot([], [], color="#4daf4a", lw=2.2, label="Carrier")
    mini_line, = ax.plot([], [], color="#e41a1c", lw=2.0, label="Mini")
    carrier_point, = ax.plot([], [], "o", color="#4daf4a", ms=5)
    mini_point, = ax.plot([], [], "o", color="#e41a1c", ms=5)
    status_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
    )
    ax.legend(loc="upper right")

    def init():
        carrier_line.set_data([], [])
        mini_line.set_data([], [])
        carrier_point.set_data([], [])
        mini_point.set_data([], [])
        status_text.set_text("")
        return carrier_line, mini_line, carrier_point, mini_point, status_text

    def update(index):
        current_rows = sampled[: index + 1]
        carrier_line.set_data(
            [r["carrier_x"] for r in current_rows],
            [r["carrier_y"] for r in current_rows],
        )
        mini_line.set_data(
            [r["mini_x"] for r in current_rows],
            [r["mini_y"] for r in current_rows],
        )
        latest = current_rows[-1]
        carrier_point.set_data([latest["carrier_x"]], [latest["carrier_y"]])
        mini_point.set_data([latest["mini_x"]], [latest["mini_y"]])
        status_text.set_text(f"t = {latest['t']:.2f} s\nphase = {latest['phase']}")
        return carrier_line, mini_line, carrier_point, mini_point, status_text

    output_path = output_dir / "trajectory_xy_full.gif"
    animation = FuncAnimation(
        fig,
        update,
        frames=len(sampled),
        init_func=init,
        interval=5,
        blit=False,
    )
    try:
        animation.save(output_path, writer=PillowWriter(fps=200))
    except Exception as exc:
        try:
            animation.save(
                output_path,
                writer=PillowWriter(fps=150),
                dpi=110,
                savefig_kwargs={"facecolor": "white", "edgecolor": "white"},
            )
        except Exception as fallback_exc:
            print(f"warning: failed to write trajectory gif ({exc}; fallback={fallback_exc})")
            plt.close(fig)
            return None
    plt.close(fig)
    return output_path


def save_xz_plot(rows, output_dir: Path):
    fig, ax = plt.subplots(figsize=(8.0, 4.5), dpi=160)
    ax.plot([r["carrier_x"] for r in rows], [r["carrier_z"] for r in rows], color="#4daf4a", lw=2.5, label="Carrier")
    ax.plot([r["mini_x"] for r in rows], [r["mini_z"] for r in rows], color="#e41a1c", lw=2.0, label="Mini")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title("Approach and Terminal Glide Profile")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    path = output_dir / "trajectory_xz.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_x_time_plot(rows, output_dir: Path, corridor_plan=None):
    """X position vs time with corridor plan publish marker."""
    if not rows:
        return None
    ts = [r["t"] - rows[0]["t"] for r in rows]
    fig, (ax_x, ax_y) = plt.subplots(2, 1, figsize=(10, 6), dpi=140, sharex=True)
    ax_x.plot(ts, [r["carrier_x"] for r in rows], color="#4daf4a", lw=1.8, label="Carrier X")
    ax_x.plot(ts, [r["mini_x"] for r in rows], color="#e41a1c", lw=1.5, label="Mini X")
    ax_x.set_ylabel("X [m]")
    ax_x.grid(True, alpha=0.3)
    ax_x.legend(fontsize=7)
    ax_y.plot(ts, [r["carrier_y"] for r in rows], color="#4daf4a", lw=1.8, label="Carrier Y")
    ax_y.plot(ts, [r["mini_y"] for r in rows], color="#e41a1c", lw=1.5, label="Mini Y")
    ax_y.set_ylabel("Y [m]")
    ax_y.set_xlabel("Time [s]")
    ax_y.grid(True, alpha=0.3)
    ax_y.legend(fontsize=7)
    # Draw corridor plan trajectory on time plot
    if corridor_plan:
        hold_sec = corridor_plan.get("hold_sec", 0.0)
        for a in (ax_x, ax_y):
            a.axvline(hold_sec, color="magenta", ls="--", lw=1.2, alpha=0.7,
                      label=f"Departure (hold={hold_sec:.1f}s)")
            a.axvline(0, color="cyan", ls=":", lw=1.0, alpha=0.5, label="CorridorPlan published")
        # Draw planned arc trajectory
        if "arc_M" in corridor_plan:
            Mx, My = corridor_plan["arc_M"]
            r = corridor_plan["arc_r"]
            phi0 = corridor_plan["arc_phi0"]
            dphi = corridor_plan.get("arc_delta_phi", corridor_plan.get("arc_dphi", 0.0))
            spd = corridor_plan.get("spd", 1.0)
            t_dur = r * abs(dphi) / max(spd, 0.1)
            N = 60
            plan_ts = [hold_sec + t_dur * i / N for i in range(N + 1)]
            plan_xs = [Mx + r * math.cos(phi0 + dphi * i / N) for i in range(N + 1)]
            plan_ys = [My + r * math.sin(phi0 + dphi * i / N) for i in range(N + 1)]
            ax_x.plot(plan_ts, plan_xs, "m--", lw=1.5, alpha=0.7, label="Plan X")
            ax_y.plot(plan_ts, plan_ys, "m--", lw=1.5, alpha=0.7, label="Plan Y")
    fig.suptitle("Position vs Time")
    fig.tight_layout()
    path = output_dir / "x_time.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_speed_plot(rows, output_dir: Path):
    # Ignore repeated/non-increasing timestamps; they can appear when status stamp
    # pauses while odom position still updates, which would create artificial speed spikes.
    time_axis = []
    carrier_speed = []
    mini_speed = []
    mini_speed_from_px4 = False
    previous = None
    for current in rows:
        if previous is None:
            previous = current
            continue
        if current.get("phase", "") == "IDLE":
            previous = current
            continue
        dt = current["t"] - previous["t"]
        if dt <= 1e-3:
            previous = current
            continue

        carrier_vx = current.get("carrier_vx", math.nan)
        carrier_vy = current.get("carrier_vy", math.nan)
        if math.isfinite(carrier_vx) and math.isfinite(carrier_vy):
            carrier_ground_speed = math.hypot(carrier_vx, carrier_vy)
        else:
            carrier_ground_speed = math.hypot(
                current["carrier_x"] - previous["carrier_x"],
                current["carrier_y"] - previous["carrier_y"],
            ) / dt

        mini_vx = current.get("mini_vx", math.nan)
        mini_vy = current.get("mini_vy", math.nan)
        if math.isfinite(mini_vx) and math.isfinite(mini_vy):
            mini_ground_speed_fd = math.hypot(mini_vx, mini_vy)
        else:
            mini_ground_speed_fd = math.hypot(
                current["mini_x"] - previous["mini_x"],
                current["mini_y"] - previous["mini_y"],
            ) / dt

        mini_px4_tas = current.get("mini_px4_true_airspeed_mps", math.nan)
        if math.isfinite(mini_px4_tas) and mini_px4_tas > 0.0:
            mini_speed.append(mini_px4_tas)
            mini_speed_from_px4 = True
        else:
            mini_speed.append(mini_ground_speed_fd)

        carrier_speed.append(carrier_ground_speed)
        time_axis.append(current["t"])
        previous = current
    fig, ax = plt.subplots(figsize=(8.0, 4.5), dpi=160)
    ax.plot(time_axis, carrier_speed, color="#4daf4a", lw=2.0, label="Carrier speed")
    mini_label = "Mini TAS (PX4)" if mini_speed_from_px4 else "Mini speed"
    ax.plot(time_axis, mini_speed, color="#e41a1c", lw=2.0, label=mini_label)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Ground speed [m/s]")
    ax.set_title("Speed Matching During Cooperative Landing")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    path = output_dir / "speed_profile.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_phase_plot(rows, output_dir: Path):
    phase_to_index = {
        "IDLE": 0,
        "TAKEOFF": 1,
        "APPROACH": 2,
        "TRACKING": 3,
        "DOCKING": 4,
        "COMPLETED": 5,
        "FAILED": 6,
    }
    fig, ax = plt.subplots(figsize=(8, 2.8), dpi=160)
    ax.step(
        [r["t"] for r in rows],
        [phase_to_index.get(r["phase"], -1) for r in rows],
        where="post",
        color="#984ea3",
        lw=2.0,
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Phase")
    ax.set_yticks(list(phase_to_index.values()))
    ax.set_yticklabels(list(phase_to_index.keys()))
    ax.set_title("Docking Phase Evolution")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "phase_timeline.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_wait_orbit_plot(rows, output_dir: Path, metadata):
    if "mini_orbit_center" not in metadata or "mini_orbit_radius" not in metadata:
        return None

    try:
        center_x, center_y = parse_pair(metadata["mini_orbit_center"])
        target_radius = float(metadata["mini_orbit_radius"])
        target_altitude = float(metadata.get("mini_takeoff_altitude", "nan"))
    except ValueError:
        return None

    orbit_rows = wait_orbit_rows(rows, metadata)
    if len(orbit_rows) < 2:
        return None

    time_axis = [r["t"] for r in orbit_rows]
    radius = [
        math.hypot(r["mini_x"] - center_x, r["mini_y"] - center_y)
        for r in orbit_rows
    ]
    altitude = [r["mini_z"] for r in orbit_rows]

    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.0), dpi=160, sharex=True)
    axes[0].plot(time_axis, radius, color="#e41a1c", lw=2.0, label="Mini orbit radius")
    axes[0].axhline(target_radius, color="#377eb8", lw=1.6, ls="--", label="Target radius")
    axes[0].set_ylabel("Radius [m]")
    axes[0].set_title("Mini Wait-Orbit Radius / Altitude")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(time_axis, altitude, color="#4daf4a", lw=2.0, label="Mini altitude")
    if math.isfinite(target_altitude):
        axes[1].axhline(target_altitude, color="#377eb8", lw=1.6, ls="--", label="Target altitude")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Altitude [m]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    fig.tight_layout()
    path = output_dir / "mini_wait_orbit_radius.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_fixed_wing_diagnostics_plot(rows, output_dir: Path):
    required = {
        "mini_tecs_true_airspeed_sp_mps",
        "mini_tecs_true_airspeed_filtered_mps",
        "mini_tecs_throttle_sp",
        "mini_tecs_underspeed_ratio",
    }
    if not required.issubset(rows[0].keys()):
        return None

    time_axis = [r["t"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(8.4, 8.0), dpi=160, sharex=True)

    axes[0].plot(time_axis, [r["mini_z"] for r in rows], color="#e41a1c", lw=2.0, label="Mini z")
    if "mini_tecs_altitude_reference_m" in rows[0]:
        reference0 = rows[0]["mini_tecs_altitude_reference_m"]
        altitude_sp_rel = [r["mini_tecs_altitude_sp_m"] - reference0 for r in rows]
        axes[0].plot(time_axis, altitude_sp_rel, color="#377eb8", lw=1.8, ls="--", label="TECS alt sp")
    axes[0].set_ylabel("Altitude [m]")
    axes[0].set_title("Fixed-Wing Wait Orbit Diagnostics")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(
        time_axis,
        [r["mini_tecs_true_airspeed_sp_mps"] for r in rows],
        color="#4daf4a",
        lw=1.8,
        label="TECS TAS sp",
    )
    axes[1].plot(
        time_axis,
        [r["mini_tecs_true_airspeed_filtered_mps"] for r in rows],
        color="#ff7f00",
        lw=2.0,
        label="TECS TAS",
    )
    if "mini_px4_true_airspeed_mps" in rows[0]:
        axes[1].plot(
            time_axis,
            [r["mini_px4_true_airspeed_mps"] for r in rows],
            color="#984ea3",
            lw=1.4,
            ls=":",
            label="AirspeedValidated TAS",
        )
    axes[1].set_ylabel("Airspeed [m/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].plot(
        time_axis,
        [r["mini_tecs_throttle_sp"] for r in rows],
        color="#0b84a5",
        lw=2.0,
        label="TECS throttle sp",
    )
    if "mini_px4_throttle_filtered" in rows[0]:
        axes[2].plot(
            time_axis,
            [r["mini_px4_throttle_filtered"] for r in rows],
            color="#a65628",
            lw=1.8,
            ls="--",
            label="Filtered throttle",
        )
    axes[2].plot(
        time_axis,
        [r["mini_tecs_underspeed_ratio"] for r in rows],
        color="#f781bf",
        lw=1.8,
        label="Underspeed ratio",
    )
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Throttle / ratio")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.tight_layout()
    path = output_dir / "fixed_wing_wait_diagnostics.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def save_summary(rows, output_dir: Path, metadata):
    summary = defaultdict(str)
    tangent_metrics = compute_tangent_exit_metrics(rows, metadata, output_dir)
    path_metrics = compute_path_efficiency_metrics(rows)
    prehold_start_metrics = compute_prehold_start_metrics(rows, metadata)
    intercept_metrics = compute_start_intercept_metrics(rows, metadata)
    accepted_window_metrics = parse_start_window_accept_metrics(output_dir)
    analysis_rows = [
        r for r in rows
        if measured_rel_distance(r) > 1e-6 or r["relative_distance"] > 1e-6
    ]
    if not analysis_rows:
        analysis_rows = rows

    status_distance_values = [r["relative_distance"] for r in analysis_rows]
    summary["samples"] = str(len(rows))
    summary["duration_sec"] = f"{analysis_rows[-1]['t']:.3f}" if analysis_rows else "0.0"
    summary["initial_distance_m"] = f"{status_distance_values[0]:.3f}" if analysis_rows else "0.0"
    summary["final_distance_m"] = f"{status_distance_values[-1]:.3f}" if analysis_rows else "0.0"
    summary["final_phase"] = analysis_rows[-1]["phase"] if analysis_rows else "UNKNOWN"
    summary["min_distance_m"] = f"{min(status_distance_values):.3f}" if analysis_rows else "0.0"
    summary["measured_final_distance_m"] = (
        f"{measured_rel_distance(analysis_rows[-1]):.3f}" if analysis_rows else "0.0"
    )
    summary["measured_min_distance_m"] = (
        f"{min(measured_rel_distance(r) for r in analysis_rows):.3f}" if analysis_rows else "0.0"
    )
    summary["mini_altitude_min_m"] = f"{min(r['mini_z'] for r in analysis_rows):.3f}" if analysis_rows else "0.0"
    summary["mini_altitude_max_m"] = f"{max(r['mini_z'] for r in analysis_rows):.3f}" if analysis_rows else "0.0"
    summary["mini_altitude_final_m"] = f"{analysis_rows[-1]['mini_z']:.3f}" if analysis_rows else "0.0"

    def _config_float(metadata_key: str, env_key: str, default: float) -> float:
        value = str(metadata.get(metadata_key, "")).strip()
        if value:
            try:
                return float(value)
            except ValueError:
                pass
        env_value = str(os.getenv(env_key) or "").strip()
        if env_value:
            try:
                return float(env_value)
            except ValueError:
                pass
        return default

    def _final_pass_hold(analysis_rows, cfg: dict[str, float]) -> float:
        best_hold = 0.0
        current_hold = 0.0
        last_t = math.nan
        last_ok = False
        for r in analysis_rows:
            phase = r.get("phase", "")
            t = r.get("t", math.nan)
            rel_x = r.get("rel_x", math.nan)
            rel_y = r.get("rel_y", math.nan)
            rel_z = r.get("rel_z", math.nan)
            distance = r.get("relative_distance", math.nan)
            rel_speed = measured_rel_speed(r)

            ok = (
                phase in {"DOCKING", "COMPLETED"} and
                math.isfinite(t) and
                math.isfinite(rel_x) and
                math.isfinite(rel_y) and
                math.isfinite(rel_z) and
                math.isfinite(distance) and
                math.isfinite(rel_speed) and
                abs(rel_x) <= cfg["xy_abs_max_m"] and
                abs(rel_y) <= cfg["xy_abs_max_m"] and
                cfg["z_min_m"] <= rel_z <= cfg["z_max_m"] and
                distance <= cfg["distance_max_m"] and
                rel_speed <= cfg["rel_speed_max_mps"]
            )

            if ok and last_ok and math.isfinite(last_t):
                dt = t - last_t
                if dt > 0.0 and dt < 1.0:
                    current_hold += dt
            elif ok:
                current_hold = 0.0
            else:
                current_hold = 0.0

            best_hold = max(best_hold, current_hold)
            last_t = t
            last_ok = ok

        return best_hold

    # FINAL_PASS: strict(v1) as final acceptance + loose as iteration gate.
    if analysis_rows:
        final_row = next((r for r in reversed(analysis_rows) if r["phase"] != "IDLE"), analysis_rows[-1])
        final_rel_x = final_row.get("rel_x", math.nan)
        final_rel_y = final_row.get("rel_y", math.nan)
        final_rel_z = final_row.get("rel_z", math.nan)
        final_rel_speed = measured_rel_speed(final_row)
        final_abs_xy_max = (
            max(abs(final_rel_x), abs(final_rel_y))
            if math.isfinite(final_rel_x) and math.isfinite(final_rel_y)
            else math.nan
        )
        summary["final_rel_x_m"] = f"{final_rel_x:.3f}" if math.isfinite(final_rel_x) else "nan"
        summary["final_rel_y_m"] = f"{final_rel_y:.3f}" if math.isfinite(final_rel_y) else "nan"
        summary["final_rel_z_m"] = f"{final_rel_z:.3f}" if math.isfinite(final_rel_z) else "nan"
        summary["final_rel_speed_mps"] = f"{final_rel_speed:.3f}" if math.isfinite(final_rel_speed) else "nan"
        summary["final_abs_xy_max_m"] = f"{final_abs_xy_max:.3f}" if math.isfinite(final_abs_xy_max) else "nan"

        v1_cfg = {
            "xy_abs_max_m": _config_float("final_pass_v1_xy_abs_max_m", "FINAL_PASS_V1_XY_ABS_MAX_M", 0.10),
            "z_min_m": _config_float("final_pass_v1_z_min_m", "FINAL_PASS_V1_Z_MIN_M", 0.15),
            "z_max_m": _config_float("final_pass_v1_z_max_m", "FINAL_PASS_V1_Z_MAX_M", 0.45),
            "distance_max_m": _config_float("final_pass_v1_distance_max_m", "FINAL_PASS_V1_DISTANCE_MAX_M", 0.30),
            "rel_speed_max_mps": _config_float("final_pass_v1_rel_speed_max_mps", "FINAL_PASS_V1_REL_SPEED_MAX_MPS", 0.40),
            "hold_min_sec": _config_float("final_pass_v1_hold_min_sec", "FINAL_PASS_V1_HOLD_MIN_SEC", 0.30),
        }
        loose_cfg = {
            "xy_abs_max_m": _config_float("final_pass_loose_xy_abs_max_m", "FINAL_PASS_LOOSE_XY_ABS_MAX_M", 0.15),
            "z_min_m": _config_float("final_pass_loose_z_min_m", "FINAL_PASS_LOOSE_Z_MIN_M", 0.15),
            "z_max_m": _config_float("final_pass_loose_z_max_m", "FINAL_PASS_LOOSE_Z_MAX_M", 0.55),
            "distance_max_m": _config_float("final_pass_loose_distance_max_m", "FINAL_PASS_LOOSE_DISTANCE_MAX_M", 0.40),
            "rel_speed_max_mps": _config_float("final_pass_loose_rel_speed_max_mps", "FINAL_PASS_LOOSE_REL_SPEED_MAX_MPS", 0.60),
            "hold_min_sec": _config_float("final_pass_loose_hold_min_sec", "FINAL_PASS_LOOSE_HOLD_MIN_SEC", 0.30),
        }

        summary["final_pass_v1_xy_abs_max_m_cfg"] = f"{v1_cfg['xy_abs_max_m']:.3f}"
        summary["final_pass_v1_z_min_m_cfg"] = f"{v1_cfg['z_min_m']:.3f}"
        summary["final_pass_v1_z_max_m_cfg"] = f"{v1_cfg['z_max_m']:.3f}"
        summary["final_pass_v1_distance_max_m_cfg"] = f"{v1_cfg['distance_max_m']:.3f}"
        summary["final_pass_v1_rel_speed_max_mps_cfg"] = f"{v1_cfg['rel_speed_max_mps']:.3f}"
        summary["final_pass_v1_hold_min_sec_cfg"] = f"{v1_cfg['hold_min_sec']:.3f}"
        summary["final_pass_loose_xy_abs_max_m_cfg"] = f"{loose_cfg['xy_abs_max_m']:.3f}"
        summary["final_pass_loose_z_min_m_cfg"] = f"{loose_cfg['z_min_m']:.3f}"
        summary["final_pass_loose_z_max_m_cfg"] = f"{loose_cfg['z_max_m']:.3f}"
        summary["final_pass_loose_distance_max_m_cfg"] = f"{loose_cfg['distance_max_m']:.3f}"
        summary["final_pass_loose_rel_speed_max_mps_cfg"] = f"{loose_cfg['rel_speed_max_mps']:.3f}"
        summary["final_pass_loose_hold_min_sec_cfg"] = f"{loose_cfg['hold_min_sec']:.3f}"

        final_phase = analysis_rows[-1]["phase"]

        v1_hold = _final_pass_hold(analysis_rows, v1_cfg)
        v1_pass = final_phase == "COMPLETED" and v1_hold >= v1_cfg["hold_min_sec"]
        summary["final_pass_v1_hold_sec"] = f"{v1_hold:.3f}"
        summary["final_pass_v1"] = "1" if v1_pass else "0"

        loose_hold = _final_pass_hold(analysis_rows, loose_cfg)
        loose_pass = final_phase == "COMPLETED" and loose_hold >= loose_cfg["hold_min_sec"]
        summary["final_pass_loose_hold_sec"] = f"{loose_hold:.3f}"
        summary["final_pass_loose"] = "1" if loose_pass else "0"

        # Backward-compatible aliases: final_pass == strict(v1).
        summary["final_pass_profile"] = "v1"
        summary["final_pass_xy_abs_max_m_cfg"] = summary["final_pass_v1_xy_abs_max_m_cfg"]
        summary["final_pass_z_min_m_cfg"] = summary["final_pass_v1_z_min_m_cfg"]
        summary["final_pass_z_max_m_cfg"] = summary["final_pass_v1_z_max_m_cfg"]
        summary["final_pass_distance_max_m_cfg"] = summary["final_pass_v1_distance_max_m_cfg"]
        summary["final_pass_rel_speed_max_mps_cfg"] = summary["final_pass_v1_rel_speed_max_mps_cfg"]
        summary["final_pass_hold_min_sec_cfg"] = summary["final_pass_v1_hold_min_sec_cfg"]
        summary["final_pass_hold_sec"] = summary["final_pass_v1_hold_sec"]
        summary["final_pass"] = summary["final_pass_v1"]
    if "mini_tecs_underspeed_ratio" in analysis_rows[0]:
        underspeed_values = finite_values(analysis_rows, "mini_tecs_underspeed_ratio")
        if underspeed_values:
            summary["mini_tecs_max_underspeed_ratio"] = f"{max(underspeed_values):.3f}"
    if "mini_tecs_true_airspeed_filtered_mps" in analysis_rows[0]:
        tas_values = finite_values(analysis_rows, "mini_tecs_true_airspeed_filtered_mps")
        if tas_values:
            summary["mini_tecs_min_tas_mps"] = f"{min(tas_values):.3f}"
            summary["mini_tecs_max_tas_mps"] = f"{max(tas_values):.3f}"
    if "mini_tecs_throttle_sp" in analysis_rows[0]:
        throttle_values = finite_values(analysis_rows, "mini_tecs_throttle_sp")
        if throttle_values:
            summary["mini_tecs_max_throttle_sp"] = f"{max(throttle_values):.3f}"
            summary["mini_tecs_min_throttle_sp"] = f"{min(throttle_values):.3f}"
    if analysis_rows:
        best_rows = [r for r in analysis_rows if r["phase"] != "IDLE"] or analysis_rows
        best_window = min(
            best_rows,
            key=lambda r: (
                r["relative_distance"] +
                0.18 * abs(r["rel_vx"]) +
                0.32 * abs(r["rel_vy"]) +
                0.12 * abs(r["rel_z"] - 0.2)
            ),
        )
        best_rel_speed = math.sqrt(
            best_window["rel_vx"] ** 2 +
            best_window["rel_vy"] ** 2 +
            best_window["rel_vz"] ** 2
        )
        best_vx = best_window["rel_vx"]
        best_vy = best_window["rel_vy"]
        best_vz = best_window["rel_vz"]
        best_score = (
            best_window["relative_distance"] +
            0.18 * abs(best_window["rel_vx"]) +
            0.32 * abs(best_window["rel_vy"]) +
            0.12 * abs(best_window["rel_z"] - 0.2)
        )
        summary["best_window_t_sec"] = f"{best_window['t']:.3f}"
        summary["best_window_phase"] = best_window["phase"]
        summary["best_window_distance_m"] = f"{best_window['relative_distance']:.3f}"
        summary["best_window_rel_vx_mps"] = f"{best_vx:.3f}"
        summary["best_window_rel_vy_mps"] = f"{best_vy:.3f}"
        summary["best_window_rel_vz_mps"] = f"{best_vz:.3f}"
        summary["best_window_rel_speed_mps"] = f"{best_rel_speed:.3f}"
        summary["best_window_score"] = f"{best_score:.3f}"

        completed_rows = [r for r in analysis_rows if r["phase"] == "COMPLETED"]
        if completed_rows:
            first_completed = completed_rows[0]
            summary["first_completed_t_sec"] = f"{first_completed['t']:.3f}"
            summary["first_completed_distance_m"] = f"{first_completed['relative_distance']:.3f}"
            summary["first_completed_rel_speed_mps"] = f"{math.sqrt(first_completed['rel_vx'] ** 2 + first_completed['rel_vy'] ** 2 + first_completed['rel_vz'] ** 2):.3f}"
            summary["post_completed_min_distance_m"] = (
                f"{min(r['relative_distance'] for r in completed_rows):.3f}"
            )
            summary["post_completed_max_distance_m"] = (
                f"{max(r['relative_distance'] for r in completed_rows):.3f}"
            )
            summary["post_completed_final_rel_speed_mps"] = (
                f"{math.sqrt(completed_rows[-1]['rel_vx'] ** 2 + completed_rows[-1]['rel_vy'] ** 2 + completed_rows[-1]['rel_vz'] ** 2):.3f}"
            )

    if "mini_orbit_center" in metadata and "mini_orbit_radius" in metadata:
        try:
            center_x, center_y = parse_pair(metadata["mini_orbit_center"])
            target_radius = float(metadata["mini_orbit_radius"])
            target_altitude = float(metadata.get("mini_takeoff_altitude", "nan"))
            orbit_rows = wait_orbit_rows(rows, metadata)
            if orbit_rows:
                radius_values = [
                    math.hypot(r["mini_x"] - center_x, r["mini_y"] - center_y)
                    for r in orbit_rows
                ]
                radius_errors = [value - target_radius for value in radius_values]
                summary["mini_wait_rows"] = str(len(orbit_rows))
                summary["mini_wait_orbit_radius_target_m"] = f"{target_radius:.3f}"
                summary["mini_wait_orbit_radius_mean_m"] = (
                    f"{sum(radius_values) / len(radius_values):.3f}"
                )
                summary["mini_wait_orbit_radius_min_m"] = f"{min(radius_values):.3f}"
                summary["mini_wait_orbit_radius_max_m"] = f"{max(radius_values):.3f}"
                summary["mini_wait_orbit_radius_drift_m"] = (
                    f"{radius_values[-1] - radius_values[0]:.3f}"
                )
                mean_radius_error = sum(radius_errors) / len(radius_errors)
                summary["mini_wait_orbit_radius_error_mean_m"] = f"{mean_radius_error:.3f}"
                summary["mini_wait_orbit_radius_abs_error_mean_m"] = (
                    f"{sum(abs(v) for v in radius_errors) / len(radius_errors):.3f}"
                )
                summary["mini_wait_orbit_radius_abs_error_max_m"] = (
                    f"{max(abs(v) for v in radius_errors):.3f}"
                )
                radius_variance = sum(
                    (value - (sum(radius_values) / len(radius_values))) ** 2
                    for value in radius_values
                ) / len(radius_values)
                summary["mini_wait_orbit_radius_std_m"] = f"{math.sqrt(radius_variance):.3f}"
                if math.isfinite(target_altitude):
                    alt_errors = [r["mini_z"] - target_altitude for r in orbit_rows]
                    summary["mini_wait_altitude_error_mean_m"] = (
                        f"{sum(alt_errors) / len(alt_errors):.3f}"
                    )
                    summary["mini_wait_altitude_abs_error_mean_m"] = (
                        f"{sum(abs(v) for v in alt_errors) / len(alt_errors):.3f}"
                    )
                summary["mini_wait_altitude_abs_error_max_m"] = (
                    f"{max(abs(v) for v in alt_errors):.3f}"
                )
        except ValueError:
            pass

    for key, value in path_metrics.items():
        if math.isfinite(value):
            summary[key] = f"{value:.3f}"

    for key, value in prehold_start_metrics.items():
        if isinstance(value, str):
            summary[key] = value
        elif math.isfinite(value):
            summary[key] = f"{value:.3f}"

    for key, value in intercept_metrics.items():
        if isinstance(value, str):
            summary[key] = value
        elif math.isfinite(value):
            summary[key] = f"{value:.3f}"

    for key, value in tangent_metrics.items():
        if isinstance(value, str):
            summary[key] = value
        elif math.isfinite(value):
            summary[key] = f"{value:.3f}"

    for key, value in accepted_window_metrics.items():
        if isinstance(value, str):
            summary[key] = value
        elif math.isfinite(value):
            summary[key] = f"{value:.3f}"

    summary_path = output_dir / "summary.txt"
    with summary_path.open("w", encoding="utf-8") as file:
        for key, value in summary.items():
            file.write(f"{key}={value}\n")
    return summary_path


def main():
    mock_mode = False
    args = sys.argv[1:]
    if args and args[0] == "--mock":
        mock_mode = True
        args = args[1:]

    if len(args) != 1:
        print("usage: generate_report.py [--mock] <result_dir>")
        raise SystemExit(1)

    output_dir = Path(args[0]).resolve()
    rows = load_rows(output_dir / "docking_log.csv")
    metadata = load_metadata(output_dir / "metadata.txt")
    if not rows:
        raise SystemExit("no rows found in docking_log.csv")

    corridor_plan = parse_corridor_plan_from_log(output_dir / "launch.log")
    if mock_mode:
        files = [
            save_speed_plot(rows, output_dir),
            save_xy_plot(rows, output_dir, corridor_plan),
            save_xy_animation_gif(rows, output_dir),
            save_x_time_plot(rows, output_dir, corridor_plan),
            save_summary(rows, output_dir, metadata),
        ]
    else:
        files = [
            save_distance_plot(rows, output_dir),
            save_signed_front_distance_plot(rows, output_dir),
            save_xy_plot(rows, output_dir),
            save_xz_plot(rows, output_dir),
            save_speed_plot(rows, output_dir),
            save_phase_plot(rows, output_dir),
            save_summary(rows, output_dir, metadata),
        ]
        intercept_diagnostics_path = save_intercept_diagnostics(rows, output_dir, metadata)
        if intercept_diagnostics_path is not None:
            files.append(intercept_diagnostics_path)
        wait_orbit_path = save_wait_orbit_plot(rows, output_dir, metadata)
        if wait_orbit_path is not None:
            files.append(wait_orbit_path)
        diagnostics_path = save_fixed_wing_diagnostics_plot(rows, output_dir)
        if diagnostics_path is not None:
            files.append(diagnostics_path)
        xy_gif_path = save_xy_animation_gif(rows, output_dir)
        if xy_gif_path is not None:
            files.append(xy_gif_path)

    manifest = output_dir / "artifacts.txt"
    with manifest.open("w", encoding="utf-8") as file:
        for path in files:
            file.write(f"{path.name}\n")

    print(output_dir)
    for path in files:
        print(path)


if __name__ == "__main__":
    main()
