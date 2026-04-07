#!/usr/bin/env python3

import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


def load_rows(csv_path: Path):
    rows = []
    with csv_path.open("r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
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


def save_xy_plot(rows, output_dir: Path):
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
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Planar Docking Trajectory (Focused)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
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
    ax_full.set_xlabel("X [m]")
    ax_full.set_ylabel("Y [m]")
    ax_full.set_title("Planar Docking Trajectory (Full)")
    ax_full.axis("equal")
    ax_full.grid(True, alpha=0.3)
    ax_full.legend()
    fig_full.tight_layout()
    fig_full.savefig(output_dir / "trajectory_xy_full.png")
    plt.close(fig_full)
    return path


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

    # FINAL_PASS (initial definition): completed + tight terminal constraints + sustained hold.
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

        final_pass_xy_abs_max_m = 0.10
        final_pass_z_min_m = 0.15
        final_pass_z_max_m = 0.45
        final_pass_distance_max_m = 0.30
        final_pass_rel_speed_max_mps = 0.40
        final_pass_hold_min_sec = 0.30

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
                abs(rel_x) <= final_pass_xy_abs_max_m and
                abs(rel_y) <= final_pass_xy_abs_max_m and
                final_pass_z_min_m <= rel_z <= final_pass_z_max_m and
                distance <= final_pass_distance_max_m and
                rel_speed <= final_pass_rel_speed_max_mps
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

        final_phase = analysis_rows[-1]["phase"]
        final_pass = final_phase == "COMPLETED" and best_hold >= final_pass_hold_min_sec
        summary["final_pass_hold_sec"] = f"{best_hold:.3f}"
        summary["final_pass"] = "1" if final_pass else "0"
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

    for key, value in tangent_metrics.items():
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
    if len(sys.argv) != 2:
        print("usage: generate_report.py <result_dir>")
        raise SystemExit(1)

    output_dir = Path(sys.argv[1]).resolve()
    rows = load_rows(output_dir / "docking_log.csv")
    metadata = load_metadata(output_dir / "metadata.txt")
    if not rows:
        raise SystemExit("no rows found in docking_log.csv")

    files = [
        save_distance_plot(rows, output_dir),
        save_xy_plot(rows, output_dir),
        save_xz_plot(rows, output_dir),
        save_speed_plot(rows, output_dir),
        save_phase_plot(rows, output_dir),
        save_summary(rows, output_dir, metadata),
    ]
    wait_orbit_path = save_wait_orbit_plot(rows, output_dir, metadata)
    if wait_orbit_path is not None:
        files.append(wait_orbit_path)
    diagnostics_path = save_fixed_wing_diagnostics_plot(rows, output_dir)
    if diagnostics_path is not None:
        files.append(diagnostics_path)

    manifest = output_dir / "artifacts.txt"
    with manifest.open("w", encoding="utf-8") as file:
        for path in files:
            file.write(f"{path.name}\n")

    print(output_dir)
    for path in files:
        print(path)


if __name__ == "__main__":
    main()
