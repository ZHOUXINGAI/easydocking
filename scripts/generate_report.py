#!/usr/bin/env python3

import csv
import math
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
    fig, ax = plt.subplots(figsize=(6.5, 6.0), dpi=160)
    ax.plot([r["carrier_x"] for r in rows], [r["carrier_y"] for r in rows], color="#4daf4a", lw=2.5, label="Carrier")
    ax.plot([r["mini_x"] for r in rows], [r["mini_y"] for r in rows], color="#e41a1c", lw=2.0, label="Mini")
    ax.scatter(rows[0]["mini_x"], rows[0]["mini_y"], color="#ff7f00", s=40, label="Mini start")
    ax.scatter(rows[-1]["mini_x"], rows[-1]["mini_y"], color="#377eb8", s=40, label="Mini end")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Planar Docking Trajectory")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    path = output_dir / "trajectory_xy.png"
    fig.savefig(path)
    plt.close(fig)
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
    carrier_speed = []
    mini_speed = []
    for previous, current in zip(rows, rows[1:]):
        dt = max(current["t"] - previous["t"], 1e-3)
        carrier_speed.append(math.hypot(
            current["carrier_x"] - previous["carrier_x"],
            current["carrier_y"] - previous["carrier_y"],
        ) / dt)
        mini_speed.append(math.hypot(
            current["mini_x"] - previous["mini_x"],
            current["mini_y"] - previous["mini_y"],
        ) / dt)
    time_axis = [r["t"] for r in rows[1:]]
    fig, ax = plt.subplots(figsize=(8.0, 4.5), dpi=160)
    ax.plot(time_axis, carrier_speed, color="#4daf4a", lw=2.0, label="Carrier speed")
    ax.plot(time_axis, mini_speed, color="#e41a1c", lw=2.0, label="Mini speed")
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


def save_summary(rows, output_dir: Path):
    summary = defaultdict(str)
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
    if not rows:
        raise SystemExit("no rows found in docking_log.csv")

    files = [
        save_distance_plot(rows, output_dir),
        save_xy_plot(rows, output_dir),
        save_xz_plot(rows, output_dir),
        save_speed_plot(rows, output_dir),
        save_phase_plot(rows, output_dir),
        save_summary(rows, output_dir),
    ]

    manifest = output_dir / "artifacts.txt"
    with manifest.open("w", encoding="utf-8") as file:
        for path in files:
            file.write(f"{path.name}\n")

    print(output_dir)
    for path in files:
        print(path)


if __name__ == "__main__":
    main()
