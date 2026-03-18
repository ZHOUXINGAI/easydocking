#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


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


def phase_color(phase: str) -> str:
    return {
        "IDLE": "#7f7f7f",
        "TAKEOFF": "#1f77b4",
        "APPROACH": "#ff7f0e",
        "TRACKING": "#2ca02c",
        "DOCKING": "#d62728",
        "COMPLETED": "#9467bd",
    }.get(phase, "#333333")


def main():
    if len(sys.argv) != 2:
        print("usage: generate_animation.py <result_dir>")
        raise SystemExit(1)

    result_dir = Path(sys.argv[1]).resolve()
    rows = load_rows(result_dir / "docking_log.csv")
    if not rows:
        raise SystemExit("no rows found")

    times = [r["t"] for r in rows]
    carrier_x = [r["carrier_x"] for r in rows]
    carrier_y = [r["carrier_y"] for r in rows]
    mini_x = [r["mini_x"] for r in rows]
    mini_y = [r["mini_y"] for r in rows]
    distances = [r["relative_distance"] for r in rows]

    min_x = min(min(carrier_x), min(mini_x)) - 3.0
    max_x = max(max(carrier_x), max(mini_x)) + 3.0
    min_y = min(min(carrier_y), min(mini_y)) - 3.0
    max_y = max(max(carrier_y), max(mini_y)) + 3.0

    stride = max(1, len(rows) // 300)
    frame_indices = list(range(0, len(rows), stride))
    if frame_indices[-1] != len(rows) - 1:
        frame_indices.append(len(rows) - 1)

    fig = plt.figure(figsize=(10.5, 6.2), dpi=150)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1.0], height_ratios=[1.0, 1.0])
    ax_xy = fig.add_subplot(gs[:, 0])
    ax_dist = fig.add_subplot(gs[0, 1])
    ax_text = fig.add_subplot(gs[1, 1])

    ax_xy.set_title("Cooperative Docking Replay")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_xlim(min_x, max_x)
    ax_xy.set_ylim(min_y, max_y)
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.25)

    ax_dist.set_title("Relative Distance")
    ax_dist.set_xlabel("Time [s]")
    ax_dist.set_ylabel("Distance [m]")
    ax_dist.set_xlim(times[0], times[-1])
    ax_dist.set_ylim(0.0, max(distances) * 1.08)
    ax_dist.grid(True, alpha=0.25)
    ax_dist.plot(times, distances, color="#0b84a5", lw=2.0)

    ax_text.axis("off")
    summary_text = ax_text.text(
        0.02,
        0.95,
        "",
        va="top",
        ha="left",
        family="monospace",
        fontsize=10,
    )

    carrier_traj, = ax_xy.plot([], [], color="#2ca02c", lw=2.4, label="Carrier")
    mini_traj, = ax_xy.plot([], [], color="#d62728", lw=2.1, label="Fixed-wing")
    carrier_dot, = ax_xy.plot([], [], marker="o", ms=8, color="#2ca02c")
    mini_dot, = ax_xy.plot([], [], marker="o", ms=8, color="#d62728")
    distance_cursor = ax_dist.axvline(times[0], color="#111111", lw=1.6, alpha=0.8)
    phase_badge = ax_xy.text(
        0.02,
        0.98,
        "",
        transform=ax_xy.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        color="white",
        bbox={"boxstyle": "round,pad=0.35", "fc": "#333333", "ec": "none", "alpha": 0.9},
    )
    ax_xy.legend(loc="lower right")

    def update(frame_idx):
        row_idx = frame_indices[frame_idx]
        row = rows[row_idx]
        carrier_traj.set_data(carrier_x[:row_idx + 1], carrier_y[:row_idx + 1])
        mini_traj.set_data(mini_x[:row_idx + 1], mini_y[:row_idx + 1])
        carrier_dot.set_data([carrier_x[row_idx]], [carrier_y[row_idx]])
        mini_dot.set_data([mini_x[row_idx]], [mini_y[row_idx]])
        distance_cursor.set_xdata([times[row_idx], times[row_idx]])

        phase = row["phase"]
        phase_badge.set_text(f"{phase}  t={row['t']:.1f}s")
        phase_badge.set_bbox(
            {"boxstyle": "round,pad=0.35", "fc": phase_color(phase), "ec": "none", "alpha": 0.92}
        )

        rel_speed = math.sqrt(row["rel_vx"] ** 2 + row["rel_vy"] ** 2 + row["rel_vz"] ** 2)
        position_error = math.sqrt(
            row["rel_x"] ** 2 + row["rel_y"] ** 2 + (row["rel_z"] - 0.8) ** 2
        )
        summary_text.set_text(
            "\n".join([
                f"time        : {row['t']:.2f} s",
                f"phase       : {phase}",
                f"distance    : {row['relative_distance']:.3f} m",
                f"true error  : {position_error:.3f} m",
                f"rel pos     : ({row['rel_x']:.3f}, {row['rel_y']:.3f}, {row['rel_z']:.3f})",
                f"rel vel     : ({row['rel_vx']:.3f}, {row['rel_vy']:.3f}, {row['rel_vz']:.3f})",
                f"|rel vel|   : {rel_speed:.3f} m/s",
            ])
        )

        return (
            carrier_traj,
            mini_traj,
            carrier_dot,
            mini_dot,
            distance_cursor,
            phase_badge,
            summary_text,
        )

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=80,
        blit=False,
    )

    output_path = result_dir / "trajectory_replay.gif"
    anim.save(output_path, writer=PillowWriter(fps=12))
    plt.close(fig)
    print(output_path)


if __name__ == "__main__":
    main()
