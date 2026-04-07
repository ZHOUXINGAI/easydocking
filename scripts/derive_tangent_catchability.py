#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="ignore") as file:
        return list(csv.DictReader(file))


def tangent_metrics(rows: list[dict[str, str]], index: int) -> dict[str, float] | None:
    if index <= 0 or index >= len(rows) - 1:
        return None

    prev_row = rows[index - 1]
    row = rows[index]
    next_row = rows[index + 1]
    dt = max(float(next_row["t"]) - float(prev_row["t"]), 1e-6)

    mini_x = float(row["mini_x"])
    mini_y = float(row["mini_y"])
    mini_vx = (float(next_row["mini_x"]) - float(prev_row["mini_x"])) / dt
    mini_vy = (float(next_row["mini_y"]) - float(prev_row["mini_y"])) / dt
    mini_speed = math.hypot(mini_vx, mini_vy)
    if mini_speed <= 1e-6:
        return None

    axis_x = mini_vx / mini_speed
    axis_y = mini_vy / mini_speed
    lateral_x = -axis_y
    lateral_y = axis_x

    carrier_x = float(row["carrier_x"])
    carrier_y = float(row["carrier_y"])
    delta_x = carrier_x - mini_x
    delta_y = carrier_y - mini_y

    carrier_vx = (float(next_row["carrier_x"]) - float(prev_row["carrier_x"])) / dt
    carrier_vy = (float(next_row["carrier_y"]) - float(prev_row["carrier_y"])) / dt
    rel_vx = carrier_vx - mini_vx
    rel_vy = carrier_vy - mini_vy

    forward_progress = delta_x * axis_x + delta_y * axis_y
    lateral_error = delta_x * lateral_x + delta_y * lateral_y
    forward_rate = rel_vx * axis_x + rel_vy * axis_y
    lateral_rate = rel_vx * lateral_x + rel_vy * lateral_y

    time_to_line = math.inf
    predicted_forward_at_line = math.nan
    if abs(lateral_rate) > 1e-6:
        time_to_line = -lateral_error / lateral_rate
        predicted_forward_at_line = forward_progress + forward_rate * time_to_line

    return {
        "t": float(row["t"]),
        "phase": row["phase"],
        "relative_distance": float(row["relative_distance"]),
        "forward_progress": forward_progress,
        "lateral_error": lateral_error,
        "forward_rate": forward_rate,
        "lateral_rate": lateral_rate,
        "time_to_line": time_to_line,
        "predicted_forward_at_line": predicted_forward_at_line,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Derive tangent catchability windows from one result dir.")
    parser.add_argument("result_dir")
    parser.add_argument("--time-to-line-max", type=float, default=8.0)
    parser.add_argument("--predicted-forward-min", type=float, default=2.0)
    parser.add_argument("--relative-distance-max", type=float, default=50.0)
    parser.add_argument("--output-csv", default="")
    args = parser.parse_args()

    result_dir = Path(args.result_dir)
    rows = read_rows(result_dir / "docking_log.csv")

    candidates: list[dict[str, float | str]] = []
    for index in range(1, len(rows) - 1):
        metrics = tangent_metrics(rows, index)
        if metrics is None:
            continue
        time_to_line = float(metrics["time_to_line"])
        predicted_forward_at_line = float(metrics["predicted_forward_at_line"])
        lateral_error = float(metrics["lateral_error"])
        lateral_rate = float(metrics["lateral_rate"])
        if not math.isfinite(time_to_line):
            continue
        if metrics["relative_distance"] > args.relative_distance_max:
            continue
        if not (0.2 <= time_to_line <= args.time_to_line_max):
            continue
        if lateral_error * lateral_rate >= 0.0:
            continue
        if predicted_forward_at_line < args.predicted_forward_min:
            continue

        score = (
            abs(predicted_forward_at_line - 6.0) +
            0.35 * abs(lateral_error) +
            0.6 * abs(time_to_line - 3.0)
        )
        enriched = dict(metrics)
        enriched["score"] = score
        candidates.append(enriched)

    candidates.sort(key=lambda item: float(item["score"]))

    if args.output_csv:
        output_path = Path(args.output_csv)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "t",
            "phase",
            "relative_distance",
            "forward_progress",
            "lateral_error",
            "forward_rate",
            "lateral_rate",
            "time_to_line",
            "predicted_forward_at_line",
            "score",
        ]
        with output_path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(candidates)

    print(f"candidate_count={len(candidates)}")
    if candidates:
        best = candidates[0]
        print(
            "best_candidate="
            f"t={float(best['t']):.3f},"
            f"phase={best['phase']},"
            f"relative_distance={float(best['relative_distance']):.3f},"
            f"forward_progress={float(best['forward_progress']):.3f},"
            f"lateral_error={float(best['lateral_error']):.3f},"
            f"time_to_line={float(best['time_to_line']):.3f},"
            f"predicted_forward_at_line={float(best['predicted_forward_at_line']):.3f},"
            f"score={float(best['score']):.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
