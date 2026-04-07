#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent


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


def run_experiment(extra_env: dict[str, str]) -> Path:
    env = os.environ.copy()
    env.update(extra_env)
    completed = subprocess.run(
        [str(ROOT_DIR / "scripts" / "run_px4_sih_orbit_tracking_experiment.sh")],
        cwd=str(ROOT_DIR),
        text=True,
        capture_output=True,
        check=True,
        env=env,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("orbit_tracking experiment produced no result dir")
    result_dir = Path(lines[-1])
    if not result_dir.exists():
        raise RuntimeError(f"result dir not found: {result_dir}")
    return result_dir


def derive_catchability(result_dir: Path) -> tuple[int, str]:
    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT_DIR / "scripts" / "derive_tangent_catchability.py"),
            str(result_dir),
        ],
        cwd=str(ROOT_DIR),
        text=True,
        capture_output=True,
        check=True,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    candidate_count = 0
    best_line = ""
    for line in lines:
        if line.startswith("candidate_count="):
            candidate_count = int(line.split("=", 1)[1])
        elif line.startswith("best_candidate="):
            best_line = line
    return candidate_count, best_line


def summarize_result(result_dir: Path) -> tuple[str, str]:
    summary = read_kv(result_dir / "summary.txt")
    min_distance = summary.get("min_distance_m", "nan")
    final_phase = summary.get("final_phase", "IDLE")
    return min_distance, final_phase


def post_time_min_distance(result_dir: Path, time_sec: float) -> tuple[str, str]:
    rows_path = result_dir / "docking_log.csv"
    with rows_path.open("r", encoding="utf-8", errors="ignore") as file:
        rows = list(csv.DictReader(file))
    filtered = [row for row in rows if float(row["t"]) >= time_sec]
    if not filtered:
        return "nan", "IDLE"
    best = min(filtered, key=lambda row: float(row["relative_distance"]))
    return best["relative_distance"], best["phase"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Search coarse carrier start geometry for tangent-catchability.")
    parser.add_argument("--carrier-offset-x-list", default="54.0")
    parser.add_argument("--carrier-offset-y-list", default="-30.0")
    parser.add_argument("--carrier-offset-angle-deg-list", default="")
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--start-delay", default="auto")
    parser.add_argument("--analysis-start-sec", type=float, default=-1.0)
    parser.add_argument("--orbit-center-x", type=float, default=10.0)
    parser.add_argument("--orbit-center-y", type=float, default=-6.0)
    parser.add_argument("--orbit-radius", type=float, default=80.0)
    parser.add_argument("--outside-margin", type=float, default=10.0)
    args = parser.parse_args()

    x_values = [value.strip() for value in args.carrier_offset_x_list.split(",") if value.strip()]
    y_values = [value.strip() for value in args.carrier_offset_y_list.split(",") if value.strip()]
    angle_values = [value.strip() for value in args.carrier_offset_angle_deg_list.split(",") if value.strip()]
    if angle_values:
        x_values = []
        y_values = []
        ring_radius = args.orbit_radius + args.outside_margin
        for angle_deg in angle_values:
            angle_rad = math.radians(float(angle_deg))
            x_values.append(f"{args.orbit_center_x + ring_radius * math.cos(angle_rad):.1f}")
            y_values.append(f"{args.orbit_center_y + ring_radius * math.sin(angle_rad):.1f}")

    print("search_timestamp=" + time.strftime("%Y%m%d_%H%M%S"))
    if angle_values:
        offset_pairs = list(zip(x_values, y_values, angle_values))
    else:
        offset_pairs = [(offset_x, offset_y, "") for offset_x in x_values for offset_y in y_values]

    for offset_x, offset_y, angle_deg in offset_pairs:
            env = {
                "CARRIER_OFFSET_X": offset_x,
                "CARRIER_OFFSET_Y": offset_y,
                "EXPERIMENT_DURATION_SEC": str(args.duration_sec),
                "START_DELAY": str(args.start_delay),
            }
            angle_suffix = f" angle_deg={angle_deg}" if angle_deg else ""
            print(f"[search] offset=({offset_x},{offset_y}){angle_suffix}", flush=True)
            result_dir = run_experiment(env)
            candidate_count, best_line = derive_catchability(result_dir)
            min_distance, final_phase = summarize_result(result_dir)
            analysis_start_sec = (
                args.analysis_start_sec
                if args.analysis_start_sec >= 0.0
                else (float(args.start_delay) if args.start_delay not in {"auto", "never", "none"} else 0.0)
            )
            post_start_min_distance, post_start_phase = post_time_min_distance(result_dir, analysis_start_sec)
            print(
                f"result={result_dir} "
                f"min_distance={min_distance} "
                f"final_phase={final_phase} "
                f"post_start_min_distance={post_start_min_distance} "
                f"post_start_phase={post_start_phase} "
                f"candidate_count={candidate_count} "
                f"{best_line}",
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
