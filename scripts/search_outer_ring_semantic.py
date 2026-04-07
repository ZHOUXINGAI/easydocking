#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent


def run_experiment(extra_env: dict[str, str]) -> Path:
    env = os.environ.copy()
    env.update(extra_env)
    completed = subprocess.run(
        [str(ROOT_DIR / "scripts" / "run_px4_sih_docking_experiment.sh")],
        cwd=str(ROOT_DIR),
        text=True,
        capture_output=True,
        check=True,
        env=env,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("experiment produced no result dir")
    result_dir = Path(lines[-1])
    if not result_dir.exists():
        raise RuntimeError(f"result dir not found: {result_dir}")
    return result_dir


def check_semantics(result_dir: Path) -> tuple[int, str]:
    completed = subprocess.run(
        [sys.executable, str(ROOT_DIR / "scripts" / "check_tangent_release_semantics.py"), str(result_dir)],
        cwd=str(ROOT_DIR),
        text=True,
        capture_output=True,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    joined = "; ".join(lines)
    return completed.returncode, joined


def main() -> int:
    parser = argparse.ArgumentParser(description="Search carrier outer-ring start angles with strong semantic check.")
    parser.add_argument("--angles-deg", default="-150,-120,-90,-60,-30,0,30,60,90,120,150,180")
    parser.add_argument("--orbit-center-x", type=float, default=10.0)
    parser.add_argument("--orbit-center-y", type=float, default=-6.0)
    parser.add_argument("--orbit-radius", type=float, default=80.0)
    parser.add_argument("--outside-margin", type=float, default=10.0)
    parser.add_argument("--start-delay", default="35.0")
    parser.add_argument("--duration-sec", type=float, default=85.0)
    args = parser.parse_args()

    angles = [float(item) for item in args.angles_deg.split(",") if item.strip()]
    ring_radius = args.orbit_radius + args.outside_margin

    print("search_timestamp=" + time.strftime("%Y%m%d_%H%M%S"))
    for angle_deg in angles:
        angle_rad = math.radians(angle_deg)
        carrier_x = args.orbit_center_x + ring_radius * math.cos(angle_rad)
        carrier_y = args.orbit_center_y + ring_radius * math.sin(angle_rad)
        env = {
            "CARRIER_OFFSET_X": f"{carrier_x:.1f}",
            "CARRIER_OFFSET_Y": f"{carrier_y:.1f}",
            "START_DELAY": str(args.start_delay),
            "EXPERIMENT_DURATION_SEC": str(args.duration_sec),
        }
        print(
            f"[search] angle_deg={angle_deg:.1f} "
            f"carrier_offset=({carrier_x:.1f},{carrier_y:.1f})",
            flush=True,
        )
        result_dir = run_experiment(env)
        semantic_code, semantic_text = check_semantics(result_dir)
        print(
            f"result={result_dir} "
            f"semantic_code={semantic_code} "
            f"{semantic_text}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
