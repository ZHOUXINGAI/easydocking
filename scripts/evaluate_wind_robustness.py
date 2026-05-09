#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import Dict, List


def read_kv_file(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    if not path.exists():
        return data
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def safe_float(value: str) -> float:
    try:
        return float(value)
    except Exception:
        return math.nan


def percentile(values: List[float], p: float) -> float:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    rank = (len(finite) - 1) * p
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return finite[lower]
    w = rank - lower
    return finite[lower] * (1.0 - w) + finite[upper] * w


def latest_result_dirs(root: Path, latest: int) -> List[Path]:
    dirs = [d for d in root.glob("*_px4_sih") if d.is_dir()]
    dirs.sort(key=lambda d: d.name)
    return dirs[-latest:]


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate wind robustness stats from SIH runs.")
    parser.add_argument("--latest", type=int, default=0)
    parser.add_argument("result_dirs", nargs="*")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    results_root = repo_root / "results"

    targets: List[Path] = []
    if args.latest > 0:
        targets.extend(latest_result_dirs(results_root, args.latest))
    for item in args.result_dirs:
        path = Path(item)
        if not path.is_absolute():
            path = repo_root / item
        if path.exists():
            targets.append(path)
    dedup = sorted({p.resolve() for p in targets}, key=lambda p: p.name)
    if not dedup:
        print("no result dirs found")
        return 1

    windy_runs = []
    final_pass_count = 0
    fields = {
        "final_abs_xy_max_m": [],
        "final_rel_speed_mps": [],
        "final_distance_m": [],
        "mini_wait_orbit_radius_abs_error_mean_m": [],
        "mini_wait_altitude_error_mean_m": [],
    }

    for run_dir in dedup:
        summary = read_kv_file(run_dir / "summary.txt")
        metadata = read_kv_file(run_dir / "metadata.txt")
        classification = read_kv_file(run_dir / "classification.txt")
        wind_enabled = metadata.get("wind_profile_enabled", "false").lower() == "true"
        if not wind_enabled:
            continue
        windy_runs.append(run_dir)
        if classification.get("classification", "") == "final-pass":
            final_pass_count += 1
        for key in fields:
            fields[key].append(safe_float(summary.get(key, "nan")))

    if not windy_runs:
        print("no windy runs found in selected results")
        return 2

    print(f"windy_runs={len(windy_runs)} final_pass={final_pass_count}")
    for key, values in fields.items():
        p95 = percentile(values, 0.95)
        p99 = percentile(values, 0.99)
        mean = (
            sum(v for v in values if math.isfinite(v)) /
            max(sum(1 for v in values if math.isfinite(v)), 1)
        )
        print(f"{key}_mean={mean:.3f}")
        print(f"{key}_p95={p95:.3f}")
        print(f"{key}_p99={p99:.3f}")
    print("runs:")
    for run_dir in windy_runs:
        print(run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
