#!/usr/bin/env python3
import argparse
import math
from pathlib import Path
from typing import Dict, List

import pandas as pd


def parse_summary(summary_path: Path) -> Dict[str, float]:
    data: Dict[str, float] = {}
    if not summary_path.exists():
        return data
    for line in summary_path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        try:
            data[key] = float(value)
        except ValueError:
            continue
    return data


def build_run_metrics(run_dir: Path) -> Dict[str, float]:
    log_path = run_dir / "docking_log.csv"
    if not log_path.exists():
        return {"run_id": run_dir.name, "error": 1.0}

    df = pd.read_csv(log_path)
    if df.empty:
        return {"run_id": run_dir.name, "error": 2.0}

    summary = parse_summary(run_dir / "summary.txt")
    tracking = df["phase"] == "TRACKING"

    core_mask = (
        tracking
        & (df["controller_terminal_distance"] < 8.4)
        & (df["controller_terminal_along_error"] < -0.6)
        & (df["controller_terminal_lateral_error"].abs() < 3.6)
        & (df["rel_z"] > 0.25)
        & (df["rel_z"] < 0.95)
        & (df["controller_tracking_lateral_speed"].abs() < 4.5)
        & (df["rel_vz"].abs() < 3.5)
    )
    entry_like_mask = core_mask & (df["controller_terminal_along_error"] > -6.0)

    metrics: Dict[str, float] = {
        "run_id": run_dir.name,
        "error": 0.0,
        "final_pass": summary.get("final_pass", math.nan),
        "final_phase": df.iloc[-1]["phase"],
        "tracking_rows": float(tracking.sum()),
        "core_rows": float(core_mask.sum()),
        "entry_like_rows": float(entry_like_mask.sum()),
        "first_tracking_t_sec": float(df.loc[tracking, "t"].iloc[0]) if tracking.any() else math.nan,
        "first_core_t_sec": float(df.loc[core_mask, "t"].iloc[0]) if core_mask.any() else math.nan,
        "first_entry_like_t_sec": float(df.loc[entry_like_mask, "t"].iloc[0]) if entry_like_mask.any() else math.nan,
        "best_track_term_dist_m": float(df.loc[tracking, "controller_terminal_distance"].min()) if tracking.any() else math.nan,
        "best_core_term_dist_m": float(df.loc[core_mask, "controller_terminal_distance"].min()) if core_mask.any() else math.nan,
        "max_along_core_m": float(df.loc[core_mask, "controller_terminal_along_error"].max()) if core_mask.any() else math.nan,
        "max_along_entry_like_m": float(df.loc[entry_like_mask, "controller_terminal_along_error"].max()) if entry_like_mask.any() else math.nan,
    }

    if core_mask.any():
        max_along_core = metrics["max_along_core_m"]
        metrics["ahead_upper_gap_m"] = max(0.0, -6.0 - max_along_core)
    else:
        metrics["ahead_upper_gap_m"] = math.nan

    return metrics


def resolve_run_dirs(result_root: Path, run_ids: List[str], prefix: str) -> List[Path]:
    if run_ids:
        dirs = [result_root / rid for rid in run_ids]
    else:
        dirs = sorted(result_root.glob(f"{prefix}*_px4_sih"))
    return [d for d in dirs if d.exists() and d.is_dir()]


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze TRACKING->DOCKING entry blockers from logs.")
    parser.add_argument("--results-root", default="results", help="Results directory root.")
    parser.add_argument("--run-id", action="append", default=[], help="Run id(s), repeatable.")
    parser.add_argument("--prefix", default="", help="Prefix filter when run-id not provided.")
    parser.add_argument("--out-csv", default="", help="Output CSV path.")
    args = parser.parse_args()

    result_root = Path(args.results_root)
    run_dirs = resolve_run_dirs(result_root, args.run_id, args.prefix)
    if not run_dirs:
        raise SystemExit("No matching run directories found.")

    rows = [build_run_metrics(run_dir) for run_dir in run_dirs]
    out = pd.DataFrame(rows)
    out = out.sort_values("run_id")

    if args.out_csv:
        out_path = Path(args.out_csv)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out.to_csv(out_path, index=False, float_format="%.6f")
        print(f"WROTE {out_path}")

    with pd.option_context("display.max_columns", None, "display.width", 200):
        print(out.to_string(index=False))


if __name__ == "__main__":
    main()
