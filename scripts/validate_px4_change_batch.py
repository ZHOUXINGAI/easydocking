#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent
RESULTS_DIR = ROOT_DIR / "results"


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


@dataclass
class RunResult:
    index: int
    result_dir: Path
    success: bool
    reason: str


def launch_experiment(cmd: list[str], cwd: Path, extra_env: dict[str, str] | None = None) -> Path:
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    completed = subprocess.run(
        cmd,
        cwd=str(cwd),
        check=True,
        text=True,
        capture_output=True,
        env=env,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError(f"experiment produced no stdout: {' '.join(cmd)}")
    result_dir = Path(lines[-1])
    if not result_dir.exists():
        raise RuntimeError(f"result dir does not exist: {result_dir}")
    return result_dir


def classify_semantic(result_dir: Path) -> tuple[bool, str]:
    launch_log = (result_dir / "launch.log").read_text(encoding="utf-8", errors="ignore")
    forbidden = [
        "fixed-wing: glide window accepted phase=TRACKING",
        "fixed-wing: glide window accepted phase=APPROACH",
        "fixed-wing: glide window accepted phase=TAKEOFF",
    ]
    for marker in forbidden:
        if marker in launch_log:
            return False, marker

    glide_lines = [
        line.strip()
        for line in launch_log.splitlines()
        if "fixed-wing: glide window accepted phase=" in line
    ]
    if glide_lines:
        return True, glide_lines[-1]
    return True, "no_glide_activation_before_timeout"


def classify_semantic_activation(result_dir: Path) -> tuple[bool, str]:
    semantic_ok, semantic_reason = classify_semantic(result_dir)
    if not semantic_ok:
        return False, semantic_reason

    start_log = (result_dir / "start_command.log").read_text(encoding="utf-8", errors="ignore")
    if "Auto START sent" not in start_log:
        return False, "auto_start_not_sent"
    if "Auto START confirmed active" not in start_log:
        return False, "auto_start_not_confirmed"

    summary = read_kv(result_dir / "summary.txt")
    best_phase = summary.get("best_window_phase", "IDLE").upper()
    final_phase = summary.get("final_phase", "IDLE").upper()
    if best_phase == "IDLE" and final_phase == "IDLE":
        return False, "controller_never_left_idle"

    return True, f"{semantic_reason}; best_phase={best_phase}; final_phase={final_phase}"


def classify_tracking_progress(result_dir: Path) -> tuple[bool, str]:
    activation_ok, activation_reason = classify_semantic_activation(result_dir)
    if not activation_ok:
        return False, activation_reason

    summary = read_kv(result_dir / "summary.txt")
    best_phase = summary.get("best_window_phase", "IDLE").upper()
    final_phase = summary.get("final_phase", "IDLE").upper()
    accepted = {"TRACKING", "DOCKING", "COMPLETED"}
    if best_phase not in accepted and final_phase not in accepted:
        return False, f"no_tracking_progress best_phase={best_phase} final_phase={final_phase}"

    return True, f"{activation_reason}; best_phase={best_phase}; final_phase={final_phase}"


def classify_orbit_only(
    result_dir: Path,
    radius_std_max: float,
    radius_abs_error_max: float,
    altitude_abs_error_max: float,
    min_wait_rows: int,
) -> tuple[bool, str]:
    summary = read_kv(result_dir / "summary.txt")
    try:
        wait_rows = int(summary.get("mini_wait_rows", "0"))
        radius_std = float(summary.get("mini_wait_orbit_radius_std_m", "inf"))
        radius_abs_error = float(summary.get("mini_wait_orbit_radius_abs_error_max_m", "inf"))
        altitude_abs_error = float(summary.get("mini_wait_altitude_abs_error_max_m", "inf"))
    except ValueError:
        return False, "invalid_orbit_summary"

    if wait_rows < min_wait_rows:
        return False, f"wait_rows<{min_wait_rows}"
    if radius_std > radius_std_max:
        return False, f"radius_std={radius_std:.3f}>{radius_std_max:.3f}"
    if radius_abs_error > radius_abs_error_max:
        return False, f"radius_abs_error={radius_abs_error:.3f}>{radius_abs_error_max:.3f}"
    if altitude_abs_error > altitude_abs_error_max:
        return False, f"altitude_abs_error={altitude_abs_error:.3f}>{altitude_abs_error_max:.3f}"
    return True, (
        f"radius_std={radius_std:.3f}, "
        f"radius_abs_error={radius_abs_error:.3f}, "
        f"altitude_abs_error={altitude_abs_error:.3f}"
    )


def classify_orbit_tracking(result_dir: Path) -> tuple[bool, str]:
    launch_log = (result_dir / "launch.log").read_text(encoding="utf-8", errors="ignore")
    if "fixed-wing: glide window accepted" in launch_log:
        return False, "unexpected_glide_activation"

    start_log = (result_dir / "start_command.log").read_text(encoding="utf-8", errors="ignore")
    if "Auto START sent" not in start_log and "publishing #1" not in start_log:
        return False, "start_not_sent"

    summary = read_kv(result_dir / "summary.txt")
    best_phase = summary.get("best_window_phase", "IDLE").upper()
    final_phase = summary.get("final_phase", "IDLE").upper()
    if best_phase not in {"TRACKING", "DOCKING", "COMPLETED"} and final_phase not in {"TRACKING", "DOCKING", "COMPLETED"}:
        return False, f"no_tracking_progress best_phase={best_phase} final_phase={final_phase}"

    try:
        min_distance = float(summary.get("min_distance_m", "inf"))
    except ValueError:
        return False, "invalid_min_distance"
    if not math.isfinite(min_distance):
        return False, "invalid_min_distance"

    return True, f"best_phase={best_phase}; final_phase={final_phase}; min_distance={min_distance:.3f}"


def classify_score_handoff_progress(result_dir: Path) -> tuple[bool, str]:
    launch_log = (result_dir / "launch.log").read_text(encoding="utf-8", errors="ignore")
    if "fixed-wing: glide score release accepted" not in launch_log:
        return False, "score_release_not_accepted"
    if "fixed-wing: glide window accepted" not in launch_log:
        return False, "glide_window_not_accepted"

    summary = read_kv(result_dir / "summary.txt")
    best_phase = summary.get("best_window_phase", "IDLE").upper()
    final_phase = summary.get("final_phase", "IDLE").upper()
    try:
        min_distance = float(summary.get("min_distance_m", "inf"))
    except ValueError:
        return False, "invalid_min_distance"
    if not math.isfinite(min_distance):
        return False, "invalid_min_distance"
    if min_distance > 12.0:
        return False, f"min_distance={min_distance:.3f}>12.000"

    return True, (
        f"best_phase={best_phase}; "
        f"final_phase={final_phase}; "
        f"min_distance={min_distance:.3f}"
    )


def classify_tangent_semantic(
    result_dir: Path,
    *,
    min_tangent_duration_sec: float,
    max_radius_drop_1s_m: float,
    max_heading_change_1s_deg: float,
) -> tuple[bool, str]:
    command = [
        sys.executable,
        str(ROOT_DIR / "scripts" / "check_tangent_release_semantics.py"),
        str(result_dir),
        "--radius-window-sec", "1.2",
        "--max-radius-drop", str(max_radius_drop_1s_m),
        "--min-tangent-duration-sec", str(min_tangent_duration_sec),
    ]
    completed = subprocess.run(
        command,
        cwd=str(ROOT_DIR),
        text=True,
        capture_output=True,
    )
    output_lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    reason = output_lines[-1] if output_lines else "no_tangent_semantic_output"
    if output_lines:
        reason = "; ".join(output_lines)
    return completed.returncode == 0, reason


def main() -> int:
    parser = argparse.ArgumentParser(description="Run repeated PX4 validation batches.")
    parser.add_argument(
        "--mode",
        choices=[
            "semantic",
            "semantic_activation",
            "tracking_progress",
            "orbit_only",
            "orbit_tracking",
            "score_handoff_progress",
            "tangent_semantic",
        ],
        required=True,
    )
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--success-min", type=int, default=9)
    parser.add_argument("--label", default="")
    parser.add_argument("--radius-std-max", type=float, default=8.0)
    parser.add_argument("--radius-abs-error-max", type=float, default=12.0)
    parser.add_argument("--altitude-abs-error-max", type=float, default=8.0)
    parser.add_argument("--min-wait-rows", type=int, default=150)
    parser.add_argument("--min-tangent-duration-sec", type=float, default=1.0)
    parser.add_argument("--max-radius-drop-1s-m", type=float, default=0.5)
    parser.add_argument("--max-heading-change-1s-deg", type=float, default=12.0)
    args = parser.parse_args()

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    label_suffix = f"_{args.label}" if args.label else ""
    batch_dir = RESULTS_DIR / f"{timestamp}_{args.mode}_batch{label_suffix}"
    batch_dir.mkdir(parents=True, exist_ok=True)

    extra_env: dict[str, str] | None = None
    if args.mode in {"semantic", "semantic_activation", "tracking_progress", "score_handoff_progress", "tangent_semantic"}:
        cmd = [str(ROOT_DIR / "scripts" / "run_px4_sih_docking_experiment.sh")]
        if args.mode == "score_handoff_progress":
            extra_env = {
                "MINI_GLIDE_RELEASE_ENABLED": "true",
                "MINI_GLIDE_RELEASE_MODE": "score_state_machine",
            }
        elif args.mode == "tangent_semantic":
            extra_env = {
                "MINI_GLIDE_RELEASE_ENABLED": "true",
                "MINI_GLIDE_RELEASE_MODE": "score_state_machine",
            }
    elif args.mode == "orbit_tracking":
        cmd = [str(ROOT_DIR / "scripts" / "run_px4_sih_orbit_tracking_experiment.sh")]
    else:
        cmd = [str(ROOT_DIR / "scripts" / "run_px4_sih_orbit_only_experiment.sh")]

    run_results: list[RunResult] = []
    max_failures = max(args.repeats - args.success_min, 0)
    for idx in range(1, args.repeats + 1):
        print(f"[batch] run {idx}/{args.repeats} mode={args.mode}", flush=True)
        result_dir = launch_experiment(cmd, ROOT_DIR, extra_env=extra_env)
        if args.mode == "semantic":
            success, reason = classify_semantic(result_dir)
        elif args.mode == "semantic_activation":
            success, reason = classify_semantic_activation(result_dir)
        elif args.mode == "tracking_progress":
            success, reason = classify_tracking_progress(result_dir)
        elif args.mode == "orbit_tracking":
            success, reason = classify_orbit_tracking(result_dir)
        elif args.mode == "score_handoff_progress":
            success, reason = classify_score_handoff_progress(result_dir)
        elif args.mode == "tangent_semantic":
            success, reason = classify_tangent_semantic(
                result_dir,
                min_tangent_duration_sec=args.min_tangent_duration_sec,
                max_radius_drop_1s_m=args.max_radius_drop_1s_m,
                max_heading_change_1s_deg=args.max_heading_change_1s_deg,
            )
        else:
            success, reason = classify_orbit_only(
                result_dir,
                radius_std_max=args.radius_std_max,
                radius_abs_error_max=args.radius_abs_error_max,
                altitude_abs_error_max=args.altitude_abs_error_max,
                min_wait_rows=args.min_wait_rows,
            )
        run_results.append(RunResult(idx, result_dir, success, reason))
        print(f"  -> {'PASS' if success else 'FAIL'} {result_dir.name} :: {reason}", flush=True)

        success_count = sum(1 for result in run_results if result.success)
        fail_count = sum(1 for result in run_results if not result.success)
        if success_count >= args.success_min:
            print(
                f"[batch] early-stop success_count={success_count} reached success_min={args.success_min}",
                flush=True,
            )
            break
        if fail_count > max_failures:
            print(
                f"[batch] early-stop fail_count={fail_count} exceeds max_failures={max_failures}",
                flush=True,
            )
            break

    success_runs = [r for r in run_results if r.success]
    fail_runs = [r for r in run_results if not r.success]
    batch_success = len(success_runs) >= args.success_min

    manifest_path = batch_dir / "manifest.csv"
    with manifest_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "result_dir", "success", "reason"])
        for result in run_results:
            writer.writerow([result.index, str(result.result_dir), int(result.success), result.reason])

    summary_path = batch_dir / "batch_summary.txt"
    with summary_path.open("w", encoding="utf-8") as file:
        file.write(f"mode={args.mode}\n")
        file.write(f"repeats={args.repeats}\n")
        file.write(f"success_min={args.success_min}\n")
        file.write(f"success_count={len(success_runs)}\n")
        file.write(f"fail_count={len(fail_runs)}\n")
        file.write(f"batch_success={str(batch_success).lower()}\n")
        if args.mode == "orbit_only":
            file.write(f"radius_std_max={args.radius_std_max}\n")
            file.write(f"radius_abs_error_max={args.radius_abs_error_max}\n")
            file.write(f"altitude_abs_error_max={args.altitude_abs_error_max}\n")
            file.write(f"min_wait_rows={args.min_wait_rows}\n")
        for result in run_results:
            file.write(
                f"run_{result.index}={result.result_dir}|{'PASS' if result.success else 'FAIL'}|{result.reason}\n"
            )

    if not batch_success:
        for result in success_runs:
            if result.result_dir.exists():
                shutil.rmtree(result.result_dir, ignore_errors=True)

    print(batch_dir)
    print(summary_path)
    print(f"success_count={len(success_runs)} fail_count={len(fail_runs)} batch_success={batch_success}")
    return 0 if batch_success else 1


if __name__ == "__main__":
    sys.exit(main())
