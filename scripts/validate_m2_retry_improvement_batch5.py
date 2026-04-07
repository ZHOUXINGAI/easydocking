#!/usr/bin/env python3

"""
M2 regression gate: run repeated PX4 SIH experiments and verify the passive retry path
(DOCKING->TRACKING->DOCKING) improves terminal lateral error when it occurs.

We follow the project rule: after a change, run 5 times; if all PASS, the change is
considered reasonable and we proceed.

Evaluation zone:
- phase in {TRACKING, DOCKING}
- relative_distance <= 10.0 m
- rel_z in [0.20, 1.20] m

Per-run PASS criteria (either path is acceptable):
A) No retry (docking_entries==1): min_lat < 0.35 m
B) Retry observed (docking_entries>=2):
   - second_entry_lat_abs <= 0.70 * first_entry_lat_abs
   - second_entry_lat_abs < 0.90 m

Batch PASS criteria:
- 5 valid runs collected
- all 5 runs PASS
- at least 1 run uses path B (retry observed)
"""

from __future__ import annotations

import csv
import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent
RUN_SCRIPT = ROOT_DIR / "scripts" / "run_px4_sih_docking_experiment.sh"

EVAL_MAX_DISTANCE_M = 10.0
EVAL_Z_MIN_M = 0.20
EVAL_Z_MAX_M = 1.20

NO_RETRY_PASS_MIN_LAT_M = 0.35
RETRY_IMPROVE_RATIO = 0.70
RETRY_SECOND_LAT_MAX_M = 0.90


def safe_float(value: str | None) -> float:
    if value is None:
        return math.nan
    text = str(value).strip()
    if not text or text.lower() == "nan":
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


@dataclass
class RunEval:
    run_dir: Path
    ok: bool
    reason: str
    min_lat: float
    first_entry_lat: float
    second_entry_lat: float
    docking_entries: int


def run_experiment(attempt: int) -> Path:
    env = os.environ.copy()
    env.setdefault("AUTO_START_TIMEOUT_SEC", "160.0")
    completed = subprocess.run(
        [str(RUN_SCRIPT)],
        cwd=str(ROOT_DIR),
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError(f"attempt {attempt}: no stdout from {RUN_SCRIPT}")
    run_dir = Path(lines[-1])
    if not run_dir.exists():
        raise RuntimeError(f"attempt {attempt}: result dir does not exist: {run_dir}")
    return run_dir


def eval_run(run_dir: Path) -> RunEval:
    rows_path = run_dir / "docking_log.csv"
    if not rows_path.exists():
        return RunEval(run_dir, False, "missing_csv", math.nan, math.nan, math.nan, 0)

    with rows_path.open() as f:
        rows = list(csv.DictReader(f))

    entry_lats: list[float] = []
    prev_phase = ""
    min_lat = math.inf
    ever_eval = False

    for row in rows:
        phase = (row.get("phase") or "").strip()
        if phase == "DOCKING" and prev_phase != "DOCKING":
            lat = abs(safe_float(row.get("controller_terminal_lateral_error")))
            entry_lats.append(lat if math.isfinite(lat) else math.nan)
        if phase:
            prev_phase = phase

        if phase not in {"TRACKING", "DOCKING"}:
            continue

        dist = safe_float(row.get("relative_distance"))
        z = safe_float(row.get("rel_z"))
        if not (math.isfinite(dist) and dist <= EVAL_MAX_DISTANCE_M):
            continue
        if not (math.isfinite(z) and EVAL_Z_MIN_M <= z <= EVAL_Z_MAX_M):
            continue

        lat = abs(safe_float(row.get("controller_terminal_lateral_error")))
        if not math.isfinite(lat):
            continue
        ever_eval = True
        min_lat = min(min_lat, lat)

    if not ever_eval or min_lat == math.inf:
        return RunEval(run_dir, False, "no_eval_samples", math.nan, math.nan, math.nan, len(entry_lats))

    docking_entries = len(entry_lats)
    first = entry_lats[0] if docking_entries >= 1 else math.nan
    second = entry_lats[1] if docking_entries >= 2 else math.nan

    # Path A: no retry
    if docking_entries <= 1:
        ok = min_lat < NO_RETRY_PASS_MIN_LAT_M
        reason = f"no_retry min_lat={min_lat:.3f}"
        return RunEval(run_dir, ok, reason, min_lat, first, second, docking_entries)

    # Path B: retry observed
    if not (math.isfinite(first) and math.isfinite(second) and first > 1e-6):
        return RunEval(run_dir, False, "retry_invalid_entry_lat", min_lat, first, second, docking_entries)

    improved = second <= RETRY_IMPROVE_RATIO * first
    bounded = second < RETRY_SECOND_LAT_MAX_M
    ok = improved and bounded
    reason = f"retry first={first:.3f} second={second:.3f} improved={improved} bounded={bounded}"
    return RunEval(run_dir, ok, reason, min_lat, first, second, docking_entries)


def main() -> int:
    target_valid = 5
    max_attempts = 10
    results: list[RunEval] = []
    retry_seen = False

    attempt = 0
    while len(results) < target_valid and attempt < max_attempts:
        attempt += 1
        print(f"[m2] run {len(results)+1}/{target_valid} (attempt {attempt}/{max_attempts})", flush=True)
        run_dir = run_experiment(attempt)
        r = eval_run(run_dir)
        if r.reason == "no_eval_samples":
            print(f"  -> SKIP {run_dir.name} (no eval samples)", flush=True)
            continue
        results.append(r)
        if r.docking_entries >= 2:
            retry_seen = True
        status = "PASS" if r.ok else "FAIL"
        print(
            f"  -> {status} {run_dir.name} "
            f"entries={r.docking_entries} min_lat={r.min_lat:.3f} "
            f"first={r.first_entry_lat:.3f} second={r.second_entry_lat:.3f} :: {r.reason}",
            flush=True,
        )

    if len(results) < target_valid:
        print(f"[m2] valid_runs={len(results)}<{target_valid}; batch FAIL", flush=True)
        return 1

    all_ok = all(r.ok for r in results)
    if not retry_seen:
        all_ok = False
        print("[m2] no retry observed in valid runs; batch FAIL", flush=True)

    print(f"[m2] all_pass={str(all_ok).lower()}", flush=True)
    if not all_ok:
        failed = [r.run_dir.name for r in results if not r.ok]
        print(f"[m2] failed_runs={','.join(failed)}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

